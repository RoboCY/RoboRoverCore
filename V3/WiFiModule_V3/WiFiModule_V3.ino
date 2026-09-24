/*
  WiFiModule_V3 - RoboRover ESP-01S WiFi Command Center
  -----------------------------------------------------
  The firmware of the rover's WiFi module. It comes already programmed;
  reflash it only if it is blank or was overwritten - see README.md here.
  11_WiFiDashboard_V3 and 12_UnboxingDemo_V3 are the Kypruino side.

  - Open WiFi access point "RoboRover-XXXX" at 192.168.4.1, always on
  - No sign-in pop-up: the phones' "is there internet?" checks (Android,
    iPhone, Windows, Firefox) get the answer they expect, so no captive-portal
    window opens - it cannot rotate to landscape anyway. Users open
    http://192.168.4.1 in Chrome / Safari. Any other address typed in the
    browser still lands on the controller.
  - HTTP :80 serves the landscape web app (gzipped, from index_html.h)
      /             controller
      /remote.html  same app as a download, drives the rover over MQTT
  - WebSocket :81 for low-latency control, UART bridge to the Kypruino
  - Optional "Dashboard + Internet" mode (Internet.ino): joins a router,
    MQTT data link for the Kypruino, optional remote control.
    Configured at runtime from the dashboard or the Kypruino, no re-flashing.

  Web UI source lives in web/index.html. After editing it run:
      python tools/build_web.py        (regenerates index_html.h)

  Browser -> ESP (WebSocket text):
    P <n>          heartbeat; ESP answers "p <n>" and sends PING to the Kypruino
    NET <command>  network settings (handled by the ESP, see Internet.ino)
    anything else  forwarded to the Kypruino as one line:
                   M <L> <R> | STOP | CALL <ID> | ODO 0 | TXT ... | SET <key> <val>

  ESP -> Browser:
    H <ssid>       sent on connect
    N {json}       network settings + status
    R <text>       reply to a NET command (OK ... / ERR ...)
    W [ssids]      Wi-Fi scan result
    p <n>          heartbeat reply (browser shows round-trip time)
    S <line>       line received from the Kypruino (T <mode>, D <distance>,
                   V <cm/s> <cm driven>, LOG ..., ERR ...)

  Safety: STOP is sent to the Kypruino on boot and whenever a dashboard
  connects or disconnects. The Kypruino's own dead-man timer stops the motors
  if M updates stop arriving.

  Build: Generic ESP8266 Module, 1MB flash (FS 64KB), CPU 160 MHz (makes TLS
  faster). Library: WebSockets (Markus Sattler).
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <WiFiClientSecureBearSSL.h>
#include <WebSocketsServer.h>
#include "index_html.h"
#include "NetConfig.h"
#include "MiniMqtt.h"

#define CAPTIVE_PORTAL 1  // 0 = only http://192.168.4.1 serves the UI

static const uint32_t UART_BAUD       = 19200;  // Kypruino side is software serial: slower is far more reliable
static const uint8_t  AP_CHANNEL      = 6;  // follows the router's channel once connected
static const uint8_t  AP_MAX_CLIENTS  = 4;
static const uint32_t PING_FWD_MIN_MS = 500;   // Kypruino auto modes time out after 2 s of silence
static const uint32_t LOCAL_DRIVE_MS  = 1500;  // dashboard driving blocks remote/reconnects this long
static const size_t   MAX_CMD_LEN     = 120;   // Kypruino line buffer is 128

static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);

ESP8266WebServer server(80);
WebSocketsServer ws(81);
DNSServer dns;

static char apSsid[24];    // RoboRover-XXXX
static char hostName[16];  // rover-xxxx  (http://rover-xxxx.local on the home network)
static uint32_t lastPingFwdMs = 0;
static uint32_t localDriveMs = 0;
static bool localDriveActive = false;

// UART line buffer, pre-filled with the "S " prefix so lines broadcast without copying
static char rxLine[2 + 160] = "S ";
static uint8_t rxLen = 0;

// Internet.ino
void netBegin();
void netLoop();
void netCommand(const char* args, int8_t wsNum);  // wsNum < 0: from the Kypruino
void netSendStatus(int8_t wsNum);                 // wsNum < 0: all dashboards
bool netHandleKypruinoLine(const char* line);     // true = consumed
void netMirrorLine(const char* sLine, size_t len);

// ---- talking to the Kypruino: take turns ----
// The Kypruino's serial port is software: it cannot listen while it talks, so
// a line that starts while it is talking arrives garbled. Lines for it wait
// here until it is quiet - not part-way through a line of its own. (The
// Kypruino does the same the other way round before it talks.)
static char txQueue[768];
static size_t txQueued = 0;
static uint32_t kypByteUs = 0;   // when the last byte from the Kypruino was read

static void flushToKypruino() {
  if (!txQueued) return;
  if (Serial.available()) return;                // it is talking: read that first
  const uint32_t quietUs = micros() - kypByteUs;
  if (quietUs < 2000) return;                    // a byte only just arrived
  if (rxLen > 0 && quietUs < 30000) return;      // part-way through a line
  Serial.write((const uint8_t*)txQueue, txQueued);
  txQueued = 0;
}

// ---- shared helpers ----
void uartLine(const char* s, size_t len) {
  if (txQueued + len + 1 > sizeof(txQueue)) return;   // full: drop it, the next command follows
  memcpy(txQueue + txQueued, s, len);
  txQueued += len;
  txQueue[txQueued++] = '\n';
  flushToKypruino();
}

void uartLine(const char* s) { uartLine(s, strlen(s)); }

void forwardPing() {
  const uint32_t now = millis();
  if (now - lastPingFwdMs >= PING_FWD_MIN_MS) {
    lastPingFwdMs = now;
    uartLine("PING", 4);
  }
}

bool localDriving() { return localDriveActive && millis() - localDriveMs < LOCAL_DRIVE_MS; }

// Old Kypruino firmware answers ESP-only lines with "ERR unknown cmd: ..."; hide those.
static bool isEchoNoise(const char* line) {
  if (strncmp(line, "ERR unknown cmd: ", 17) != 0) return false;
  const char* c = line + 17;
  return !strncmp(c, "PING", 4) || !strncmp(c, "NET", 3) || !strncmp(c, "MSG", 3);
}

// ---- UART ----
static void pumpUart() {
  int budget = 128;  // bound the time spent here per loop
  while (budget-- > 0 && Serial.available()) {
    const char c = (char)Serial.read();
    kypByteUs = micros();
    if (c == '\r') continue;
    if (c != '\n') {
      if (rxLen < sizeof(rxLine) - 3) rxLine[2 + rxLen++] = c;  // truncate long lines
      continue;
    }
    if (rxLen == 0) continue;
    const size_t n = 2 + rxLen;
    rxLine[n] = '\0';
    rxLen = 0;
    const char* line = rxLine + 2;
    if (isEchoNoise(line) || netHandleKypruinoLine(line)) continue;
    if (ws.connectedClients() > 0) ws.broadcastTXT(rxLine, n);
    netMirrorLine(rxLine, n);
  }
}

// ---- HTTP ----
static void sendApp(bool download) {
  if (!download && server.header("If-None-Match") == INDEX_HTML_ETAG) {
    server.send(304);
    return;
  }
  server.sendHeader(F("Content-Encoding"), F("gzip"));
  if (download) {
    server.sendHeader(F("Content-Disposition"), F("attachment; filename=\"roborover-remote.html\""));
  } else {
    server.sendHeader(F("Cache-Control"), F("no-cache"));
    server.sendHeader(F("ETag"), F(INDEX_HTML_ETAG));
  }
  server.send_P(200, PSTR("text/html; charset=utf-8"), (PGM_P)INDEX_HTML_GZ, INDEX_HTML_GZ_LEN);
}

// Internet checks. Answered the way each system expects, so no sign-in
// pop-up opens (its window cannot rotate to landscape); users open
// http://192.168.4.1 in their browser instead.
//   iPhone / Mac:  captive.apple.com/hotspot-detect.html -> "Success" page
//   Android:       .../generate_204 (and gen_204)          -> empty 204
//   Windows:       msftconnecttest.com/connecttest.txt      -> fixed text
//   Firefox:       detectportal.firefox.com/success.txt     -> "success"
static bool isAppleProbe() {
  const String host = server.hostHeader();
  const String& uri = server.uri();
  return host.indexOf(F("apple.com")) >= 0 || host.indexOf(F("appleiphonecell.com")) >= 0 ||
         host.indexOf(F("ibook.info")) >= 0 || host.indexOf(F("itools.info")) >= 0 ||
         host.indexOf(F("airport.us")) >= 0 || host.indexOf(F("thinkdifferent.us")) >= 0 ||
         uri.endsWith(F("hotspot-detect.html")) || uri.endsWith(F("/success.html"));
}

static bool answerProbe() {
  const String& uri = server.uri();
  const bool known = isAppleProbe() || uri.endsWith(F("generate_204")) || uri.endsWith(F("gen_204")) ||
                     uri.endsWith(F("connecttest.txt")) || uri.endsWith(F("ncsi.txt")) ||
                     uri.endsWith(F("success.txt")) || uri.endsWith(F("canonical.html"));
  if (!known) return false;
  server.sendHeader(F("Cache-Control"), F("no-store"));
  if (isAppleProbe()) {
    server.send(200, F("text/html"), F("<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>"));
  } else if (uri.endsWith(F("generate_204")) || uri.endsWith(F("gen_204"))) {
    server.send(204);
  } else if (uri.endsWith(F("connecttest.txt"))) {
    server.send(200, F("text/plain"), F("Microsoft Connect Test"));
  } else if (uri.endsWith(F("ncsi.txt"))) {
    server.send(200, F("text/plain"), F("Microsoft NCSI"));
  } else {
    server.send(200, F("text/plain"), F("success\n"));   // Firefox and friends
  }
  return true;
}

static void handleNotFound() {
  if (answerProbe()) return;
#if CAPTIVE_PORTAL
  // Any other address typed in a browser on the rover's WiFi: show the controller
  if (server.client().localIP() == AP_IP) {
    server.sendHeader(F("Location"), F("http://192.168.4.1/"));
    server.sendHeader(F("Cache-Control"), F("no-store"));
    server.send(302);
    return;
  }
#endif
  server.send(404, F("text/plain"), F("Not found"));
}

// ---- WebSocket ----
static void noteLocalDrive(const char* p) {
  char* end;
  const long l = strtol(p + 2, &end, 10);
  const long r = strtol(end, nullptr, 10);
  localDriveActive = l || r;
  localDriveMs = millis();
}

static void handleWsText(uint8_t num, char* p, size_t len) {
  while (len && (uint8_t)p[len - 1] <= ' ') len--;
  while (len && (uint8_t)*p <= ' ') { p++; len--; }
  if (len == 0 || len > MAX_CMD_LEN) return;
  p[len] = '\0';  // payload buffer is longer than the trimmed text

  if (p[0] == 'P' && (len == 1 || p[1] == ' ')) {  // heartbeat
    p[0] = 'p';
    ws.sendTXT(num, p, len);
    forwardPing();
    return;
  }

  // One command per message: refuse control characters (no line injection)
  for (size_t i = 0; i < len; i++) {
    if ((uint8_t)p[i] < ' ') return;
  }
  if (!strncmp(p, "NET", 3) && (len == 3 || p[3] == ' ')) {
    netCommand(p + 3, num);
    return;
  }
  if (p[0] == 'M' && p[1] == ' ') noteLocalDrive(p);
  uartLine(p, len);
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED: {
      char hello[32];
      const int n = snprintf(hello, sizeof(hello), "H %s", apSsid);
      ws.sendTXT(num, hello, n);
      netSendStatus(num);
      uartLine("STOP");
      break;
    }
    case WStype_DISCONNECTED:
      localDriveActive = false;
      uartLine("STOP");
      break;
    case WStype_TEXT:
      handleWsText(num, (char*)payload, len);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.setRxBufferSize(1024);  // room for Kypruino NET bursts while MQTT reconnects
  Serial.begin(UART_BAUD);
  // Leading newline flushes any boot noise out of the Kypruino's line buffer
  Serial.print(F("\nSTOP\n"));

  const uint32_t id = ESP.getChipId();
  snprintf(apSsid, sizeof(apSsid), "RoboRover-%04X", (unsigned)(id & 0xFFFF));
  snprintf(hostName, sizeof(hostName), "rover-%04x", (unsigned)(id & 0xFFFF));

  netBegin();  // loads settings, starts the AP (+ router connection)

#if CAPTIVE_PORTAL
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", AP_IP);
#endif

  server.collectHeaders("If-None-Match");
  server.on("/", HTTP_GET, [] { sendApp(false); });
  server.on("/remote.html", HTTP_GET, [] { sendApp(true); });
  server.on("/favicon.ico", HTTP_GET, [] { server.send(204); });
  server.onNotFound(handleNotFound);
  server.begin();

  ws.begin();
  ws.onEvent(onWsEvent);
  ws.enableHeartbeat(2000, 1500, 2);  // drop phones that walked away (-> STOP)

  Serial.print(F("NET BOOT\n"));  // Kypruino code may (re)send its network settings now
}

void loop() {
  ws.loop();  // WebSocket first for lowest control latency
  pumpUart();
  flushToKypruino();
  netLoop();
  server.handleClient();
#if CAPTIVE_PORTAL
  dns.processNextRequest();
#endif
}
