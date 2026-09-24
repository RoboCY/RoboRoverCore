// ===========================================================================
//  Internet: joins a router (Dashboard + Internet mode), MQTT data link for
//  the Kypruino, optional remote control. The dashboard AP always stays up,
//  so a wrong password or missing router never locks anyone out.
//
//  Kypruino -> ESP lines (handled here, not forwarded to the dashboard):
//    PUB <topic> <data>     publish to <base>/<topic>   ("/topic" = absolute)
//    PUBR <topic> <data>    same, retained
//    SUB <topic>            extra subscription (kept until the ESP restarts)
//    NET <command>          settings, see netCommand()
//  ESP -> Kypruino:
//    MSG <topic> <data>     message on <base>/in/# or a SUB topic
//    NET BOOT | NET WIFI UP <ip> | NET WIFI DOWN | NET MQTT UP | NET MQTT DOWN
//    NET OK ... | NET ERR ... | NET STATUS ...
//
//  MQTT topics under <base>:
//    status   "online"/"offline" (retained, last will)
//    in/#     -> Kypruino as MSG
//    rc       remote control: M <L> <R> | STOP | CALL <ID> | ODO 0 | P <n>
//    out      rover output for the remote page (only while remote is on)
// ===========================================================================

static const uint32_t STA_TIMEOUT_MS  = 20000;
static const uint32_t RC_TIMEOUT_MS   = 1000;  // stop if remote M updates stop
static const size_t   MAX_MSG_LEN     = 110;   // Kypruino SoftwareSerial buffer is small
static const uint8_t  MAX_SUBS        = 6;

enum WifiState : uint8_t { W_OFF, W_CONNECTING, W_UP, W_FAILED };

static NetConfig cfg;   // active settings
static NetConfig edit;  // staged by NET commands, applied by NET SAVE

static WifiState wState = W_OFF;
static uint32_t wAt = 0, wWait = 0, wBackoff = 0;
static const char* wWhy = "";
static bool mdnsStarted = false;

static MiniMqtt mqtt;
static WiFiClient mqttPlain;
static BearSSL::WiFiClientSecure mqttTls;
static bool mqttWasUp = false;
static uint8_t mqttLastState = 255;
static uint32_t mAt = 0, mWait = 0;
static char clientId[24];
static char rcTopic[72], outTopic[72];

static char subs[MAX_SUBS][48];
static uint8_t nSubs = 0;

static bool rcDriving = false;
static uint32_t rcLastM = 0, rcDeniedAt = 0;

struct RateLimit {
  uint32_t at = 0;
  uint8_t n = 0;
  bool ok(uint8_t perSecond) {
    const uint32_t t = millis();
    if (t - at >= 1000) { at = t; n = 0; }
    if (n >= perSecond) return false;
    n++;
    return true;
  }
};
static RateLimit pubRate, outRate;

// Small JSON writer for status messages
struct Json {
  char* b;
  size_t cap, n = 0;
  bool first = true;
  Json(char* buf, size_t size) : b(buf), cap(size) { b[0] = 0; }
  void raw(const char* s) {
    while (*s && n + 1 < cap) b[n++] = *s++;
    b[n] = 0;
  }
  void val(const char* s) {
    raw("\"");
    for (; *s && n + 8 < cap; s++) {
      const char c = *s;
      if (c == '"' || c == '\\') { b[n++] = '\\'; b[n++] = c; }
      else if ((uint8_t)c < 0x20) n += snprintf(b + n, cap - n, "\\u%04x", c);
      else b[n++] = c;
    }
    b[n] = 0;
    raw("\"");
  }
  void key(const char* k) {
    raw(first ? "\"" : ",\"");
    first = false;
    raw(k);
    raw("\":");
  }
  void str(const char* k, const char* v) { key(k); val(v); }
  void num(const char* k, long v) {
    char t[12];
    snprintf(t, sizeof(t), "%ld", v);
    key(k);
    raw(t);
  }
};

// ---- helpers ----
static bool wantSta() { return cfg.mode == NET_INTERNET && cfg.ssid[0]; }

static void topicFor(const char* rel, char* out, size_t n) {
  if (rel[0] == '/') snprintf(out, n, "%s", rel + 1);
  else snprintf(out, n, "%s/%s", cfg.base, rel);
}

static void applyTopics() {
  topicFor("rc", rcTopic, sizeof(rcTopic));
  topicFor("out", outTopic, sizeof(outTopic));
}

static const char* wifiStateName() {
  if (!wantSta()) return "off";
  switch (wState) {
    case W_UP: return "up";
    case W_CONNECTING: return "connecting";
    default: return "failed";
  }
}

static const char* mqttStateName() {
  if (wState != W_UP || !cfg.host[0]) return "off";
  switch (mqtt.state()) {
    case MiniMqtt::UP: return "up";
    case MiniMqtt::WAIT_ACK: return "connecting";
    default: return "down";
  }
}

static const char* mqttErrorText() {
  switch (mqtt.lastError()) {
    case 0: return "";
    case 4: return "wrong user or password";
    case 5: return "not authorized";
    case MiniMqtt::ERR_TIMEOUT: return "no answer";
    case MiniMqtt::ERR_NET: return "cannot reach broker";
    default: return "refused";
  }
}

static void reply(int8_t num, const char* msg) {
  char b[96];
  if (num < 0) {
    snprintf(b, sizeof(b), "NET %s", msg);
    uartLine(b);
  } else {
    const int n = snprintf(b, sizeof(b), "R %s", msg);
    ws.sendTXT(num, b, n);
  }
}

static void publishOut(const char* s) {
  if (mqttWasUp) mqtt.publish(outTopic, s);
}

static void remoteStop() {
  if (!rcDriving) return;
  rcDriving = false;
  uartLine("M 0 0");
}

// ---- status ----
void netSendStatus(int8_t num) {
  char buf[720];
  Json j(buf, sizeof(buf));
  j.raw("N {");
  j.num("mode", cfg.mode);
  j.str("ssid", cfg.ssid);
  j.num("pass", cfg.pass[0] != 0);
  j.str("host", cfg.host);
  j.num("port", cfg.port);
  j.num("tls", cfg.tls);
  j.str("user", cfg.user);
  j.num("mpass", cfg.mpass[0] != 0);
  j.str("base", cfg.base);
  j.num("remote", cfg.remote);
  j.num("rcmax", cfg.rcMax);
  j.str("wifi", wifiStateName());
  j.str("why", wWhy);
  j.str("mdns", hostName);
  if (wState == W_UP) {
    j.str("ip", WiFi.localIP().toString().c_str());
    j.num("rssi", WiFi.RSSI());
  }
  j.str("mqtt", mqttStateName());
  j.str("merr", mqttErrorText());
  j.raw("}");
  if (num < 0) {
    if (ws.connectedClients() > 0) ws.broadcastTXT(buf, j.n);
  } else {
    ws.sendTXT(num, buf, j.n);
  }
}

static void uartStatus() {
  char b[112];
  snprintf(b, sizeof(b), "NET STATUS %s wifi=%s ip=%s mqtt=%s",
           cfg.mode == NET_INTERNET ? "internet" : "dashboard", wifiStateName(),
           wState == W_UP ? WiFi.localIP().toString().c_str() : "-", mqttStateName());
  uartLine(b);
}

// ---- Wi-Fi client ----
static void startSta() {
  WiFi.begin(cfg.ssid, cfg.pass[0] ? cfg.pass : nullptr);
  wState = W_CONNECTING;
  wAt = millis();
  wWhy = "";
  netSendStatus(-1);
}

static void staFailed(const char* why, uint32_t retryMs) {
  WiFi.disconnect();  // stop searching so the dashboard AP stays steady
  wState = W_FAILED;
  wWhy = why;
  wAt = millis();
  wWait = retryMs;
  netSendStatus(-1);
}

static uint32_t nextBackoff() {
  wBackoff = wBackoff ? min<uint32_t>(wBackoff * 2, 300000UL) : 30000UL;
  return wBackoff;
}

static void mqttDown() {
  mqttWasUp = false;
  mAt = millis();
  mWait = 2000;
  remoteStop();
  uartLine("NET MQTT DOWN");
}

static void wifiUp() {
  wState = W_UP;
  wWhy = "";
  wBackoff = 0;
  mAt = millis();
  mWait = 0;
  if (!mdnsStarted && MDNS.begin(hostName)) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
  }
  char b[40];
  snprintf(b, sizeof(b), "NET WIFI UP %s", WiFi.localIP().toString().c_str());
  uartLine(b);
  netSendStatus(-1);
}

static void wifiDown() {
  mqtt.disconnect();
  if (mqttWasUp) mqttDown();
  uartLine("NET WIFI DOWN");
}

static void applyWifi() {
  if (wState == W_UP) wifiDown();
  WiFi.disconnect();
  wState = W_OFF;
  wBackoff = 0;
  if (wantSta()) {
    WiFi.mode(WIFI_AP_STA);
    startSta();
  } else {
    WiFi.mode(WIFI_AP);
  }
}

static void onScanDone(int count) {
  char buf[600];
  Json j(buf, sizeof(buf));
  j.raw("W [");
  // Strongest first, no duplicates, at most 15
  bool used[64] = {false};
  const int n = min(count, 64);
  for (int listed = 0; listed < 15; listed++) {
    int best = -1;
    for (int i = 0; i < n; i++) {
      if (!used[i] && WiFi.SSID(i).length() && (best < 0 || WiFi.RSSI(i) > WiFi.RSSI(best))) best = i;
    }
    if (best < 0) break;
    used[best] = true;
    const String name = WiFi.SSID(best);
    for (int i = 0; i < n; i++) {
      if (!used[i] && WiFi.SSID(i) == name) used[i] = true;
    }
    if (listed) j.raw(",");
    j.val(name.c_str());
  }
  j.raw("]");
  WiFi.scanDelete();
  if (!wantSta()) WiFi.enableSTA(false);
  if (ws.connectedClients() > 0) ws.broadcastTXT(buf, j.n);
}

static void startScan() {
  if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) return;
  if (!(WiFi.getMode() & WIFI_STA)) WiFi.enableSTA(true);
  WiFi.scanNetworksAsync(onScanDone);
}

// ---- MQTT ----
static void onMqttMessage(const char* topic, const uint8_t* p, size_t len);

static void mqttConnect() {
  Client* c = &mqttPlain;
  if (cfg.tls) {
    mqttTls.setInsecure();
    mqttTls.setBufferSizes(1024, 512);
    mqttTls.setTimeout(4000);
    mqttTls.setNoDelay(true);
    c = &mqttTls;
  } else {
    mqttPlain.setTimeout(2500);
    mqttPlain.setNoDelay(true);
  }
  char will[80];
  topicFor("status", will, sizeof(will));
  mqtt.connect(*c, cfg.host, cfg.port, clientId, cfg.user, cfg.mpass, will, "offline");
  mAt = millis();
  mWait = mWait < 5000 ? 5000 : min<uint32_t>(mWait * 2, 60000UL);
}

static void mqttUp() {
  mqttWasUp = true;
  mWait = 0;
  char t[80];
  topicFor("status", t, sizeof(t));
  mqtt.publish(t, "online", true);
  topicFor("in/#", t, sizeof(t));
  mqtt.subscribe(t);
  mqtt.subscribe(rcTopic);
  for (uint8_t i = 0; i < nSubs; i++) {
    topicFor(subs[i], t, sizeof(t));
    mqtt.subscribe(t);
  }
  uartLine("NET MQTT UP");
}

static void mqttLoop() {
  if (!cfg.host[0]) return;
  mqtt.loop();
  const bool isUp = mqtt.state() == MiniMqtt::UP;
  if (isUp && !mqttWasUp) mqttUp();
  if (!isUp && mqttWasUp) mqttDown();
  // Reconnecting can block for a moment, so never while someone drives locally
  if (mqtt.state() == MiniMqtt::DOWN && millis() - mAt >= mWait && !localDriving()) mqttConnect();
  if (mqtt.state() != mqttLastState) {
    mqttLastState = mqtt.state();
    netSendStatus(-1);
  }
}

static void remoteCommand(const uint8_t* p, size_t len) {
  char cmd[48];
  while (len && (uint8_t)p[len - 1] <= ' ') len--;
  if (len == 0 || len >= sizeof(cmd)) return;
  memcpy(cmd, p, len);
  cmd[len] = 0;
  const uint32_t now = millis();

  if (cmd[0] == 'P' && (cmd[1] == 0 || cmd[1] == ' ')) {  // heartbeat: answer even when disabled
    cmd[0] = 'p';
    publishOut(cmd);
    if (cfg.remote) forwardPing();
    return;
  }
  if (!cfg.remote) {
    if (now - rcDeniedAt > 5000) {
      rcDeniedAt = now;
      publishOut("S ERR remote control is turned off on this rover");
    }
    return;
  }
  if (strcmp(cmd, "STOP") == 0) {
    rcDriving = false;
    uartLine("STOP");
    return;
  }
  if (strncmp(cmd, "CALL ", 5) == 0 || strcmp(cmd, "ODO 0") == 0) {
    uartLine(cmd);
    return;
  }
  if (cmd[0] == 'M' && cmd[1] == ' ') {
    char* end;
    long l = strtol(cmd + 2, &end, 10);
    long r = strtol(end, nullptr, 10);
    l = constrain(l, -255L, 255L) * cfg.rcMax / 100;
    r = constrain(r, -255L, 255L) * cfg.rcMax / 100;
    rcLastM = now;
    if (localDriving()) return;  // the dashboard in the room wins
    char b[24];
    snprintf(b, sizeof(b), "M %ld %ld", l, r);
    uartLine(b);
    rcDriving = l || r;
    return;
  }
  publishOut("S ERR remote: only M, STOP, CALL and ODO are allowed");
}

static void onMqttMessage(const char* topic, const uint8_t* p, size_t len) {
  if (strcmp(topic, rcTopic) == 0) {
    remoteCommand(p, len);
    return;
  }
  // "S MSG <topic> <payload>" - prefix kept so the same buffer feeds the dashboard console
  char line[2 + MAX_MSG_LEN + 1];
  const size_t bl = strlen(cfg.base);
  const bool under = strncmp(topic, cfg.base, bl) == 0 && topic[bl] == '/';
  int n = snprintf(line, sizeof(line), under ? "S MSG %s " : "S MSG /%s ", under ? topic + bl + 1 : topic);
  if (n < 0 || n >= (int)sizeof(line) - 1) return;
  for (size_t i = 0; i < len && n < (int)sizeof(line) - 1; i++) {
    line[n++] = p[i] < ' ' ? ' ' : (char)p[i];
  }
  line[n] = 0;
  uartLine(line + 2, n - 2);
  if (ws.connectedClients() > 0) ws.broadcastTXT(line, n);
}

// ---- commands from Kypruino / dashboard ----
static bool setStr(char* dst, size_t cap, const char* v) {
  if (strlen(v) >= cap) return false;
  strcpy(dst, v);
  return true;
}
#define SET_STR(field, v) setStr(edit.field, sizeof(edit.field), v)

static void saveConfig(int8_t num) {
  if (memcmp(&cfg, &edit, sizeof(cfg)) == 0) {
    reply(num, "OK unchanged");
    return;
  }
  const NetConfig old = cfg;
  cfg = edit;
  netCfgSave(cfg);
  edit = cfg;
  applyTopics();
  reply(num, "OK saved");

  if (mqtt.state() != MiniMqtt::DOWN || mqttWasUp) {
    mqtt.disconnect();
    if (mqttWasUp) mqttDown();
  }
  mWait = 0;
  if (old.mode != cfg.mode || strcmp(old.ssid, cfg.ssid) || strcmp(old.pass, cfg.pass)) applyWifi();
  netSendStatus(-1);
}

// args: "<KEY> [value...]"
void netCommand(const char* a, int8_t num) {
  while (*a == ' ') a++;
  const char* sp = strchr(a, ' ');
  const size_t kl = sp ? (size_t)(sp - a) : strlen(a);
  char key[8];
  if (kl >= sizeof(key)) { reply(num, "ERR unknown command"); return; }
  memcpy(key, a, kl);
  key[kl] = 0;
  const char* v = sp ? sp + 1 : "";

  if (!kl || !strcasecmp(key, "GET") || !strcasecmp(key, "STATUS")) {
    if (num < 0) uartStatus(); else netSendStatus(num);
    return;
  }
  // Kypruino: ignore replies/events echoed back by old firmware
  if (!strcasecmp(key, "OK") || !strcasecmp(key, "ERR")) return;
  if (!strcasecmp(key, "BEGIN")) { edit = cfg; return; }
  if (!strcasecmp(key, "SAVE")) { saveConfig(num); return; }
  if (!strcasecmp(key, "SCAN")) { startScan(); return; }
  if (!strcasecmp(key, "FORGET")) {
    edit = cfg;
    edit.ssid[0] = edit.pass[0] = 0;
    edit.mode = NET_DASHBOARD;
    saveConfig(num);
    return;
  }
  if (!strcasecmp(key, "RESET")) {
    netCfgDefaults(edit, ESP.getChipId());
    saveConfig(num);
    return;
  }

  bool ok = true;
  if (!strcasecmp(key, "MODE")) {
    if (!strcasecmp(v, "internet") || !strcmp(v, "1")) edit.mode = NET_INTERNET;
    else if (!strcasecmp(v, "dashboard") || !strcmp(v, "0")) edit.mode = NET_DASHBOARD;
    else ok = false;
  } else if (!strcasecmp(key, "SSID")) {
    ok = SET_STR(ssid, v);
  } else if (!strcasecmp(key, "PASS")) {
    const size_t l = strlen(v);
    ok = (l == 0 || (l >= 8 && l <= 63)) && SET_STR(pass, v);
  } else if (!strcasecmp(key, "BROKER")) {  // host [port]
    char host[sizeof(edit.host)];
    const char* ps = strchr(v, ' ');
    const size_t hl = ps ? (size_t)(ps - v) : strlen(v);
    ok = hl < sizeof(host);
    if (ok) {
      memcpy(host, v, hl);
      host[hl] = 0;
      strcpy(edit.host, host);
      if (ps) {
        const long port = atol(ps + 1);
        ok = port > 0 && port < 65536;
        if (ok) edit.port = port;
      }
    }
  } else if (!strcasecmp(key, "PORT")) {
    const long port = atol(v);
    ok = port > 0 && port < 65536;
    if (ok) edit.port = port;
  } else if (!strcasecmp(key, "TLS")) {
    edit.tls = atoi(v) != 0;
  } else if (!strcasecmp(key, "USER")) {
    ok = SET_STR(user, v);
  } else if (!strcasecmp(key, "MPASS")) {
    ok = SET_STR(mpass, v);
  } else if (!strcasecmp(key, "BASE")) {
    size_t l = strlen(v);
    while (l && v[l - 1] == '/') l--;
    ok = l > 0 && l < sizeof(edit.base) && !strpbrk(v, "#+") && v[0] != '/';
    if (ok) {
      memcpy(edit.base, v, l);
      edit.base[l] = 0;
    }
  } else if (!strcasecmp(key, "REMOTE")) {
    edit.remote = atoi(v) != 0;
  } else if (!strcasecmp(key, "RCMAX")) {
    const int m = atoi(v);
    ok = m >= 10 && m <= 100;
    if (ok) edit.rcMax = m;
  } else {
    reply(num, "ERR unknown command");
    return;
  }
  if (!ok) {
    char b[40];
    snprintf(b, sizeof(b), "ERR bad value for %s", key);
    reply(num, b);
  }
}

static void kypPub(const char* s, bool retain) {
  while (*s == ' ') s++;
  const char* sp = strchr(s, ' ');
  const size_t tl = sp ? (size_t)(sp - s) : strlen(s);
  char rel[64], topic[128];
  if (!tl || tl >= sizeof(rel)) return;
  if (!mqttWasUp || !pubRate.ok(20)) return;
  memcpy(rel, s, tl);
  rel[tl] = 0;
  topicFor(rel, topic, sizeof(topic));
  mqtt.publish(topic, sp ? sp + 1 : "", retain);
}

static void kypSub(const char* s) {
  while (*s == ' ') s++;
  if (!*s || strlen(s) >= sizeof(subs[0])) { uartLine("NET ERR bad topic"); return; }
  for (uint8_t i = 0; i < nSubs; i++) {
    if (strcmp(subs[i], s) == 0) return;
  }
  if (nSubs >= MAX_SUBS) { uartLine("NET ERR too many subscriptions"); return; }
  strcpy(subs[nSubs++], s);
  if (mqttWasUp) {
    char t[80];
    topicFor(s, t, sizeof(t));
    mqtt.subscribe(t);
  }
}

// ---- entry points used by the main sketch ----
bool netHandleKypruinoLine(const char* line) {
  if (!strncmp(line, "NET ", 4)) { netCommand(line + 4, -1); return true; }
  if (!strcmp(line, "NET")) { uartStatus(); return true; }
  if (!strncmp(line, "PUB ", 4)) { kypPub(line + 4, false); return true; }
  if (!strncmp(line, "PUBR ", 5)) { kypPub(line + 5, true); return true; }
  if (!strncmp(line, "SUB ", 4)) { kypSub(line + 4); return true; }
  return false;
}

// Kypruino output ("S ..." line) -> remote page
void netMirrorLine(const char* sLine, size_t len) {
  if (cfg.remote && mqttWasUp && outRate.ok(10)) mqtt.publish(outTopic, (const uint8_t*)sLine, len);
}

void netBegin() {
  netCfgLoad(cfg, ESP.getChipId());
  edit = cfg;
  applyTopics();
  snprintf(clientId, sizeof(clientId), "roborover-%06x", (unsigned)ESP.getChipId());
  mqtt.onMessage(onMqttMessage);

  WiFi.persistent(false);  // settings live in NetConfig, not the SDK's flash area
  WiFi.setAutoReconnect(false);
  // Never auto-join a network some older sketch saved (checked first: the SDK stores this flag in flash)
  if (WiFi.getAutoConnect()) WiFi.setAutoConnect(false);
  WiFi.mode(wantSta() ? WIFI_AP_STA : WIFI_AP);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.hostname(hostName);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);
  WiFi.softAP(apSsid, nullptr, AP_CHANNEL, false, AP_MAX_CLIENTS);
  if (wantSta()) startSta();
}

void netLoop() {
  const uint32_t now = millis();
  if (rcDriving && now - rcLastM > RC_TIMEOUT_MS) remoteStop();
  if (!wantSta()) return;
  if (mdnsStarted) MDNS.update();

  const wl_status_t st = WiFi.status();
  switch (wState) {
    case W_CONNECTING:
      if (st == WL_CONNECTED) wifiUp();
      else if (st == WL_WRONG_PASSWORD) staFailed("wrong password", 300000UL);
      else if (now - wAt > STA_TIMEOUT_MS) staFailed(st == WL_NO_SSID_AVAIL ? "network not found" : "no answer", nextBackoff());
      break;
    case W_UP:
      if (st != WL_CONNECTED) {
        wifiDown();
        staFailed("connection lost", 3000);
      } else {
        mqttLoop();
      }
      break;
    case W_FAILED:
      // Searching makes the dashboard AP stutter, so wait while someone drives
      if (now - wAt >= wWait && !localDriving()) startSta();
      break;
    default:
      startSta();
      break;
  }
}
