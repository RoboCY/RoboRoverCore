/*
  RoboRover ESP-01S WiFi Joystick + Console Bridge
  ------------------------------------------------
  - AP-only, open network (no password)
  - HTTP on port 80 serves a single-page landscape UI
  - WebSocket on port 81 for low-latency control + console
  - UART bridge to Kypruino (line-based protocol)

  Protocol to Kypruino (newline-terminated):
    - M <L> <R>          (L/R in -255..255)
    - STOP
    - CALL <ID>          (LIGHTMODE, DANCE, BEEP, PIANO, G1, G2)
    - TXT <anything...>  (freeform)

  Telemetry from Kypruino (newline-terminated) is forwarded to web console as-is.
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

ESP8266WebServer server(80);
WebSocketsServer ws(81);

static const uint32_t UART_BAUD = 57600;

static String apSsid;
static String uartLineBuf;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
  <title>RoboRover WiFi Control</title>
  <style>
    :root { --bg:#0b0f14; --panel:#121a24; --muted:#7e8aa0; --txt:#e7eefc; --accent:#3aa0ff; --danger:#ff4d4d; }
    html,body { height:100%; margin:0; background:var(--bg); color:var(--txt); font-family:system-ui, -apple-system, Segoe UI, Roboto, Arial; }
    .wrap { display:flex; height:100%; gap:10px; padding:10px; box-sizing:border-box; }
    .left { flex: 1.3; background:var(--panel); border-radius:16px; position:relative; overflow:hidden; }
    .right { flex: 1; display:flex; flex-direction:column; gap:10px; min-width: 320px; }
    .card { background:var(--panel); border-radius:16px; padding:12px; box-sizing:border-box; }

    .row { display:flex; align-items:center; gap:10px; }
    .row > * { flex:1; }
    .btn { border:0; border-radius:12px; padding:12px 10px; font-weight:700; background:#1c2736; color:var(--txt); }
    .btn:active { transform:scale(0.99); }
    .btnStop { background:var(--danger); }
    .btnOn { outline:2px solid var(--accent); }

    .grid { display:grid; grid-template-columns: 1fr 1fr; gap:10px; }
    .small { font-size:12px; color:var(--muted); }
    input[type="range"] { width:100%; }

    .hud { position:absolute; left:12px; top:12px; background:rgba(0,0,0,.35); padding:8px 10px; border-radius:12px; font-size:12px; }
    .dot { display:inline-block; width:10px; height:10px; border-radius:99px; background:#555; margin-right:8px; vertical-align:middle; }
    .dot.on { background:#3dff88; }
    .joy { position:absolute; inset:0; touch-action:none; }
    .stick { position:absolute; width:80px; height:80px; border-radius:999px; background:rgba(58,160,255,.25); border:2px solid rgba(58,160,255,.6); transform:translate(-50%,-50%); pointer-events:none; display:none; }
    .center { position:absolute; left:50%; top:50%; width:10px; height:10px; background:rgba(255,255,255,.25); border-radius:99px; transform:translate(-50%,-50%); }

    .console { height: 34vh; display:flex; flex-direction:column; gap:8px; }
    .log { flex:1; background:#0a0e13; border-radius:12px; padding:10px; overflow:auto; font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size:12px; }
    .log .t { color:#9ad0ff; }
    .log .e { color:#ff9a9a; }
    .log .l { color:#cfd7e6; }
    .sendRow { display:flex; gap:8px; }
    .sendRow input { flex:1; padding:10px; border-radius:12px; border:1px solid #263244; background:#0a0e13; color:var(--txt); }

    .pillRow { display:flex; gap:8px; flex-wrap:wrap; margin-top:8px; }
    .pill { background:#0a0e13; border:1px solid #263244; padding:6px 10px; border-radius:999px; font-size:12px; color:var(--muted); }

    .statusLine{
      font-size:11px;
      color:var(--muted);
      white-space:nowrap;
      overflow:hidden;
      text-overflow:ellipsis;
      max-width: 260px;
      text-align:right;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="left" id="joyArea">
      <div class="hud">
        <span class="dot" id="dot"></span><span id="net">disconnected</span><br>
        <span class="small" id="mix">L 0 | R 0</span>
      </div>
      <div class="center"></div>
      <div class="stick" id="stick"></div>
      <div class="joy" id="joy"></div>
    </div>

    <div class="right">
      <div class="card">
        <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:8px;">
          <div style="font-weight:800;">Control</div>
          <div id="statusLine" class="statusLine">mode: -- | ws: -- | last: --</div>
        </div>

        <div style="margin-top:10px;">
          <div class="row">
            <div>
              <div style="font-weight:700;">Speed</div>
              <div class="small">Scales joystick output</div>
            </div>
            <div style="flex:1.5;">
              <input id="spd" type="range" min="40" max="255" value="200">
              <div class="small"><span id="spdVal">200</span>/255</div>
            </div>
          </div>
        </div>
      </div>

      <div class="card">
        <div style="font-weight:800; margin-bottom:10px;">Functions</div>
        <div class="grid">
          <button class="btn" data-call="LIGHTMODE">Light Mode</button>
          <button class="btn" data-call="DANCE" id="danceBtn">Robot Dance</button>
          <button class="btn" data-call="BEEP">Beep</button>
          <button class="btn" data-call="PIANO">Piano Demo</button>
          <button class="btn" data-call="G1">Custom A</button>
          <button class="btn" data-call="G2">Custom B</button>
        </div>
      </div>
    </div>
  </div>

<script>
(() => {
  const dot = document.getElementById('dot');
  const net = document.getElementById('net');
  const mixEl = document.getElementById('mix');
  const stick = document.getElementById('stick');
  const joy = document.getElementById('joy');

  const spd = document.getElementById('spd');
  const spdVal = document.getElementById('spdVal');
  const danceBtn = document.getElementById('danceBtn');

  const statusLine = document.getElementById('statusLine');

  let ws;

  let modeStr = "--";
  let wsStr = "down";
  let lastTelemMs = 0;

  let active = false;
  let cx = 0, cy = 0;
  let vx = 0, vy = 0;  // -1..1
  let lastSent = 0;
  let lastL = 0, lastR = 0;

  function clamp(v, a, b){ return Math.max(a, Math.min(b, v)); }
  function now(){ return Date.now(); }

  function updateStatusLine(ageOverride=null){
    if (!statusLine) return;
    let lastStr = "--";
    if (lastTelemMs) {
      const age = (ageOverride !== null) ? ageOverride : ((Date.now() - lastTelemMs)/1000);
      lastStr = age.toFixed(1) + "s";
    }
    statusLine.textContent = `mode: ${modeStr} | ws: ${wsStr} | last: ${lastStr}`;
  }

  function wsSend(s){
    if (ws && ws.readyState === 1) ws.send(s);
  }

  function connect(){
    const host = location.hostname || "192.168.4.1";
    ws = new WebSocket(`ws://${host}:81/`);

    ws.onopen = () => {
      dot.classList.add('on'); net.textContent = "connected";
      wsStr = "ok";
      updateStatusLine();
      wsSend("HELLO");
    };

    ws.onclose = () => {
      dot.classList.remove('on'); net.textContent = "disconnected";
      wsStr = "down";
      updateStatusLine();
      setTimeout(connect, 500); // Faster reconnect
    };

    ws.onerror = (err) => {
      dot.classList.remove('on'); net.textContent = "error";
      wsStr = "error";
      updateStatusLine();
    };

    ws.onmessage = (ev) => {
      const msg = String(ev.data || "");
      if (msg.startsWith("S ")) {
        const line = msg.slice(2);

        // Telemetry parsing (minimal format: "T <mode>")
        if (line.startsWith("T ")) {
          lastTelemMs = Date.now();

          const mode = line.substring(2).trim();

          // mode: 0=teleop, 1=lightfollow, 2=dance
          if (mode === '0') modeStr = 'teleop';
          else if (mode === '1') modeStr = 'lightfollow';
          else if (mode === '2') modeStr = 'dance';

          if (modeStr === "dance") danceBtn.classList.add('btnOn');
          if (modeStr === "teleop") danceBtn.classList.remove('btnOn');

          updateStatusLine(0.0);
        }
      }
    };
  }

  function arcadeMix(x, y){
    // x,y in -1..1
    let L = y + x;
    let R = y - x;
    L = clamp(L, -1, 1);
    R = clamp(R, -1, 1);

    let maxPWM = parseInt(spd.value, 10) || 200;

    const l255 = Math.round(L * maxPWM);
    const r255 = Math.round(R * maxPWM);
    return [l255, r255];
  }

  function sendMotors(force=false){
    const t = now();
    if (!force && (t - lastSent) < 50) return; // ~20Hz rate limit (reduced WiFi load)
    
    const [l255, r255] = arcadeMix(vx, vy);
    mixEl.textContent = `L ${l255} | R ${r255}`;

    // When active (joystick held), always send to keep dead-man timer alive
    // When inactive, only send if values actually changed
    const shouldSend = force || active || (l255 !== lastL || r255 !== lastR);
    
    if (shouldSend) {
      lastSent = t;
      lastL = l255;
      lastR = r255;
      wsSend(`M ${l255} ${r255}`);
    }
  }

  function stop(){
    vx = 0; vy = 0;
    lastL = 0; lastR = 0;
    mixEl.textContent = `L 0 | R 0`;
    wsSend("STOP");
  }

  // Joystick touch / mouse
  function pointerDown(e){
    active = true;
    const r = joy.getBoundingClientRect();
    cx = r.left + r.width/2;
    cy = r.top + r.height/2;
    stick.style.display = 'block';
    pointerMove(e);
    sendMotors(true);
  }

  function pointerMove(e){
    if (!active) return;
    const p = (e.touches && e.touches[0]) ? e.touches[0] : e;
    const rect = joy.getBoundingClientRect();
    const centerX = rect.left + rect.width/2;
    const centerY = rect.top + rect.height/2;

    const dx = p.clientX - centerX;
    const dy = p.clientY - centerY;

    const r = Math.min(Math.max(joy.clientWidth, joy.clientHeight) * 0.28, 180);
    const nx = clamp(dx / r, -1, 1);
    const ny = clamp(dy / r, -1, 1);

    vx = -nx; // Invert: push left = turn left
    vy = -ny; // Invert: push up = forward

    stick.style.left = (rect.width/2 + nx * r) + "px";
    stick.style.top  = (rect.height/2 + ny * r) + "px";
  }

  function pointerUp(){
    if (!active) return;
    active = false;
    stick.style.display = 'none';
    stop();
  }

  joy.addEventListener('touchstart', (e)=>{ e.preventDefault(); pointerDown(e); }, {passive:false});
  joy.addEventListener('touchmove',  (e)=>{ e.preventDefault(); pointerMove(e); }, {passive:false});
  joy.addEventListener('touchend',   (e)=>{ e.preventDefault(); pointerUp(); }, {passive:false});
  joy.addEventListener('touchcancel', (e)=>{ e.preventDefault(); pointerUp(); }, {passive:false});

  joy.addEventListener('mousedown', (e)=>{ pointerDown(e); });
  window.addEventListener('mousemove', (e)=>{ pointerMove(e); });
  window.addEventListener('mouseup', ()=>{ pointerUp(); });

  // Periodic motor updates while active - 50ms for 20Hz (reduced WiFi load)
  setInterval(()=>{ if (active) sendMotors(false); }, 50);

  // Status line refresh for age
  setInterval(()=>{ updateStatusLine(); }, 250);

  // UI controls
  spd.addEventListener('input', ()=>{ spdVal.textContent = spd.value; if (active) sendMotors(true); });

  document.querySelectorAll('[data-call]').forEach(btn=>{
    btn.addEventListener('click', ()=>{
      const id = btn.getAttribute('data-call');
      wsSend(`CALL ${id}`);
    });
  });

  // Boot
  spdVal.textContent = spd.value;
  updateStatusLine();
  connect();

})();
</script>
</body>
</html>
)HTML";

// ---- Helpers ----
static void sendToKypruinoLine(const String& lineNoNl) {
  Serial.print(lineNoNl);
  Serial.print('\n');
}

static void wsBroadcastLine(const String& s) {
  // WebSockets v2.7.2 requires non-const lvalue String&
  String out = String("S ") + s;
  ws.broadcastTXT(out);
}

static void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleNotFound() {
  server.send(404, "text/plain; charset=utf-8", "Not found");
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) {
    ws.sendTXT(num, "OK connected");
    sendToKypruinoLine("STOP"); // safety
    return;
  }
  if (type == WStype_DISCONNECTED) {
    sendToKypruinoLine("STOP"); // safety
    return;
  }
  if (type != WStype_TEXT) return;

  String msg;
  msg.reserve(len + 1);
  for (size_t i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  if (msg.length() == 0) return;

  // Accept and forward:
  //  - "M L R"
  //  - "STOP"
  //  - "CALL ID"
  //  - "TXT ..."
  //  - else forward raw
  if (msg == "STOP") { sendToKypruinoLine("STOP"); return; }
  if (msg.startsWith("M ")) { sendToKypruinoLine(msg); return; }
  if (msg.startsWith("CALL ")) { sendToKypruinoLine(msg); return; }
  if (msg.startsWith("TXT ")) { sendToKypruinoLine(msg); return; }

  sendToKypruinoLine(msg);
}

void setup() {
  Serial.begin(UART_BAUD);
  delay(50);

  uint32_t chip = ESP.getChipId();
  char ssid[32];
  snprintf(ssid, sizeof(ssid), "RoboRover-%04X", (unsigned)(chip & 0xFFFF));
  apSsid = ssid;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid.c_str()); // open AP
  delay(100);

  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();

  ws.begin();
  ws.onEvent(onWsEvent);

  // Safety stop
  sendToKypruinoLine("STOP");
}

void loop() {
  ws.loop(); // Process WebSocket first for low latency
  server.handleClient();

  // UART -> WS (line-based) - limit processing to prevent blocking
  uint8_t charCount = 0;
  while (Serial.available() && charCount++ < 64) { // Process max 64 chars per loop
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (uartLineBuf.length()) {
        wsBroadcastLine(uartLineBuf);
        uartLineBuf = "";
      }
    } else {
      if (uartLineBuf.length() < 200) uartLineBuf += c; // Limit buffer size
    }
  }
  
  yield(); // Give ESP time for WiFi housekeeping
}
