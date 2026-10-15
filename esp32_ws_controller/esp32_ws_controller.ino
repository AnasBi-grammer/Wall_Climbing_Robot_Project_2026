/*
  WallCrawler - Wi-Fi AP + ArduinoOTA + WebSocket control page
  NO actuator code yet: the ESP only receives, stores and prints the control values.

  Libraries to install (Library Manager or ZIP):
    - "ESP Async WebServer" by ESP32Async
    - "Async TCP" by ESP32Async
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ---------- Settings ----------
const char* AP_SSID  = "WallBot";
const char* AP_PASS  = "wallbot123";     // at least 8 characters
const int   AP_CHANNEL = 6;
const int   AP_MAX_CLIENTS = 4;

const char* OTA_HOSTNAME = "wallbot";
const char* OTA_PASS     = "ota1234";

// ---------- Latest values received from the page (range -100..100, mode 0..100, arm 0/1) ----------
volatile int   ctl_lx = 0, ctl_ly = 0, ctl_rx = 0, ctl_ry = 0;
volatile int   ctl_mode = 0;
volatile int   ctl_arm  = 0;
volatile uint32_t lastPacketMs = 0;      // for the future failsafe

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------- The web page (stored in flash, no internet/CDN needed) ----------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>WallCrawler Controller</title>
<style>
  :root { --bg:#111418; --panel:#1b2027; --line:#2c3440; --fg:#e8edf2; --dim:#8b97a6; --acc:#3aa0ff; --ok:#3ddc84; --bad:#ff5c5c; }
  * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
  html,body { margin:0; height:100%; background:var(--bg); color:var(--fg);
              font-family:system-ui,Segoe UI,Roboto,sans-serif; overflow:hidden;
              touch-action:none; user-select:none; -webkit-user-select:none; }
  .wrap { height:100%; display:flex; flex-direction:column; padding:10px; gap:10px; }
  .top { display:flex; align-items:center; justify-content:space-between; gap:10px; }
  .status { display:flex; align-items:center; gap:8px; font-size:14px; }
  .dot { width:12px; height:12px; border-radius:50%; background:var(--bad); }
  .dot.on { background:var(--ok); }
  .mid { display:flex; gap:10px; align-items:center; }
  .mid > * { flex:1; }
  button.arm { padding:12px; font-size:16px; font-weight:600; border-radius:10px; border:2px solid var(--line);
               background:var(--panel); color:var(--fg); }
  button.arm.on { border-color:var(--bad); background:#3a1d1d; color:#ffb3b3; }
  .slider { background:var(--panel); border:1px solid var(--line); border-radius:10px; padding:8px 12px; font-size:13px; color:var(--dim); }
  .slider input { width:100%; margin-top:6px; }
  .sticks { flex:1; display:flex; align-items:center; justify-content:space-around; gap:10px; min-height:0; }
  .stick { position:relative; width:min(42vw, 42vh); height:min(42vw, 42vh); border-radius:50%;
           background:radial-gradient(circle, #232a33 0%, var(--panel) 70%); border:2px solid var(--line); }
  .stick::before, .stick::after { content:""; position:absolute; background:var(--line); }
  .stick::before { left:50%; top:8%; bottom:8%; width:1px; }
  .stick::after  { top:50%; left:8%; right:8%; height:1px; }
  .knob { position:absolute; left:50%; top:50%; width:32%; height:32%; margin:-16% 0 0 -16%;
          border-radius:50%; background:var(--acc); box-shadow:0 2px 12px rgba(0,0,0,.5); }
  .readout { font:12px/1.5 ui-monospace,Menlo,Consolas,monospace; color:var(--dim); text-align:center; }
</style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div class="status"><span class="dot" id="dot"></span><span id="stat">Connecting...</span></div>
    <div class="status" id="rtt">-- ms</div>
  </div>

  <div class="mid">
    <button class="arm" id="arm">ARM: OFF</button>
    <div class="slider">Mode (ground &harr; wall)
      <input type="range" id="mode" min="0" max="100" value="0">
    </div>
  </div>

  <div class="sticks">
    <div class="stick" id="stickL"><div class="knob"></div></div>
    <div class="stick" id="stickR"><div class="knob"></div></div>
  </div>

  <div class="readout" id="readout">L(0,0)  R(0,0)  mode 0  arm 0</div>
</div>

<script>
(function () {
  const dot = document.getElementById('dot');
  const stat = document.getElementById('stat');
  const rttEl = document.getElementById('rtt');
  const readout = document.getElementById('readout');
  const armBtn = document.getElementById('arm');
  const modeEl = document.getElementById('mode');

  let arm = 0;
  let ws = null;

  // ---- WebSocket with auto-reconnect ----
  function connect() {
    ws = new WebSocket('ws://' + location.host + '/ws');
    ws.onopen = () => { dot.classList.add('on'); stat.textContent = 'Connected'; };
    ws.onclose = () => {
      dot.classList.remove('on'); stat.textContent = 'Disconnected - retrying...';
      rttEl.textContent = '-- ms';
      setTimeout(connect, 1000);
    };
    ws.onerror = () => { try { ws.close(); } catch (e) {} };
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string' && ev.data[0] === 'P') {
        const t = parseFloat(ev.data.split(',')[1]);
        rttEl.textContent = Math.round(performance.now() - t) + ' ms';
      }
    };
  }
  connect();

  // ---- Virtual joystick (pointer events, springs back to centre) ----
  function makeStick(el) {
    const knob = el.querySelector('.knob');
    const s = { x: 0, y: 0 };
    let activeId = null;

    function update(e) {
      const r = el.getBoundingClientRect();
      const R = r.width / 2;
      const max = R * 0.68;
      let dx = e.clientX - (r.left + R);
      let dy = e.clientY - (r.top + R);
      const d = Math.hypot(dx, dy);
      if (d > max) { dx *= max / d; dy *= max / d; }
      knob.style.transform = 'translate(' + dx + 'px,' + dy + 'px)';
      s.x = Math.round(dx / max * 100);
      s.y = Math.round(-dy / max * 100);   // up = positive
    }
    function reset() {
      activeId = null; s.x = 0; s.y = 0;
      knob.style.transform = 'translate(0,0)';
    }
    el.addEventListener('pointerdown', (e) => {
      activeId = e.pointerId; el.setPointerCapture(activeId); update(e); e.preventDefault();
    });
    el.addEventListener('pointermove', (e) => { if (e.pointerId === activeId) update(e); });
    el.addEventListener('pointerup', (e) => { if (e.pointerId === activeId) reset(); });
    el.addEventListener('pointercancel', reset);
    el.addEventListener('lostpointercapture', reset);
    return s;
  }
  const L = makeStick(document.getElementById('stickL'));
  const R = makeStick(document.getElementById('stickR'));

  // ---- Arm toggle (data only for now) ----
  armBtn.addEventListener('click', () => {
    arm = arm ? 0 : 1;
    armBtn.textContent = 'ARM: ' + (arm ? 'ON' : 'OFF');
    armBtn.classList.toggle('on', !!arm);
  });

  // ---- Send control packet at ~30 Hz ----
  setInterval(() => {
    const mode = parseInt(modeEl.value, 10);
    readout.textContent = 'L(' + L.x + ',' + L.y + ')  R(' + R.x + ',' + R.y + ')  mode ' + mode + '  arm ' + arm;
    if (ws && ws.readyState === 1) {
      ws.send('J,' + L.x + ',' + L.y + ',' + R.x + ',' + R.y + ',' + mode + ',' + arm);
    }
  }, 33);

  // ---- Ping every second to measure latency ----
  setInterval(() => {
    if (ws && ws.readyState === 1) ws.send('P,' + performance.now());
  }, 1000);

  // If the page is hidden, force arm off for safety
  document.addEventListener('visibilitychange', () => {
    if (document.hidden && arm) armBtn.click();
  });
})();
</script>
</body>
</html>
)rawliteral";

// ---------- WebSocket events ----------
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WS client #%u connected\n", client->id());
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("WS client #%u disconnected\n", client->id());
      // Reset stored values when the controller disappears
      ctl_lx = ctl_ly = ctl_rx = ctl_ry = ctl_mode = ctl_arm = 0;
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) break;

      char buf[64];
      size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
      memcpy(buf, data, n);
      buf[n] = '\0';

      if (buf[0] == 'J') {
        int lx, ly, rx, ry, mode, arm;
        if (sscanf(buf, "J,%d,%d,%d,%d,%d,%d", &lx, &ly, &rx, &ry, &mode, &arm) == 6) {
          ctl_lx = constrain(lx, -100, 100);
          ctl_ly = constrain(ly, -100, 100);
          ctl_rx = constrain(rx, -100, 100);
          ctl_ry = constrain(ry, -100, 100);
          ctl_mode = constrain(mode, 0, 100);
          ctl_arm = arm ? 1 : 0;
          lastPacketMs = millis();
        }
      } else if (buf[0] == 'P') {
        client->text(buf);              // echo back for latency measurement
      }
      break;
    }

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Access point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // ArduinoOTA
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([]() {
    // TODO later: set ESC outputs to minimum here
    ctl_arm = 0;
    Serial.println("OTA start");
  });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA end"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%\r", p * 100 / t);
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error[%u]\n", e); });
  ArduinoOTA.begin();

  // Web server + WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
  });
  server.begin();
  Serial.println("Web server started: http://192.168.4.1");
}

void loop() {
  ArduinoOTA.handle();
  ws.cleanupClients();

  // Print the latest values 5 times per second
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.printf("L(%d,%d) R(%d,%d) mode=%d arm=%d age=%ums\n",
                  ctl_lx, ctl_ly, ctl_rx, ctl_ry, ctl_mode, ctl_arm,
                  (unsigned)(millis() - lastPacketMs));
  }

  // Future failsafe goes here:
  // if (millis() - lastPacketMs > 300) { /* zero throttle, disarm */ }
}
