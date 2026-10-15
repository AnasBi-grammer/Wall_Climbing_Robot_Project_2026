/*
  WallCrawler - Wi-Fi AP + ArduinoOTA + WebSocket slider page
  Step 2: FRONT and BACK MG996R tilt servos, controlled by sliders only.
  Still NO ESC / motor / wheel-servo code.

  Libraries to install (Library Manager or ZIP):
    - "ESP Async WebServer" by ESP32Async
    - "Async TCP" by ESP32Async
    - "ESP32Servo" by Kevin Harrington / John K. Bennett
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>

// ---------- Network settings ----------
const char* AP_SSID  = "WallBot";
const char* AP_PASS  = "wallbot123";     // at least 8 characters
const int   AP_CHANNEL = 6;
const int   AP_MAX_CLIENTS = 4;

const char* OTA_HOSTNAME = "wallbot";
const char* OTA_PASS     = "ota1234";

// ---------- MG996R servo settings ----------
const int SERVO_F_PIN = 13;          // FRONT MG996R signal pin (change to your wiring)
const int SERVO_B_PIN = 19;          // BACK  MG996R signal pin (change to your wiring)
const int SERVO_MIN_US = 500;        // pulse width for 0 deg
const int SERVO_MAX_US = 2400;       // pulse width for 180 deg (calibrate if the ends buzz)

const int FRONT_DEFAULT_DEG = 0;     // front rests at 0, slider range 0..180
const int BACK_DEFAULT_DEG  = 90;    // back rests at 90, slider range 0..180 (90 each side)

const float SLEW_DEG_PER_STEP = 5.0; // speed limit: 5 deg per 20 ms (~250 deg/s)
const uint32_t FAILSAFE_MS = 300;    // no packet for this long -> servos return to defaults

// ---------- Latest values received from the page ----------
volatile int ctl_front = FRONT_DEFAULT_DEG;   // 0..180 (degrees)
volatile int ctl_back  = BACK_DEFAULT_DEG;    // 0..180 (degrees)
volatile int ctl_arm   = 0;                   // 0/1
volatile uint32_t lastPacketMs = 0;

Servo servoF, servoB;
float curF = FRONT_DEFAULT_DEG;      // current angles, for slew limiting
float curB = BACK_DEFAULT_DEG;

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
              font-family:system-ui,Segoe UI,Roboto,sans-serif; overflow:hidden; user-select:none; -webkit-user-select:none; }
  .wrap { height:100%; display:flex; flex-direction:column; padding:12px; gap:12px; max-width:700px; margin:0 auto; }
  .top { display:flex; align-items:center; justify-content:space-between; gap:10px; }
  .status { display:flex; align-items:center; gap:8px; font-size:14px; }
  .dot { width:12px; height:12px; border-radius:50%; background:var(--bad); }
  .dot.on { background:var(--ok); }
  button { font-family:inherit; }
  button.arm { padding:14px; font-size:18px; font-weight:700; border-radius:12px; border:2px solid var(--line);
               background:var(--panel); color:var(--fg); }
  button.arm.on { border-color:var(--bad); background:#3a1d1d; color:#ffb3b3; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:14px; padding:14px 16px; }
  .card h2 { margin:0 0 4px; font-size:13px; letter-spacing:.08em; text-transform:uppercase; color:var(--dim); font-weight:600; }
  .row { display:flex; align-items:baseline; justify-content:space-between; }
  .val { font:700 34px ui-monospace,Menlo,Consolas,monospace; }
  .val small { font-size:16px; color:var(--dim); }
  .ends { display:flex; justify-content:space-between; font-size:12px; color:var(--dim); margin-top:2px; }
  input[type=range] { -webkit-appearance:none; appearance:none; width:100%; height:46px; background:transparent;
                      touch-action:manipulation; margin:6px 0 0; }
  input[type=range]::-webkit-slider-runnable-track { height:10px; border-radius:5px; background:var(--line); }
  input[type=range]::-moz-range-track { height:10px; border-radius:5px; background:var(--line); }
  input[type=range]::-webkit-slider-thumb { -webkit-appearance:none; width:44px; height:44px; border-radius:50%;
                      background:var(--acc); border:none; margin-top:-17px; box-shadow:0 2px 10px rgba(0,0,0,.5); }
  input[type=range]::-moz-range-thumb { width:44px; height:44px; border-radius:50%; background:var(--acc); border:none; }
  input[type=range]:disabled { opacity:.45; }
  button.small { padding:6px 12px; font-size:13px; border-radius:8px; border:1px solid var(--line);
                 background:transparent; color:var(--dim); }
  .hint { font-size:12px; color:var(--dim); text-align:center; }
</style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div class="status"><span class="dot" id="dot"></span><span id="stat">Connecting...</span></div>
    <div class="status" id="rtt">-- ms</div>
  </div>

  <button class="arm" id="arm">ARM: OFF</button>

  <div class="card">
    <h2>Front servo</h2>
    <div class="row"><div class="val"><span id="fv">0</span><small>&deg;</small></div></div>
    <input type="range" id="front" min="0" max="180" value="0" disabled>
    <div class="ends"><span>0&deg; (rest)</span><span>180&deg;</span></div>
  </div>

  <div class="card">
    <h2>Back servo</h2>
    <div class="row">
      <div class="val"><span id="bv">90</span><small>&deg;</small></div>
      <button class="small" id="bcenter" disabled>Centre 90&deg;</button>
    </div>
    <input type="range" id="back" min="0" max="180" value="90" disabled>
    <div class="ends"><span>0&deg;</span><span>90&deg; (rest)</span><span>180&deg;</span></div>
  </div>

  <div class="hint">Sliders are locked until ARM is ON. Turning ARM on or off resets both sliders to rest.</div>
</div>

<script>
(function () {
  const dot = document.getElementById('dot');
  const stat = document.getElementById('stat');
  const rttEl = document.getElementById('rtt');
  const armBtn = document.getElementById('arm');
  const front = document.getElementById('front');
  const back = document.getElementById('back');
  const fv = document.getElementById('fv');
  const bv = document.getElementById('bv');
  const bcenter = document.getElementById('bcenter');

  const FRONT_REST = 0, BACK_REST = 90;
  let arm = 0;
  let ws = null;

  function showValues() { fv.textContent = front.value; bv.textContent = back.value; }
  front.addEventListener('input', showValues);
  back.addEventListener('input', showValues);
  bcenter.addEventListener('click', () => { back.value = BACK_REST; showValues(); });

  function resetSliders() { front.value = FRONT_REST; back.value = BACK_REST; showValues(); }

  function setArm(v) {
    arm = v ? 1 : 0;
    resetSliders();                       // nothing jumps when arming/disarming
    front.disabled = back.disabled = bcenter.disabled = !arm;
    armBtn.textContent = 'ARM: ' + (arm ? 'ON' : 'OFF');
    armBtn.classList.toggle('on', !!arm);
  }
  armBtn.addEventListener('click', () => setArm(!arm));

  // ---- WebSocket with auto-reconnect ----
  function connect() {
    ws = new WebSocket('ws://' + location.host + '/ws');
    ws.onopen = () => { dot.classList.add('on'); stat.textContent = 'Connected'; };
    ws.onclose = () => {
      dot.classList.remove('on'); stat.textContent = 'Disconnected - retrying...';
      rttEl.textContent = '-- ms';
      if (arm) setArm(0);                 // lose the link -> disarm and reset the page
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

  // ---- Send control packet at ~30 Hz:  S,front,back,arm ----
  setInterval(() => {
    if (ws && ws.readyState === 1) {
      ws.send('S,' + front.value + ',' + back.value + ',' + arm);
    }
  }, 33);

  // ---- Ping every second to measure latency ----
  setInterval(() => {
    if (ws && ws.readyState === 1) ws.send('P,' + performance.now());
  }, 1000);

  // Hidden page (app switched, screen off) -> disarm
  document.addEventListener('visibilitychange', () => { if (document.hidden && arm) setArm(0); });
})();
</script>
</body>
</html>
)rawliteral";

// ---------- WebSocket events ----------
void resetControls() {
  ctl_front = FRONT_DEFAULT_DEG;
  ctl_back  = BACK_DEFAULT_DEG;
  ctl_arm   = 0;
}

void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WS client #%u connected\n", client->id());
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("WS client #%u disconnected\n", client->id());
      resetControls();
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) break;

      char buf[64];
      size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
      memcpy(buf, data, n);
      buf[n] = '\0';

      if (buf[0] == 'S') {
        int f, b, a;
        if (sscanf(buf, "S,%d,%d,%d", &f, &b, &a) == 3) {
          ctl_front = constrain(f, 0, 180);
          ctl_back  = constrain(b, 0, 180);
          ctl_arm   = a ? 1 : 0;
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

// ---------- Servo handling (called every 20 ms) ----------
float slew(float cur, float target) {
  float d = target - cur;
  if (d >  SLEW_DEG_PER_STEP) d =  SLEW_DEG_PER_STEP;
  if (d < -SLEW_DEG_PER_STEP) d = -SLEW_DEG_PER_STEP;
  return cur + d;
}

void servosToDefault() {
  curF = FRONT_DEFAULT_DEG;
  curB = BACK_DEFAULT_DEG;
  servoF.write(FRONT_DEFAULT_DEG);
  servoB.write(BACK_DEFAULT_DEG);
}

void updateServos() {
  bool linkOk = (millis() - lastPacketMs) < FAILSAFE_MS;

  float targetF = FRONT_DEFAULT_DEG;   // disarmed or link lost -> defaults
  float targetB = BACK_DEFAULT_DEG;
  if (ctl_arm && linkOk) {
    targetF = ctl_front;               // the sliders are the servo angles
    targetB = ctl_back;
  }

  curF = slew(curF, constrain(targetF, 0, 180));
  curB = slew(curB, constrain(targetB, 0, 180));

  servoF.write((int)lroundf(curF));
  servoB.write((int)lroundf(curB));
}

void setup() {
  Serial.begin(115200);

  // Servos (attach first so they go to rest immediately)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoF.setPeriodHertz(50);
  servoB.setPeriodHertz(50);
  servoF.attach(SERVO_F_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoB.attach(SERVO_B_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servosToDefault();

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
    resetControls();
    servosToDefault();
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

  // Servo update every 20 ms
  static uint32_t lastServo = 0;
  if (millis() - lastServo >= 20) {
    lastServo = millis();
    updateServos();
  }

  // Print status 5 times per second
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.printf("front=%d back=%d arm=%d age=%ums | servoF=%.0f servoB=%.0f\n",
                  ctl_front, ctl_back, ctl_arm,
                  (unsigned)(millis() - lastPacketMs), curF, curB);
  }
}
