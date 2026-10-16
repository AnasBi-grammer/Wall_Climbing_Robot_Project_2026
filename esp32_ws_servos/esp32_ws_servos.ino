/*
  WallCrawler - Wi-Fi AP + ArduinoOTA + WebSocket slider page
  Step 3: FRONT and BACK MG996R tilt servos + two BLHeli ESCs (RS2205 motors), all by sliders.
  Still NO wheel-servo code.

  !!! REMOVE THE PROPELLERS for all first tests !!!

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
const int SERVO_F_PIN = 18;          // FRONT MG996R signal pin (change to your wiring)
const int SERVO_B_PIN = 19;          // BACK  MG996R signal pin (change to your wiring)
const int SERVO_MIN_US = 500;        // pulse width for 0 deg
const int SERVO_MAX_US = 2400;       // pulse width for 180 deg (calibrate if the ends buzz)

// ---------- ESC settings (standard 50 Hz PWM, 1000..2000 us) ----------
const int ESC_A_PIN = 25;            // ESC 1 signal pin (change to your wiring)
const int ESC_B_PIN = 26;            // ESC 2 signal pin (change to your wiring)
const int ESC_MIN_US = 1000;         // full reverse
const int ESC_NEUTRAL_US = 1500;     // zero throttle
const int ESC_MAX_US = 2000;         // full throttle

const float THROTTLE_LIMIT_PCT = 30.0;     // slider 100% = 30% of full ESC range. RAISE ONLY AFTER TESTING.
const uint32_t ESC_ARM_HOLD_MS = 3000;     // hold zero throttle this long after boot so the ESCs arm
const float RAMP_UP_PER_STEP   = 1.5;      // slider % per 20 ms going up  (~75 % per second)
const float RAMP_DOWN_PER_STEP = 5.0;      // slider % per 20 ms going down

const int FRONT_DEFAULT_DEG = 90;     // front rests at 90, slider range 0..180 (90 each side)
const int BACK_DEFAULT_DEG  = 90;    // back rests at 90, slider range 0..180 (90 each side)

const float SLEW_DEG_PER_STEP = 5.0; // speed limit: 5 deg per 20 ms (~250 deg/s)
const uint32_t FAILSAFE_MS = 300;    // no packet for this long -> servos return to defaults

// ---------- MG90S steering servos (front wheels) ----------
const int STEER_L_PIN = 27;          // left  steering servo signal pin (change to your wiring)
const int STEER_R_PIN = 32;          // right steering servo signal pin (change to your wiring)
const int STEER_CENTER_DEG = 90;
const int STEER_TRAVEL_DEG = 45;     // +/- from centre. Use 60 only if the frame allows it
const bool STEER_L_INVERT = false;   // set true if that servo turns the wrong way
const bool STEER_R_INVERT = false;

// ---------- Latest values received from the page ----------
volatile int ctl_front = FRONT_DEFAULT_DEG;   // 0..180 (degrees)
volatile int ctl_back  = BACK_DEFAULT_DEG;    // 0..180 (degrees)
volatile int ctl_thrA  = 0;                   // 0..100 (%)
volatile int ctl_thrB  = 0;                   // 0..100 (%)
volatile int ctl_steerL = STEER_CENTER_DEG;
volatile int ctl_steerR = STEER_CENTER_DEG;
volatile bool visionOn = false;
volatile int  vis_steer = STEER_CENTER_DEG;
volatile uint32_t lastVisionMs = 0;
uint32_t controllerId = 0;                 // id of the web-page client (the one sending S packets)
const uint32_t VISION_TIMEOUT_MS = 300;
volatile int ctl_arm   = 0;                   // 0/1
volatile uint32_t lastPacketMs = 0;

Servo servoF, servoB;
Servo escA, escB;
Servo servoSL, servoSR;
float curSL = STEER_CENTER_DEG, curSR = STEER_CENTER_DEG;
float curThrA = 0, curThrB = 0;      // current throttle (slider %), after ramping
bool  throttleUnlocked = false;      // true only after both throttles were seen at 0 while armed
uint32_t bootMs = 0;
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
<title>WallBot Controller</title>
<style>
  :root { --bg:#111418; --panel:#1b2027; --line:#2c3440; --fg:#e8edf2; --dim:#8b97a6; --acc:#3aa0ff; --ok:#3ddc84; --bad:#ff5c5c; }
  * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
  html,body { margin:0; height:100%; background:var(--bg); color:var(--fg);
              font-family:system-ui,Segoe UI,Roboto,sans-serif; overflow:auto; user-select:none; -webkit-user-select:none; }
  .wrap { min-height:100%; display:flex; flex-direction:column; padding:12px 12px 48px; gap:12px; max-width:700px; margin:0 auto; }
  .top { display:flex; align-items:center; justify-content:space-between; gap:10px; }
  .status { display:flex; align-items:center; gap:8px; font-size:14px; }
  .dot { width:12px; height:12px; border-radius:50%; background:var(--bad); }
  .dot.on { background:var(--ok); }
  button { font-family:inherit; }
  button.arm { padding:14px; font-size:18px; font-weight:700; border-radius:12px; border:2px solid var(--line);
               background:var(--panel); color:var(--fg); }
  button.arm.on { border-color:var(--bad); background:#3a1d1d; color:#ffb3b3; }
  button.vis.on { border-color:var(--ok); background:#1d3a28; color:#b3ffd0; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:14px; padding:8px 14px; }
  .card h2 { margin:0 0 4px; font-size:13px; letter-spacing:.08em; text-transform:uppercase; color:var(--dim); font-weight:600; }
  .row { display:flex; align-items:baseline; justify-content:space-between; }
  .val { font:700 26px ui-monospace,Menlo,Consolas,monospace; }
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
  <button class="arm vis" id="visionBtn" style="padding:10px;font-size:15px;" disabled>VISION: OFF</button>

  <div class="card">
    <h2>Front servo</h2>
    <div class="row">
      <div class="val"><span id="fv">90</span><small>&deg;</small></div>
    </div>
      <input type="range" id="front" min="0" max="180" value="90" disabled>
      <div class="ends"><span>0&deg;</span><span>90&deg; (rest)</span><span>180&deg;</span></div>
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

  <div class="card">
    <h2>Steering left (MG90S)</h2>
    <div class="row"><div class="val"><span id="slv">90</span><small>&deg;</small></div></div>
    <input type="range" id="steerL" min="45" max="135" value="90" disabled>
    <div class="ends"><span>45&deg;</span><span>90&deg; (centre)</span><span>135&deg;</span></div>
  </div>

  <div class="card">
    <h2>Steering right (MG90S)</h2>
    <div class="row"><div class="val"><span id="srv">90</span><small>&deg;</small></div></div>
    <input type="range" id="steerR" min="45" max="135" value="90" disabled>
    <div class="ends"><span>45&deg;</span><span>90&deg; (centre)</span><span>135&deg;</span></div>
  </div>

  <div class="card">
    <h2>Motor 1 (ESC 1)</h2>
    <div class="row"><div class="val"><span id="av">0</span><small>%</small></div></div>
    <input type="range" id="thrA" min="-100" max="100" value="0" disabled>
    <div class="ends"><span>-100% (rev)</span><span>0</span><span>100% (fwd)</span></div>
  </div>

  <div class="card">
    <h2>Motor 2 (ESC 2)</h2>
    <div class="row">
      <div class="val"><span id="mv">0</span><small>%</small></div>
      <label class="small" style="display:flex;align-items:center;gap:6px;padding:6px 10px;border:1px solid var(--line);border-radius:8px;">
        <input type="checkbox" id="link" style="width:18px;height:18px;"> Link both
      </label>
    </div>
    <input type="range" id="thrB" min="-100" max="100" value="0" disabled>
    <div class="ends"><span>-100% (rev)</span><span>0</span><span>100% (fwd)</span></div>
  </div>

  <button class="arm" id="stop" style="padding:10px;font-size:15px;">MOTORS TO 0</button>
  <button class="arm" id="centerAll" style="padding:10px;font-size:15px;" disabled>CENTRE SERVOS (90&deg;)</button>

  <div class="hint">Sliders are locked until ARM is ON. Turning ARM on or off resets all sliders to rest. Motors only spin after both motor sliders are at 0 once armed.</div>
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
  const thrA = document.getElementById('thrA');
  const thrB = document.getElementById('thrB');
  const av = document.getElementById('av');
  const mv = document.getElementById('mv');
  const link = document.getElementById('link');
  const stopBtn = document.getElementById('stop');
  const centerBtn = document.getElementById('centerAll');
  const visionBtn = document.getElementById('visionBtn');
  let vision = 0;
  const steerL = document.getElementById('steerL');
  const steerR = document.getElementById('steerR');
  const slv = document.getElementById('slv');
  const srv = document.getElementById('srv');

  const FRONT_REST = 90, BACK_REST = 90;
  let arm = 0;
  let ws = null;

  function showValues() {
    fv.textContent = front.value; bv.textContent = back.value;
    av.textContent = thrA.value;  mv.textContent = thrB.value;
    slv.textContent = steerL.value; srv.textContent = steerR.value;
  }
  front.addEventListener('input', showValues);
  back.addEventListener('input', showValues);
  steerL.addEventListener('input', showValues);
  steerR.addEventListener('input', showValues);
  thrA.addEventListener('input', () => { if (link.checked) thrB.value = thrA.value; showValues(); });
  thrB.addEventListener('input', () => { if (link.checked) thrA.value = thrB.value; showValues(); });
  stopBtn.addEventListener('click', () => { thrA.value = 0; thrB.value = 0; showValues(); });
  centerBtn.addEventListener('click', () => { front.value = FRONT_REST; back.value = BACK_REST; showValues(); });
  bcenter.addEventListener('click', () => { back.value = BACK_REST; showValues(); });

  function setVisionUI(v) {
    vision = v ? 1 : 0;
    visionBtn.textContent = 'VISION: ' + (vision ? 'ON (steering locked)' : 'OFF');
    visionBtn.classList.toggle('on', !!vision);
    steerL.disabled = steerR.disabled = !arm || !!vision;
  }
  visionBtn.addEventListener('click', () => {
    if (ws && ws.readyState === 1) ws.send('V,' + (vision ? 0 : 1));
  });

  function resetSliders() {
    front.value = FRONT_REST; back.value = BACK_REST;
    thrA.value = 0; thrB.value = 0; showValues();
    steerL.value = 90; steerR.value = 90;
  }

  function setArm(v) {
    arm = v ? 1 : 0;
    resetSliders();                       // nothing jumps when arming/disarming
    front.disabled = back.disabled = bcenter.disabled = thrA.disabled = thrB.disabled = centerBtn.disabled = steerL.disabled = steerR.disabled = !arm;
    visionBtn.disabled = !arm;
    setVisionUI(arm ? vision : 0);
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
      if (typeof ev.data !== 'string') return;
      if (ev.data[0] === 'P') {
        const t = parseFloat(ev.data.split(',')[1]);
        rttEl.textContent = Math.round(performance.now() - t) + ' ms';
      } else if (ev.data[0] === 'V') {
        setVisionUI(ev.data.split(',')[1] === '1');
      }
    };
  }
  connect();

  // ---- Send control packet at ~30 Hz:  S,front,back,thrA,thrB,arm ----
  setInterval(() => {
    if (ws && ws.readyState === 1) {
      ws.send('S,' + front.value + ',' + back.value + ',' + thrA.value + ',' + thrB.value + ',' +
      steerL.value + ',' + steerR.value + ',' + arm);
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
  ctl_thrA  = 0;
  ctl_thrB  = 0;
  ctl_arm   = 0;
  ctl_steerL = STEER_CENTER_DEG;
  ctl_steerR = STEER_CENTER_DEG;
  visionOn = false;
  vis_steer = STEER_CENTER_DEG;
}

void setVision(bool on) {
  visionOn = on;
  vis_steer = STEER_CENTER_DEG;
  lastVisionMs = 0;                        // stale until the first H packet arrives
  ws.textAll(on ? "V,1" : "V,0");
}

void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WS client #%u connected\n", client->id());
      client->text(visionOn ? "V,1" : "V,0");      // tell the new client the current state
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("WS client #%u disconnected\n", client->id());
      if (client->id() == controllerId) {           // only the web page disarms; Python dropping doesn't 
        resetControls();
        ws.textAll("V,0");
      }
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) break;

      char buf[64];
      size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
      memcpy(buf, data, n);
      buf[n] = '\0';

      if (buf[0] == 'S') {
        int f, b, ta, tb, sl, sr, a;
        if (sscanf(buf, "S,%d,%d,%d,%d,%d,%d,%d", &f, &b, &ta, &tb, &sl, &sr, &a) == 7) {
          ctl_steerL = constrain(sl, STEER_CENTER_DEG - STEER_TRAVEL_DEG, STEER_CENTER_DEG + STEER_TRAVEL_DEG);
          ctl_steerR = constrain(sr, STEER_CENTER_DEG - STEER_TRAVEL_DEG, STEER_CENTER_DEG + STEER_TRAVEL_DEG);
          ctl_front = constrain(f, 0, 180);
          ctl_back  = constrain(b, 0, 180);
          ctl_thrA  = constrain(ta, -100, 100);
          ctl_thrB  = constrain(tb, -100, 100);
          ctl_arm   = a ? 1 : 0;
          lastPacketMs = millis();
          controllerId = client->id();
          if (!ctl_arm && visionOn) setVision(false);   // disarm always turns vision off
        }
      } else if (buf[0] == 'V') {
        int v = 0;
        if (sscanf(buf, "V,%d", &v) == 1) setVision(v && ctl_arm);   // only possible while armed
      } else if (buf[0] == 'H') {
        int h;
        if (sscanf(buf, "H,%d", &h) == 1) {
          vis_steer = constrain(h, STEER_CENTER_DEG - STEER_TRAVEL_DEG, STEER_CENTER_DEG + STEER_TRAVEL_DEG);
          lastVisionMs = millis();
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
  curSL = curSR = STEER_CENTER_DEG;
  servoSL.write(STEER_L_INVERT ? 180 - STEER_CENTER_DEG : STEER_CENTER_DEG);
  servoSR.write(STEER_R_INVERT ? 180 - STEER_CENTER_DEG : STEER_CENTER_DEG);
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
  float targetSL = STEER_CENTER_DEG, targetSR = STEER_CENTER_DEG;
    if (ctl_arm && linkOk) {
    if (visionOn) {
      bool visFresh = (millis() - lastVisionMs) < VISION_TIMEOUT_MS;
      targetSL = targetSR = visFresh ? vis_steer : STEER_CENTER_DEG;
    } else {
      targetSL = ctl_steerL;
      targetSR = ctl_steerR;
    }
  }
  targetSL = constrain(targetSL, STEER_CENTER_DEG - STEER_TRAVEL_DEG, STEER_CENTER_DEG + STEER_TRAVEL_DEG);
  targetSR = constrain(targetSR, STEER_CENTER_DEG - STEER_TRAVEL_DEG, STEER_CENTER_DEG + STEER_TRAVEL_DEG);
  curSL = slew(curSL, targetSL);
  curSR = slew(curSR, targetSR);
  int outSL = (int)lroundf(curSL), outSR = (int)lroundf(curSR);
  servoSL.write(STEER_L_INVERT ? 180 - outSL : outSL);
  servoSR.write(STEER_R_INVERT ? 180 - outSR : outSR);
}

// ---------- ESC handling (called every 20 ms) ----------
int throttleToUs(float pct) {
  float frac = (pct / 100.0f) * (THROTTLE_LIMIT_PCT / 100.0f);   // -1..+1 after the limit
  return ESC_NEUTRAL_US + (int)lroundf((ESC_MAX_US - ESC_NEUTRAL_US) * frac);
}

void escsToZero() {
  curThrA = curThrB = 0;
  escA.writeMicroseconds(ESC_NEUTRAL_US);
  escB.writeMicroseconds(ESC_NEUTRAL_US);
}

float ramp(float cur, float target) {
  float d = target - cur;
  float limit = (fabsf(target) < fabsf(cur)) ? RAMP_DOWN_PER_STEP : RAMP_UP_PER_STEP;
  if (d >  limit) d =  limit;
  if (d < -limit) d = -limit;
  return cur + d;
}

void updateEscs() {
  bool linkOk   = (millis() - lastPacketMs) < FAILSAFE_MS;
  bool bootDone = (millis() - bootMs) > ESC_ARM_HOLD_MS;
  bool armed    = ctl_arm && linkOk && bootDone;

  static bool prevArmed = false;
  if (armed && !prevArmed) throttleUnlocked = false;   // every (re)arm needs a fresh zero
  prevArmed = armed;

  if (!armed) {                        // disarmed, link lost, or ESCs still arming: stop at once
    throttleUnlocked = false;
    escsToZero();
    return;
  }

  if (!throttleUnlocked && ctl_thrA == 0 && ctl_thrB == 0) throttleUnlocked = true;

  float targetA = throttleUnlocked ? ctl_thrA : 0;
  float targetB = throttleUnlocked ? ctl_thrB : 0;

  curThrA = ramp(curThrA, targetA);
  curThrB = ramp(curThrB, targetB);

  escA.writeMicroseconds(throttleToUs(curThrA));
  escB.writeMicroseconds(throttleToUs(curThrB));
}

void setup() {
  Serial.begin(115200);
  bootMs = millis();

  // ESCs first: send zero throttle immediately so they arm safely
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  escA.setPeriodHertz(50);
  escB.setPeriodHertz(50);
  escA.attach(ESC_A_PIN, ESC_MIN_US, ESC_MAX_US);
  escB.attach(ESC_B_PIN, ESC_MIN_US, ESC_MAX_US);
  escsToZero();

  // Servos (attach first so they go to rest immediately)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoF.setPeriodHertz(50);
  servoB.setPeriodHertz(50);
  servoF.attach(SERVO_F_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoB.attach(SERVO_B_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoSL.setPeriodHertz(50);
  servoSR.setPeriodHertz(50);
  servoSL.attach(STEER_L_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoSR.attach(STEER_R_PIN, SERVO_MIN_US, SERVO_MAX_US);
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
    resetControls();
    escsToZero();
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
    updateEscs();
  }

  // Print status 5 times per second
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.printf("front=%d back=%d thrA=%d thrB=%d arm=%d age=%ums | servoF=%.0f servoB=%.0f escA=%dus escB=%dus unlocked=%d\n",
                  ctl_front, ctl_back, ctl_thrA, ctl_thrB, ctl_arm,
                  (unsigned)(millis() - lastPacketMs), curF, curB,
                  throttleToUs(curThrA), throttleToUs(curThrB), throttleUnlocked);
  }
}
