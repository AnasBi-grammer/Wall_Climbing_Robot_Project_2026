/*
  WallBot - Wi-Fi AP + ArduinoOTA + WebSocket slider page

  Step 4:
    - MG996R tilt servos REMOVED (propeller pads are fixed now)
    - 4 DC wheel motors through 2x HW-095 (L298N) drivers, 2 sliders:
        FRONT wheels (2 motors) and BACK wheels (2 motors), each -255..+255
    - 2 BLHeli bidirectional ESCs (propellers), 2 sliders (unchanged)
    - 2 MG90S steering servos, 2 sliders (unchanged), + vision hook (unchanged)

  !!! REMOVE THE PROPELLERS and LIFT THE WHEELS for all first tests !!!

  Libraries (Library Manager or ZIP):
    - "ESP Async WebServer" by ESP32Async
    - "Async TCP" by ESP32Async
    - "ESP32Servo" by Kevin Harrington / John K. Bennett
  Works with ESP32 Arduino core 2.x and 3.x (the PWM calls are chosen automatically).

  ---------------- WIRING OF THE DC MOTORS (HW-095 / L298N) ----------------
  Keep the ENA and ENB jumpers ON (EN pins tied to 5 V). Speed is made by PWM on the IN pins:
     forward : INa = PWM, INb = LOW      reverse : INa = LOW, INb = PWM      stop : both LOW (brake)

  Driver 1 = FRONT wheels                 Driver 2 = BACK wheels
    IN1 <- GPIO 13  (front-left  A)         IN1 <- GPIO 22  (back-left  A)
    IN2 <- GPIO 18  (front-left  B)         IN2 <- GPIO 23  (back-left  B)
    IN3 <- GPIO 19  (front-right A)         IN3 <- GPIO 4   (back-right A)
    IN4 <- GPIO 21  (front-right B)         IN4 <- GPIO 33  (back-right B)
    OUT1/OUT2 -> front-left motor           OUT1/OUT2 -> back-left motor
    OUT3/OUT4 -> front-right motor          OUT3/OUT4 -> back-right motor
  Both drivers: 12V/VMS terminal <- XL4015 output (+), GND <- common ground (also tied to ESP32 GND).
  Do NOT use the driver's 5 V pin to power the ESP32 (the LM2596 does that).
  Optional but recommended: 10k pull-down resistor from each of the 8 IN pins to GND.

  Pins already used elsewhere in this sketch:
    ESC 1 = 25, ESC 2 = 26, steering servos = 27 and 32
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ---------- Network settings ----------
const char* AP_SSID  = "WallBot";
const char* AP_PASS  = "wallbot123";     // at least 8 characters
const int   AP_CHANNEL = 6;
const int   AP_MAX_CLIENTS = 4;

const char* OTA_HOSTNAME = "wallbot";
const char* OTA_PASS     = "ota1234";

// ---------- Common timing ----------
const uint32_t FAILSAFE_MS = 300;    // no packet for this long -> everything stops / returns to rest
                                     // (the wall-specific failsafe will be designed later)

// ---------- Servo pulse range (MG90S steering servos) ----------
const int SERVO_MIN_US = 500;        // pulse width for 0 deg
const int SERVO_MAX_US = 2400;       // pulse width for 180 deg (calibrate if the ends buzz)

// ---------- ESC settings (propellers, standard 50 Hz PWM, 1000..2000 us) ----------
const int ESC_A_PIN = 25;            // ESC 1 signal pin
const int ESC_B_PIN = 26;            // ESC 2 signal pin
const int ESC_MIN_US = 1050;         // full reverse
const int ESC_NEUTRAL_US = 1488;     // zero throttle
const int ESC_MAX_US = 1800;         // full throttle

const float THROTTLE_LIMIT_PCT = 30.0;     // slider 100% = 30% of full ESC range. RAISE ONLY AFTER TESTING.
const uint32_t ESC_ARM_HOLD_MS = 3000;     // hold zero throttle this long after boot so the ESCs arm
const float RAMP_UP_PER_STEP   = 1.5;      // slider % per 20 ms going up  (~75 % per second)
const float RAMP_DOWN_PER_STEP = 5.0;      // slider % per 20 ms going down

// ---------- MG90S steering servos (front wheels) ----------
const int STEER_L_PIN = 27;          // left  steering servo signal pin
const int STEER_R_PIN = 32;          // right steering servo signal pin
const int STEER_CENTER_DEG = 90;
const int STEER_TRAVEL_DEG = 45;     // +/- from centre. Use 60 only if the frame allows it
const bool STEER_L_INVERT = false;   // set true if that servo turns the wrong way
const bool STEER_R_INVERT = false;
const float SLEW_DEG_PER_STEP = 5.0; // steering speed limit: 5 deg per 20 ms (~250 deg/s)

// ---------- DC wheel motors (4x, through 2x HW-095 / L298N) ----------
const uint32_t DC_PWM_FREQ = 5000;   // Hz. L298N is slow, so keep it <= ~10 kHz. 1-2 kHz is more efficient but whines
const uint8_t  DC_PWM_BITS = 8;      // duty 0..255
const int DC_MIN_PWM = 60;           // slider value 1 -> this duty (DC motors do not start below ~25 %). Set 0 for raw PWM
const int DC_MAX_PWM = 255;          // duty cap. Lower it if the motors get more voltage than they like
const float DC_RAMP_UP_PER_STEP   = 12.0;   // slider units per 20 ms going up   (~600/s, 0 -> full in ~0.45 s)
const float DC_RAMP_DOWN_PER_STEP = 25.0;   // slider units per 20 ms going down / through zero (a full reversal takes ~0.6 s)

struct DcMotor {
  uint8_t pinA, pinB;                // L298N IN pins
  uint8_t chA, chB;                  // LEDC channels (8..15 = separate timer group from the servos/ESCs)
  bool invert;                       // flip if this wheel turns the wrong way
};

DcMotor motors[4] = {
  { 13, 18,  8,  9, false },         // 0 front-left   (driver 1, IN1/IN2)
  { 19, 21, 10, 11, true  },         // 1 front-right  (driver 1, IN3/IN4)  mirrored mounting -> inverted
  { 22, 23, 12, 13, false },         // 2 back-left    (driver 2, IN1/IN2)
  {  4, 33, 14, 15, true  },         // 3 back-right   (driver 2, IN3/IN4)  mirrored mounting -> inverted
};
const int FRONT_MOTORS[2] = { 0, 1 };
const int BACK_MOTORS[2]  = { 2, 3 };

// ---------- Latest values received from the page ----------
volatile int ctl_dcF   = 0;                   // -255..255 front wheels
volatile int ctl_dcB   = 0;                   // -255..255 back wheels
volatile int ctl_thrA  = 0;                   // -100..100 (%) propeller ESC 1
volatile int ctl_thrB  = 0;                   // -100..100 (%) propeller ESC 2
volatile int ctl_steerL = STEER_CENTER_DEG;
volatile int ctl_steerR = STEER_CENTER_DEG;
volatile bool visionOn = false;
volatile int  vis_steer = STEER_CENTER_DEG;
volatile uint32_t lastVisionMs = 0;
uint32_t controllerId = 0;                    // id of the web-page client (the one sending S packets)
const uint32_t VISION_TIMEOUT_MS = 300;
volatile int ctl_arm   = 0;                   // 0/1
volatile uint32_t lastPacketMs = 0;

Servo escA, escB;
Servo servoSL, servoSR;
float curSL = STEER_CENTER_DEG, curSR = STEER_CENTER_DEG;
float curThrA = 0, curThrB = 0;      // current throttle (slider %), after ramping
bool  throttleUnlocked = false;      // true only after both throttles were seen at 0 while armed
float curDcF = 0, curDcB = 0;        // current wheel speed (slider units), after ramping
bool  dcUnlocked = false;            // true only after both wheel sliders were seen at 0 while armed
uint32_t bootMs = 0;

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
    <h2>Front wheels (2 DC motors)</h2>
    <div class="row">
      <div class="val"><span id="dfv">0</span><small> / 255</small></div>
      <label class="small" style="display:flex;align-items:center;gap:6px;padding:6px 10px;border:1px solid var(--line);border-radius:8px;">
        <input type="checkbox" id="linkDc" checked style="width:18px;height:18px;"> Link front &amp; back
      </label>
    </div>
    <input type="range" id="dcF" min="-255" max="255" value="0" disabled>
    <div class="ends"><span>-255 (rev)</span><span>0</span><span>255 (fwd)</span></div>
  </div>

  <div class="card">
    <h2>Back wheels (2 DC motors)</h2>
    <div class="row"><div class="val"><span id="dbv">0</span><small> / 255</small></div></div>
    <input type="range" id="dcB" min="-255" max="255" value="0" disabled>
    <div class="ends"><span>-255 (rev)</span><span>0</span><span>255 (fwd)</span></div>
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
    <h2>Propeller 1 (ESC 1)</h2>
    <div class="row"><div class="val"><span id="av">0</span><small>%</small></div></div>
    <input type="range" id="thrA" min="-100" max="100" value="0" disabled>
    <div class="ends"><span>-100% (rev)</span><span>0</span><span>100% (fwd)</span></div>
  </div>

  <div class="card">
    <h2>Propeller 2 (ESC 2)</h2>
    <div class="row">
      <div class="val"><span id="mv">0</span><small>%</small></div>
      <label class="small" style="display:flex;align-items:center;gap:6px;padding:6px 10px;border:1px solid var(--line);border-radius:8px;">
        <input type="checkbox" id="link" style="width:18px;height:18px;"> Link both
      </label>
    </div>
    <input type="range" id="thrB" min="-100" max="100" value="0" disabled>
    <div class="ends"><span>-100% (rev)</span><span>0</span><span>100% (fwd)</span></div>
  </div>

  <button class="arm" id="stop" style="padding:10px;font-size:15px;">ALL MOTORS TO 0</button>
  <button class="arm" id="centerAll" style="padding:10px;font-size:15px;" disabled>CENTRE STEERING (90&deg;)</button>

  <div class="hint">Sliders are locked until ARM is ON. Turning ARM on or off resets all sliders to rest. Wheels and propellers only move after their sliders have been at 0 once while armed.</div>
</div>

<script>
(function () {
  const dot = document.getElementById('dot');
  const stat = document.getElementById('stat');
  const rttEl = document.getElementById('rtt');
  const armBtn = document.getElementById('arm');
  const dcF = document.getElementById('dcF');
  const dcB = document.getElementById('dcB');
  const dfv = document.getElementById('dfv');
  const dbv = document.getElementById('dbv');
  const linkDc = document.getElementById('linkDc');
  const thrA = document.getElementById('thrA');
  const thrB = document.getElementById('thrB');
  const av = document.getElementById('av');
  const mv = document.getElementById('mv');
  const link = document.getElementById('link');
  const stopBtn = document.getElementById('stop');
  const centerBtn = document.getElementById('centerAll');
  const visionBtn = document.getElementById('visionBtn');
  const steerL = document.getElementById('steerL');
  const steerR = document.getElementById('steerR');
  const slv = document.getElementById('slv');
  const srv = document.getElementById('srv');

  const STEER_REST = 90;
  let vision = 0;
  let arm = 0;
  let ws = null;

  function showValues() {
    dfv.textContent = dcF.value;    dbv.textContent = dcB.value;
    av.textContent = thrA.value;    mv.textContent = thrB.value;
    slv.textContent = steerL.value; srv.textContent = steerR.value;
  }
  dcF.addEventListener('input', () => { if (linkDc.checked) dcB.value = dcF.value; showValues(); });
  dcB.addEventListener('input', () => { if (linkDc.checked) dcF.value = dcB.value; showValues(); });
  linkDc.addEventListener('change', () => { if (linkDc.checked) { dcB.value = dcF.value; showValues(); } });
  thrA.addEventListener('input', () => { if (link.checked) thrB.value = thrA.value; showValues(); });
  thrB.addEventListener('input', () => { if (link.checked) thrA.value = thrB.value; showValues(); });
  steerL.addEventListener('input', showValues);
  steerR.addEventListener('input', showValues);
  stopBtn.addEventListener('click', () => {
    dcF.value = 0; dcB.value = 0; thrA.value = 0; thrB.value = 0; showValues();
  });
  centerBtn.addEventListener('click', () => {
    steerL.value = STEER_REST; steerR.value = STEER_REST; showValues();
  });

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
    dcF.value = 0; dcB.value = 0;
    thrA.value = 0; thrB.value = 0;
    steerL.value = STEER_REST; steerR.value = STEER_REST;
    showValues();
  }

  function setArm(v) {
    arm = v ? 1 : 0;
    resetSliders();                       // nothing jumps when arming/disarming
    dcF.disabled = dcB.disabled = thrA.disabled = thrB.disabled = centerBtn.disabled = steerL.disabled = steerR.disabled = !arm;
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

  // ---- Send control packet at ~30 Hz:  S,dcFront,dcBack,thrA,thrB,steerL,steerR,arm ----
  setInterval(() => {
    if (ws && ws.readyState === 1) {
      ws.send('S,' + dcF.value + ',' + dcB.value + ',' + thrA.value + ',' + thrB.value + ',' +
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
  ctl_dcF   = 0;
  ctl_dcB   = 0;
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
        // S,dcFront,dcBack,thrA,thrB,steerL,steerR,arm
        int df, db, ta, tb, sl, sr, a;
        if (sscanf(buf, "S,%d,%d,%d,%d,%d,%d,%d", &df, &db, &ta, &tb, &sl, &sr, &a) == 7) {
          ctl_dcF   = constrain(df, -255, 255);
          ctl_dcB   = constrain(db, -255, 255);
          ctl_thrA  = constrain(ta, -100, 100);
          ctl_thrB  = constrain(tb, -100, 100);
          ctl_steerL = constrain(sl, STEER_CENTER_DEG - STEER_TRAVEL_DEG, STEER_CENTER_DEG + STEER_TRAVEL_DEG);
          ctl_steerR = constrain(sr, STEER_CENTER_DEG - STEER_TRAVEL_DEG, STEER_CENTER_DEG + STEER_TRAVEL_DEG);
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

// ---------- Steering servo handling (called every 20 ms) ----------
float slew(float cur, float target) {
  float d = target - cur;
  if (d >  SLEW_DEG_PER_STEP) d =  SLEW_DEG_PER_STEP;
  if (d < -SLEW_DEG_PER_STEP) d = -SLEW_DEG_PER_STEP;
  return cur + d;
}

void servosToDefault() {
  curSL = curSR = STEER_CENTER_DEG;
  servoSL.write(STEER_L_INVERT ? 180 - STEER_CENTER_DEG : STEER_CENTER_DEG);
  servoSR.write(STEER_R_INVERT ? 180 - STEER_CENTER_DEG : STEER_CENTER_DEG);
}

void updateServos() {
  bool linkOk = (millis() - lastPacketMs) < FAILSAFE_MS;

  float targetSL = STEER_CENTER_DEG, targetSR = STEER_CENTER_DEG;   // disarmed or link lost -> centre
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

// ---------- ESC handling (propellers, called every 20 ms) ----------
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

// ---------- DC wheel motor handling (called every 20 ms) ----------
void pwmBegin(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, DC_PWM_FREQ, DC_PWM_BITS, ch);
#else
  ledcSetup(ch, DC_PWM_FREQ, DC_PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// signedDuty: -255..255 (already after dead-band mapping). Positive = forward.
void driveMotor(const DcMotor& m, int signedDuty) {
  if (m.invert) signedDuty = -signedDuty;
  if (signedDuty > 0) {
    pwmWrite(m.pinA, m.chA, (uint32_t)signedDuty);
    pwmWrite(m.pinB, m.chB, 0);
  } else if (signedDuty < 0) {
    pwmWrite(m.pinA, m.chA, 0);
    pwmWrite(m.pinB, m.chB, (uint32_t)(-signedDuty));
  } else {
    pwmWrite(m.pinA, m.chA, 0);        // both inputs LOW with EN high = brake
    pwmWrite(m.pinB, m.chB, 0);
  }
}

// Slider value -255..255 -> duty with dead-band compensation and cap
int dcToDuty(int v) {
  if (v == 0) return 0;
  int mag = abs(v);
  if (mag > 255) mag = 255;
  int duty = DC_MIN_PWM + (mag * (DC_MAX_PWM - DC_MIN_PWM)) / 255;
  if (duty > 255) duty = 255;
  return v > 0 ? duty : -duty;
}

void dcWriteAll(int dutyFront, int dutyBack) {
  for (int i = 0; i < 2; i++) driveMotor(motors[FRONT_MOTORS[i]], dutyFront);
  for (int i = 0; i < 2; i++) driveMotor(motors[BACK_MOTORS[i]],  dutyBack);
}

void dcToZero() {
  curDcF = curDcB = 0;
  dcWriteAll(0, 0);
}

// Going through zero (reversal) counts as slowing down, so reversing is always gentle
float rampDc(float cur, float target) {
  bool slowing = (cur * target < 0) || (fabsf(target) < fabsf(cur));
  float limit = slowing ? DC_RAMP_DOWN_PER_STEP : DC_RAMP_UP_PER_STEP;
  float d = target - cur;
  if (d >  limit) d =  limit;
  if (d < -limit) d = -limit;
  return cur + d;
}

void updateDc() {
  bool linkOk = (millis() - lastPacketMs) < FAILSAFE_MS;
  bool armed  = ctl_arm && linkOk;

  static bool prevArmed = false;
  if (armed && !prevArmed) dcUnlocked = false;         // every (re)arm needs a fresh zero
  prevArmed = armed;

  if (!armed) {                                        // disarmed or link lost: stop at once
    dcUnlocked = false;
    dcToZero();
    return;
  }

  if (!dcUnlocked && ctl_dcF == 0 && ctl_dcB == 0) dcUnlocked = true;

  float targetF = dcUnlocked ? ctl_dcF : 0;
  float targetB = dcUnlocked ? ctl_dcB : 0;

  curDcF = rampDc(curDcF, targetF);
  curDcB = rampDc(curDcB, targetB);

  dcWriteAll(dcToDuty((int)lroundf(curDcF)), dcToDuty((int)lroundf(curDcB)));
}

void setup() {
  Serial.begin(115200);
  bootMs = millis();

  // 0) Motor driver inputs LOW immediately, so the wheels cannot twitch while the rest starts up
  for (int i = 0; i < 4; i++) {
    pinMode(motors[i].pinA, OUTPUT); digitalWrite(motors[i].pinA, LOW);
    pinMode(motors[i].pinB, OUTPUT); digitalWrite(motors[i].pinB, LOW);
  }

  // 1) ESCs: send zero throttle immediately so they arm safely
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  escA.setPeriodHertz(50);
  escB.setPeriodHertz(50);
  escA.attach(ESC_A_PIN, ESC_MIN_US, ESC_MAX_US);
  escB.attach(ESC_B_PIN, ESC_MIN_US, ESC_MAX_US);
  escsToZero();

  // 2) Steering servos to centre
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoSL.setPeriodHertz(50);
  servoSR.setPeriodHertz(50);
  servoSL.attach(STEER_L_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoSR.attach(STEER_R_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servosToDefault();

  // 3) DC motor PWM (after servos/ESCs; uses LEDC channels 8..15 = a different timer group)
  for (int i = 0; i < 4; i++) {
    pwmBegin(motors[i].pinA, motors[i].chA);
    pwmBegin(motors[i].pinB, motors[i].chB);
  }
  dcToZero();

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
    dcToZero();
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

  // Actuator update every 20 ms
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate >= 20) {
    lastUpdate = millis();
    updateServos();
    updateEscs();
    updateDc();
  }

  // Print status 5 times per second
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.printf("dcF=%d dcB=%d thrA=%d thrB=%d arm=%d age=%ums | wheels F=%.0f B=%.0f dcUnlocked=%d | escA=%dus escB=%dus thrUnlocked=%d | steer L=%.0f R=%.0f\n",
                  ctl_dcF, ctl_dcB, ctl_thrA, ctl_thrB, ctl_arm,
                  (unsigned)(millis() - lastPacketMs),
                  curDcF, curDcB, dcUnlocked,
                  throttleToUs(curThrA), throttleToUs(curThrB), throttleUnlocked,
                  curSL, curSR);
  }
}
