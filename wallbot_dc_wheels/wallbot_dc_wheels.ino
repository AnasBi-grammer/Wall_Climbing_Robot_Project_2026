/*
  WallBot - Wi-Fi AP + ArduinoOTA + WebSocket slider page

  Step 5:
    - Steering servos REMOVED. Turning is done by driving the LEFT and RIGHT sides at different speeds.
    - 6 DC wheel motors (3 per side) through 3x HW-095 (L298N) drivers:
        front pair (driver F), middle pair (driver M), back pair (driver B)
    - Two drive modes on the page:  ROWS  (front / middle / back sliders, with "link all rows")
                                    SIDES (left / right sliders, with "link both sides")
    - 2 BLHeli bidirectional ESCs (propellers), 2 sliders (unchanged)
    - Vision hook now sends LEFT/RIGHT side speeds:   H,<left>,<right>

  !!! REMOVE THE PROPELLERS and LIFT THE WHEELS for all first tests !!!

  Libraries (Library Manager or ZIP):
    - "ESP Async WebServer" by ESP32Async
    - "Async TCP" by ESP32Async
    - "ESP32Servo" by Kevin Harrington / John K. Bennett   (now only used to make the ESC signals)
  Works with ESP32 Arduino core 2.x and 3.x (the PWM calls are chosen automatically).

  ======================================= PIN TABLE =======================================
  GPIO | Function                         | Goes to
  -----+----------------------------------+-----------------------------------------------
   25  | ESC 1 signal (propeller 1)       | ESC 1 signal wire (+ common GND)
   26  | ESC 2 signal (propeller 2)       | ESC 2 signal wire (+ common GND)
   13  | Front-left   IN1  (motor 0, A)   | Driver F  IN1
   18  | Front-left   IN2  (motor 0, B)   | Driver F  IN2
   19  | Front-right  IN3  (motor 1, A)   | Driver F  IN3
   21  | Front-right  IN4  (motor 1, B)   | Driver F  IN4
   27  | Middle-left  IN1  (motor 2, A)   | Driver M  IN1
   32  | Middle-left  IN2  (motor 2, B)   | Driver M  IN2
   16  | Middle-right IN3  (motor 3, A)   | Driver M  IN3   (not usable on WROVER/PSRAM modules)
   17  | Middle-right IN4  (motor 3, B)   | Driver M  IN4   (not usable on WROVER/PSRAM modules)
   22  | Back-left    IN1  (motor 4, A)   | Driver B  IN1
   23  | Back-left    IN2  (motor 4, B)   | Driver B  IN2
    4  | Back-right   IN3  (motor 5, A)   | Driver B  IN3
   33  | Back-right   IN4  (motor 5, B)   | Driver B  IN4
  Free input-only pins (sensors, e.g. battery voltage on ADC1): 34, 35, 36, 39
  Avoid (boot/flash/serial): 0, 1, 2, 3, 5, 6-11, 12, 14, 15
  =========================================================================================

  Driver OUT terminals:  OUT1/OUT2 -> left motor of that pair,  OUT3/OUT4 -> right motor of that pair
  Keep the ENA and ENB jumpers ON on all three drivers (speed is made by PWM on the IN pins).
    forward : INa = PWM, INb = LOW     reverse : INa = LOW, INb = PWM     stop : both LOW (brake)

  Motor power:  Driver F and Driver B  <- XL4015 output
                Driver M               <- the buck that used to power the servos, set to 8.0 V BEFORE connecting
                (set the XL4015 to 8 V too, so all six motors get the same voltage and run at the same speed)
  Common ground: battery -, ESCs, all bucks, all three drivers and the ESP32 must share one ground.
  Do NOT power the ESP32 from a driver's 5 V pin (the LM2596 does that).
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
const uint32_t FAILSAFE_MS = 300;    // no packet for this long -> everything stops
                                     // (the wall-specific failsafe will be designed later)

// ---------- ESC settings (propellers, standard 50 Hz PWM, 1000..2000 us) ----------
const int ESC_A_PIN = 25;            // ESC 1 signal pin
const int ESC_B_PIN = 26;            // ESC 2 signal pin
const int ESC_MIN_US = 1000;         // full reverse
const int ESC_NEUTRAL_US = 1500;     // zero throttle
const int ESC_MAX_US = 2000;         // full throttle

const float THROTTLE_LIMIT_PCT = 30.0;     // slider 100% = 30% of full ESC range. RAISE ONLY AFTER TESTING.
const uint32_t ESC_ARM_HOLD_MS = 3000;     // hold zero throttle this long after boot so the ESCs arm
const float RAMP_UP_PER_STEP   = 1.5;      // slider % per 20 ms going up  (~75 % per second)
const float RAMP_DOWN_PER_STEP = 5.0;      // slider % per 20 ms going down

// ---------- DC wheel motors (6x, through 3x HW-095 / L298N) ----------
const uint32_t DC_PWM_FREQ = 5000;   // Hz. L298N is slow, so keep it <= ~10 kHz
const uint8_t  DC_PWM_BITS = 8;      // duty 0..255
const int DC_MIN_PWM = 60;           // slider value 1 -> this duty (DC motors do not start below ~25 %). Set 0 for raw PWM
const int DC_MAX_PWM = 255;          // duty cap. Lower it if the motors get more voltage than they like
const float DC_RAMP_UP_PER_STEP   = 12.0;   // slider units per 20 ms going up   (~600/s, 0 -> full in ~0.45 s)
const float DC_RAMP_DOWN_PER_STEP = 25.0;   // slider units per 20 ms going down / through zero (a full reversal ~0.6 s)

const int VISION_MAX_SPEED = 160;    // the camera can never command more than this (slider units, 0..255)

#define NUM_MOTORS 6

struct DcMotor {
  uint8_t pinA, pinB;                // L298N IN pins
  uint8_t chA, chB;                  // LEDC channels 4..15 (channels 0..3 stay free for the ESC signals)
  bool invert;                       // flip if this wheel turns the wrong way
  bool left;                         // true = left side, false = right side
};

//  Index order everywhere (packets, arrays):  0=FL  1=FR  2=ML  3=MR  4=BL  5=BR
DcMotor motors[NUM_MOTORS] = {
  // pinA pinB chA chB invert left
  {  13,  18,  4,  5, false, true  },   // 0 front-left    (driver F, IN1/IN2)
  {  19,  21,  6,  7, true,  false },   // 1 front-right   (driver F, IN3/IN4)  mirrored mounting -> inverted
  {  27,  32,  8,  9, false, true  },   // 2 middle-left   (driver M, IN1/IN2)
  {  16,  17, 10, 11, true,  false },   // 3 middle-right  (driver M, IN3/IN4)  mirrored mounting -> inverted
  {  22,  23, 12, 13, false, true  },   // 4 back-left     (driver B, IN1/IN2)
  {   4,  33, 14, 15, true,  false },   // 5 back-right    (driver B, IN3/IN4)  mirrored mounting -> inverted
};

// ---------- Latest values received from the page ----------
volatile int ctl_m[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};   // -255..255 per motor (the page computes them)
volatile int ctl_thrA  = 0;                   // -100..100 (%) propeller ESC 1
volatile int ctl_thrB  = 0;                   // -100..100 (%) propeller ESC 2
volatile int ctl_arm   = 0;                   // 0/1
volatile uint32_t lastPacketMs = 0;

// ---------- Vision (camera) side commands ----------
volatile bool visionOn = false;
volatile int  vis_left  = 0;                  // -255..255, sent by the Python client as H,<left>,<right>
volatile int  vis_right = 0;
volatile uint32_t lastVisionMs = 0;
uint32_t controllerId = 0;                    // id of the web-page client (the one sending S packets)
const uint32_t VISION_TIMEOUT_MS = 300;

Servo escA, escB;
float curThrA = 0, curThrB = 0;      // current throttle (slider %), after ramping
bool  throttleUnlocked = false;      // true only after both throttles were seen at 0 while armed
float curDc[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};   // current wheel speeds (slider units), after ramping
bool  dcUnlocked = false;            // true only after all wheel sliders were seen at 0 while armed
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
  .panel { display:flex; flex-direction:column; gap:12px; }
  .hidden { display:none; }
  .top { display:flex; align-items:center; justify-content:space-between; gap:10px; }
  .status { display:flex; align-items:center; gap:8px; font-size:14px; }
  .dot { width:12px; height:12px; border-radius:50%; background:var(--bad); }
  .dot.on { background:var(--ok); }
  button { font-family:inherit; }
  button.arm { padding:14px; font-size:18px; font-weight:700; border-radius:12px; border:2px solid var(--line);
               background:var(--panel); color:var(--fg); }
  button.arm.on { border-color:var(--bad); background:#3a1d1d; color:#ffb3b3; }
  button.vis.on { border-color:var(--ok); background:#1d3a28; color:#b3ffd0; }
  button:disabled { opacity:.5; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:14px; padding:8px 14px; }
  .card h2 { margin:0 0 4px; font-size:13px; letter-spacing:.08em; text-transform:uppercase; color:var(--dim); font-weight:600; }
  .row { display:flex; align-items:baseline; justify-content:space-between; }
  .val { font:700 26px ui-monospace,Menlo,Consolas,monospace; }
  .val small { font-size:16px; color:var(--dim); }
  .ends { display:flex; justify-content:space-between; font-size:12px; color:var(--dim); margin-top:2px; }
  .lnk { display:flex; align-items:center; gap:6px; padding:6px 10px; border:1px solid var(--line); border-radius:8px;
         font-size:13px; color:var(--dim); }
  .lnk input { width:18px; height:18px; }
  input[type=range] { -webkit-appearance:none; appearance:none; width:100%; height:46px; background:transparent;
                      touch-action:manipulation; margin:6px 0 0; }
  input[type=range]::-webkit-slider-runnable-track { height:10px; border-radius:5px; background:var(--line); }
  input[type=range]::-moz-range-track { height:10px; border-radius:5px; background:var(--line); }
  input[type=range]::-webkit-slider-thumb { -webkit-appearance:none; width:44px; height:44px; border-radius:50%;
                      background:var(--acc); border:none; margin-top:-17px; box-shadow:0 2px 10px rgba(0,0,0,.5); }
  input[type=range]::-moz-range-thumb { width:44px; height:44px; border-radius:50%; background:var(--acc); border:none; }
  input[type=range]:disabled { opacity:.45; }
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
  <button class="arm" id="modeBtn" style="padding:10px;font-size:15px;" disabled>DRIVE MODE: ROWS (front / middle / back)</button>

  <!-- ===== ROWS mode ===== -->
  <div class="panel" id="rowsPanel">
    <div class="card">
      <h2>Front wheels (2 DC)</h2>
      <div class="row">
        <div class="val"><span id="rfv">0</span><small> / 255</small></div>
        <label class="lnk"><input type="checkbox" id="linkRows" checked> Link all rows (all 6 DCs)</label>
      </div>
      <input type="range" id="rowF" min="-255" max="255" value="0" disabled>
      <div class="ends"><span>-255 (rev)</span><span>0</span><span>255 (fwd)</span></div>
    </div>

    <div class="card">
      <h2>Middle wheels (2 DC)</h2>
      <div class="row"><div class="val"><span id="rmv">0</span><small> / 255</small></div></div>
      <input type="range" id="rowM" min="-255" max="255" value="0" disabled>
      <div class="ends"><span>-255 (rev)</span><span>0</span><span>255 (fwd)</span></div>
    </div>

    <div class="card">
      <h2>Back wheels (2 DC)</h2>
      <div class="row"><div class="val"><span id="rbv">0</span><small> / 255</small></div></div>
      <input type="range" id="rowB" min="-255" max="255" value="0" disabled>
      <div class="ends"><span>-255 (rev)</span><span>0</span><span>255 (fwd)</span></div>
    </div>
  </div>

  <!-- ===== SIDES mode ===== -->
  <div class="panel hidden" id="sidesPanel">
    <div class="card">
      <h2>Left side (3 DC)</h2>
      <div class="row">
        <div class="val"><span id="slv">0</span><small> / 255</small></div>
        <label class="lnk"><input type="checkbox" id="linkSides"> Link both sides (straight)</label>
      </div>
      <input type="range" id="sideL" min="-255" max="255" value="0" disabled>
      <div class="ends"><span>-255 (rev)</span><span>0</span><span>255 (fwd)</span></div>
    </div>

    <div class="card">
      <h2>Right side (3 DC)</h2>
      <div class="row"><div class="val"><span id="srv">0</span><small> / 255</small></div></div>
      <input type="range" id="sideR" min="-255" max="255" value="0" disabled>
      <div class="ends"><span>-255 (rev)</span><span>0</span><span>255 (fwd)</span></div>
    </div>
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
      <label class="lnk"><input type="checkbox" id="link"> Link both</label>
    </div>
    <input type="range" id="thrB" min="-100" max="100" value="0" disabled>
    <div class="ends"><span>-100% (rev)</span><span>0</span><span>100% (fwd)</span></div>
  </div>

  <button class="arm" id="stop" style="padding:10px;font-size:15px;">ALL MOTORS TO 0</button>

  <div class="hint">Sliders are locked until ARM is ON. Turning ARM on/off or changing drive mode resets all sliders to 0. Wheels and propellers only move after their sliders have been at 0 once while armed. To turn, use SIDES mode and drive one side more than the other.</div>
</div>

<script>
(function () {
  const $ = (id) => document.getElementById(id);
  const dot = $('dot'), stat = $('stat'), rttEl = $('rtt');
  const armBtn = $('arm'), visionBtn = $('visionBtn'), modeBtn = $('modeBtn'), stopBtn = $('stop');
  const rowsPanel = $('rowsPanel'), sidesPanel = $('sidesPanel');

  const rowF = $('rowF'), rowM = $('rowM'), rowB = $('rowB');
  const rfv = $('rfv'), rmv = $('rmv'), rbv = $('rbv');
  const linkRows = $('linkRows');

  const sideL = $('sideL'), sideR = $('sideR');
  const slv = $('slv'), srv = $('srv');
  const linkSides = $('linkSides');

  const thrA = $('thrA'), thrB = $('thrB');
  const av = $('av'), mv = $('mv');
  const link = $('link');

  const wheelSliders = [rowF, rowM, rowB, sideL, sideR];

  let arm = 0;
  let vision = 0;
  let driveMode = 'rows';              // 'rows' or 'sides'
  let ws = null;

  function showValues() {
    rfv.textContent = rowF.value;  rmv.textContent = rowM.value;  rbv.textContent = rowB.value;
    slv.textContent = sideL.value; srv.textContent = sideR.value;
    av.textContent = thrA.value;   mv.textContent = thrB.value;
  }

  function zeroWheels() { wheelSliders.forEach((s) => { s.value = 0; }); }

  function updateLocks() {
    const lockWheels = !arm || !!vision;      // wheels are locked when disarmed or when the camera drives
    wheelSliders.forEach((s) => { s.disabled = lockWheels; });
    modeBtn.disabled = lockWheels;
    thrA.disabled = thrB.disabled = !arm;
    visionBtn.disabled = !arm;
  }

  // ---- rows: link all three together ----
  [rowF, rowM, rowB].forEach((s) => s.addEventListener('input', () => {
    if (linkRows.checked) { rowF.value = rowM.value = rowB.value = s.value; }
    showValues();
  }));
  linkRows.addEventListener('change', () => {
    if (linkRows.checked) { rowM.value = rowB.value = rowF.value; showValues(); }
  });

  // ---- sides: link both ----
  sideL.addEventListener('input', () => { if (linkSides.checked) sideR.value = sideL.value; showValues(); });
  sideR.addEventListener('input', () => { if (linkSides.checked) sideL.value = sideR.value; showValues(); });
  linkSides.addEventListener('change', () => { if (linkSides.checked) { sideR.value = sideL.value; showValues(); } });

  // ---- propellers ----
  thrA.addEventListener('input', () => { if (link.checked) thrB.value = thrA.value; showValues(); });
  thrB.addEventListener('input', () => { if (link.checked) thrA.value = thrB.value; showValues(); });

  stopBtn.addEventListener('click', () => { zeroWheels(); thrA.value = 0; thrB.value = 0; showValues(); });

  // ---- drive mode ----
  function setDriveMode(m) {
    driveMode = m;
    zeroWheels(); showValues();              // nothing jumps when the mode changes
    rowsPanel.classList.toggle('hidden', m !== 'rows');
    sidesPanel.classList.toggle('hidden', m !== 'sides');
    modeBtn.textContent = (m === 'rows')
      ? 'DRIVE MODE: ROWS (front / middle / back)'
      : 'DRIVE MODE: SIDES (left / right)';
  }
  modeBtn.addEventListener('click', () => setDriveMode(driveMode === 'rows' ? 'sides' : 'rows'));

  // ---- vision ----
  function setVisionUI(v) {
    vision = v ? 1 : 0;
    if (vision) { zeroWheels(); showValues(); }
    visionBtn.textContent = 'VISION: ' + (vision ? 'ON (wheels locked, camera drives)' : 'OFF');
    visionBtn.classList.toggle('on', !!vision);
    updateLocks();
  }
  visionBtn.addEventListener('click', () => {
    if (ws && ws.readyState === 1) ws.send('V,' + (vision ? 0 : 1));
  });

  // ---- arm ----
  function setArm(v) {
    arm = v ? 1 : 0;
    zeroWheels(); thrA.value = 0; thrB.value = 0; showValues();   // nothing jumps when arming/disarming
    setVisionUI(arm ? vision : 0);                                // also refreshes the locks
    armBtn.textContent = 'ARM: ' + (arm ? 'ON' : 'OFF');
    armBtn.classList.toggle('on', !!arm);
  }
  armBtn.addEventListener('click', () => setArm(!arm));

  // ---- the six wheel values, order: FL, FR, ML, MR, BL, BR ----
  function motorValues() {
    if (driveMode === 'rows') {
      const f = +rowF.value, m = +rowM.value, b = +rowB.value;
      return [f, f, m, m, b, b];
    }
    const l = +sideL.value, r = +sideR.value;
    return [l, r, l, r, l, r];
  }

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

  // ---- Send control packet at ~30 Hz:  S,FL,FR,ML,MR,BL,BR,thrA,thrB,arm ----
  setInterval(() => {
    if (ws && ws.readyState === 1) {
      ws.send('S,' + motorValues().join(',') + ',' + thrA.value + ',' + thrB.value + ',' + arm);
    }
  }, 33);

  // ---- Ping every second to measure latency ----
  setInterval(() => {
    if (ws && ws.readyState === 1) ws.send('P,' + performance.now());
  }, 1000);

  // Hidden page (app switched, screen off) -> disarm
  document.addEventListener('visibilitychange', () => { if (document.hidden && arm) setArm(0); });

  showValues();
  updateLocks();
})();
</script>
</body>
</html>
)rawliteral";

// ---------- WebSocket events ----------
void resetControls() {
  for (int i = 0; i < NUM_MOTORS; i++) ctl_m[i] = 0;
  ctl_thrA = 0;
  ctl_thrB = 0;
  ctl_arm  = 0;
  visionOn = false;
  vis_left = vis_right = 0;
}

void setVision(bool on) {
  visionOn = on;
  vis_left = vis_right = 0;
  lastVisionMs = 0;                        // stale until the first H packet arrives
  ws.textAll(on ? "V,1" : "V,0");
}

void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
  (void)srv;
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
        // S,FL,FR,ML,MR,BL,BR,thrA,thrB,arm
        int m[NUM_MOTORS], ta, tb, a;
        if (sscanf(buf, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                   &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &ta, &tb, &a) == 9) {
          for (int i = 0; i < NUM_MOTORS; i++) ctl_m[i] = constrain(m[i], -255, 255);
          ctl_thrA = constrain(ta, -100, 100);
          ctl_thrB = constrain(tb, -100, 100);
          ctl_arm  = a ? 1 : 0;
          lastPacketMs = millis();
          controllerId = client->id();
          if (!ctl_arm && visionOn) setVision(false);   // disarm always turns vision off
        }
      } else if (buf[0] == 'V') {
        int v = 0;
        if (sscanf(buf, "V,%d", &v) == 1) setVision(v && ctl_arm);   // only possible while armed
      } else if (buf[0] == 'H') {
        // H,<left>,<right>   side speeds from the camera (Python client)
        int l, r;
        if (sscanf(buf, "H,%d,%d", &l, &r) == 2) {
          vis_left  = constrain(l, -VISION_MAX_SPEED, VISION_MAX_SPEED);
          vis_right = constrain(r, -VISION_MAX_SPEED, VISION_MAX_SPEED);
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
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
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

void dcToZero() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    curDc[i] = 0;
    driveMotor(motors[i], 0);
  }
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

  // The page keeps all wheel sliders at 0 until the user moves them, so this unlocks right after arming
  bool allZero = true;
  for (int i = 0; i < NUM_MOTORS; i++) if (ctl_m[i] != 0) allZero = false;
  if (!dcUnlocked && allZero) dcUnlocked = true;

  float target[NUM_MOTORS];
  if (!dcUnlocked) {
    for (int i = 0; i < NUM_MOTORS; i++) target[i] = 0;
  } else if (visionOn) {
    // Camera control: left/right side speeds. No fresh packet -> stop (never keep driving blind)
    bool fresh = (millis() - lastVisionMs) < VISION_TIMEOUT_MS;
    float l = fresh ? vis_left  : 0;
    float r = fresh ? vis_right : 0;
    for (int i = 0; i < NUM_MOTORS; i++) target[i] = motors[i].left ? l : r;
  } else {
    for (int i = 0; i < NUM_MOTORS; i++) target[i] = ctl_m[i];
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    curDc[i] = rampDc(curDc[i], target[i]);
    driveMotor(motors[i], dcToDuty((int)lroundf(curDc[i])));
  }
}

void setup() {
  Serial.begin(115200);
  bootMs = millis();

  // 0) Motor driver inputs LOW immediately, so the wheels cannot twitch while the rest starts up
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].pinA, OUTPUT); digitalWrite(motors[i].pinA, LOW);
    pinMode(motors[i].pinB, OUTPUT); digitalWrite(motors[i].pinB, LOW);
  }

  // 1) ESCs: send zero throttle immediately so they arm safely
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  escA.setPeriodHertz(50);
  escB.setPeriodHertz(50);
  escA.attach(ESC_A_PIN, ESC_MIN_US, ESC_MAX_US);
  escB.attach(ESC_B_PIN, ESC_MIN_US, ESC_MAX_US);
  escsToZero();

  // 2) DC motor PWM (after the ESCs; uses LEDC channels 4..15)
  for (int i = 0; i < NUM_MOTORS; i++) {
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
    updateEscs();
    updateDc();
  }

  // Print status 5 times per second
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.printf("arm=%d age=%ums vision=%d | wheels FL%.0f FR%.0f ML%.0f MR%.0f BL%.0f BR%.0f unl=%d | thrA=%d thrB=%d escA=%dus escB=%dus unl=%d\n",
                  ctl_arm, (unsigned)(millis() - lastPacketMs), (int)visionOn,
                  curDc[0], curDc[1], curDc[2], curDc[3], curDc[4], curDc[5], dcUnlocked,
                  ctl_thrA, ctl_thrB, throttleToUs(curThrA), throttleToUs(curThrB), throttleUnlocked);
  }
}
