#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <mcp2515.h>
#include "pin_config.h"

#define SERIAL_BAUD 115200

// ===================================
// CONFIGURACION CAN
// ===================================
#define CAN_SPEED   CAN_250KBPS
#define CAN_CLOCK   MCP_16MHZ

const char* AP_SSID = "T2CAN_MONITOR";
const char* AP_PASS = "12345678";

static const size_t MAX_IDS = 80;
static const size_t MAX_SIGNALS = 16;
// ===================================

WebServer server(80);
MCP2515 canBus(MCP2515_CS);
struct can_frame rxFrame;

struct FrameState {
  bool valid;
  uint32_t id;
  bool ext;
  bool rtr;
  uint8_t dlc;
  uint8_t data[8];
  uint8_t changedMask[8];
  uint32_t createdOrder;
  uint32_t updates;
  uint32_t lastUpdateMs;
};

FrameState states[MAX_IDS];
uint32_t nextOrder = 1;
bool canStarted = false;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// =========================
// SEÑALES DECODIFICADAS
// =========================
enum SignalType {
  SIGNAL_HEX = 0,
  SIGNAL_MAP = 1,
  SIGNAL_U24_LE_DIV128 = 2,
  SIGNAL_S8_KMH = 3
};

struct SignalRule {
  bool enabled;
  const char* name;
  uint32_t id;
  bool ext;
  uint8_t byteIndex;
  SignalType type;

  uint8_t mapValue1;
  const char* mapLabel1;
  uint8_t mapValue2;
  const char* mapLabel2;
  uint8_t mapValue3;
  const char* mapLabel3;
};

SignalRule signalRules[MAX_SIGNALS] = {
  {
    true,
    "Dirección",
    0x18A,
    false,
    6,
    SIGNAL_MAP,
    0x20, "Reversa",
    0x40, "Adelante",
    0x00, "Detenido/Neutro"
  },
  {
    true,
    "Horómetro",
    0x183,
    false,
    0,
    SIGNAL_U24_LE_DIV128,
    0x00, "",
    0x00, "",
    0x00, ""
  },
  {
    true,
    "Velocidad",
    0x205,
    false,
    2,
    SIGNAL_S8_KMH,
    0x00, "",
    0x00, "",
    0x00, ""
  },
  {
    false,
    "Horquillas",
    0x000,
    false,
    0,
    SIGNAL_HEX,
    0x00, "",
    0x00, "",
    0x00, ""
  }
};

void hardResetMCP2515() {
  pinMode(MCP2515_RST, OUTPUT);
  digitalWrite(MCP2515_RST, HIGH);
  delay(10);
  digitalWrite(MCP2515_RST, LOW);
  delay(20);
  digitalWrite(MCP2515_RST, HIGH);
  delay(50);
}

String byteToHex(uint8_t v) {
  char b[3];
  snprintf(b, sizeof(b), "%02X", v);
  return String(b);
}

int findFrameIndexInArray(FrameState* arr, uint32_t id, bool ext) {
  for (size_t i = 0; i < MAX_IDS; i++) {
    if (arr[i].valid && arr[i].id == id && arr[i].ext == ext) {
      return (int)i;
    }
  }
  return -1;
}

int findFrameIndex(uint32_t id, bool ext) {
  return findFrameIndexInArray(states, id, ext);
}

int findFreeIndex() {
  for (size_t i = 0; i < MAX_IDS; i++) {
    if (!states[i].valid) return (int)i;
  }

  uint32_t oldestOrder = 0xFFFFFFFF;
  int oldestIdx = 0;
  for (size_t i = 0; i < MAX_IDS; i++) {
    if (states[i].createdOrder < oldestOrder) {
      oldestOrder = states[i].createdOrder;
      oldestIdx = (int)i;
    }
  }
  return oldestIdx;
}

void clearStates() {
  portENTER_CRITICAL(&mux);
  memset(states, 0, sizeof(states));
  nextOrder = 1;
  portEXIT_CRITICAL(&mux);
}

void updateStateFromFrame(const struct can_frame &frame) {
  uint32_t rawId = frame.can_id;
  bool ext = rawId & CAN_EFF_FLAG;
  bool rtr = rawId & CAN_RTR_FLAG;
  uint32_t id = ext ? (rawId & CAN_EFF_MASK) : (rawId & CAN_SFF_MASK);

  portENTER_CRITICAL(&mux);

  int idx = findFrameIndex(id, ext);
  if (idx < 0) idx = findFreeIndex();

  FrameState &s = states[idx];

  if (!s.valid || s.id != id || s.ext != ext) {
    memset(&s, 0, sizeof(s));
    s.valid = true;
    s.id = id;
    s.ext = ext;
    s.rtr = rtr;
    s.dlc = frame.can_dlc;
    s.createdOrder = nextOrder++;
    s.updates = 1;
    s.lastUpdateMs = millis();

    for (int i = 0; i < 8; i++) {
      if (i < frame.can_dlc) {
        s.data[i] = frame.data[i];
        s.changedMask[i] = 0xFF;
      } else {
        s.data[i] = 0;
        s.changedMask[i] = 0;
      }
    }
  } else {
    s.rtr = rtr;
    s.dlc = frame.can_dlc;
    s.updates++;
    s.lastUpdateMs = millis();

    for (int i = 0; i < 8; i++) {
      if (i < frame.can_dlc) {
        s.changedMask[i] = (s.data[i] != frame.data[i]) ? 1 : 0;
        s.data[i] = frame.data[i];
      } else {
        s.changedMask[i] = 0;
        s.data[i] = 0;
      }
    }
  }

  portEXIT_CRITICAL(&mux);
}

bool startCAN() {
  SPI.begin(MCP2515_SCLK, MCP2515_MISO, MCP2515_MOSI, MCP2515_CS);
  pinMode(MCP2515_CS, OUTPUT);
  digitalWrite(MCP2515_CS, HIGH);

  hardResetMCP2515();

  canBus.reset();
  delay(100);

  auto err = canBus.setBitrate(CAN_SPEED, CAN_CLOCK);
  Serial.printf("setBitrate -> %d\n", (int)err);
  if (err != MCP2515::ERROR_OK) return false;

  err = canBus.setListenOnlyMode();
  Serial.printf("setListenOnlyMode -> %d\n", (int)err);
  if (err != MCP2515::ERROR_OK) return false;

  canStarted = true;
  return true;
}

const char PAGE_INDEX[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1.0">
  <title>Plataforma remota lector CAN</title>
  <style>
    :root{
      --bg:#0c1220;
      --panel:#141c31;
      --panel2:#10172b;
      --line:#2b385e;
      --text:#eef3ff;
      --muted:#9ba9cf;
      --chip:#1c2747;
      --chipBorder:#334471;
      --flash:#ffd166;
      --flashText:#2a1c00;
      --accent:#7da8ff;
      --good:#79e0b8;
    }
    *{box-sizing:border-box}
    body{
      margin:0;
      font-family:Inter,Segoe UI,Arial,sans-serif;
      background:linear-gradient(180deg,#0a1020,#10182d);
      color:var(--text);
    }
    .wrap{
      max-width:1400px;
      margin:auto;
      padding:18px;
    }
    .top{
      background:linear-gradient(180deg,var(--panel),var(--panel2));
      border:1px solid var(--line);
      border-radius:18px;
      padding:18px;
      margin-bottom:14px;
      box-shadow:0 8px 24px rgba(0,0,0,.22);
    }
    h1{
      margin:0 0 6px;
      font-size:28px;
    }
    .sub{
      margin:0;
      color:var(--muted);
      font-size:14px;
    }
    .toolbar{
      display:flex;
      justify-content:space-between;
      align-items:center;
      flex-wrap:wrap;
      gap:10px;
      background:linear-gradient(180deg,var(--panel),var(--panel2));
      border:1px solid var(--line);
      border-radius:18px;
      padding:14px 16px;
      margin-bottom:14px;
    }
    .left,.right{
      display:flex;
      align-items:center;
      gap:10px;
      flex-wrap:wrap;
    }
    button,input{
      background:#111936;
      color:var(--text);
      border:1px solid #344372;
      border-radius:12px;
      padding:10px 12px;
      font-weight:700;
    }
    button{cursor:pointer}
    .pill{
      padding:9px 12px;
      border-radius:999px;
      border:1px solid var(--line);
      color:var(--muted);
      background:#111936;
      font-size:13px;
    }
    .layout{
      display:grid;
      grid-template-columns: 1.5fr .9fr;
      gap:14px;
      align-items:start;
    }
    .card{
      background:linear-gradient(180deg,var(--panel),var(--panel2));
      border:1px solid var(--line);
      border-radius:18px;
      overflow:hidden;
      box-shadow:0 8px 24px rgba(0,0,0,.22);
    }
    .cardTitle{
      padding:16px 18px;
      border-bottom:1px solid var(--line);
      font-size:16px;
      font-weight:800;
      color:var(--accent);
    }
    .tableWrap{
      overflow:auto;
      max-height:76vh;
    }
    table{
      width:100%;
      border-collapse:collapse;
      min-width:760px;
    }
    thead th{
      position:sticky;
      top:0;
      z-index:2;
      background:#16203b;
      color:#b8c4e8;
      text-align:left;
      font-size:12px;
      padding:12px;
      border-bottom:1px solid var(--line);
    }
    tbody td{
      padding:12px;
      border-bottom:1px solid rgba(75,91,140,.35);
      font-size:14px;
      vertical-align:middle;
    }
    tbody tr:nth-child(odd){
      background:rgba(255,255,255,.015);
    }
    .mono{
      font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
    }
    .idTag{
      display:inline-block;
      padding:6px 10px;
      border-radius:10px;
      background:#111936;
      border:1px solid #344372;
      font-weight:800;
    }
    .bytes{
      display:flex;
      gap:8px;
      flex-wrap:wrap;
    }
    .byte{
      min-width:42px;
      text-align:center;
      padding:8px 10px;
      border-radius:10px;
      background:var(--chip);
      border:1px solid var(--chipBorder);
      font-weight:800;
      transition:background-color .15s ease, color .15s ease, transform .15s ease;
    }
    .byte.flash{
      background:var(--flash);
      color:var(--flashText);
      border-color:#ffde8a;
      transform:scale(1.04);
    }
    .muted{
      color:var(--muted);
    }
    .signals{
      padding:14px;
      display:flex;
      flex-direction:column;
      gap:12px;
    }
    .signalCard{
      border:1px solid var(--line);
      background:rgba(255,255,255,.02);
      border-radius:14px;
      padding:14px;
    }
    .signalName{
      font-size:15px;
      font-weight:800;
      margin-bottom:8px;
      color:var(--good);
    }
    .signalValue{
      font-size:22px;
      font-weight:900;
      margin-bottom:8px;
    }
    .signalMeta{
      font-size:12px;
      color:var(--muted);
      line-height:1.5;
    }
    .emptyNote{
      color:var(--muted);
      padding:16px;
      font-size:14px;
    }
    @media (max-width:1100px){
      .layout{
        grid-template-columns:1fr;
      }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <section class="top">
      <h1>Plataforma remota lector CAN</h1>
      <p class="sub">GISULMEX Research and Development · Monitor compacto por ID + panel lateral de señales decodificadas.</p>
    </section>

    <section class="toolbar">
      <div class="left">
        <button id="pauseBtn">Pause</button>
        <button id="clearBtn">Clear</button>
        <span class="pill">250 kbps</span>
        <span class="pill">Listen Only</span>
        <span class="pill">192.168.4.1</span>
      </div>
      <div class="right">
        <input id="idFilter" placeholder="Filtrar ID: 123 o 0x1A5">
        <span class="pill">IDs: <span id="idCount">0</span></span>
      </div>
    </section>

    <section class="layout">
      <section class="card">
        <div class="cardTitle">Tramas CAN agrupadas</div>
        <div class="tableWrap">
          <table>
            <thead>
              <tr>
                <th>ID</th>
                <th>DLC</th>
                <th>HEX</th>
                <th>Updates</th>
              </tr>
            </thead>
            <tbody id="body"></tbody>
          </table>
        </div>
      </section>

      <section class="card">
        <div class="cardTitle">Señales decodificadas</div>
        <div class="signals" id="signalsPanel"></div>
      </section>
    </section>
  </div>

<script>
let paused = false;
let items = [];
let signals = [];
const flashMap = new Map();

const bodyEl = document.getElementById('body');
const idCountEl = document.getElementById('idCount');
const pauseBtn = document.getElementById('pauseBtn');
const clearBtn = document.getElementById('clearBtn');
const idFilterEl = document.getElementById('idFilter');
const signalsPanelEl = document.getElementById('signalsPanel');

function normalizeFilter(v){
  return v.trim().toLowerCase();
}

function matchId(item, f) {
  if (!f) return true;
  const dec = String(item.id);
  const hexNoPrefix = Number(item.id).toString(16).toLowerCase();
  const hex = '0x' + hexNoPrefix;
  return f === dec || f === hex || f === hexNoPrefix;
}

function bytesHtml(item) {
  let html = '<div class="bytes">';
  for (let i = 0; i < item.dlc; i++) {
    const key = item.id + '_' + i;
    const flash = flashMap.get(key) ? 'flash' : '';
    html += `<div class="byte mono ${flash}">${item.data[i]}</div>`;
  }
  html += '</div>';
  return html;
}

function renderFrames() {
  const filter = normalizeFilter(idFilterEl.value);

  const filtered = items
    .filter(x => matchId(x, filter))
    .sort((a,b) => a.order - b.order);

  bodyEl.innerHTML = filtered.map(item => `
    <tr>
      <td class="mono"><span class="idTag">0x${Number(item.id).toString(16).toUpperCase()}</span></td>
      <td class="mono">${item.dlc}</td>
      <td>${bytesHtml(item)}</td>
      <td class="mono muted">${item.updates}</td>
    </tr>
  `).join('');

  idCountEl.textContent = filtered.length;
}

function renderSignals() {
  if (!signals.length) {
    signalsPanelEl.innerHTML = `<div class="emptyNote">Aún no hay señales configuradas o disponibles.</div>`;
    return;
  }

  signalsPanelEl.innerHTML = signals.map(sig => `
    <div class="signalCard">
      <div class="signalName">${sig.name}</div>
      <div class="signalValue mono">${sig.value}</div>
      <div class="signalMeta">
        ID: 0x${Number(sig.id).toString(16).toUpperCase()}<br>
        Byte inicial: ${sig.byte_index}<br>
        Valor bruto: ${sig.raw}
      </div>
    </div>
  `).join('');
}

function renderAll() {
  renderFrames();
  renderSignals();
}

function updateFlashes(newItems) {
  for (const item of newItems) {
    if (!Array.isArray(item.changed)) continue;
    for (let i = 0; i < item.changed.length; i++) {
      if (item.changed[i] === "01") {
        const key = item.id + '_' + i;
        flashMap.set(key, true);
        setTimeout(() => {
          flashMap.delete(key);
          renderFrames();
        }, 320);
      }
    }
  }
}

async function fetchData() {
  if (paused) return;

  try {
    const r = await fetch('/data', { cache: 'no-store' });
    const j = await r.json();
    items = j.items || [];
    signals = j.signals || [];
    updateFlashes(items);
    renderAll();
  } catch (e) {}
}

pauseBtn.addEventListener('click', () => {
  paused = !paused;
  pauseBtn.textContent = paused ? 'Resume' : 'Pause';
});

clearBtn.addEventListener('click', async () => {
  try {
    await fetch('/clear', { method:'POST' });
    items = [];
    signals = [];
    flashMap.clear();
    renderAll();
  } catch (e) {}
});

idFilterEl.addEventListener('input', renderFrames);

setInterval(fetchData, 180);
fetchData();
</script>
</body>
</html>
)rawliteral";

String decodeSignalValue(const SignalRule& rule, const FrameState& frame) {
  if (rule.byteIndex >= frame.dlc) return "N/A";

  if (rule.type == SIGNAL_HEX) {
    return "0x" + byteToHex(frame.data[rule.byteIndex]);
  }

  if (rule.type == SIGNAL_MAP) {
    uint8_t rawByte = frame.data[rule.byteIndex];
    if (rawByte == rule.mapValue1) return String(rule.mapLabel1);
    if (rawByte == rule.mapValue2) return String(rule.mapLabel2);
    if (rawByte == rule.mapValue3) return String(rule.mapLabel3);
    return "0x" + byteToHex(rawByte);
  }

  if (rule.type == SIGNAL_U24_LE_DIV128) {
    if (rule.byteIndex + 2 >= frame.dlc) return "N/A";

    uint32_t raw =
      ((uint32_t)frame.data[rule.byteIndex]) |
      ((uint32_t)frame.data[rule.byteIndex + 1] << 8) |
      ((uint32_t)frame.data[rule.byteIndex + 2] << 16);

    float hours = raw / 128.0f;

    char out[24];
    snprintf(out, sizeof(out), "%.2f h", hours);
    return String(out);
  }

  if (rule.type == SIGNAL_S8_KMH) {
    int8_t signedSpeed = (int8_t)frame.data[rule.byteIndex];

    float speedKmh = signedSpeed / 4.75f;

    char out[24];
    snprintf(out, sizeof(out), "%.1f km/h", speedKmh);
    return String(out);
  }

  return "N/A";
}

String getRawSignalString(const SignalRule& rule, const FrameState& frame) {
  if (rule.byteIndex >= frame.dlc) return "N/A";

  if (rule.type == SIGNAL_U24_LE_DIV128) {
    if (rule.byteIndex + 2 >= frame.dlc) return "N/A";

    String s = "0x";
    s += byteToHex(frame.data[rule.byteIndex + 2]);
    s += byteToHex(frame.data[rule.byteIndex + 1]);
    s += byteToHex(frame.data[rule.byteIndex]);
    return s;
  }

  return "0x" + byteToHex(frame.data[rule.byteIndex]);
}

void handleRoot() {
  server.send_P(200, "text/html", PAGE_INDEX);
}

void handleClear() {
  clearStates();
  server.send(200, "text/plain", "OK");
}

void handleData() {
  FrameState snap[MAX_IDS];

  portENTER_CRITICAL(&mux);
  memcpy(snap, states, sizeof(states));
  portEXIT_CRITICAL(&mux);

  String json = "{\"items\":[";
  bool first = true;

  for (size_t i = 0; i < MAX_IDS; i++) {
    if (!snap[i].valid) continue;

    if (!first) json += ",";
    first = false;

    json += "{";
    json += "\"id\":" + String(snap[i].id) + ",";
    json += "\"ext\":" + String(snap[i].ext ? "true" : "false") + ",";
    json += "\"rtr\":" + String(snap[i].rtr ? "true" : "false") + ",";
    json += "\"dlc\":" + String(snap[i].dlc) + ",";
    json += "\"order\":" + String(snap[i].createdOrder) + ",";
    json += "\"updates\":" + String(snap[i].updates) + ",";
    json += "\"data\":[";
    for (int b = 0; b < snap[i].dlc; b++) {
      if (b) json += ",";
      json += "\"" + byteToHex(snap[i].data[b]) + "\"";
    }
    json += "],";
    json += "\"changed\":[";
    for (int b = 0; b < snap[i].dlc; b++) {
      if (b) json += ",";
      json += "\"" + String(snap[i].changedMask[b] ? "01" : "00") + "\"";
    }
    json += "]";
    json += "}";
  }

  json += "],\"signals\":[";
  bool firstSignal = true;

  for (size_t s = 0; s < MAX_SIGNALS; s++) {
    if (!signalRules[s].enabled) continue;

    int idx = findFrameIndexInArray(snap, signalRules[s].id, signalRules[s].ext);
    if (idx < 0) continue;

    const FrameState &frame = snap[idx];
    if (!frame.valid) continue;

    String value = decodeSignalValue(signalRules[s], frame);
    String raw = getRawSignalString(signalRules[s], frame);

    if (!firstSignal) json += ",";
    firstSignal = false;

    json += "{";
    json += "\"name\":\"" + String(signalRules[s].name) + "\",";
    json += "\"id\":" + String(signalRules[s].id) + ",";
    json += "\"byte_index\":" + String(signalRules[s].byteIndex) + ",";
    json += "\"raw\":\"" + raw + "\",";
    json += "\"value\":\"" + value + "\"";
    json += "}";
  }

  json += "]}";
  server.send(200, "application/json", json);
}

void startWiFiAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println();
  Serial.println("WiFi AP iniciado");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("PASS: ");
  Serial.println(AP_PASS);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

void startWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/clear", HTTP_POST, handleClear);
  server.begin();

  Serial.println("Servidor web iniciado");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(2000);

  Serial.println();
  Serial.println("=== T2CAN COMPACT MONITOR ===");
  Serial.println("Objetivo: 250 kbps, Listen Only, Accept All");
  Serial.printf("MCP2515_CS   = %d\n", MCP2515_CS);
  Serial.printf("MCP2515_SCLK = %d\n", MCP2515_SCLK);
  Serial.printf("MCP2515_MOSI = %d\n", MCP2515_MOSI);
  Serial.printf("MCP2515_MISO = %d\n", MCP2515_MISO);
  Serial.printf("MCP2515_RST  = %d\n", MCP2515_RST);

  clearStates();

  if (!startCAN()) {
    Serial.println("No se pudo iniciar MCP2515");
    while (true) delay(1000);
  }

  Serial.println("MCP2515 listo");
  Serial.println("Modo: Listen Only");
  Serial.println("Velocidad: 250 kbps");
  Serial.println("Clock: 16 MHz");

  startWiFiAP();
  startWebServer();
}

void loop() {
  server.handleClient();

  if (!canStarted) return;

  auto err = canBus.readMessage(&rxFrame);

  if (err == MCP2515::ERROR_OK) {
    updateStateFromFrame(rxFrame);
  } else if (err != MCP2515::ERROR_NOMSG) {
    delay(5);
  }
}