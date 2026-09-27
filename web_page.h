// ============================================================
// web_page.h — dashboard HTML embedded as a PROGMEM raw string
// Source of truth: data/index.html (kept in sync manually)
// ============================================================
#ifndef WEB_PAGE_H
#define WEB_PAGE_H

#include <Arduino.h>

static const char INDEX_HTML[] PROGMEM = R"HTMLDOC(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Pyrolysis Controller</title>
<style>
  :root {
    --bg: #1a1a2e;
    --card: #16213e;
    --teal: #4ecca3;
    --red: #e94560;
    --yellow: #ffd700;
    --text: #eaeaea;
    --text-dim: #8f9bb3;
    --border: #2a3358;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 0;
    background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    max-width: 480px; margin: 0 auto;
    padding-bottom: 70px;
  }
  header {
    background: var(--card);
    padding: 14px 16px;
    display: flex; justify-content: space-between; align-items: center;
    border-bottom: 1px solid var(--border);
    position: sticky; top: 0; z-index: 10;
  }
  header h1 { font-size: 16px; margin: 0; }
  .badge {
    padding: 4px 10px; border-radius: 12px; font-size: 11px; font-weight: 600;
    text-transform: uppercase; letter-spacing: 0.5px;
  }
  .badge.IDLE { background: #444; color: #ccc; }
  .badge.RUNNING { background: var(--teal); color: #06251c; }
  .badge.AUTOTUNE { background: var(--yellow); color: #3a2e00; }
  .badge.SHUTDOWN { background: var(--red); color: #fff; }
  .badge.LEARNING { background: #7b61ff; color: #fff; }

  .tabs {
    position: fixed; bottom: 0; left: 0; right: 0;
    max-width: 480px; margin: 0 auto;
    display: flex; background: var(--card); border-top: 1px solid var(--border);
    z-index: 20;
  }
  .tab-btn {
    flex: 1; background: none; border: none; color: var(--text-dim);
    padding: 10px 2px; font-size: 10px; text-align: center; cursor: pointer;
  }
  .tab-btn.active { color: var(--teal); }
  .tab-btn .ic { display: block; font-size: 18px; margin-bottom: 2px; }

  .page { display: none; padding: 14px; }
  .page.active { display: block; }

  .card {
    background: var(--card); border: 1px solid var(--border);
    border-radius: 10px; padding: 14px; margin-bottom: 14px;
  }
  .card h3 { margin: 0 0 10px 0; font-size: 13px; color: var(--text-dim); text-transform: uppercase; letter-spacing: 0.5px; }

  .row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
  .row:last-child { margin-bottom: 0; }
  .label { color: var(--text-dim); font-size: 13px; }
  .value { font-weight: 600; font-size: 15px; }
  .value.big { font-size: 22px; }

  .gauge-wrap { display: flex; justify-content: center; margin: 6px 0; }

  .progress-bar { width: 100%; height: 18px; background: #0d1229; border-radius: 9px; overflow: hidden; border: 1px solid var(--border); }
  .progress-fill { height: 100%; background: linear-gradient(90deg, var(--teal), #2fa88a); transition: width 0.4s; }

  .toggle-row { display: flex; justify-content: space-between; align-items: center; padding: 8px 0; border-bottom: 1px solid var(--border); }
  .toggle-row:last-child { border-bottom: none; }
  .switch { position: relative; width: 46px; height: 26px; }
  .switch input { opacity: 0; width: 0; height: 0; }
  .slider-toggle {
    position: absolute; cursor: pointer; inset: 0; background: #333;
    border-radius: 26px; transition: 0.2s;
  }
  .slider-toggle:before {
    content: ""; position: absolute; height: 20px; width: 20px; left: 3px; bottom: 3px;
    background: white; border-radius: 50%; transition: 0.2s;
  }
  input:checked + .slider-toggle { background: var(--teal); }
  input:checked + .slider-toggle:before { transform: translateX(20px); }

  button.btn {
    border: none; border-radius: 8px; padding: 12px; font-size: 14px; font-weight: 700;
    cursor: pointer; color: #fff;
  }
  .btn-start { background: var(--teal); color: #06251c; }
  .btn-stop { background: var(--red); }
  .btn-tune { background: var(--yellow); color: #3a2e00; }
  .btn-secondary { background: #33406e; color: #fff; }
  .btn-row { display: flex; gap: 8px; }
  .btn-row button { flex: 1; }

  input[type=number], input[type=text] {
    background: #0d1229; border: 1px solid var(--border); color: var(--text);
    border-radius: 6px; padding: 8px; font-size: 14px; width: 100%;
  }
  label.field-label { font-size: 12px; color: var(--text-dim); display: block; margin-bottom: 4px; margin-top: 10px; }

  input[type=range] { width: 100%; accent-color: var(--teal); }

  table { width: 100%; border-collapse: collapse; font-size: 12px; }
  th, td { padding: 6px 4px; text-align: left; border-bottom: 1px solid var(--border); }
  th { color: var(--text-dim); font-weight: 600; }
  tr.recommended { background: rgba(78, 204, 163, 0.15); color: var(--teal); font-weight: 700; }
  .table-scroll { max-height: 300px; overflow-y: auto; }

  canvas { width: 100%; background: #0d1229; border-radius: 8px; }

  #atProgress { display: none; background: rgba(255,215,0,0.1); border: 1px solid var(--yellow); border-radius: 8px; padding: 10px; margin-top: 10px; color: var(--yellow); font-size: 13px; }

  .conn-dot { width: 9px; height: 9px; border-radius: 50%; display: inline-block; margin-right: 5px; }
  .conn-dot.on { background: var(--teal); }
  .conn-dot.off { background: var(--red); }

  .grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
</style>
</head>
<body>

<header>
  <h1>Pyrolysis Controller</h1>
  <span class="badge IDLE" id="stateBadge">IDLE</span>
</header>

<!-- ============ TAB 1: DASHBOARD ============ -->
<div class="page active" id="page-dashboard">

  <div class="card">
    <h3>Suhu</h3>
    <div class="gauge-wrap"><svg id="tempGauge" width="220" height="140" viewBox="0 0 220 140"></svg></div>
    <div class="row"><span class="label">Suhu saat ini</span><span class="value big" id="txtSuhu">-- &deg;C</span></div>
    <div class="row"><span class="label">Setpoint</span><span class="value" id="txtSetpoint">-- &deg;C</span></div>
  </div>

  <div class="card">
    <h3>Grafik Suhu Real-time (10 menit)</h3>
    <canvas id="tempChart" width="440" height="180"></canvas>
  </div>

  <div class="card grid2">
    <div>
      <div class="label">Fan PWM</div>
      <div class="value big" id="txtFanPct">--%</div>
      <div class="progress-bar"><div class="progress-fill" id="fanBar" style="width:0%; background:linear-gradient(90deg,#4ecca3,#2fa88a);"></div></div>
    </div>
    <div>
      <div class="label">Efisiensi</div>
      <div class="value big" id="txtEff">-- mL/jam</div>
    </div>
  </div>

  <div class="card">
    <h3>Volume Output</h3>
    <div class="row"><span class="label" id="txtVolLabel">0 / 1000 mL</span><span class="value" id="txtVolPct">0%</span></div>
    <div class="progress-bar"><div class="progress-fill" id="volBar" style="width:0%"></div></div>
    <div class="row" style="margin-top:10px;"><span class="label">Durasi</span><span class="value" id="txtDuration">00:00:00</span></div>
    <div class="row"><span class="label">Estimasi selesai</span><span class="value" id="txtEstFinish">--</span></div>
  </div>

  <div class="card">
    <h3>Kontrol Cepat</h3>
    <div class="toggle-row">
      <span class="label">Fan</span>
      <label class="switch"><input type="checkbox" id="togFan" onchange="sendToggle('TOGGLE_FAN', this.checked)"><span class="slider-toggle"></span></label>
    </div>
    <div class="toggle-row">
      <span class="label">Oil Pump</span>
      <label class="switch"><input type="checkbox" id="togOil" onchange="sendToggle('TOGGLE_OIL', this.checked)"><span class="slider-toggle"></span></label>
    </div>
    <div class="toggle-row">
      <span class="label">Water Pump</span>
      <label class="switch"><input type="checkbox" id="togWater" onchange="sendToggle('TOGGLE_WATER', this.checked)"><span class="slider-toggle"></span></label>
    </div>
  </div>
</div>

<!-- ============ TAB 2: KONTROL ============ -->
<div class="page" id="page-kontrol">
  <div class="card">
    <h3>Aksi Utama</h3>
    <div class="btn-row">
      <button class="btn btn-start" onclick="sendCmd('START')">START</button>
      <button class="btn btn-stop" onclick="sendCmd('STOP')">STOP</button>
    </div>
    <div class="btn-row" style="margin-top:8px;">
      <button class="btn btn-tune" onclick="sendCmd('AUTOTUNE')">AUTO-TUNE</button>
    </div>
    <div id="atProgress"></div>
  </div>

  <div class="card">
    <h3>Parameter Proses</h3>
    <label class="field-label">Setpoint Suhu (&deg;C)</label>
    <input type="number" id="inSetpoint" step="1">
    <button class="btn btn-secondary" style="width:100%; margin-top:6px;" onclick="setNumField('SET_SETPOINT','inSetpoint')">Set Setpoint</button>

    <label class="field-label">Target Volume (mL)</label>
    <input type="number" id="inTargetVol" step="10">
    <button class="btn btn-secondary" style="width:100%; margin-top:6px;" onclick="setNumField('SET_TARGET_VOL','inTargetVol')">Set Target Volume</button>

    <label class="field-label">T Shutdown (&deg;C)</label>
    <input type="number" id="inTshutdown" step="1">
    <button class="btn btn-secondary" style="width:100%; margin-top:6px;" onclick="setNumField('SET_TSHUTDOWN','inTshutdown')">Set T Shutdown</button>
  </div>

  <div class="card">
    <h3>Bobot Learning (auto-normalize = 1.0)</h3>
    <label class="field-label">w1 - Produktivitas: <span id="lblW1">0.60</span></label>
    <input type="range" id="rngW1" min="0" max="1" step="0.01" value="0.6" oninput="onWeightChange('w1')">
    <label class="field-label">w2 - Pemakaian PWM: <span id="lblW2">0.20</span></label>
    <input type="range" id="rngW2" min="0" max="1" step="0.01" value="0.2" oninput="onWeightChange('w2')">
    <label class="field-label">w3 - ISE: <span id="lblW3">0.20</span></label>
    <input type="range" id="rngW3" min="0" max="1" step="0.01" value="0.2" oninput="onWeightChange('w3')">
    <button class="btn btn-secondary" style="width:100%; margin-top:10px;" onclick="applyWeights()">Terapkan Bobot</button>
  </div>
</div>

<!-- ============ TAB 3: PID SETTINGS ============ -->
<div class="page" id="page-pid">
  <div class="card">
    <h3>PID Aktif</h3>
    <div class="row"><span class="label">Kp</span><span class="value" id="txtKp">--</span></div>
    <div class="row"><span class="label">Ki</span><span class="value" id="txtKi">--</span></div>
    <div class="row"><span class="label">Kd</span><span class="value" id="txtKd">--</span></div>
    <div class="row" style="margin-top:8px;"><span class="label" id="txtSpInfo">Parameter untuk --&deg;C (0 runs)</span></div>
  </div>

  <div class="card">
    <h3>Override Manual</h3>
    <label class="field-label">Kp</label>
    <input type="number" id="inKp" step="0.01">
    <button class="btn btn-secondary" style="width:100%; margin-top:6px;" onclick="setNumField('SET_KP','inKp')">Set Kp</button>
    <label class="field-label">Ki</label>
    <input type="number" id="inKi" step="0.001">
    <button class="btn btn-secondary" style="width:100%; margin-top:6px;" onclick="setNumField('SET_KI','inKi')">Set Ki</button>
    <label class="field-label">Kd</label>
    <input type="number" id="inKd" step="0.01">
    <button class="btn btn-secondary" style="width:100%; margin-top:6px;" onclick="setNumField('SET_KD','inKd')">Set Kd</button>
  </div>

  <div class="card">
    <h3>Gain Scheduling (3 Zona)</h3>
    <table>
      <tr><th>Zona</th><th>Range</th><th>Kp mult.</th></tr>
      <tr><td>Cold</td><td>&lt; 200&deg;C</td><td>1.30x</td></tr>
      <tr><td>Mid</td><td>200-400&deg;C</td><td>1.00x</td></tr>
      <tr><td>Hot</td><td>&gt; 400&deg;C</td><td>0.70x</td></tr>
    </table>
  </div>

  <div class="card btn-row">
    <button class="btn btn-secondary" onclick="sendCmd('APPLY_BEST')">APPLY BEST PARAMS</button>
    <button class="btn btn-stop" onclick="sendCmd('RESET_DEFAULT')">RESET DEFAULT</button>
  </div>
</div>

<!-- ============ TAB 4: EFFICIENCY MAP ============ -->
<div class="page" id="page-efficiency">
  <div class="card">
    <h3>Peta Efisiensi per Setpoint</h3>
    <div class="table-scroll">
      <table>
        <thead><tr><th>SP (&deg;C)</th><th>Runs</th><th>Eff (mL/h)</th><th>Score</th><th></th></tr></thead>
        <tbody id="effTableBody"></tbody>
      </table>
    </div>
  </div>

  <div class="card">
    <h3>Grafik Efisiensi per Setpoint</h3>
    <svg id="effBarChart" width="440" height="180" viewBox="0 0 440 180"></svg>
  </div>

  <div class="card">
    <h3>Riwayat ISE (turun = makin pintar)</h3>
    <canvas id="iseChart" width="440" height="160"></canvas>
  </div>
</div>

<!-- ============ TAB 5: DATA LOG ============ -->
<div class="page" id="page-log">
  <div class="card">
    <h3>Status SD Card</h3>
    <div class="row"><span class="label">SD Card</span><span class="value" id="txtSdStatus">--</span></div>
    <div class="row"><span class="label">Free Heap</span><span class="value" id="txtHeap">--</span></div>
  </div>

  <div class="card">
    <h3>Data Log (50 baris terakhir)</h3>
    <div class="table-scroll">
      <table>
        <thead><tr><th>Waktu</th><th>Suhu</th><th>SP</th><th>PWM</th><th>Vol</th></tr></thead>
        <tbody id="logTableBody"></tbody>
      </table>
    </div>
    <button class="btn btn-secondary" style="width:100%; margin-top:10px;" onclick="exportCSV()">EXPORT CSV</button>
  </div>
</div>

<div class="tabs">
  <button class="tab-btn active" data-page="dashboard" onclick="showPage('dashboard')"><span class="ic">&#9679;</span>Dashboard</button>
  <button class="tab-btn" data-page="kontrol" onclick="showPage('kontrol')"><span class="ic">&#9654;</span>Kontrol</button>
  <button class="tab-btn" data-page="pid" onclick="showPage('pid')"><span class="ic">&#9881;</span>PID</button>
  <button class="tab-btn" data-page="efficiency" onclick="showPage('efficiency')"><span class="ic">&#9733;</span>Efficiency</button>
  <button class="tab-btn" data-page="log" onclick="showPage('log')"><span class="ic">&#8801;</span>Data Log</button>
</div>

<script>
// ------------------------------------------------------------
// WebSocket connection
// ------------------------------------------------------------
var ws = null;
var wsHost = location.hostname || "192.168.4.1";
var lastState = null;
var effMapCache = { entries: [], recommended_setpoint: 0 };
var historyRows = [];
var logBuffer = [];
var chartData = []; // {t, suhu, setpoint, pwm}
var lastChartPush = 0;
var CHART_INTERVAL_MS = 6000;
var MAX_CHART_POINTS = 100;
var pageLoadTime = Date.now();

function connectWS() {
  ws = new WebSocket("ws://" + wsHost + ":81/");
  ws.onopen = function() {
    document.title = "Pyrolysis Controller (connected)";
    ws.send(JSON.stringify({cmd:"GET_EFFICIENCY_MAP"}));
    ws.send(JSON.stringify({cmd:"GET_HISTORY"}));
  };
  ws.onclose = function() { setTimeout(connectWS, 2000); };
  ws.onerror = function() { ws.close(); };
  ws.onmessage = function(evt) {
    var msg;
    try { msg = JSON.parse(evt.data); } catch(e) { return; }
    if (msg.type === "efficiency_map") { handleEfficiencyMap(msg); return; }
    if (msg.type === "history") { handleHistory(msg); return; }
    if (msg.event === "emergency") { alert("EMERGENCY! Suhu: " + msg.suhu + " C"); return; }
    handleState(msg);
  };
}
connectWS();

function sendCmd(cmd) { if (ws && ws.readyState === 1) ws.send(JSON.stringify({cmd:cmd})); }
function sendCmdVal(cmd, value) { if (ws && ws.readyState === 1) ws.send(JSON.stringify({cmd:cmd, value:value})); }
function sendToggle(cmd, checked) { sendCmdVal(cmd, checked); }
function setNumField(cmd, inputId) {
  var v = parseFloat(document.getElementById(inputId).value);
  if (!isNaN(v)) sendCmdVal(cmd, v);
}

// ------------------------------------------------------------
// Tab navigation
// ------------------------------------------------------------
function showPage(name) {
  document.querySelectorAll(".page").forEach(function(p){ p.classList.remove("active"); });
  document.getElementById("page-" + name).classList.add("active");
  document.querySelectorAll(".tab-btn").forEach(function(b){ b.classList.remove("active"); });
  document.querySelector('.tab-btn[data-page="' + name + '"]').classList.add("active");
  if (name === "efficiency") { sendCmd("GET_EFFICIENCY_MAP"); sendCmd("GET_HISTORY"); }
}

// ------------------------------------------------------------
// Incoming state handling
// ------------------------------------------------------------
function fmtHMS(ms) {
  if (ms == null || ms < 0 || !isFinite(ms)) return "--";
  var s = Math.floor(ms/1000);
  var h = Math.floor(s/3600), m = Math.floor((s%3600)/60), sec = s%60;
  function p(n){ return n<10 ? "0"+n : ""+n; }
  return p(h)+":"+p(m)+":"+p(sec);
}

function handleState(d) {
  lastState = d;

  var badge = document.getElementById("stateBadge");
  badge.textContent = d.state;
  badge.className = "badge " + d.state;

  document.getElementById("txtSuhu").textContent = d.suhu.toFixed(1) + " °C";
  document.getElementById("txtSetpoint").textContent = d.setpoint.toFixed(1) + " °C";
  drawTempGauge(d.suhu);

  document.getElementById("txtFanPct").textContent = d.fan_pct + "%";
  document.getElementById("fanBar").style.width = d.fan_pct + "%";
  document.getElementById("txtEff").textContent = d.efficiency.toFixed(0) + " mL/jam";

  var volPct = d.target_vol > 0 ? Math.min(100, (d.volume_ml / d.target_vol) * 100) : 0;
  document.getElementById("txtVolLabel").textContent = d.volume_ml.toFixed(0) + " / " + d.target_vol.toFixed(0) + " mL";
  document.getElementById("txtVolPct").textContent = volPct.toFixed(0) + "%";
  document.getElementById("volBar").style.width = volPct + "%";

  document.getElementById("txtDuration").textContent = fmtHMS(d.duration_ms);
  document.getElementById("txtEstFinish").textContent = (d.efficiency > 0 && d.est_finish_ms >= 0) ? fmtHMS(d.est_finish_ms) : "--";

  document.getElementById("togFan").checked = !!d.fan_enabled;
  document.getElementById("togOil").checked = !!d.oil_pump;
  document.getElementById("togWater").checked = !!d.water_pump;

  document.getElementById("txtKp").textContent = d.kp.toFixed(3);
  document.getElementById("txtKi").textContent = d.ki.toFixed(4);
  document.getElementById("txtKd").textContent = d.kd.toFixed(3);

  var runsForSp = 0;
  effMapCache.entries.forEach(function(e){ if (Math.abs(e.setpoint - d.setpoint) < 0.01) runsForSp = e.runs; });
  document.getElementById("txtSpInfo").textContent = "Parameter untuk " + d.setpoint.toFixed(0) + "°C (" + runsForSp + " runs)";

  document.getElementById("txtSdStatus").textContent = d.sd_ok ? "OK" : "ERROR";
  document.getElementById("txtHeap").textContent = d.heap_free + " bytes" + (d.heap_free < 10000 ? " (LOW!)" : "");

  if (!document.activeElement || document.activeElement.tagName !== "INPUT") {
    setIfEmpty("inSetpoint", d.setpoint);
    setIfEmpty("inTargetVol", d.target_vol);
    setIfEmpty("inTshutdown", null);
    setIfEmpty("inKp", d.kp);
    setIfEmpty("inKi", d.ki);
    setIfEmpty("inKd", d.kd);
  }

  // Autotune progress
  var at = document.getElementById("atProgress");
  if (d.state === "AUTOTUNE") {
    at.style.display = "block";
    at.textContent = "Cycle " + d.at_cycle + "/4 | " + (d.at_heating ? "HEATING" : "COOLING") +
                      " | Peak: " + d.at_peak.toFixed(1) + "°C | Valley: " + d.at_valley.toFixed(1) + "°C";
  } else {
    at.style.display = "none";
  }

  // Rolling chart buffer (throttled to ~1 point / 6s => 100 points = 10 min)
  var now = Date.now();
  if (now - lastChartPush >= CHART_INTERVAL_MS) {
    chartData.push({t: now, suhu: d.suhu, setpoint: d.setpoint, pwm: d.fan_pct});
    if (chartData.length > MAX_CHART_POINTS) chartData.shift();
    lastChartPush = now;
    drawTempChart();
  }

  // Data log rolling buffer (50 rows)
  logBuffer.push({
    t: now, elapsed: now - pageLoadTime, suhu: d.suhu, setpoint: d.setpoint, pwm: d.fan_pct, pct: d.fan_pct,
    error: d.error, kp: d.kp, ki: d.ki, kd: d.kd, volume: d.volume_ml, efficiency: d.efficiency,
    ise: d.ise, oil: d.oil_pump, water: d.water_pump, state: d.state
  });
  if (logBuffer.length > 50) logBuffer.shift();
  renderLogTable();
}

function setIfEmpty(id, val) {
  if (val == null) return;
  var el = document.getElementById(id);
  if (el.value === "") el.value = val;
}

// ------------------------------------------------------------
// SVG temperature gauge (0-800C semicircular arc)
// ------------------------------------------------------------
function drawTempGauge(temp) {
  var svg = document.getElementById("tempGauge");
  var pct = Math.max(0, Math.min(1, temp / 800));
  var cx = 110, cy = 120, r = 90;
  var startAngle = Math.PI, endAngle = 0; // left to right, top semicircle
  function pt(angle) { return [cx + r*Math.cos(angle), cy - r*Math.sin(angle)]; }
  var bgStart = pt(Math.PI), bgEnd = pt(0);
  var valAngle = Math.PI - pct * Math.PI;
  var valEnd = pt(valAngle);
  var largeArc = pct > 0.5 ? 1 : 0;

  var color = temp > 700 ? "#e94560" : (temp > 500 ? "#ffd700" : "#4ecca3");

  svg.innerHTML =
    '<path d="M ' + bgStart[0] + ' ' + bgStart[1] + ' A ' + r + ' ' + r + ' 0 1 1 ' + bgEnd[0] + ' ' + bgEnd[1] + '" ' +
      'stroke="#2a3358" stroke-width="16" fill="none" stroke-linecap="round"/>' +
    '<path d="M ' + bgStart[0] + ' ' + bgStart[1] + ' A ' + r + ' ' + r + ' 0 ' + largeArc + ' 1 ' + valEnd[0] + ' ' + valEnd[1] + '" ' +
      'stroke="' + color + '" stroke-width="16" fill="none" stroke-linecap="round"/>' +
    '<text x="110" y="105" text-anchor="middle" font-size="26" font-weight="700" fill="' + color + '">' + temp.toFixed(0) + '</text>' +
    '<text x="110" y="125" text-anchor="middle" font-size="12" fill="#8f9bb3">&#176;C</text>';
}

// ------------------------------------------------------------
// Canvas: temperature history chart
// ------------------------------------------------------------
function drawTempChart() {
  var canvas = document.getElementById("tempChart");
  var ctx = canvas.getContext("2d");
  var w = canvas.width, h = canvas.height;
  ctx.clearRect(0,0,w,h);
  if (chartData.length < 2) return;

  var maxT = 800, minT = 0;
  var padL = 30, padR = 10, padT = 10, padB = 20;
  var plotW = w - padL - padR, plotH = h - padT - padB;

  ctx.strokeStyle = "#2a3358";
  ctx.lineWidth = 1;
  for (var i=0;i<=4;i++) {
    var y = padT + (plotH/4)*i;
    ctx.beginPath(); ctx.moveTo(padL,y); ctx.lineTo(w-padR,y); ctx.stroke();
    ctx.fillStyle = "#8f9bb3"; ctx.font = "9px monospace";
    ctx.fillText(((maxT - (maxT-minT)/4*i)).toFixed(0), 2, y+3);
  }

  function drawSeries(key, color) {
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
    chartData.forEach(function(pt, i) {
      var x = padL + (i/(MAX_CHART_POINTS-1)) * plotW;
      var v = Math.max(minT, Math.min(maxT, pt[key]));
      var y = padT + plotH - ((v-minT)/(maxT-minT))*plotH;
      if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    });
    ctx.stroke();
  }
  drawSeries("suhu", "#4ecca3");
  drawSeries("setpoint", "#e94560");
}

// ------------------------------------------------------------
// Efficiency map + history
// ------------------------------------------------------------
function handleEfficiencyMap(msg) {
  effMapCache = { entries: msg.entries || [], recommended_setpoint: msg.recommended_setpoint || 0 };
  renderEffTable();
  drawEffBarChart();
}

function renderEffTable() {
  var body = document.getElementById("effTableBody");
  body.innerHTML = "";
  effMapCache.entries.forEach(function(e) {
    var tr = document.createElement("tr");
    if (Math.abs(e.setpoint - effMapCache.recommended_setpoint) < 0.01) tr.className = "recommended";
    tr.innerHTML = "<td>" + e.setpoint.toFixed(0) + "</td><td>" + e.runs + "</td><td>" +
                   e.avg_efficiency.toFixed(0) + "</td><td>" + e.avg_score.toFixed(0) + "</td>" +
                   "<td><button class='btn btn-secondary' style='padding:4px 8px;font-size:11px;' onclick='useSetpoint(" + e.setpoint + ")'>Gunakan</button></td>";
    body.appendChild(tr);
  });
}

function useSetpoint(sp) {
  sendCmdVal("SET_SETPOINT", sp);
  showPage("kontrol");
}

function drawEffBarChart() {
  var svg = document.getElementById("effBarChart");
  var entries = effMapCache.entries;
  if (!entries.length) { svg.innerHTML = ""; return; }
  var w = 440, h = 180, padB = 24, padT = 10;
  var maxEff = Math.max.apply(null, entries.map(function(e){return e.avg_efficiency;})) || 1;
  var barW = (w - 20) / entries.length;
  var html = "";
  entries.forEach(function(e, i) {
    var barH = (e.avg_efficiency / maxEff) * (h - padB - padT);
    var x = 10 + i*barW;
    var y = h - padB - barH;
    var color = Math.abs(e.setpoint - effMapCache.recommended_setpoint) < 0.01 ? "#4ecca3" : "#33406e";
    html += '<rect x="' + (x+4) + '" y="' + y + '" width="' + (barW-8) + '" height="' + barH + '" fill="' + color + '" rx="3"/>';
    html += '<text x="' + (x+barW/2) + '" y="' + (h-padB+14) + '" text-anchor="middle" font-size="10" fill="#8f9bb3">' + e.setpoint.toFixed(0) + '</text>';
    html += '<text x="' + (x+barW/2) + '" y="' + (y-4) + '" text-anchor="middle" font-size="10" fill="#eaeaea">' + e.avg_efficiency.toFixed(0) + '</text>';
  });
  svg.innerHTML = html;
}

function handleHistory(msg) {
  var lines = (msg.csv || "").split("\n").filter(function(l){ return l.trim().length>0; });
  historyRows = [];
  for (var i=1;i<lines.length;i++) { // skip header
    var cols = lines[i].split(",");
    if (cols.length < 11) continue;
    historyRows.push({
      run_num: parseInt(cols[0]), setpoint: parseFloat(cols[1]), total_vol_ml: parseFloat(cols[2]),
      duration_ms: parseFloat(cols[3]), efficiency_ml_h: parseFloat(cols[4]), score: parseFloat(cols[5]),
      ise: parseFloat(cols[6]), avg_pwm: parseFloat(cols[7])
    });
  }
  drawIseChart();
}

function drawIseChart() {
  var canvas = document.getElementById("iseChart");
  var ctx = canvas.getContext("2d");
  var w = canvas.width, h = canvas.height;
  ctx.clearRect(0,0,w,h);
  if (historyRows.length < 2) return;

  var maxIse = Math.max.apply(null, historyRows.map(function(r){return r.ise;}));
  var minIse = Math.min.apply(null, historyRows.map(function(r){return r.ise;}));
  if (maxIse === minIse) maxIse = minIse + 1;
  var padL = 35, padR = 10, padT = 10, padB = 20;
  var plotW = w - padL - padR, plotH = h - padT - padB;

  ctx.strokeStyle = "#7b61ff"; ctx.lineWidth = 2; ctx.beginPath();
  historyRows.forEach(function(r, i) {
    var x = padL + (i/(historyRows.length-1)) * plotW;
    var y = padT + plotH - ((r.ise-minIse)/(maxIse-minIse))*plotH;
    if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    ctx.fillStyle = "#7b61ff";
    ctx.beginPath(); ctx.arc(x,y,3,0,7); ctx.fill();
  });
  ctx.stroke();

  ctx.fillStyle = "#8f9bb3"; ctx.font = "9px monospace";
  ctx.fillText(maxIse.toFixed(0), 2, padT+8);
  ctx.fillText(minIse.toFixed(0), 2, h-padB);
}

// ------------------------------------------------------------
// Weight sliders (auto-normalize to sum = 1.0)
// ------------------------------------------------------------
function onWeightChange(which) {
  var w1 = parseFloat(document.getElementById("rngW1").value);
  var w2 = parseFloat(document.getElementById("rngW2").value);
  var w3 = parseFloat(document.getElementById("rngW3").value);
  var sum = w1+w2+w3;
  if (sum <= 0) sum = 1;
  w1/=sum; w2/=sum; w3/=sum;
  document.getElementById("rngW1").value = w1;
  document.getElementById("rngW2").value = w2;
  document.getElementById("rngW3").value = w3;
  document.getElementById("lblW1").textContent = w1.toFixed(2);
  document.getElementById("lblW2").textContent = w2.toFixed(2);
  document.getElementById("lblW3").textContent = w3.toFixed(2);
}

function applyWeights() {
  var w1 = parseFloat(document.getElementById("rngW1").value);
  var w2 = parseFloat(document.getElementById("rngW2").value);
  var w3 = parseFloat(document.getElementById("rngW3").value);
  if (ws && ws.readyState === 1) ws.send(JSON.stringify({cmd:"SET_WEIGHTS", w1:w1, w2:w2, w3:w3}));
}

// ------------------------------------------------------------
// Data log table + CSV export
// ------------------------------------------------------------
function renderLogTable() {
  var body = document.getElementById("logTableBody");
  body.innerHTML = "";
  for (var i = logBuffer.length - 1; i >= 0; i--) {
    var r = logBuffer[i];
    var tr = document.createElement("tr");
    tr.innerHTML = "<td>" + fmtHMS(r.elapsed) + "</td><td>" + r.suhu.toFixed(1) + "</td><td>" +
                   r.setpoint.toFixed(1) + "</td><td>" + r.pct + "%</td><td>" + r.volume.toFixed(1) + "</td>";
    body.appendChild(tr);
  }
}

function exportCSV() {
  var header = "timestamp_ms,elapsed_s,suhu_c,setpoint_c,fan_pwm,fan_pct,error,kp,ki,kd,volume_ml,efficiency_ml_h,ise,oil_pump,water_pump,state\n";
  var rows = logBuffer.map(function(r) {
    return [r.t, (r.elapsed/1000).toFixed(1), r.suhu, r.setpoint, r.pwm, r.pct, r.error,
            r.kp, r.ki, r.kd, r.volume, r.efficiency, r.ise, r.oil, r.water, r.state].join(",");
  }).join("\n");
  var blob = new Blob([header + rows], {type: "text/csv"});
  var url = URL.createObjectURL(blob);
  var a = document.createElement("a");
  var now = new Date();
  var fname = "pyrolysis_log_" + now.toISOString().slice(0,19).replace(/[:T]/g,"-") + ".csv";
  a.href = url; a.download = fname;
  document.body.appendChild(a); a.click(); document.body.removeChild(a);
  URL.revokeObjectURL(url);
}
</script>
</body>
</html>

)HTMLDOC";

#endif // WEB_PAGE_H
