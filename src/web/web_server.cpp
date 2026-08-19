#include "web/web_server.h"

#if CCM_WEB_ENABLED

#include <ArduinoJson.h>
#include <WiFi.h>

#include <cstring>

#include "meth/meth_config.h"
#include "web/WebApiLogic.hpp"

namespace web {

namespace {
constexpr const char* kRatioWarning = "Mixture ratio is user selected and not sensor verified.";
constexpr const char* kUiFontStack = "'Arial Black','Segoe UI',Arial,sans-serif";

const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Foxbody Cabin Master</title><style>
*{box-sizing:border-box}
body{font-family:{{UI_FONT_STACK}};background:#000;color:#f3f6fa;margin:0;line-height:1.35}
header{padding:16px 18px;background:#000;border-bottom:2px solid #3a1212}
h1{margin:0;font-size:clamp(28px,5vw,44px);letter-spacing:.5px}
.sub{margin-top:6px;color:#b8c4d8;font-size:clamp(13px,2.2vw,18px)}
main{padding:14px;display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px;max-width:1320px;margin:0 auto}
.card{border:2px solid #5c1717;border-radius:12px;padding:14px;background:#120606;min-width:0}
.card h2{margin:0 0 10px 0;font-size:clamp(22px,3.3vw,30px)}
pre{margin:0;background:#050000;border:1px solid #3a1212;border-radius:8px;padding:10px;font-size:clamp(14px,2.25vw,20px);white-space:pre-wrap;word-break:break-word;overflow-wrap:anywhere}
.warn{font-size:clamp(14px,2.3vw,18px);font-weight:700;color:#ffd27d}
.actions{display:grid;grid-template-columns:repeat(2,minmax(130px,1fr));gap:8px}
button{font:inherit;font-size:clamp(16px,2.6vw,22px);font-weight:800;padding:12px;border-radius:10px;border:2px solid #8e1b1b;background:#4a0b0b;color:#fff4f4}
button:active{transform:scale(.99)}
a.link{color:#ff8a8a;font-size:clamp(16px,2.4vw,22px);text-decoration:none}
@media (max-width:760px){.actions{grid-template-columns:1fr}}
</style></head>
<body>
<header><h1>Foxbody Cabin Master</h1><div class='sub'>Race-safe dashboard • High-contrast • Large block text</div></header>
<main>
<div class='card'><h2>Live Vehicle</h2><pre id='live'>Connecting...</pre></div>
<div class='card'><h2>CAN Status</h2><pre id='canStatus'>Loading CAN status...</pre></div>
<div class='card'><h2>Analog Sensors</h2><pre id='sensorHealth'>Loading sensor diagnostics...</pre></div>
<div class='card'><h2>Water Meth</h2><p class='warn' id='ratioWarn'></p><pre id='methState'>Loading...</pre><div class='actions'><button onclick="methCmd('arm')">ARM</button><button onclick="methCmd('disarm')">DISARM</button><button onclick="setRatio()">Set Ratio</button><button onclick="methCmd('clear_faults')">Clear Faults</button></div></div>
<div class='card'><h2>Knock Monitor</h2><pre id='knockState'>Loading...</pre><div class='actions'><button onclick="knockCmd('toggle')">Enable/Disable</button><button onclick="knockCmd('reset_baseline')">Reset Baseline</button><button onclick="knockCmd('simulate')">Simulate Knock Event</button><button onclick="knockCmd('clear_events')">Clear Events</button></div></div>
<div class='card'><h2>Race Performance</h2><div class='actions'><button onclick="raceCmd('start_accel')">Start Accel</button><button onclick="raceCmd('start_lap')">Start Lap</button><button onclick="raceCmd('stop')">Stop</button><button onclick="raceCmd('reset')">Reset</button></div><pre id='race'>Loading race data...</pre></div>
<div class='card'><h2>Pages</h2><p><a class='link' href='/can'>Open dedicated CAN status page</a></p><p><a class='link' href='/knock'>Open Knock Sense page</a></p><p class='sub'>Dashboard • Race • Settings • LED • Water Meth • Taillights • Diagnostics • CAN Status • Knock Sense</p></div>
</main>
<script>
document.getElementById('ratioWarn').textContent='{{RATIO_WARNING}}';
function pretty(text){try{return JSON.stringify(JSON.parse(text),null,2);}catch(_){return text;}}
const ws=new WebSocket(`ws://${location.host}/ws`);ws.onmessage=e=>{document.getElementById('live').textContent=pretty(e.data);};
async function raceCmd(action){await fetch('/api/race/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});}
async function methCmd(action){if(action==='arm'&&!confirm('ARM meth injection?'))return;const payload=action==='clear_faults'?{clear_faults:true}:{armed:action==='arm'};const r=await fetch('/api/meth',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});if(!r.ok){const e=await r.json();alert('Error: '+(e.error||r.status));}}
async function knockCmd(action){const stateResp=await fetch('/api/knock/state');const st=await stateResp.json();let payload={};if(action==='toggle')payload={enabled:!st.enabled};else if(action==='reset_baseline')payload={reset_baseline:true};else if(action==='simulate')payload={simulate_event:true};else if(action==='clear_events')payload={clear_event_count:true};const r=await fetch('/api/knock',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});if(!r.ok){const e=await r.json();alert('Error: '+(e.error||r.status));}}
async function setRatio(){const v=prompt('Methanol % in tank (0-100):');if(v===null)return;const n=parseInt(v,10);if(isNaN(n)||n<0||n>100){alert('Invalid value');return;}await fetch('/api/meth',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ratio:n})});}
async function updatePanels(){
  try{const race=await fetch('/api/race/state');document.getElementById('race').textContent=pretty(await race.text());}catch(_){}
  try{const can=await fetch('/api/can/status');document.getElementById('canStatus').textContent=pretty(await can.text());}catch(_){}
  try{const meth=await fetch('/api/meth/state');document.getElementById('methState').textContent=pretty(await meth.text());}catch(_){}
  try{const knock=await fetch('/api/knock/state');document.getElementById('knockState').textContent=pretty(await knock.text());}catch(_){}
  try{const sensors=await fetch('/api/diagnostics');document.getElementById('sensorHealth').textContent=pretty(await sensors.text());}catch(_){}
}
setInterval(updatePanels,1000);updatePanels();
</script></body></html>
)HTML";

const char kCanStatusHtml[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Foxbody CAN Status</title><style>
*{box-sizing:border-box}body{font-family:{{UI_FONT_STACK}};background:#000;color:#f3f6fa;margin:0}
header{padding:16px 18px;background:#000;border-bottom:2px solid #3a1212}
h1{margin:0;font-size:clamp(28px,5vw,44px)}main{padding:14px;max-width:1200px;margin:0 auto}
.card{border:2px solid #5c1717;border-radius:12px;padding:14px;background:#120606}
pre{margin:0;background:#050000;border:1px solid #3a1212;border-radius:8px;padding:10px;font-size:clamp(14px,2.4vw,22px);white-space:pre-wrap;word-break:break-word;overflow-wrap:anywhere}
a{display:inline-block;margin-bottom:10px;color:#ff8a8a;font-size:clamp(16px,2.2vw,22px)}
</style></head>
<body><header><h1>CAN Status</h1></header><main><a href='/'>← Back to Dashboard</a><div class='card'><pre id='canStatus'>Loading CAN status...</pre></div></main>
<script>
function pretty(text){try{return JSON.stringify(JSON.parse(text),null,2);}catch(_){return text;}}
async function tick(){try{const r=await fetch('/api/can/status');document.getElementById('canStatus').textContent=pretty(await r.text());}catch(_){}}
setInterval(tick,600);tick();
</script></body></html>
)HTML";

const char kKnockHtml[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Knock Sense</title><style>
*{box-sizing:border-box}
body{font-family:{{UI_FONT_STACK}};background:#000;color:#f3f6fa;margin:0;line-height:1.35}
header{padding:16px 18px;background:#000;border-bottom:2px solid #3a1212;display:flex;align-items:center;gap:16px}
h1{margin:0;font-size:clamp(24px,4vw,40px);letter-spacing:.5px}
.badge{padding:4px 14px;border-radius:20px;font-size:clamp(13px,1.8vw,17px);font-weight:700}
.badge-ok{background:#1a3a1a;color:#00e676;border:1px solid #00e676}
.badge-warn{background:#3a2800;color:#ffb300;border:1px solid #ffb300}
.badge-crit{background:#3a0000;color:#ff5252;border:1px solid #ff5252}
.badge-off{background:#1a1a2a;color:#667788;border:1px solid #445566}
main{padding:14px;display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:12px;max-width:1400px;margin:0 auto}
.card{border:2px solid #5c1717;border-radius:12px;padding:14px;background:#120606;min-width:0}
.card h2{margin:0 0 10px;font-size:clamp(18px,2.5vw,26px)}
.metric-row{display:flex;align-items:center;gap:10px;margin:6px 0}
.metric-name{min-width:110px;color:#8fa8c0;font-size:clamp(13px,1.8vw,17px)}
.metric-val{font-size:clamp(15px,2.2vw,21px);font-weight:700}
.bar-wrap{flex:1;background:#050000;border-radius:4px;height:16px;overflow:hidden;border:1px solid #3a1212}
.bar-fill{height:100%;border-radius:4px;transition:width .3s,background .3s}
.note{font-size:clamp(12px,1.6vw,15px);color:#7a8a9a;margin-top:6px}
.actions{display:grid;grid-template-columns:repeat(2,minmax(120px,1fr));gap:8px;margin-top:10px}
button{font:inherit;font-size:clamp(14px,2vw,19px);font-weight:700;padding:10px;border-radius:9px;border:2px solid #8e1b1b;background:#4a0b0b;color:#fff4f4;cursor:pointer}
button:active{transform:scale(.98)}
button.danger{border-color:#882222;background:#3a1010}
button.sim{border-color:#7a5a00;background:#2a1e00;color:#ffd060}
a.back{color:#ff8a8a;font-size:clamp(14px,2vw,18px);text-decoration:none}
canvas{display:block;width:100%;height:140px;background:#0a0e14;border-radius:8px;border:1px solid #2a3a50}
.setting-row{display:flex;align-items:center;gap:8px;margin:5px 0}
.setting-row label{min-width:170px;font-size:clamp(12px,1.7vw,16px);color:#8fa8c0}
.setting-row input,.setting-row select{background:#141c28;border:1px solid #33445f;color:#f3f6fa;border-radius:6px;padding:5px 8px;font-size:clamp(13px,1.8vw,17px);width:100px}
</style></head>
<body>
<header>
  <h1>&#9201; Knock Sense</h1>
  <span id='hdrBadge' class='badge badge-off'>OFFLINE</span>
  <span style='flex:1'></span>
  <a class='back' href='/'>&#8592; Dashboard</a>
</header>
<main>
<div class='card'><h2>Live Levels</h2>
  <div class='metric-row'><span class='metric-name'>Knock Energy</span><span class='metric-val' id='mEnergy'>--</span><div class='bar-wrap'><div class='bar-fill' id='bEnergy' style='width:0%;background:#2255aa'></div></div></div>
  <div class='metric-row'><span class='metric-name'>Baseline</span><span class='metric-val' id='mBaseline'>--</span><div class='bar-wrap'><div class='bar-fill' id='bBaseline' style='width:0%;background:#22aa55'></div></div></div>
  <div class='metric-row'><span class='metric-name'>Threshold</span><span class='metric-val' id='mThreshold'>--</span><div class='bar-wrap'><div class='bar-fill' id='bThreshold' style='width:100%;background:#aa7722'></div></div></div>
  <p class='note'>Bars show % of current threshold level.</p>
</div>
<div class='card'><h2>Sensor Health</h2>
  <div class='metric-row'><span class='metric-name'>Status</span><span class='metric-val' id='mSensor'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Enabled</span><span class='metric-val' id='mEnabled'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Signal Valid</span><span class='metric-val' id='mValid'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Baseline Learned</span><span class='metric-val' id='mLearned'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Sensor Fault</span><span class='metric-val' id='mFault'>--</span></div>
  <div class='metric-row'><span class='metric-name'>ADC Clipping</span><span class='metric-val' id='mClip'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Clip High</span><span class='metric-val' id='mClipHi'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Clip Low</span><span class='metric-val' id='mClipLo'>--</span></div>
  <div class='metric-row'><span class='metric-name'>SD Logging</span><span class='metric-val' id='mLog'>--</span></div>
  <p class='note'>† Not ECU knock control — supplemental warning and logging only.</p>
</div>
<div class='card'><h2>Events</h2>
  <div class='metric-row'><span class='metric-name'>Event Count</span><span class='metric-val' id='mCount'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Warning Active</span><span class='metric-val' id='mWarn'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Critical Active</span><span class='metric-val' id='mCrit'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Response Mode</span><span class='metric-val' id='mResp'>--</span></div>
  <br><b style='font-size:clamp(14px,2vw,18px)'>Last Event</b>
  <div class='metric-row'><span class='metric-name'>RPM</span><span class='metric-val' id='mLRpm'>--</span></div>
  <div class='metric-row'><span class='metric-name'>Boost kPa</span><span class='metric-val' id='mLBoost'>--</span></div>
  <div class='metric-row'><span class='metric-name'>IAT °C</span><span class='metric-val' id='mLIat'>--</span></div>
</div>
<div class='card' style='grid-column:span 2'><h2>History Chart (last 30 s)</h2>
  <canvas id='chart'></canvas>
  <p class='note'>Blue = energy &nbsp; Green = baseline &nbsp; Orange = threshold</p>
</div>
<div class='card'><h2>Controls</h2>
  <div class='actions'>
    <button onclick="knockAction('toggle')">Enable / Disable</button>
    <button onclick="knockAction('reset_baseline')">Reset Baseline</button>
    <button onclick="knockAction('clear_events')">Clear Events</button>
    <button class='sim' onclick="knockAction('simulate')">Simulate Event</button>
  </div>
  <p class='note'>Simulate only works in demo / dev builds.</p>
</div>
<div class='card'><h2>Settings</h2>
  <div class='setting-row'><label>Signal Gain</label><input id='sGain' type='number' step='0.25' min='0.25' max='8'></div>
  <div class='setting-row'><label>Threshold ×</label><input id='sMult' type='number' step='0.1' min='1' max='10'></div>
  <div class='setting-row'><label>Boost Enable kPa</label><input id='sBoost' type='number' step='1'></div>
  <div class='setting-row'><label>RPM Enable</label><input id='sRpm' type='number' step='100'></div>
  <div class='setting-row'><label>Response Mode</label>
    <select id='sResp'>
      <option value='0'>Log Only</option>
      <option value='1'>Warn Only (default)</option>
      <option value='2'>Meth Enable</option>
      <option value='3'>Safety Shutdown</option>
    </select></div>
  <div class='actions' style='margin-top:8px'>
    <button onclick='saveSettings()'>Save Settings</button>
  </div>
  <p class='note' style='color:#ffb300'>&#9888; Meth Enable and Safety Shutdown modes alter active systems. Use with caution.</p>
</div>
</main>
<script>
const kRespNames=['Log Only','Warn Only','Meth Enable','Safety Shutdown'];
let chartData={energy:[],baseline:[],threshold:[]};
const MAX_CHART_POINTS=60;
async function knockAction(action){
  const stateResp=await fetch('/api/knock/state');
  const st=await stateResp.json();
  let payload={};
  if(action==='toggle')payload={enabled:!st.enabled};
  else if(action==='reset_baseline')payload={reset_baseline:true};
  else if(action==='simulate')payload={simulate_event:true};
  else if(action==='clear_events')payload={clear_event_count:true};
  const r=await fetch('/api/knock',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
  if(!r.ok){const e=await r.json();alert('Error: '+(e.error||r.status));}
}
async function saveSettings(){
  const multEl=document.getElementById('sMult');
  const gainEl=document.getElementById('sGain');
  const boostEl=document.getElementById('sBoost');
  const rpmEl=document.getElementById('sRpm');
  const respEl=document.getElementById('sResp');
  const mult=parseFloat(multEl.value);
  const gain=parseFloat(gainEl.value);
  const boost=parseFloat(boostEl.value);
  const rpm=parseInt(rpmEl.value);
  const resp=parseInt(respEl.value);
  if(isNaN(mult)||mult<1){alert('Threshold multiplier must be a number >= 1');return;}
  if(isNaN(gain)||gain<0.25||gain>8){alert('Signal gain must be between 0.25 and 8');return;}
  if(isNaN(boost)||boost<0){alert('Boost enable kPa must be a valid number');return;}
  if(isNaN(rpm)||rpm<0){alert('RPM enable must be a valid number');return;}
  if(isNaN(resp)){alert('Response mode is required');return;}
  const payload={
    knock_gain:gain,
    knock_threshold_multiplier:mult,
    knock_boost_enable_kpa:boost,
    knock_rpm_enable_min:rpm,
    knock_response_mode:resp
  };
  if(resp>=2&&!confirm('Response mode '+kRespNames[resp]+' will affect active systems. Continue?'))return;
  const r=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
  if(!r.ok)alert('Settings save failed');
}
function setV(id,v,unit){const el=document.getElementById(id);if(el)el.textContent=(v!==undefined?v:'--')+(unit||'');}
function setBar(id,pct){const el=document.getElementById(id);if(!el)return;el.style.width=Math.min(100,Math.max(0,pct))+'%';if(pct>=90)el.style.background='#cc2222';else if(pct>=60)el.style.background='#cc7700';else el.style.background=(id==='bEnergy'?'#2255aa':id==='bBaseline'?'#22aa55':'#aa7722');}
async function loadSettings(){
  try{const r=await fetch('/api/settings');const d=await r.json();
  document.getElementById('sGain').value=d.knock_gain||1.0;
  document.getElementById('sMult').value=d.knock_threshold_multiplier||2.5;
  document.getElementById('sBoost').value=d.knock_boost_enable_kpa||120;
  document.getElementById('sRpm').value=d.knock_rpm_enable_min||2500;
  document.getElementById('sResp').value=d.knock_response_mode||1;
  }catch(_){}
}
function drawChart(){
  const canvas=document.getElementById('chart');
  if(!canvas)return;
  const w=canvas.offsetWidth||600;canvas.width=w;canvas.height=140;
  const ctx=canvas.getContext('2d');
  ctx.clearRect(0,0,w,140);
  const n=chartData.energy.length;if(n<2)return;
  const all=chartData.energy.concat(chartData.baseline,chartData.threshold);
  const maxV=Math.max(...all,1)*1.2;
  function drawLine(arr,color){
    ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;
    arr.forEach((v,i)=>{
      const x=(i/(n-1))*(w-4)+2;const y=130-(v/maxV)*120;
      if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
    });ctx.stroke();
  }
  ctx.strokeStyle='#1a2540';ctx.lineWidth=1;
  for(let g=0;g<=4;g++){const y=130-(g/4)*120;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}
  drawLine(chartData.baseline,'#22aa55');
  drawLine(chartData.threshold,'#aa7722');
  drawLine(chartData.energy,'#5588dd');
}
async function tick(){
  try{
    const r=await fetch('/api/knock/state');const d=await r.json();
    const thresh=d.knock_threshold||1;
    const ePct=(d.knock_energy/thresh*100).toFixed(0);
    const bPct=(d.knock_baseline/thresh*100).toFixed(0);
    setV('mEnergy',(d.knock_energy||0).toFixed(1),' raw ('+ePct+'%)');
    setV('mBaseline',(d.knock_baseline||0).toFixed(1),' raw ('+bPct+'%)');
    setV('mThreshold',(d.knock_threshold||0).toFixed(1),' raw');
    setBar('bEnergy',parseFloat(ePct));setBar('bBaseline',parseFloat(bPct));
    setV('mSensor',d.knock_sensor_fault?'FAULT':d.knock_clipping_detected?'CLIPPING':!d.knock_baseline_learned?'LEARNING':!d.knock_signal_valid?'NOISY':'OK');
    setV('mEnabled',d.enabled?'YES':'NO');
    setV('mValid',d.knock_signal_valid?'YES':'NO');
    setV('mLearned',d.knock_baseline_learned?'YES':'NO');
    setV('mFault',d.knock_sensor_fault?'YES':'NO');
    setV('mClip',d.knock_clipping_detected?'YES':'NO');
    setV('mClipHi',d.knock_signal_clip_high_count);
    setV('mClipLo',d.knock_signal_clip_low_count);
    setV('mLog',d.knock_logging_active?'ACTIVE':'--');
    setV('mCount',d.knock_event_count);
    setV('mWarn',d.knock_warning_active?'⚠ YES':'NO');
    setV('mCrit',d.knock_critical_active?'🔴 YES':'NO');
    setV('mResp',kRespNames[d.knock_response_mode]||d.knock_response_mode);
    setV('mLRpm',d.knock_last_event_rpm,' RPM');
    setV('mLBoost',d.knock_last_event_boost_kpa,' kPa');
    setV('mLIat',d.knock_last_event_iat_c,' °C');
    const badge=document.getElementById('hdrBadge');
    if(d.knock_critical_active){badge.className='badge badge-crit';badge.textContent='CRITICAL';}
    else if(d.knock_warning_active){badge.className='badge badge-warn';badge.textContent='WARNING';}
    else if(d.knock_enabled||d.enabled){badge.className='badge badge-ok';badge.textContent='ONLINE';}
    else{badge.className='badge badge-off';badge.textContent='DISABLED';}
    chartData.energy.push(d.knock_energy||0);
    chartData.baseline.push(d.knock_baseline||0);
    chartData.threshold.push(d.knock_threshold||0);
    if(chartData.energy.length>MAX_CHART_POINTS){chartData.energy.shift();chartData.baseline.shift();chartData.threshold.shift();}
    drawChart();
  }catch(_){}
}
loadSettings();setInterval(tick,500);tick();
</script></body></html>
)HTML";

String toHexColor(uint32_t c) {
  char buf[10];
  snprintf(buf, sizeof(buf), "#%06lX", static_cast<unsigned long>(c & 0xFFFFFF));
  return String(buf);
}

uint32_t parseColor(const String& s) {
  if (s.length() == 7 && s[0] == '#') {
    return static_cast<uint32_t>(strtoul(s.substring(1).c_str(), nullptr, 16));
  }
  return static_cast<uint32_t>(strtoul(s.c_str(), nullptr, 0));
}

bool constantTimeEquals(const char* a, const String& b) {
  if (!a) return false;
  const size_t alen = strlen(a);
  const size_t blen = b.length();
  if (alen != blen) return false;
  uint8_t diff = 0;
  for (size_t i = 0; i < alen; ++i) {
    diff |= static_cast<uint8_t>(a[i]) ^ static_cast<uint8_t>(b[i]);
  }
  return diff == 0;
}

uint8_t sanitizeMethRatioValue(uint8_t ratio) {
  return meth::sanitizeRatio(ratio);
}

}  // namespace

bool WebServerManager::begin(state::VehicleStateStore* stateStore, settings::SettingsManager* settingsMgr, canbus::CanManager* canMgr,
                             race::RacePerformanceManager* raceMgr) {
  stateStore_ = stateStore;
  settingsMgr_ = settingsMgr;
  canMgr_ = canMgr;
  raceMgr_ = raceMgr;
  if (!stateStore_ || !settingsMgr_ || !canMgr_ || !raceMgr_) return false;

  ws_.onEvent([this](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type, void*, uint8_t*, size_t) {
    if (type == WS_EVT_CONNECT || type == WS_EVT_DISCONNECT) {
      stateStore_->mutate([this](state::VehicleState& s) { s.web_connected_clients = ws_.count(); });
    }
  });
  server_.addHandler(&ws_);

  server_.on("/", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    String html(kIndexHtml);
    html.replace("{{RATIO_WARNING}}", kRatioWarning);
    html.replace("{{UI_FONT_STACK}}", kUiFontStack);
    req->send(200, "text/html", html);
  });
  server_.on("/can", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    String html(kCanStatusHtml);
    html.replace("{{UI_FONT_STACK}}", kUiFontStack);
    req->send(200, "text/html", html);
  });

  server_.on("/api/meth/state", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    const state::VehicleState s = stateStore_->read();
    JsonDocument doc;
    doc["online"] = s.meth_online;
    doc["armed"] = s.meth_desired_armed;
    doc["ratio_percent"] = s.meth_selected_ratio_percent;
    doc["meth_state"] = static_cast<uint8_t>(s.meth_state);
    doc["pump_duty"] = s.meth_pump_duty;
    doc["tank_level"] = s.meth_tank_level;
    doc["flow_status"] = s.meth_flow_status;
    doc["boost_kpa"] = s.boost_kpa;
    doc["boost_psi"] = s.boost_psi;
    doc["intake_temp_c"] = s.intake_temp;
    doc["engine_bay_temp_c"] = s.engine_bay_temp;
    doc["meth_pressure_psi"] = s.meth_pressure_psi;
    doc["fuel_pressure_psi"] = s.fuel_pressure_psi;
    doc["oil_pressure_psi"] = s.oil_pressure_psi;
    doc["iat_valid"] = s.intake_temp_valid;
    doc["meth_pressure_valid"] = s.meth_pressure_valid;
    String body;
    serializeJson(doc, body);
    req->send(200, "application/json", body);
  });
  server_.on("/api/knock/state", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    const state::VehicleState s = stateStore_->read();
    JsonDocument doc;
    doc["knock_enabled"] = s.knock_enabled;
    doc["knock_signal_valid"] = s.knock_signal_valid;
    doc["knock_energy"] = s.knock_energy;
    doc["knock_baseline"] = s.knock_baseline;
    doc["knock_threshold"] = s.knock_threshold;
    doc["knock_warning_active"] = s.knock_warning_active;
    doc["knock_critical_active"] = s.knock_critical_active;
    doc["knock_event_count"] = s.knock_event_count;
    doc["knock_last_event_rpm"] = s.knock_last_event_rpm;
    doc["knock_last_event_boost_kpa"] = s.knock_last_event_boost_kpa;
    doc["knock_last_event_iat_c"] = s.knock_last_event_iat_c;
    doc["knock_sensor_fault"] = s.knock_sensor_fault;
    doc["knock_clipping_detected"] = s.knock_clipping_detected;
    doc["knock_baseline_learned"] = s.knock_baseline_learned;
    doc["knock_gain"] = s.knock_gain;
    doc["knock_response_mode"] = s.knock_response_mode;
    doc["knock_logging_active"] = s.knock_logging_active;
    doc["knock_online"] = s.knock_online;
    doc["knock_signal_clip_high_count"] = s.knock_signal_clip_high_count;
    doc["knock_signal_clip_low_count"] = s.knock_signal_clip_low_count;
    // Legacy / shorter aliases kept for backward compatibility
    doc["enabled"] = s.knock_enabled;
    doc["energy"] = s.knock_energy;
    doc["baseline"] = s.knock_baseline;
    doc["threshold"] = s.knock_threshold;
    doc["event_count"] = s.knock_event_count;
    doc["last_event_rpm"] = s.knock_last_event_rpm;
    doc["last_event_boost_kpa"] = s.knock_last_event_boost_kpa;
    doc["signal_valid"] = s.knock_signal_valid;
    doc["warning_active"] = s.knock_warning_active;
    doc["critical_active"] = s.knock_critical_active;
    doc["baseline_learned"] = s.knock_baseline_learned;
    doc["sensor_fault"] = s.knock_sensor_fault;
    doc["clipping_detected"] = s.knock_clipping_detected;
    doc["response_mode"] = s.knock_response_mode;
    String body;
    serializeJson(doc, body);
    req->send(200, "application/json", body);
  });
  server_.on("/api/state", HTTP_GET, [this](AsyncWebServerRequest* req) { sendState(req); });
  server_.on("/api/settings", HTTP_GET, [this](AsyncWebServerRequest* req) { sendSettings(req); });
  server_.on("/api/diagnostics", HTTP_GET, [this](AsyncWebServerRequest* req) { sendDiagnostics(req); });
  server_.on("/api/can/status", HTTP_GET, [this](AsyncWebServerRequest* req) { sendCanStatus(req); });
  server_.on("/api/race/state", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    req->send(200, "application/json", stateJson());
  });
  server_.on("/api/race/history", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    req->send(200, "application/json", raceMgr_->historyJson());
  });
  server_.on("/api/race/records", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    req->send(200, "application/json", raceMgr_->recordsJson());
  });
  server_.on("/api/race/export", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    const uint16_t written = raceMgr_->exportHistoryToLog();
    JsonDocument out;
    out["ok"] = true;
    out["exported_entries"] = written;
    out["target"] = "/logs/race";
    String body;
    serializeJson(out, body);
    req->send(200, "application/json", body);
  });

  server_.on("/knock", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    String html(kKnockHtml);
    html.replace("{{UI_FONT_STACK}}", kUiFontStack);
    req->send(200, "text/html", html);
  });

  server_.on("/api/settings", HTTP_POST,
    [this](AsyncWebServerRequest* req) {
      if (!checkAuth(req)) return;
      if (!req->_tempObject) { req->send(400); return; }
      JsonDocument doc;
      deserializeJson(doc, (char*)req->_tempObject);
      free(req->_tempObject); req->_tempObject = nullptr;
      JsonObject obj = doc.as<JsonObject>();
      stateStore_->mutate([&](state::VehicleState& s) {
        if (obj.containsKey("display_brightness")) s.display_brightness = obj["display_brightness"].as<uint8_t>();
        if (obj.containsKey("night_mode")) s.night_mode_enabled = obj["night_mode"].as<bool>();
        if (obj.containsKey("tach_pulses_per_rev10")) s.pulses_per_rev10 = obj["tach_pulses_per_rev10"].as<uint8_t>();
        if (obj.containsKey("tach_scaling_mode")) s.tach_scaling_mode = obj["tach_scaling_mode"].as<uint8_t>();
        if (obj.containsKey("led_global_brightness")) s.led_global_brightness = obj["led_global_brightness"].as<uint8_t>();
        if (obj.containsKey("led_theme")) s.led_theme = obj["led_theme"].as<uint8_t>();
        if (obj.containsKey("meth_ratio")) s.meth_selected_ratio_percent = sanitizeMethRatioValue(obj["meth_ratio"].as<uint8_t>());
        if (obj.containsKey("knock_enabled")) s.knock_enabled = obj["knock_enabled"].as<bool>();
        if (obj.containsKey("knock_adc_pin")) s.knock_adc_pin = obj["knock_adc_pin"].as<uint8_t>();
        if (obj.containsKey("knock_boost_enable_kpa")) s.knock_boost_enable_kpa = obj["knock_boost_enable_kpa"].as<float>();
        if (obj.containsKey("knock_rpm_enable_min")) s.knock_rpm_enable_min = obj["knock_rpm_enable_min"].as<uint16_t>();
        if (obj.containsKey("knock_gain")) s.knock_gain = obj["knock_gain"].as<float>();
        if (obj.containsKey("knock_threshold_multiplier")) s.knock_threshold_multiplier = obj["knock_threshold_multiplier"].as<float>();
        if (obj.containsKey("knock_threshold_offset")) s.knock_threshold_offset = obj["knock_threshold_offset"].as<float>();
        if (obj.containsKey("knock_event_cooldown_ms")) s.knock_event_cooldown_ms = obj["knock_event_cooldown_ms"].as<uint16_t>();
        if (obj.containsKey("knock_warning_threshold_count")) s.knock_warning_threshold_count = obj["knock_warning_threshold_count"].as<uint8_t>();
        if (obj.containsKey("knock_critical_threshold_count")) s.knock_critical_threshold_count = obj["knock_critical_threshold_count"].as<uint8_t>();
        if (obj.containsKey("knock_baseline_learning_enabled")) s.knock_baseline_learning_enabled = obj["knock_baseline_learning_enabled"].as<bool>();
        if (obj.containsKey("knock_demo_mode_enabled")) s.knock_demo_mode_enabled = obj["knock_demo_mode_enabled"].as<bool>();
        if (obj.containsKey("knock_response_mode")) s.knock_response_mode = obj["knock_response_mode"].as<uint8_t>();
        if (obj.containsKey("race_use_metric_targets")) s.race_use_metric_targets = obj["race_use_metric_targets"].as<bool>();
        if (obj.containsKey("race_auto_start")) s.race_auto_start = obj["race_auto_start"].as<bool>();
        if (obj.containsKey("race_min_satellites")) s.race_min_satellites = obj["race_min_satellites"].as<uint8_t>();
        if (obj.containsKey("race_sample_min_ms")) s.race_sample_min_ms = obj["race_sample_min_ms"].as<uint16_t>();
        if (obj.containsKey("race_sample_max_ms")) s.race_sample_max_ms = obj["race_sample_max_ms"].as<uint16_t>();
        if (obj.containsKey("race_start_finish_radius_m")) s.race_start_finish_radius_m = obj["race_start_finish_radius_m"].as<float>();
        if (obj.containsKey("analog_sensors_enabled")) s.analog_sensors_enabled = obj["analog_sensors_enabled"].as<bool>();
        if (obj.containsKey("analog_sensor_sample_ms")) s.analog_sensor_sample_ms = obj["analog_sensor_sample_ms"].as<uint16_t>();
        if (obj.containsKey("thermistor_pullup_ohms")) s.thermistor_pullup_ohms = obj["thermistor_pullup_ohms"].as<float>();
        if (obj.containsKey("iat_adc_pin")) s.iat_adc_pin = obj["iat_adc_pin"].as<uint8_t>();
        if (obj.containsKey("engine_bay_adc_pin")) s.engine_bay_adc_pin = obj["engine_bay_adc_pin"].as<uint8_t>();
        if (obj.containsKey("cabin_temp_adc_pin")) s.cabin_temp_adc_pin = obj["cabin_temp_adc_pin"].as<uint8_t>();
        if (obj.containsKey("ambient_temp_adc_pin")) s.ambient_temp_adc_pin = obj["ambient_temp_adc_pin"].as<uint8_t>();
        if (obj.containsKey("oil_pressure_adc_pin")) s.oil_pressure_adc_pin = obj["oil_pressure_adc_pin"].as<uint8_t>();
        if (obj.containsKey("fuel_pressure_adc_pin")) s.fuel_pressure_adc_pin = obj["fuel_pressure_adc_pin"].as<uint8_t>();
        if (obj.containsKey("meth_pressure_adc_pin")) s.meth_pressure_adc_pin = obj["meth_pressure_adc_pin"].as<uint8_t>();
        if (obj.containsKey("boost_ref_pressure_adc_pin")) s.boost_ref_pressure_adc_pin = obj["boost_ref_pressure_adc_pin"].as<uint8_t>();
        if (obj.containsKey("spare_pressure_1_adc_pin")) s.spare_pressure_1_adc_pin = obj["spare_pressure_1_adc_pin"].as<uint8_t>();
        if (obj.containsKey("spare_pressure_2_adc_pin")) s.spare_pressure_2_adc_pin = obj["spare_pressure_2_adc_pin"].as<uint8_t>();
        if (obj.containsKey("iat_sensor_enabled")) s.iat_sensor_enabled = obj["iat_sensor_enabled"].as<bool>();
        if (obj.containsKey("engine_bay_sensor_enabled")) s.engine_bay_sensor_enabled = obj["engine_bay_sensor_enabled"].as<bool>();
        if (obj.containsKey("cabin_temp_sensor_enabled")) s.cabin_temp_sensor_enabled = obj["cabin_temp_sensor_enabled"].as<bool>();
        if (obj.containsKey("ambient_temp_sensor_enabled")) s.ambient_temp_sensor_enabled = obj["ambient_temp_sensor_enabled"].as<bool>();
        if (obj.containsKey("oil_pressure_sensor_enabled")) s.oil_pressure_sensor_enabled = obj["oil_pressure_sensor_enabled"].as<bool>();
        if (obj.containsKey("fuel_pressure_sensor_enabled")) s.fuel_pressure_sensor_enabled = obj["fuel_pressure_sensor_enabled"].as<bool>();
        if (obj.containsKey("meth_pressure_sensor_enabled")) s.meth_pressure_sensor_enabled = obj["meth_pressure_sensor_enabled"].as<bool>();
        if (obj.containsKey("boost_ref_pressure_sensor_enabled")) s.boost_ref_pressure_sensor_enabled = obj["boost_ref_pressure_sensor_enabled"].as<bool>();
        if (obj.containsKey("spare_pressure_1_sensor_enabled")) s.spare_pressure_1_sensor_enabled = obj["spare_pressure_1_sensor_enabled"].as<bool>();
        if (obj.containsKey("spare_pressure_2_sensor_enabled")) s.spare_pressure_2_sensor_enabled = obj["spare_pressure_2_sensor_enabled"].as<bool>();
        if (obj.containsKey("pressure_sensor_min_v")) s.pressure_sensor_min_v = obj["pressure_sensor_min_v"].as<float>();
        if (obj.containsKey("pressure_sensor_max_v")) s.pressure_sensor_max_v = obj["pressure_sensor_max_v"].as<float>();
        if (obj.containsKey("pressure_sensor_max_psi")) s.pressure_sensor_max_psi = obj["pressure_sensor_max_psi"].as<float>();
      });
      state::VehicleState snapshot = stateStore_->read();
      settingsMgr_->updateFromState(snapshot);
      settingsMgr_->save();
      req->send(200, "application/json", "{\"ok\":true}");
    },
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) { req->_tempObject = malloc(total + 1); }
      if (req->_tempObject) {
        memcpy((uint8_t*)req->_tempObject + index, data, len);
        if (index + len == total) ((uint8_t*)req->_tempObject)[total] = '\0';
      }
    });

  server_.on("/api/knock", HTTP_POST,
    [this](AsyncWebServerRequest* req) {
      if (!checkAuth(req)) return;
      if (!req->_tempObject) { req->send(400); return; }
      JsonDocument doc;
      deserializeJson(doc, (char*)req->_tempObject);
      free(req->_tempObject); req->_tempObject = nullptr;
      JsonObject obj = doc.as<JsonObject>();

      // Validate the named action before entering the state mutation.
      static const char* const kValidActions[] = {
        "enable", "disable", "reset_baseline", "clear_events", "simulate_event"
      };
      if (obj.containsKey("action")) {
        const String action = obj["action"].as<String>();
        bool found = false;
        for (const char* v : kValidActions) { if (action == v) { found = true; break; } }
        if (!found) {
          req->send(400, "application/json", "{\"error\":\"unrecognized action\"}");
          return;
        }
      }

      stateStore_->mutate([&](state::VehicleState& s) {
        // Named action field (new canonical API)
        if (obj.containsKey("action")) {
          const String action = obj["action"].as<String>();
          if (action == "enable")          { s.knock_enabled = true; }
          else if (action == "disable")    { s.knock_enabled = false; }
          else if (action == "reset_baseline") { s.knock_reset_baseline_request = true; }
          else if (action == "clear_events")   { s.knock_clear_event_count_request = true; }
          else if (action == "simulate_event") {
            if (web::knockSimulationAllowed(
#if defined(DEMO_MODE) && (DEMO_MODE == 1)
                    true,
#else
                    false,
#endif
                    s.knock_demo_mode_enabled)) s.knock_simulate_event_request = true;
          }
        }
        // Existing key-value fields (backward compatible)
        if (obj.containsKey("enabled")) s.knock_enabled = obj["enabled"].as<bool>();
        if (obj.containsKey("adc_pin")) s.knock_adc_pin = obj["adc_pin"].as<uint8_t>();
        if (obj.containsKey("boost_enable_kpa")) s.knock_boost_enable_kpa = obj["boost_enable_kpa"].as<float>();
        if (obj.containsKey("rpm_enable_min")) s.knock_rpm_enable_min = obj["rpm_enable_min"].as<uint16_t>();
        if (obj.containsKey("gain")) s.knock_gain = obj["gain"].as<float>();
        if (obj.containsKey("threshold_multiplier")) s.knock_threshold_multiplier = obj["threshold_multiplier"].as<float>();
        if (obj.containsKey("threshold_offset")) s.knock_threshold_offset = obj["threshold_offset"].as<float>();
        if (obj.containsKey("event_cooldown_ms")) s.knock_event_cooldown_ms = obj["event_cooldown_ms"].as<uint16_t>();
        if (obj.containsKey("warning_threshold_count")) s.knock_warning_threshold_count = obj["warning_threshold_count"].as<uint8_t>();
        if (obj.containsKey("critical_threshold_count")) s.knock_critical_threshold_count = obj["critical_threshold_count"].as<uint8_t>();
        if (obj.containsKey("baseline_learning_enabled")) s.knock_baseline_learning_enabled = obj["baseline_learning_enabled"].as<bool>();
        if (obj.containsKey("demo_mode_enabled")) s.knock_demo_mode_enabled = obj["demo_mode_enabled"].as<bool>();
        if (obj.containsKey("response_mode")) s.knock_response_mode = obj["response_mode"].as<uint8_t>();
        // Legacy boolean command keys for backward compatibility
        if ((obj["reset_baseline"] | false) == true) s.knock_reset_baseline_request = true;
        if ((obj["clear_event_count"] | false) == true) s.knock_clear_event_count_request = true;
        if ((obj["simulate_event"] | false) == true) {
          if (web::knockSimulationAllowed(
#if defined(DEMO_MODE) && (DEMO_MODE == 1)
                  true,
#else
                  false,
#endif
                  s.knock_demo_mode_enabled)) s.knock_simulate_event_request = true;
        }
      });
      settingsMgr_->updateFromState(stateStore_->read());
      settingsMgr_->save();
      req->send(200, "application/json", "{\"ok\":true}");
    },
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) { req->_tempObject = malloc(total + 1); }
      if (req->_tempObject) {
        memcpy((uint8_t*)req->_tempObject + index, data, len);
        if (index + len == total) ((uint8_t*)req->_tempObject)[total] = '\0';
      }
    });

  server_.on("/api/led", HTTP_POST,
    [this](AsyncWebServerRequest* req) {
      if (!checkAuth(req)) return;
      if (!req->_tempObject) { req->send(400); return; }
      JsonDocument doc;
      deserializeJson(doc, (char*)req->_tempObject);
      free(req->_tempObject); req->_tempObject = nullptr;
      JsonObject obj = doc.as<JsonObject>();
      const int channel = obj["channel"] | 0;
      if (!web::ledChannelInRange(channel)) {
        req->send(400, "application/json", "{\"error\":\"invalid channel\"}");
        return;
      }
      stateStore_->mutate([&](state::VehicleState& s) {
        const bool enabled = obj["enabled"] | true;
        const uint8_t mode = obj["mode"] | 1;
        const uint8_t br = obj["brightness"] | 180;
        String colorStr = obj["color"] | "#00FF80";
        const uint32_t color = parseColor(colorStr);
        if (channel == 1) {
          s.led_channel_1_enabled = true;
          s.led_channel_1_mode = state::LedMode::RPM_GAUGE;
          s.led_channel_1_brightness = 180;
          s.led_channel_1_color = 0xFFFFFF;
        } else if (channel == 2) {
          s.led_channel_2_enabled = enabled; s.led_channel_2_mode = static_cast<state::LedMode>(mode);
          s.led_channel_2_brightness = br; s.led_channel_2_color = color;
        } else {
          s.led_channel_3_enabled = enabled; s.led_channel_3_mode = static_cast<state::LedMode>(mode);
          s.led_channel_3_brightness = br; s.led_channel_3_color = color;
        }
        if ((obj["preview_startup"] | false) == true) s.led_startup_preview = true;
      });
      settingsMgr_->updateFromState(stateStore_->read());
      settingsMgr_->save();
      req->send(200, "application/json", "{\"ok\":true}");
    },
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) { req->_tempObject = malloc(total + 1); }
      if (req->_tempObject) {
        memcpy((uint8_t*)req->_tempObject + index, data, len);
        if (index + len == total) ((uint8_t*)req->_tempObject)[total] = '\0';
      }
    });

  server_.on("/api/meth", HTTP_POST,
    [this](AsyncWebServerRequest* req) {
      if (!checkAuth(req)) return;
      if (!req->_tempObject) { req->send(400); return; }
      JsonDocument doc;
      deserializeJson(doc, (char*)req->_tempObject);
      free(req->_tempObject); req->_tempObject = nullptr;
      JsonObject obj = doc.as<JsonObject>();
      const bool confirm = obj["confirm"] | false;
      if (obj.containsKey("ratio")) {
        stateStore_->mutate([&](state::VehicleState& s) {
          s.meth_selected_ratio_percent = sanitizeMethRatioValue(obj["ratio"].as<uint8_t>());
        });
      }
      if (obj.containsKey("armed")) {
        const bool armed = obj["armed"].as<bool>();
        const bool sent = canMgr_->sendMethArm(armed);
        stateStore_->mutate([&](state::VehicleState& s) { s.meth_desired_armed = sent && armed; });
        if (armed && !sent) {
          req->send(409, "application/json", "{\"error\":\"meth arm rejected by safety policy\"}");
          return;
        }
      }
      if (obj.containsKey("manual_test_duty")) {
        if (!confirm) { req->send(400, "application/json", "{\"error\":\"confirmation required\"}"); return; }
        const uint8_t duty = obj["manual_test_duty"].as<uint8_t>();
        if (duty == 0) { req->send(400, "application/json", "{\"error\":\"manual_test_duty must be > 0\"}"); return; }
        if (!canMgr_->sendMethManualTest(duty)) {
          const state::VehicleState snapshot = stateStore_->read();
          JsonDocument out;
          out["error"] = "manual test rejected by safety policy";
          out["reason"] = web::manualTestRejectReasonText(snapshot.meth_manual_test_reject_reason);
          out["cooldown_ms_remaining"] = snapshot.meth_manual_test_cooldown_ms_remaining;
          String body; serializeJson(out, body);
          req->send(409, "application/json", body); return;
        }
      }
      if ((obj["clear_faults"] | false) == true) canMgr_->sendMethClearFaults();
      settingsMgr_->updateFromState(stateStore_->read());
      settingsMgr_->save();
      JsonDocument out;
      out["ok"] = true;
      out["warning"] = kRatioWarning;
      String body; serializeJson(out, body);
      req->send(200, "application/json", body);
    },
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) { req->_tempObject = malloc(total + 1); }
      if (req->_tempObject) {
        memcpy((uint8_t*)req->_tempObject + index, data, len);
        if (index + len == total) ((uint8_t*)req->_tempObject)[total] = '\0';
      }
    });

  server_.on("/api/race/control", HTTP_POST,
    [this](AsyncWebServerRequest* req) {
      if (!checkAuth(req)) return;
      if (!req->_tempObject) { req->send(400); return; }
      JsonDocument doc;
      deserializeJson(doc, (char*)req->_tempObject);
      free(req->_tempObject); req->_tempObject = nullptr;
      JsonObject obj = doc.as<JsonObject>();
      const String action = obj["action"] | "";
      if (action == "start_accel") { raceMgr_->startRun(state::RaceMode::ACCEL); }
      else if (action == "start_lap") { raceMgr_->startRun(state::RaceMode::LAP); }
      else if (action == "stop") { raceMgr_->stopRun(); }
      else if (action == "reset") { raceMgr_->resetSession(); }
      else if (action == "set_start_finish") { raceMgr_->setStartFinishPointFromCurrentFix(); }
      else if (action == "mark_lap") { raceMgr_->markLap(); }
      else { req->send(400, "application/json", "{\"error\":\"invalid action\"}"); return; }
      req->send(200, "application/json", "{\"ok\":true}");
    },
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) { req->_tempObject = malloc(total + 1); }
      if (req->_tempObject) {
        memcpy((uint8_t*)req->_tempObject + index, data, len);
        if (index + len == total) ((uint8_t*)req->_tempObject)[total] = '\0';
      }
    });

  server_.on("/api/taillights", HTTP_POST,
    [this](AsyncWebServerRequest* req) {
      if (!checkAuth(req)) return;
      if (!req->_tempObject) { req->send(400); return; }
      JsonDocument doc;
      deserializeJson(doc, (char*)req->_tempObject);
      free(req->_tempObject); req->_tempObject = nullptr;
      JsonObject obj = doc.as<JsonObject>();
      if (obj.containsKey("mode")) {
        const uint8_t mode = obj["mode"].as<uint8_t>();
        if (mode <= can_protocol::taillight_mode::DEMO) { canMgr_->sendTaillightMode(mode); }
        else { req->send(400, "application/json", "{\"error\":\"invalid taillight mode\"}"); return; }
      }
      if (obj.containsKey("brightness")) canMgr_->sendTaillightBrightness(obj["brightness"].as<uint8_t>());
      if ((obj["clear_override"] | false) == true) canMgr_->clearTaillightOverride();
      if (obj.containsKey("anim_id")) {
        canMgr_->sendTaillightCustomAnimation(obj["anim_id"].as<uint8_t>(), obj["duration_ms"] | 500, obj["param0"] | 0, obj["param1"] | 0);
      }
      req->send(200, "application/json", "{\"ok\":true}");
    },
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) { req->_tempObject = malloc(total + 1); }
      if (req->_tempObject) {
        memcpy((uint8_t*)req->_tempObject + index, data, len);
        if (index + len == total) ((uint8_t*)req->_tempObject)[total] = '\0';
      }
    });

  server_.on("/api/tach", HTTP_POST,
    [this](AsyncWebServerRequest* req) {
      if (!checkAuth(req)) return;
      if (!req->_tempObject) { req->send(400); return; }
      JsonDocument doc;
      deserializeJson(doc, (char*)req->_tempObject);
      free(req->_tempObject); req->_tempObject = nullptr;
      JsonObject obj = doc.as<JsonObject>();
      stateStore_->mutate([&](state::VehicleState& s) {
        if (obj.containsKey("pulses_per_rev10")) s.pulses_per_rev10 = obj["pulses_per_rev10"].as<uint8_t>();
        if (obj.containsKey("tach_scaling_mode")) s.tach_scaling_mode = obj["tach_scaling_mode"].as<uint8_t>();
      });
      settingsMgr_->updateFromState(stateStore_->read());
      settingsMgr_->save();
      req->send(200, "application/json", "{\"ok\":true}");
    },
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) { req->_tempObject = malloc(total + 1); }
      if (req->_tempObject) {
        memcpy((uint8_t*)req->_tempObject + index, data, len);
        if (index + len == total) ((uint8_t*)req->_tempObject)[total] = '\0';
      }
    });

  server_.begin();
  return true;
}

bool WebServerManager::checkAuth(AsyncWebServerRequest* request) const {
  if (!request || !settingsMgr_) return false;
  const char* pass = settingsMgr_->data().web_password;
  if (!pass || pass[0] == '\0') return true;
  String candidate;
  const AsyncWebHeader* header = request->getHeader("X-Auth-Token");
  if (header) {
    candidate = header->value();
  }
  if (!constantTimeEquals(pass, candidate)) {
    request->send(401, "application/json", "{\"error\":\"unauthorized\"}");
    return false;
  }
  return true;
}

String WebServerManager::stateJson() const {
  JsonDocument doc;
  const state::VehicleState s = stateStore_->read();

  doc["rpm"] = s.rpm;
  doc["vehicle_speed"] = s.speed;
  doc["gps_fix"] = s.gps_fix;
  doc["gps_satellites"] = s.gps_satellites;
  doc["gps_satellites_in_view"] = s.gps_satellites_in_view;
  doc["gps_fix_quality"] = s.gps_fix_quality;
  doc["gps_fix_mode"] = s.gps_fix_mode;
  doc["imu_online"] = s.imu_online;
  doc["imu_g_lateral"] = s.imu_g_lateral;
  doc["imu_g_longitudinal"] = s.imu_g_longitudinal;
  doc["imu_g_total"] = s.imu_g_total;
  doc["imu_g_peak"] = s.imu_g_peak;
  doc["battery_voltage"] = s.battery_voltage;
  doc["microsquirt_online"] = s.microsquirt_online;
  doc["microsquirt_last_ms"] = s.microsquirt_last_ms;
  doc["microsquirt_frame_count"] = s.microsquirt_frame_count;
  doc["microsquirt_map_kpa"] = s.microsquirt_map_kpa;
  doc["microsquirt_baro_kpa"] = s.microsquirt_baro_kpa;
  doc["microsquirt_tps_percent"] = s.microsquirt_tps_percent;
  doc["microsquirt_spark_deg"] = s.microsquirt_spark_deg;
  doc["microsquirt_pw1_ms"] = s.microsquirt_pw1_ms;
  doc["microsquirt_pw2_ms"] = s.microsquirt_pw2_ms;
  doc["microsquirt_afr_target"] = s.microsquirt_afr_target;
  doc["cabin_temp"] = s.cabin_temp;
  doc["outside_temp"] = s.outside_temp;
  doc["engine_bay_temp"] = s.engine_bay_temp;
  doc["iat"] = s.intake_temp;
  doc["intercooler_temp"] = s.intercooler_temp;
  doc["oil_pressure_psi"] = s.oil_pressure_psi;
  doc["fuel_pressure_psi"] = s.fuel_pressure_psi;
  doc["meth_pressure_psi"] = s.meth_pressure_psi;
  doc["boost_ref_pressure_psi"] = s.boost_ref_pressure_psi;
  doc["spare_pressure_1_psi"] = s.spare_pressure_1_psi;
  doc["spare_pressure_2_psi"] = s.spare_pressure_2_psi;
  doc["intake_temp_valid"] = s.intake_temp_valid;
  doc["engine_bay_temp_valid"] = s.engine_bay_temp_valid;
  doc["cabin_temp_valid"] = s.cabin_temp_valid;
  doc["outside_temp_valid"] = s.outside_temp_valid;
  doc["oil_pressure_valid"] = s.oil_pressure_valid;
  doc["fuel_pressure_valid"] = s.fuel_pressure_valid;
  doc["meth_pressure_valid"] = s.meth_pressure_valid;
  doc["boost_ref_pressure_valid"] = s.boost_ref_pressure_valid;
  doc["spare_pressure_1_valid"] = s.spare_pressure_1_valid;
  doc["spare_pressure_2_valid"] = s.spare_pressure_2_valid;
  doc["analog_sensor_fault_flags"] = s.analog_sensor_fault_flags;
  doc["analog_sensors_enabled"] = s.analog_sensors_enabled;
  doc["last_analog_sensor_ms"] = s.last_analog_sensor_ms;
  doc["meth_state"] = static_cast<uint8_t>(s.meth_state);
  doc["pump_duty"] = s.meth_pump_duty;
  doc["tank_level"] = s.meth_tank_level;
  doc["selected_meth_ratio"] = s.meth_selected_ratio_percent;
  doc["meth_fault_flags"] = s.meth_fault_flags;
  doc["meth_manual_test_reject_reason"] = web::manualTestRejectReasonText(s.meth_manual_test_reject_reason);
  doc["meth_manual_test_cooldown_ms_remaining"] = s.meth_manual_test_cooldown_ms_remaining;
  doc["knock_enabled"] = s.knock_enabled;
  doc["knock_energy"] = s.knock_energy;
  doc["knock_baseline"] = s.knock_baseline;
  doc["knock_threshold"] = s.knock_threshold;
  doc["knock_event_count"] = s.knock_event_count;
  doc["knock_last_event_rpm"] = s.knock_last_event_rpm;
  doc["knock_last_event_boost_kpa"] = s.knock_last_event_boost_kpa;
  doc["knock_signal_valid"] = s.knock_signal_valid;
  doc["knock_warning_active"] = s.knock_warning_active;
  doc["knock_critical_active"] = s.knock_critical_active;
  doc["knock_baseline_learned"] = s.knock_baseline_learned;
  doc["knock_sensor_fault"] = s.knock_sensor_fault;
  doc["knock_clipping_detected"] = s.knock_clipping_detected;
  doc["knock_response_mode"] = s.knock_response_mode;
  doc["can_node_online"] = s.can_online;
  doc["taillight_state_left"] = s.taillight_left_state;
  doc["taillight_state_right"] = s.taillight_right_state;
  doc["faults"] = s.fault_flags;
  doc["warning"] = kRatioWarning;
  doc["led1_color"] = toHexColor(s.led_channel_1_color);
  doc["led2_color"] = toHexColor(s.led_channel_2_color);
  doc["led3_color"] = toHexColor(s.led_channel_3_color);
  doc["gps_latitude"] = s.gps_latitude;
  doc["gps_longitude"] = s.gps_longitude;
  doc["race_mode"] = static_cast<uint8_t>(s.race_mode);
  doc["race_enabled"] = s.race_enabled;
  doc["race_running"] = s.race_running;
  doc["race_quality_percent"] = s.race_quality_percent;
  doc["race_validation_flags"] = s.race_validation_flags;
  doc["race_data_valid"] = s.race_data_valid;
  doc["race_elapsed_ms"] = s.race_elapsed_ms;
  doc["race_distance_m"] = s.race_distance_m;
  doc["race_0_30_s"] = s.race_0_30_s;
  doc["race_0_60_s"] = s.race_0_60_s;
  doc["race_60_130_s"] = s.race_60_130_s;
  doc["race_100_150_kph_s"] = s.race_100_150_kph_s;
  doc["race_eighth_mile_et_s"] = s.race_eighth_mile_et_s;
  doc["race_quarter_mile_et_s"] = s.race_quarter_mile_et_s;
  doc["race_eighth_mile_trap_mph"] = s.race_eighth_mile_trap_mph;
  doc["race_quarter_mile_trap_mph"] = s.race_quarter_mile_trap_mph;
  doc["race_lap_count"] = s.race_lap_count;
  doc["race_last_lap_s"] = s.race_last_lap_s;
  doc["race_best_lap_s"] = s.race_best_lap_s;
  doc["race_lap_delta_s"] = s.race_lap_delta_s;
  doc["race_start_point_set"] = s.race_start_point_set;
  doc["race_start_latitude"] = s.race_start_latitude;
  doc["race_start_longitude"] = s.race_start_longitude;
  doc["race_start_finish_radius_m"] = s.race_start_finish_radius_m;

  String out;
  serializeJson(doc, out);
  return out;
}

String WebServerManager::canStatusJson() const {
  JsonDocument doc;
  const state::VehicleState s = stateStore_->read();
  doc["can_online"] = s.can_online;
  doc["can_rx_count"] = s.can_rx_count;
  doc["can_tx_count"] = s.can_tx_count;
  doc["can_bad_checksum_count"] = s.can_bad_checksum_count;
  doc["can_last_rx_id"] = s.can_last_rx_id;
  doc["can_last_tx_id"] = s.can_last_tx_id;
  doc["can_last_rx_ms"] = s.can_last_rx_ms;
  doc["can_last_tx_ms"] = s.can_last_tx_ms;
  doc["microsquirt_online"] = s.microsquirt_online;
  doc["microsquirt_last_id"] = s.microsquirt_last_id;
  doc["microsquirt_last_ms"] = s.microsquirt_last_ms;
  doc["microsquirt_frame_count"] = s.microsquirt_frame_count;
  doc["microsquirt_invalid_count"] = s.microsquirt_invalid_count;
  doc["telemetry_recording"] = s.telemetry_recording;
  doc["telemetry_queue_depth"] = s.telemetry_queue_depth;
  doc["telemetry_queue_high_water"] = s.telemetry_queue_high_water;
  doc["telemetry_captured_count"] = s.telemetry_captured_count;
  doc["telemetry_written_count"] = s.telemetry_written_count;
  doc["telemetry_dropped_count"] = s.telemetry_dropped_count;
  doc["telemetry_write_errors"] = s.telemetry_write_errors;
  doc["telemetry_recovery_count"] = s.telemetry_recovery_count;
  doc["taillight_online"] = s.taillight_online;
  doc["meth_online"] = s.meth_online;
  doc["gps_stale"] = s.gps_stale;
  doc["fault_flags"] = s.fault_flags;
  doc["meth_fault_flags"] = s.meth_fault_flags;
  doc["master_state"] = s.master_state;
  doc["knock_enabled"] = s.knock_enabled;
  doc["knock_signal_valid"] = s.knock_signal_valid;
  doc["knock_energy"] = s.knock_energy;
  doc["knock_baseline"] = s.knock_baseline;
  doc["knock_threshold"] = s.knock_threshold;
  doc["knock_warning_active"] = s.knock_warning_active;
  doc["knock_critical_active"] = s.knock_critical_active;
  doc["knock_event_count"] = s.knock_event_count;
  doc["knock_last_event_rpm"] = s.knock_last_event_rpm;
  doc["knock_last_event_boost_kpa"] = s.knock_last_event_boost_kpa;
  doc["knock_last_event_iat_c"] = s.knock_last_event_iat_c;
  doc["knock_sensor_fault"] = s.knock_sensor_fault;
  doc["knock_clipping_detected"] = s.knock_clipping_detected;
  doc["knock_baseline_learned"] = s.knock_baseline_learned;
  doc["knock_response_mode"] = s.knock_response_mode;
  doc["knock_logging_active"] = s.knock_logging_active;
  doc["knock_online"] = s.knock_online;
  doc["oil_pressure_psi"] = s.oil_pressure_psi;
  doc["fuel_pressure_psi"] = s.fuel_pressure_psi;
  doc["meth_pressure_psi"] = s.meth_pressure_psi;
  doc["boost_ref_pressure_psi"] = s.boost_ref_pressure_psi;
  doc["analog_sensor_fault_flags"] = s.analog_sensor_fault_flags;
  doc["analog_sensors_enabled"] = s.analog_sensors_enabled;
  doc["last_analog_sensor_ms"] = s.last_analog_sensor_ms;
  doc["uptime_ms"] = s.uptime_ms;
  doc["reset_reason"] = s.reset_reason;
  doc["brownout_reset_count"] = s.brownout_reset_count;
  doc["watchdog_reset_count"] = s.watchdog_reset_count;
  doc["meth_manual_test_reject_reason"] = web::manualTestRejectReasonText(s.meth_manual_test_reject_reason);
  doc["meth_manual_test_cooldown_ms_remaining"] = s.meth_manual_test_cooldown_ms_remaining;
  doc["web_connected_clients"] = s.web_connected_clients;
  String out;
  serializeJson(doc, out);
  return out;
}

void WebServerManager::sendState(AsyncWebServerRequest* request) const {
  if (!checkAuth(request)) return;
  request->send(200, "application/json", stateJson());
}

void WebServerManager::sendSettings(AsyncWebServerRequest* request) const {
  if (!checkAuth(request)) return;
  const auto& st = settingsMgr_->data();
  JsonDocument doc;
  doc["display_brightness"] = st.display_brightness;
  doc["night_mode"] = st.night_mode_enabled;
  doc["tach_pulses_per_rev10"] = st.tach_pulses_per_rev10;
  doc["tach_scaling_mode"] = st.tach_scaling_mode;
  doc["led_global_brightness"] = st.led_global_brightness;
  doc["led_theme"] = st.led_theme;
  doc["meth_ratio"] = st.meth_selected_ratio_percent;
  doc["race_use_metric_targets"] = st.race_use_metric_targets;
  doc["race_auto_start"] = st.race_auto_start;
  doc["race_min_satellites"] = st.race_min_satellites;
  doc["race_sample_min_ms"] = st.race_sample_min_ms;
  doc["race_sample_max_ms"] = st.race_sample_max_ms;
  doc["race_start_finish_radius_m"] = st.race_start_finish_radius_m;
  doc["race_start_latitude"] = st.race_start_latitude;
  doc["race_start_longitude"] = st.race_start_longitude;
  doc["race_start_point_set"] = st.race_start_point_set;
  doc["wifi_ap_mode"] = st.wifi_ap_mode;
  doc["knock_enabled"] = st.knock_enabled;
  doc["knock_adc_pin"] = st.knock_adc_pin;
  doc["knock_boost_enable_kpa"] = st.knock_boost_enable_kpa;
  doc["knock_rpm_enable_min"] = st.knock_rpm_enable_min;
  doc["knock_gain"] = st.knock_gain;
  doc["knock_threshold_multiplier"] = st.knock_threshold_multiplier;
  doc["knock_threshold_offset"] = st.knock_threshold_offset;
  doc["knock_event_cooldown_ms"] = st.knock_event_cooldown_ms;
  doc["knock_warning_threshold_count"] = st.knock_warning_threshold_count;
  doc["knock_critical_threshold_count"] = st.knock_critical_threshold_count;
  doc["knock_baseline_learning_enabled"] = st.knock_baseline_learning_enabled;
  doc["knock_demo_mode_enabled"] = st.knock_demo_mode_enabled;
  doc["knock_response_mode"] = st.knock_response_mode;
  doc["analog_sensors_enabled"] = st.analog_sensors_enabled;
  doc["analog_sensor_sample_ms"] = st.analog_sensor_sample_ms;
  doc["thermistor_pullup_ohms"] = st.thermistor_pullup_ohms;
  doc["iat_adc_pin"] = st.iat_adc_pin;
  doc["engine_bay_adc_pin"] = st.engine_bay_adc_pin;
  doc["cabin_temp_adc_pin"] = st.cabin_temp_adc_pin;
  doc["ambient_temp_adc_pin"] = st.ambient_temp_adc_pin;
  doc["oil_pressure_adc_pin"] = st.oil_pressure_adc_pin;
  doc["fuel_pressure_adc_pin"] = st.fuel_pressure_adc_pin;
  doc["meth_pressure_adc_pin"] = st.meth_pressure_adc_pin;
  doc["boost_ref_pressure_adc_pin"] = st.boost_ref_pressure_adc_pin;
  doc["spare_pressure_1_adc_pin"] = st.spare_pressure_1_adc_pin;
  doc["spare_pressure_2_adc_pin"] = st.spare_pressure_2_adc_pin;
  doc["iat_sensor_enabled"] = st.iat_sensor_enabled;
  doc["engine_bay_sensor_enabled"] = st.engine_bay_sensor_enabled;
  doc["cabin_temp_sensor_enabled"] = st.cabin_temp_sensor_enabled;
  doc["ambient_temp_sensor_enabled"] = st.ambient_temp_sensor_enabled;
  doc["oil_pressure_sensor_enabled"] = st.oil_pressure_sensor_enabled;
  doc["fuel_pressure_sensor_enabled"] = st.fuel_pressure_sensor_enabled;
  doc["meth_pressure_sensor_enabled"] = st.meth_pressure_sensor_enabled;
  doc["boost_ref_pressure_sensor_enabled"] = st.boost_ref_pressure_sensor_enabled;
  doc["spare_pressure_1_sensor_enabled"] = st.spare_pressure_1_sensor_enabled;
  doc["spare_pressure_2_sensor_enabled"] = st.spare_pressure_2_sensor_enabled;
  doc["pressure_sensor_min_v"] = st.pressure_sensor_min_v;
  doc["pressure_sensor_max_v"] = st.pressure_sensor_max_v;
  doc["pressure_sensor_max_psi"] = st.pressure_sensor_max_psi;
  doc["warning"] = kRatioWarning;

  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void WebServerManager::sendDiagnostics(AsyncWebServerRequest* request) const {
  if (!checkAuth(request)) return;
  const state::VehicleState s = stateStore_->read();
  JsonDocument doc;
  doc["can_rx_count"] = s.can_rx_count;
  doc["can_tx_count"] = s.can_tx_count;
  doc["can_bad_checksum_count"] = s.can_bad_checksum_count;
  doc["last_rx_ms"] = s.can_last_rx_ms;
  doc["last_tx_ms"] = s.can_last_tx_ms;
  doc["node_online"] = s.can_online;
  doc["esp_die_temp_c"] = s.esp_die_temp_c;
  doc["heap_free"] = s.heap_free_bytes;
  doc["uptime_ms"] = s.uptime_ms;
  doc["reset_reason"] = s.reset_reason;
  doc["wifi_connected"] = s.wifi_connected;
  doc["gps_raw_fix"] = s.gps_fix;
  doc["gps_satellites"] = s.gps_satellites;
  doc["gps_satellites_in_view"] = s.gps_satellites_in_view;
  doc["gps_fix_quality"] = s.gps_fix_quality;
  doc["gps_fix_mode"] = s.gps_fix_mode;
  doc["tach_input_frequency_hz"] = s.tach_input_frequency_hz;
  doc["tach_generated_frequency_hz"] = s.tach_generated_frequency_hz;
  doc["sd_mounted"] = s.sd_mounted;
  doc["sd_size_bytes"] = s.sd_size_bytes;
  doc["sd_used_bytes"] = s.sd_used_bytes;
  doc["current_log_file"] = s.current_log_file;
  doc["last_sd_write_status"] = s.last_sd_write_status;
  doc["sd_write_error_count"] = s.sd_write_error_count;
  doc["touch_online"] = s.touch_online;
  doc["imu_online"] = s.imu_online;
  doc["imu_g_lateral"] = s.imu_g_lateral;
  doc["imu_g_longitudinal"] = s.imu_g_longitudinal;
  doc["imu_g_total"] = s.imu_g_total;
  doc["imu_g_peak"] = s.imu_g_peak;
  doc["ui_fps"] = s.ui_fps;
  doc["race_running"] = s.race_running;
  doc["race_quality_percent"] = s.race_quality_percent;
  doc["race_validation_flags"] = s.race_validation_flags;
  doc["brownout_reset_count"] = s.brownout_reset_count;
  doc["watchdog_reset_count"] = s.watchdog_reset_count;
  doc["meth_manual_test_reject_reason"] = web::manualTestRejectReasonText(s.meth_manual_test_reject_reason);
  doc["meth_manual_test_cooldown_ms_remaining"] = s.meth_manual_test_cooldown_ms_remaining;
  doc["knock_energy"] = s.knock_energy;
  doc["knock_baseline"] = s.knock_baseline;
  doc["knock_threshold"] = s.knock_threshold;
  doc["knock_event_count"] = s.knock_event_count;
  doc["knock_last_event_rpm"] = s.knock_last_event_rpm;
  doc["knock_last_event_boost_kpa"] = s.knock_last_event_boost_kpa;
  doc["knock_signal_valid"] = s.knock_signal_valid;
  doc["knock_warning_active"] = s.knock_warning_active;
  doc["knock_critical_active"] = s.knock_critical_active;
  doc["knock_baseline_learned"] = s.knock_baseline_learned;
  doc["knock_sensor_fault"] = s.knock_sensor_fault;
  doc["knock_clipping_detected"] = s.knock_clipping_detected;
  doc["knock_signal_clip_high_count"] = s.knock_signal_clip_high_count;
  doc["knock_signal_clip_low_count"] = s.knock_signal_clip_low_count;
  doc["intake_temp_c"] = s.intake_temp;
  doc["engine_bay_temp_c"] = s.engine_bay_temp;
  doc["cabin_temp_c"] = s.cabin_temp;
  doc["ambient_temp_c"] = s.outside_temp;
  doc["oil_pressure_psi"] = s.oil_pressure_psi;
  doc["fuel_pressure_psi"] = s.fuel_pressure_psi;
  doc["meth_pressure_psi"] = s.meth_pressure_psi;
  doc["boost_ref_pressure_psi"] = s.boost_ref_pressure_psi;
  doc["spare_pressure_1_psi"] = s.spare_pressure_1_psi;
  doc["spare_pressure_2_psi"] = s.spare_pressure_2_psi;
  doc["intake_temp_valid"] = s.intake_temp_valid;
  doc["engine_bay_temp_valid"] = s.engine_bay_temp_valid;
  doc["cabin_temp_valid"] = s.cabin_temp_valid;
  doc["outside_temp_valid"] = s.outside_temp_valid;
  doc["oil_pressure_valid"] = s.oil_pressure_valid;
  doc["fuel_pressure_valid"] = s.fuel_pressure_valid;
  doc["meth_pressure_valid"] = s.meth_pressure_valid;
  doc["boost_ref_pressure_valid"] = s.boost_ref_pressure_valid;
  doc["spare_pressure_1_valid"] = s.spare_pressure_1_valid;
  doc["spare_pressure_2_valid"] = s.spare_pressure_2_valid;
  doc["analog_sensor_fault_flags"] = s.analog_sensor_fault_flags;

  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void WebServerManager::sendCanStatus(AsyncWebServerRequest* request) const {
  if (!checkAuth(request)) return;
  request->send(200, "application/json", canStatusJson());
}

// Static buffer for WS live-push serialization — avoids a heap String allocation each tick.
static char s_wsLiveBuf[1536];

void WebServerManager::tick() {
  if (!stateStore_) return;
  ws_.cleanupClients();

  const uint32_t nowMs = millis();
  if ((nowMs - lastWsPushMs_) < 1000) return;  // 1 s — half the push rate
  lastWsPushMs_ = nowMs;

  stateStore_->mutate([this](state::VehicleState& s) {
    s.web_connected_clients = ws_.count();
    s.wifi_connected = WiFi.status() == WL_CONNECTED;
  });

  if (ws_.count() == 0) return;

  // Heap guard — skip the push if we're running critically low to avoid
  // fragmenting what remains and triggering a std::bad_alloc / abort().
  if (ESP.getFreeHeap() < 20000) return;

  try {
    // Build a slim "live data" document (~30 fields) instead of the full
    // 85-field stateJson(). The browser fetches stateJson() once via HTTP GET
    // on page-load; the WS stream only needs the fast-changing values.
    JsonDocument doc;
    const state::VehicleState s = stateStore_->read();

    // Core vehicle
    doc["rpm"] = s.rpm;
    doc["vehicle_speed"] = s.speed;
    doc["gps_fix"] = s.gps_fix;
    doc["battery_voltage"] = s.battery_voltage;
    doc["faults"] = s.fault_flags;
    doc["meth_fault_flags"] = s.meth_fault_flags;
    doc["can_node_online"] = s.can_online;

    // Temperatures
    doc["cabin_temp"] = s.cabin_temp;
    doc["outside_temp"] = s.outside_temp;
    doc["engine_bay_temp"] = s.engine_bay_temp;
    doc["iat"] = s.intake_temp;
    doc["intercooler_temp"] = s.intercooler_temp;

    // Pressures
    doc["boost_ref_pressure_psi"] = s.boost_ref_pressure_psi;
    doc["oil_pressure_psi"] = s.oil_pressure_psi;
    doc["fuel_pressure_psi"] = s.fuel_pressure_psi;
    doc["meth_pressure_psi"] = s.meth_pressure_psi;
    doc["spare_pressure_1_psi"] = s.spare_pressure_1_psi;
    doc["spare_pressure_2_psi"] = s.spare_pressure_2_psi;

    // Meth
    doc["meth_state"] = static_cast<uint8_t>(s.meth_state);
    doc["pump_duty"] = s.meth_pump_duty;
    doc["tank_level"] = s.meth_tank_level;
    doc["selected_meth_ratio"] = s.meth_selected_ratio_percent;

    // Knock
    doc["knock_energy"] = s.knock_energy;
    doc["knock_warning_active"] = s.knock_warning_active;
    doc["knock_critical_active"] = s.knock_critical_active;
    doc["knock_signal_valid"] = s.knock_signal_valid;
    doc["knock_event_count"] = s.knock_event_count;

    // Race / lap
    doc["race_running"] = s.race_running;
    doc["race_elapsed_ms"] = s.race_elapsed_ms;
    doc["race_quality_percent"] = s.race_quality_percent;

    // Taillights
    doc["taillight_state_left"] = s.taillight_left_state;
    doc["taillight_state_right"] = s.taillight_right_state;

    // Serialize directly into static buffer — no heap String allocation.
    const size_t len = serializeJson(doc, s_wsLiveBuf, sizeof(s_wsLiveBuf));
    if (len > 0 && len < sizeof(s_wsLiveBuf)) {
      ws_.textAll(s_wsLiveBuf);
    }
  } catch (...) {
    // std::bad_alloc or other heap failure — skip this push, try again next tick
  }
}

}  // namespace web

#endif  // CCM_WEB_ENABLED
