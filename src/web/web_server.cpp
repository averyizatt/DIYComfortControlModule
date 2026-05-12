#include "web/web_server.h"

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <WiFi.h>

#include <cstring>

#include "meth/meth_config.h"

namespace web {

namespace {
constexpr const char* kRatioWarning = "Mixture ratio is user selected and not sensor verified.";
constexpr const char* kUiFontStack = "'Arial Black','Segoe UI',Arial,sans-serif";

const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Foxbody Cabin Master</title><style>
*{box-sizing:border-box}
body{font-family:{{UI_FONT_STACK}};background:#0f1114;color:#f3f6fa;margin:0;line-height:1.35}
header{padding:16px 18px;background:#161c24;border-bottom:2px solid #2f3f55}
h1{margin:0;font-size:clamp(28px,5vw,44px);letter-spacing:.5px}
.sub{margin-top:6px;color:#b8c4d8;font-size:clamp(13px,2.2vw,18px)}
main{padding:14px;display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px;max-width:1320px;margin:0 auto}
.card{border:2px solid #33445f;border-radius:12px;padding:14px;background:#171d27;min-width:0}
.card h2{margin:0 0 10px 0;font-size:clamp(22px,3.3vw,30px)}
pre{margin:0;background:#0f141c;border:1px solid #2f3d51;border-radius:8px;padding:10px;font-size:clamp(14px,2.25vw,20px);white-space:pre-wrap;word-break:break-word;overflow-wrap:anywhere}
.warn{font-size:clamp(14px,2.3vw,18px);font-weight:700;color:#ffd27d}
.actions{display:grid;grid-template-columns:repeat(2,minmax(130px,1fr));gap:8px}
button{font:inherit;font-size:clamp(16px,2.6vw,22px);font-weight:800;padding:12px;border-radius:10px;border:2px solid #4c6488;background:#243246;color:#f0f4ff}
button:active{transform:scale(.99)}
a.link{color:#8ec8ff;font-size:clamp(16px,2.4vw,22px);text-decoration:none}
@media (max-width:760px){.actions{grid-template-columns:1fr}}
</style></head>
<body>
<header><h1>Foxbody Cabin Master</h1><div class='sub'>Race-safe dashboard • High-contrast • Large block text</div></header>
<main>
<div class='card'><h2>Live Vehicle</h2><pre id='live'>Connecting...</pre></div>
<div class='card'><h2>CAN Status</h2><pre id='canStatus'>Loading CAN status...</pre></div>
<div class='card'><h2>Water Meth</h2><p class='warn' id='ratioWarn'></p></div>
<div class='card'><h2>Race Performance</h2><div class='actions'><button onclick="raceCmd('start_accel')">Start Accel</button><button onclick="raceCmd('start_lap')">Start Lap</button><button onclick="raceCmd('stop')">Stop</button><button onclick="raceCmd('reset')">Reset</button></div><pre id='race'>Loading race data...</pre></div>
<div class='card'><h2>Pages</h2><p><a class='link' href='/can'>Open dedicated CAN status page</a></p><p class='sub'>Dashboard • Race • Settings • LED • Water Meth • Taillights • Diagnostics • CAN Status</p></div>
</main>
<script>
document.getElementById('ratioWarn').textContent='{{RATIO_WARNING}}';
function pretty(text){try{return JSON.stringify(JSON.parse(text),null,2);}catch(_){return text;}}
const ws=new WebSocket(`ws://${location.host}/ws`);ws.onmessage=e=>{document.getElementById('live').textContent=pretty(e.data);};
async function raceCmd(action){await fetch('/api/race/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});}
async function updatePanels(){
  try{const race=await fetch('/api/race/state');document.getElementById('race').textContent=pretty(await race.text());}catch(_){}
  try{const can=await fetch('/api/can/status');document.getElementById('canStatus').textContent=pretty(await can.text());}catch(_){}
}
setInterval(updatePanels,1000);updatePanels();
</script></body></html>
)HTML";

const char kCanStatusHtml[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Foxbody CAN Status</title><style>
*{box-sizing:border-box}body{font-family:{{UI_FONT_STACK}};background:#0f1114;color:#f3f6fa;margin:0}
header{padding:16px 18px;background:#161c24;border-bottom:2px solid #2f3f55}
h1{margin:0;font-size:clamp(28px,5vw,44px)}main{padding:14px;max-width:1200px;margin:0 auto}
.card{border:2px solid #33445f;border-radius:12px;padding:14px;background:#171d27}
pre{margin:0;background:#0f141c;border:1px solid #2f3d51;border-radius:8px;padding:10px;font-size:clamp(14px,2.4vw,22px);white-space:pre-wrap;word-break:break-word;overflow-wrap:anywhere}
a{display:inline-block;margin-bottom:10px;color:#8ec8ff;font-size:clamp(16px,2.2vw,22px)}
</style></head>
<body><header><h1>CAN Status</h1></header><main><a href='/'>← Back to Dashboard</a><div class='card'><pre id='canStatus'>Loading CAN status...</pre></div></main>
<script>
function pretty(text){try{return JSON.stringify(JSON.parse(text),null,2);}catch(_){return text;}}
async function tick(){try{const r=await fetch('/api/can/status');document.getElementById('canStatus').textContent=pretty(await r.text());}catch(_){}} 
setInterval(tick,600);tick();
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

uint8_t sanitizeMethBoostTriggerKpa(uint8_t kpa) {
  return static_cast<uint8_t>(constrain(static_cast<int>(kpa), meth::kMinBoostTriggerKpa, meth::kMaxBoostTriggerKpa));
}

int8_t sanitizeMethIatThresholdC(int value) {
  return static_cast<int8_t>(constrain(value, meth::kMinIatThresholdC, meth::kMaxIatThresholdC));
}

void sanitizeMethConfig(state::VehicleState& s) {
  s.meth_selected_ratio_percent = sanitizeMethRatioValue(s.meth_selected_ratio_percent);
  s.meth_boost_trigger_kpa = sanitizeMethBoostTriggerKpa(s.meth_boost_trigger_kpa);
  s.meth_iat_safety_threshold = sanitizeMethIatThresholdC(s.meth_iat_safety_threshold);
  if (static_cast<uint8_t>(s.meth_can_loss_behavior) > static_cast<uint8_t>(state::MethCanLossBehavior::HOLD_LAST_VALID)) {
    s.meth_can_loss_behavior = state::MethCanLossBehavior::DISARM;
  }
}

const char* manualTestRejectReasonText(uint8_t code) {
  switch (code) {
    case 1:
      return "offline";
    case 2:
      return "fault";
    case 3:
      return "cooldown";
    case 4:
      return "duty_zero";
    case 5:
      return "duty_over_max";
    default:
      return "none";
  }
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
    DynamicJsonDocument out(256);
    out["ok"] = true;
    out["exported_entries"] = written;
    out["target"] = "/logs/race";
    String body;
    serializeJson(out, body);
    req->send(200, "application/json", body);
  });

  auto settingsHandler = new AsyncCallbackJsonWebHandler("/api/settings", [this](AsyncWebServerRequest* req, JsonVariant& json) {
    if (!checkAuth(req)) return;
    JsonObject obj = json.as<JsonObject>();
    stateStore_->mutate([&](state::VehicleState& s) {
      if (obj.containsKey("display_brightness")) s.display_brightness = obj["display_brightness"].as<uint8_t>();
      if (obj.containsKey("night_mode")) s.night_mode_enabled = obj["night_mode"].as<bool>();
      if (obj.containsKey("tach_pulses_per_rev10")) s.pulses_per_rev10 = obj["tach_pulses_per_rev10"].as<uint8_t>();
      if (obj.containsKey("tach_scaling_mode")) s.tach_scaling_mode = obj["tach_scaling_mode"].as<uint8_t>();
      if (obj.containsKey("led_global_brightness")) s.led_global_brightness = obj["led_global_brightness"].as<uint8_t>();
      if (obj.containsKey("led_theme")) s.led_theme = obj["led_theme"].as<uint8_t>();
      if (obj.containsKey("meth_ratio")) s.meth_selected_ratio_percent = sanitizeMethRatioValue(obj["meth_ratio"].as<uint8_t>());
      if (obj.containsKey("meth_boost_trigger_kpa")) s.meth_boost_trigger_kpa = sanitizeMethBoostTriggerKpa(obj["meth_boost_trigger_kpa"].as<uint8_t>());
      if (obj.containsKey("meth_iat_safety_threshold")) s.meth_iat_safety_threshold = sanitizeMethIatThresholdC(obj["meth_iat_safety_threshold"].as<int>());
      if (obj.containsKey("meth_max_pump_duty")) s.meth_max_pump_duty = obj["meth_max_pump_duty"].as<uint8_t>();
      if (obj.containsKey("can_loss_behavior")) {
        const uint8_t behavior = obj["can_loss_behavior"].as<uint8_t>();
        s.meth_can_loss_behavior =
            (behavior <= static_cast<uint8_t>(state::MethCanLossBehavior::HOLD_LAST_VALID)) ? static_cast<state::MethCanLossBehavior>(behavior)
                                                                                              : state::MethCanLossBehavior::DISARM;
      }
      if (obj.containsKey("race_use_metric_targets")) s.race_use_metric_targets = obj["race_use_metric_targets"].as<bool>();
      if (obj.containsKey("race_auto_start")) s.race_auto_start = obj["race_auto_start"].as<bool>();
      if (obj.containsKey("race_min_satellites")) s.race_min_satellites = obj["race_min_satellites"].as<uint8_t>();
      if (obj.containsKey("race_sample_min_ms")) s.race_sample_min_ms = obj["race_sample_min_ms"].as<uint16_t>();
      if (obj.containsKey("race_sample_max_ms")) s.race_sample_max_ms = obj["race_sample_max_ms"].as<uint16_t>();
      if (obj.containsKey("race_start_finish_radius_m")) s.race_start_finish_radius_m = obj["race_start_finish_radius_m"].as<float>();
      sanitizeMethConfig(s);
    });

    state::VehicleState snapshot = stateStore_->read();
    settingsMgr_->updateFromState(snapshot);
    settingsMgr_->save();

    req->send(200, "application/json", "{\"ok\":true}");
  });
  server_.addHandler(settingsHandler);

  auto ledHandler = new AsyncCallbackJsonWebHandler("/api/led", [this](AsyncWebServerRequest* req, JsonVariant& json) {
    if (!checkAuth(req)) return;
    JsonObject obj = json.as<JsonObject>();
    const int channel = obj["channel"] | 0;
    if (channel < 1 || channel > 3) {
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
        s.led_channel_1_enabled = enabled;
        s.led_channel_1_mode = static_cast<state::LedMode>(mode);
        s.led_channel_1_brightness = br;
        s.led_channel_1_color = color;
      } else if (channel == 2) {
        s.led_channel_2_enabled = enabled;
        s.led_channel_2_mode = static_cast<state::LedMode>(mode);
        s.led_channel_2_brightness = br;
        s.led_channel_2_color = color;
      } else {
        s.led_channel_3_enabled = enabled;
        s.led_channel_3_mode = static_cast<state::LedMode>(mode);
        s.led_channel_3_brightness = br;
        s.led_channel_3_color = color;
      }

      if ((obj["preview_startup"] | false) == true) {
        s.led_startup_preview = true;
      }
    });

    settingsMgr_->updateFromState(stateStore_->read());
    settingsMgr_->save();
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server_.addHandler(ledHandler);

  auto methHandler = new AsyncCallbackJsonWebHandler("/api/meth", [this](AsyncWebServerRequest* req, JsonVariant& json) {
    if (!checkAuth(req)) return;
    JsonObject obj = json.as<JsonObject>();
    const bool confirm = obj["confirm"] | false;

    if (obj.containsKey("ratio")) {
      stateStore_->mutate([&](state::VehicleState& s) {
        s.meth_selected_ratio_percent = sanitizeMethRatioValue(obj["ratio"].as<uint8_t>());
        sanitizeMethConfig(s);
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
      if (!confirm) {
        req->send(400, "application/json", "{\"error\":\"confirmation required\"}");
        return;
      }
      const uint8_t duty = obj["manual_test_duty"].as<uint8_t>();
      if (duty == 0) {
        req->send(400, "application/json", "{\"error\":\"manual_test_duty must be > 0\"}");
        return;
      }
      if (!canMgr_->sendMethManualTest(duty)) {
        const state::VehicleState snapshot = stateStore_->read();
        DynamicJsonDocument out(256);
        out["error"] = "manual test rejected by safety policy";
        out["reason"] = manualTestRejectReasonText(snapshot.meth_manual_test_reject_reason);
        out["cooldown_ms_remaining"] = snapshot.meth_manual_test_cooldown_ms_remaining;
        String body;
        serializeJson(out, body);
        req->send(409, "application/json", body);
        return;
      }
    }
    if ((obj["clear_faults"] | false) == true) canMgr_->sendMethClearFaults();

    stateStore_->mutate([&](state::VehicleState& s) {
      if (obj.containsKey("boost_trigger_kpa")) s.meth_boost_trigger_kpa = sanitizeMethBoostTriggerKpa(obj["boost_trigger_kpa"].as<uint8_t>());
      if (obj.containsKey("iat_threshold_c")) s.meth_iat_safety_threshold = sanitizeMethIatThresholdC(obj["iat_threshold_c"].as<int>());
      if (obj.containsKey("max_pump_duty")) s.meth_max_pump_duty = obj["max_pump_duty"].as<uint8_t>();
      sanitizeMethConfig(s);
    });

    settingsMgr_->updateFromState(stateStore_->read());
    settingsMgr_->save();

    DynamicJsonDocument out(256);
    out["ok"] = true;
    out["warning"] = kRatioWarning;
    String body;
    serializeJson(out, body);
    req->send(200, "application/json", body);
  });
  server_.addHandler(methHandler);

  auto raceControlHandler = new AsyncCallbackJsonWebHandler("/api/race/control", [this](AsyncWebServerRequest* req, JsonVariant& json) {
    if (!checkAuth(req)) return;
    JsonObject obj = json.as<JsonObject>();
    const String action = obj["action"] | "";

    if (action == "start_accel") {
      raceMgr_->startRun(state::RaceMode::ACCEL);
    } else if (action == "start_lap") {
      raceMgr_->startRun(state::RaceMode::LAP);
    } else if (action == "stop") {
      raceMgr_->stopRun();
    } else if (action == "reset") {
      raceMgr_->resetSession();
    } else if (action == "set_start_finish") {
      raceMgr_->setStartFinishPointFromCurrentFix();
    } else if (action == "mark_lap") {
      raceMgr_->markLap();
    } else {
      req->send(400, "application/json", "{\"error\":\"invalid action\"}");
      return;
    }
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server_.addHandler(raceControlHandler);

  auto tailHandler = new AsyncCallbackJsonWebHandler("/api/taillights", [this](AsyncWebServerRequest* req, JsonVariant& json) {
    if (!checkAuth(req)) return;
    JsonObject obj = json.as<JsonObject>();
    if (obj.containsKey("brightness")) canMgr_->sendTaillightBrightness(obj["brightness"].as<uint8_t>());
    if ((obj["clear_override"] | false) == true) canMgr_->clearTaillightOverride();
    if (obj.containsKey("anim_id")) {
      canMgr_->sendTaillightCustomAnimation(obj["anim_id"].as<uint8_t>(), obj["duration_ms"] | 500, obj["param0"] | 0, obj["param1"] | 0);
    }
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server_.addHandler(tailHandler);

  auto tachHandler = new AsyncCallbackJsonWebHandler("/api/tach", [this](AsyncWebServerRequest* req, JsonVariant& json) {
    if (!checkAuth(req)) return;
    JsonObject obj = json.as<JsonObject>();
    stateStore_->mutate([&](state::VehicleState& s) {
      if (obj.containsKey("pulses_per_rev10")) s.pulses_per_rev10 = obj["pulses_per_rev10"].as<uint8_t>();
      if (obj.containsKey("tach_scaling_mode")) s.tach_scaling_mode = obj["tach_scaling_mode"].as<uint8_t>();
    });
    settingsMgr_->updateFromState(stateStore_->read());
    settingsMgr_->save();
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server_.addHandler(tachHandler);

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
  DynamicJsonDocument doc(4096);
  const state::VehicleState s = stateStore_->read();

  doc["rpm"] = s.rpm;
  doc["vehicle_speed"] = s.speed;
  doc["gps_fix"] = s.gps_fix;
  doc["battery_voltage"] = s.battery_voltage;
  doc["cabin_temp"] = s.cabin_temp;
  doc["outside_temp"] = s.outside_temp;
  doc["engine_bay_temp"] = s.engine_bay_temp;
  doc["iat"] = s.intake_temp;
  doc["intercooler_temp"] = s.intercooler_temp;
  doc["meth_state"] = static_cast<uint8_t>(s.meth_state);
  doc["pump_duty"] = s.meth_pump_duty;
  doc["tank_level"] = s.meth_tank_level;
  doc["selected_meth_ratio"] = s.meth_selected_ratio_percent;
  doc["meth_manual_test_reject_reason"] = manualTestRejectReasonText(s.meth_manual_test_reject_reason);
  doc["meth_manual_test_cooldown_ms_remaining"] = s.meth_manual_test_cooldown_ms_remaining;
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
  DynamicJsonDocument doc(2048);
  const state::VehicleState s = stateStore_->read();
  doc["can_online"] = s.can_online;
  doc["can_rx_count"] = s.can_rx_count;
  doc["can_tx_count"] = s.can_tx_count;
  doc["can_bad_checksum_count"] = s.can_bad_checksum_count;
  doc["can_last_rx_id"] = s.can_last_rx_id;
  doc["can_last_tx_id"] = s.can_last_tx_id;
  doc["can_last_rx_ms"] = s.can_last_rx_ms;
  doc["can_last_tx_ms"] = s.can_last_tx_ms;
  doc["taillight_online"] = s.taillight_online;
  doc["meth_online"] = s.meth_online;
  doc["gps_stale"] = s.gps_stale;
  doc["fault_flags"] = s.fault_flags;
  doc["master_state"] = s.master_state;
  doc["uptime_ms"] = s.uptime_ms;
  doc["reset_reason"] = s.reset_reason;
  doc["brownout_reset_count"] = s.brownout_reset_count;
  doc["watchdog_reset_count"] = s.watchdog_reset_count;
  doc["meth_manual_test_reject_reason"] = manualTestRejectReasonText(s.meth_manual_test_reject_reason);
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
  DynamicJsonDocument doc(1024);
  doc["display_brightness"] = st.display_brightness;
  doc["night_mode"] = st.night_mode_enabled;
  doc["tach_pulses_per_rev10"] = st.tach_pulses_per_rev10;
  doc["tach_scaling_mode"] = st.tach_scaling_mode;
  doc["led_global_brightness"] = st.led_global_brightness;
  doc["led_theme"] = st.led_theme;
  doc["meth_ratio"] = st.meth_selected_ratio_percent;
  doc["meth_boost_trigger_kpa"] = st.meth_boost_trigger_kpa;
  doc["meth_iat_safety_threshold"] = st.meth_iat_safety_threshold;
  doc["meth_max_pump_duty"] = st.meth_max_pump_duty;
  doc["can_loss_behavior"] = st.meth_can_loss_behavior;
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
  doc["warning"] = kRatioWarning;

  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void WebServerManager::sendDiagnostics(AsyncWebServerRequest* request) const {
  if (!checkAuth(request)) return;
  const state::VehicleState s = stateStore_->read();
  DynamicJsonDocument doc(1536);
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
  doc["tach_input_frequency_hz"] = s.tach_input_frequency_hz;
  doc["tach_generated_frequency_hz"] = s.tach_generated_frequency_hz;
  doc["sd_mounted"] = s.sd_mounted;
  doc["sd_size_bytes"] = s.sd_size_bytes;
  doc["sd_used_bytes"] = s.sd_used_bytes;
  doc["current_log_file"] = s.current_log_file;
  doc["last_sd_write_status"] = s.last_sd_write_status;
  doc["sd_write_error_count"] = s.sd_write_error_count;
  doc["touch_online"] = s.touch_online;
  doc["ui_fps"] = s.ui_fps;
  doc["race_running"] = s.race_running;
  doc["race_quality_percent"] = s.race_quality_percent;
  doc["race_validation_flags"] = s.race_validation_flags;
  doc["brownout_reset_count"] = s.brownout_reset_count;
  doc["watchdog_reset_count"] = s.watchdog_reset_count;
  doc["meth_manual_test_reject_reason"] = manualTestRejectReasonText(s.meth_manual_test_reject_reason);
  doc["meth_manual_test_cooldown_ms_remaining"] = s.meth_manual_test_cooldown_ms_remaining;

  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void WebServerManager::sendCanStatus(AsyncWebServerRequest* request) const {
  if (!checkAuth(request)) return;
  request->send(200, "application/json", canStatusJson());
}

void WebServerManager::tick() {
  if (!stateStore_) return;
  ws_.cleanupClients();

  const uint32_t nowMs = millis();
  if ((nowMs - lastWsPushMs_) < 500) return;
  lastWsPushMs_ = nowMs;

  stateStore_->mutate([this](state::VehicleState& s) {
    s.web_connected_clients = ws_.count();
    s.wifi_connected = WiFi.status() == WL_CONNECTED;
  });

  const String payload = stateJson();
  ws_.textAll(payload);
}

}  // namespace web
