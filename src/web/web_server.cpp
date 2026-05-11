#include "web/web_server.h"

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <WiFi.h>

namespace web {

namespace {
constexpr const char* kRatioWarning = "Mixture ratio is user selected and not sensor verified.";

const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Foxbody Cabin Master</title><style>body{font-family:Arial;background:#111;color:#eee;margin:0}header{padding:12px;background:#1b1b1b}main{padding:12px}.card{border:1px solid #333;border-radius:8px;padding:10px;margin:8px 0}button{padding:8px 10px;margin:4px}input,select{margin:4px}</style></head>
<body><header><h2>Foxbody Cabin Master Dashboard</h2></header><main>
<div class='card'><h3>Live</h3><pre id='live'>connecting...</pre></div>
<div class='card'><h3>Water Meth</h3><p>Mixture ratio is user selected and not sensor verified.</p></div>
<div class='card'><h3>Pages</h3><p>Dashboard • Settings • LED • Water Meth • Taillights • Diagnostics</p></div>
<script>
const ws=new WebSocket(`ws://${location.host}/ws`);ws.onmessage=e=>{document.getElementById('live').textContent=e.data;};
</script></main></body></html>
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
}  // namespace

bool WebServerManager::begin(state::VehicleStateStore* stateStore, settings::SettingsManager* settingsMgr, canbus::CanManager* canMgr) {
  stateStore_ = stateStore;
  settingsMgr_ = settingsMgr;
  canMgr_ = canMgr;
  if (!stateStore_ || !settingsMgr_ || !canMgr_) return false;

  ws_.onEvent([this](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type, void*, uint8_t*, size_t) {
    if (type == WS_EVT_CONNECT || type == WS_EVT_DISCONNECT) {
      stateStore_->mutate([this](state::VehicleState& s) { s.web_connected_clients = ws_.count(); });
    }
  });
  server_.addHandler(&ws_);

  server_.on("/", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    req->send(200, "text/html", kIndexHtml);
  });

  server_.on("/api/state", HTTP_GET, [this](AsyncWebServerRequest* req) { sendState(req); });
  server_.on("/api/settings", HTTP_GET, [this](AsyncWebServerRequest* req) { sendSettings(req); });
  server_.on("/api/diagnostics", HTTP_GET, [this](AsyncWebServerRequest* req) { sendDiagnostics(req); });

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
      if (obj.containsKey("meth_ratio")) s.meth_selected_ratio_percent = obj["meth_ratio"].as<uint8_t>();
      if (obj.containsKey("meth_boost_trigger_kpa")) s.meth_boost_trigger_kpa = obj["meth_boost_trigger_kpa"].as<uint8_t>();
      if (obj.containsKey("meth_iat_safety_threshold")) s.meth_iat_safety_threshold = obj["meth_iat_safety_threshold"].as<int8_t>();
      if (obj.containsKey("meth_max_pump_duty")) s.meth_max_pump_duty = obj["meth_max_pump_duty"].as<uint8_t>();
      if (obj.containsKey("can_loss_behavior")) {
        s.meth_can_loss_behavior = static_cast<state::MethCanLossBehavior>(obj["can_loss_behavior"].as<uint8_t>());
      }
    });

    settings::AppSettings st = settingsMgr_->data();
    state::VehicleState snapshot = stateStore_->read();
    (void)st;
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
      const uint32_t color = parseColor(String(static_cast<const char*>(obj["color"] | "#00FF80")));

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
      stateStore_->mutate([&](state::VehicleState& s) { s.meth_selected_ratio_percent = obj["ratio"].as<uint8_t>(); });
    }
    if (obj.containsKey("armed")) {
      const bool armed = obj["armed"].as<bool>();
      stateStore_->mutate([&](state::VehicleState& s) { s.meth_desired_armed = armed; });
      canMgr_->sendMethArm(armed);
    }
    if (obj.containsKey("manual_test_duty")) {
      if (!confirm) {
        req->send(400, "application/json", "{\"error\":\"confirmation required\"}");
        return;
      }
      canMgr_->sendMethManualTest(obj["manual_test_duty"].as<uint8_t>());
    }
    if ((obj["clear_faults"] | false) == true) canMgr_->sendMethClearFaults();

    stateStore_->mutate([&](state::VehicleState& s) {
      if (obj.containsKey("boost_trigger_kpa")) s.meth_boost_trigger_kpa = obj["boost_trigger_kpa"].as<uint8_t>();
      if (obj.containsKey("iat_threshold_c")) s.meth_iat_safety_threshold = obj["iat_threshold_c"].as<int8_t>();
      if (obj.containsKey("max_pump_duty")) s.meth_max_pump_duty = obj["max_pump_duty"].as<uint8_t>();
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
  if (!request->hasHeader("X-Auth-Token")) {
    request->send(401, "application/json", "{\"error\":\"unauthorized\"}");
    return false;
  }
  return request->getHeader("X-Auth-Token")->value() == String(pass);
}

String WebServerManager::stateJson() const {
  DynamicJsonDocument doc(2048);
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
  doc["can_node_online"] = s.can_online;
  doc["taillight_state_left"] = s.taillight_left_state;
  doc["taillight_state_right"] = s.taillight_right_state;
  doc["faults"] = s.fault_flags;
  doc["warning"] = kRatioWarning;
  doc["led1_color"] = toHexColor(s.led_channel_1_color);
  doc["led2_color"] = toHexColor(s.led_channel_2_color);
  doc["led3_color"] = toHexColor(s.led_channel_3_color);

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

  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
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
