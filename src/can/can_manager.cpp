#include "can/can_manager.h"

#if __has_include(<driver/twai.h>)
#include <driver/twai.h>
#define CCM_HAS_TWAI 1
#else
#define CCM_HAS_TWAI 0
#endif

namespace canbus {

namespace {
constexpr uint32_t kTaillightTimeoutMs = 500;
constexpr uint32_t kMethTimeoutMs = 250;
constexpr uint32_t kGpsStaleTimeoutMs = 1000;
constexpr uint32_t kManualTestTimeoutMs = 5000;
constexpr uint32_t kMethConfigBroadcastIntervalMs = 500;

void packMasterHeartbeat(const state::VehicleState& s, can_protocol::CanFrame& out) {
  out.id = can_protocol::ID_MASTER_HEARTBEAT;
  out.dlc = 8;
  out.data[0] = s.master_state;                              // state
  out.data[1] = s.ui_page;                                   // UI page
  out.data[2] = s.input_flags;                               // input flags
  out.data[3] = can_protocol::tempToOffset40(static_cast<int>(s.cabin_temp));
  out.data[4] = can_protocol::tempToOffset40(static_cast<int>(s.outside_temp));
  out.data[5] = can_protocol::voltsTo10(s.battery_voltage);  // volts*10
  out.data[6] = static_cast<uint8_t>(s.fault_flags & 0xFF);
  out.data[7] = static_cast<uint8_t>((s.uptime_ms / 1000UL) & 0xFF);
}

void packTachState(const state::VehicleState& s, can_protocol::CanFrame& out) {
  out.id = can_protocol::ID_TACH_RPM_STATE;
  out.dlc = 8;
  can_protocol::encodeU16BE(s.rpm, out.data[0], out.data[1]);
  can_protocol::encodeU16BE(s.generated_tach_hz10, out.data[2], out.data[3]);
  out.data[4] = s.tach_source;
  out.data[5] = s.tach_status_flags;
  out.data[6] = s.pulses_per_rev10;
  out.data[7] = 0;
}

void packGpsState(const state::VehicleState& s, can_protocol::CanFrame& out) {
  out.id = can_protocol::ID_GPS_STATE;
  out.dlc = 8;
  can_protocol::encodeU16BE(static_cast<uint16_t>(s.speed * 10.0f), out.data[0], out.data[1]);
  const uint16_t altitudeRaw = static_cast<uint16_t>(s.gps_altitude_m);
  can_protocol::encodeU16BE(altitudeRaw, out.data[2], out.data[3]);
  out.data[4] = s.gps_satellites;
  out.data[5] = s.gps_fix_type;
  out.data[6] = s.gps_status_flags;
  out.data[7] = 0;
}

can_protocol::CanFrame packMethConfigState(const state::VehicleState& s) {
  const meth::DesiredConfig desired = meth::fromVehicleState(s);
  return meth::toCanBroadcast(desired);
}
}  // namespace

bool CanManager::begin(bool tryHardwareCan) {
  hwCanReady_ = false;

#if CCM_HAS_TWAI
  if (tryHardwareCan) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
      hwCanReady_ = true;
    }
  }
#else
  (void)tryHardwareCan;
#endif

#if defined(DEMO_MODE) && (DEMO_MODE == 1)
  if (!hwCanReady_) {
    // Demo mode intentionally runs without hardware CAN.
    state::g_vehicle_state.mutate([](state::VehicleState& s) { s.can_online = true; });
    return true;
  }
#endif

  state::g_vehicle_state.mutate([this](state::VehicleState& s) { s.can_online = hwCanReady_; });
  return hwCanReady_;
}

void CanManager::tick() {
  const uint32_t nowMs = millis();
  bool mustStopManualTest = false;

  can_protocol::CanFrame frame{};
  while (receiveFrame(frame)) {
    dispatchFrame(frame, nowMs);
  }

#if defined(DEMO_MODE) && (DEMO_MODE == 1)
  if (!hwCanReady_) {
    runDemoGenerator(nowMs);
  }
#endif

  sendScheduledFrames(nowMs);
  updateTimeouts(nowMs);

  // Safety: stop manual pump test on timeout or Back input.
  state::g_vehicle_state.mutate([this, nowMs, &mustStopManualTest](state::VehicleState& s) {
    const bool backPressed = (s.input_flags & can_protocol::input_flag::BACK) != 0;
    const bool timedOut = s.manual_test_running && ((nowMs - manualTestStartMs_) > kManualTestTimeoutMs);
    if ((s.manual_test_running && backPressed) || timedOut) {
      s.meth_pump_duty = 0;
      s.meth_state = state::MethState::OFF;
      s.manual_test_running = false;
      mustStopManualTest = true;
    }

    s.uptime_ms = nowMs;
  });
  if (mustStopManualTest) {
    sendMethStopManualTest();
  }
}

bool CanManager::sendTaillightBrightness(uint8_t brightness) {
  return sendFrame(can_protocol::packTaillightBrightness(brightness));
}

bool CanManager::sendTaillightOverride(uint8_t leftState, uint8_t rightState) {
  return sendFrame(can_protocol::packTaillightOverride(leftState, rightState));
}

bool CanManager::clearTaillightOverride() {
  return sendFrame(can_protocol::packTaillightClearOverride());
}

bool CanManager::sendTaillightCustomAnimation(uint8_t animId, uint16_t durationMs, uint8_t param0, uint8_t param1) {
  return sendFrame(can_protocol::packTaillightCustomAnimation(animId, durationMs, param0, param1));
}

bool CanManager::sendMethArm(bool armed) {
  if (!armed) {
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.meth_state = state::MethState::OFF;
      s.meth_pump_duty = 0;
    });
  }
  return sendFrame(can_protocol::packMethArm(armed));
}

bool CanManager::sendMethManualTest(uint8_t duty) {
  manualTestStartMs_ = millis();
  state::g_vehicle_state.mutate([duty](state::VehicleState& s) {
    s.meth_state = state::MethState::TEST;
    s.meth_pump_duty = duty;
    s.manual_test_running = true;
  });
  return sendFrame(can_protocol::packMethManualTest(duty));
}

bool CanManager::sendMethStopManualTest() {
  state::g_vehicle_state.mutate([](state::VehicleState& s) {
    s.meth_pump_duty = 0;
    s.manual_test_running = false;
    if (s.meth_state == state::MethState::TEST) {
      s.meth_state = state::MethState::OFF;
    }
  });
  return sendFrame(can_protocol::packMethStopManualTest());
}

bool CanManager::sendMethSetBoostThreshold(uint8_t kpa) {
  return sendFrame(can_protocol::packMethSetBoostThreshold(kpa));
}

bool CanManager::sendMethSetIatThreshold(int8_t tempC) {
  return sendFrame(can_protocol::packMethSetIatThreshold(tempC));
}

bool CanManager::sendMethClearFaults() {
  return sendFrame(can_protocol::packMethClearFaults());
}

bool CanManager::sendMethConfigBroadcast() {
  state::g_vehicle_state.mutate([&](state::VehicleState& live) { live.meth_config_version++; });
  state::VehicleState s = state::g_vehicle_state.read();
  return sendFrame(packMethConfigState(s));
}

bool CanManager::sendFrame(const can_protocol::CanFrame& frame) {
  bool sent = false;
#if CCM_HAS_TWAI
  if (hwCanReady_) {
    twai_message_t tx{};
    tx.identifier = frame.id;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data_length_code = frame.dlc;
    for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
      tx.data[i] = frame.data[i];
    }
    sent = twai_transmit(&tx, 0) == ESP_OK;
  } else {
    sent = true;
  }
#else
  sent = true;
#endif

  if (sent) {
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.can_tx_count++;
      s.can_last_tx_id = frame.id;
      s.can_last_tx_ms = millis();
    });
  }
  return sent;
}

bool CanManager::receiveFrame(can_protocol::CanFrame& frame) {
#if CCM_HAS_TWAI
  if (hwCanReady_) {
    twai_message_t rx{};
    if (twai_receive(&rx, 0) != ESP_OK) {
      return false;
    }
    frame.id = static_cast<uint16_t>(rx.identifier & 0x7FFU);
    frame.dlc = rx.data_length_code;
    for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
      frame.data[i] = rx.data[i];
    }
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.can_rx_count++;
      s.can_last_rx_id = frame.id;
      s.can_last_rx_ms = millis();
    });
    return true;
  }
#endif
  return false;
}

void CanManager::dispatchFrame(const can_protocol::CanFrame& frame, uint32_t nowMs) {
  if (frame.id == can_protocol::ID_TAILLIGHT_STATE) {
    can_protocol::TaillightState msg{};
    if (!can_protocol::unpackTaillightState(frame, msg)) return;

    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.taillight_left_state = msg.left_state;
      s.taillight_right_state = msg.right_state;
      s.taillight_input_flags = msg.input_flags;
      s.taillight_brightness = msg.brightness;
      s.taillight_die_temp_c = msg.die_temp_c;
      s.taillight_thermal_derate = msg.thermal_derate;
      s.last_taillight_ms = nowMs;
      s.taillight_online = true;
      s.can_online = true;
    });
    return;
  }

  if (frame.id == can_protocol::ID_TAILLIGHT_FAULT) {
    can_protocol::TaillightFault msg{};
    if (!can_protocol::unpackTaillightFault(frame, msg)) return;

    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.fault_flags |= 0x0001;
      if (msg.severity >= static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL)) {
        s.master_state = static_cast<uint8_t>(can_protocol::MasterState::FAULT);
      }
    });
    return;
  }

  if (frame.id == can_protocol::ID_ENGINE_METH_STATE) {
    can_protocol::EngineMethState msg{};
    if (!can_protocol::unpackEngineMethState(frame, msg)) return;

    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.meth_state = static_cast<state::MethState>(msg.meth_state);
      s.meth_pump_duty = msg.pump_duty;
      s.meth_tank_level = msg.tank_level;
      s.meth_flow_status = msg.flow_status;
      s.boost_kpa = msg.boost_kpa;
      s.intake_temp = msg.iat_c;
      s.engine_bay_temp = msg.engine_bay_c;
      s.fault_flags = static_cast<uint16_t>((s.fault_flags & 0xFF00U) | msg.fault_flags);
      s.last_meth_ms = nowMs;
      s.meth_online = true;
      s.can_online = true;

      // Safety latch: if fault, force local view of pump to zero.
      if (s.meth_state == state::MethState::FAULT) {
        s.meth_pump_duty = 0;
      }
    });
    return;
  }

  if (frame.id == can_protocol::ID_ENGINE_METH_FAULT) {
    can_protocol::EngineMethFault msg{};
    if (!can_protocol::unpackEngineMethFault(frame, msg)) return;

    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.fault_flags |= 0x0010;
      if (msg.severity >= static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL)) {
        s.master_state = static_cast<uint8_t>(can_protocol::MasterState::FAULT);
        s.meth_state = state::MethState::FAULT;
        s.meth_pump_duty = 0;
      }
    });
    return;
  }

  if (frame.id == can_protocol::ID_MASTER_COMMAND && frame.dlc >= 1) {
    const uint8_t cmd = frame.data[0];
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      switch (cmd) {
        case can_protocol::master_command::SET_UI_PAGE:
          if (frame.dlc >= 2) s.ui_page = frame.data[1];
          break;
        case can_protocol::master_command::SET_BRIGHTNESS:
          break;  // Handled by UI/display task.
        case can_protocol::master_command::TRIGGER_TACH_SWEEP:
          s.tach_status_flags |= (1U << 2);
          break;
        case can_protocol::master_command::SET_DRIVE_MODE:
          break;
        default:
          break;
      }
    });
  }

  if (frame.id == can_protocol::ID_METH_CONFIG_REQUEST) {
    can_protocol::MethConfigRequest req{};
    if (!can_protocol::unpackMethConfigRequest(frame, req)) return;
    (void)req;
    lastMethConfigTxMs_ = 0;  // Force immediate rebroadcast on next scheduler tick.
    return;
  }

  if (frame.id == can_protocol::ID_METH_CONFIG_ACK) {
    can_protocol::MethConfigAck ack{};
    if (!can_protocol::unpackMethConfigAck(frame, ack)) return;
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.meth_config_version = ack.accepted_version;
      if (ack.active_ratio_percent <= 100 || ack.active_ratio_percent == 255) {
        s.meth_selected_ratio_percent = ack.active_ratio_percent;
      }
      if (ack.status == 0) {
        s.meth_online = true;
      } else if (ack.status == 3) {
        s.meth_state = state::MethState::FAULT;
        s.meth_pump_duty = 0;
      }
    });
    return;
  }
}

void CanManager::sendScheduledFrames(uint32_t nowMs) {
  state::VehicleState snapshot = state::g_vehicle_state.read();

  if ((nowMs - lastHeartbeatTxMs_) >= 100) {
    can_protocol::CanFrame hb{};
    packMasterHeartbeat(snapshot, hb);
    sendFrame(hb);
    lastHeartbeatTxMs_ = nowMs;
  }

  if ((nowMs - lastTachTxMs_) >= 20) {
    can_protocol::CanFrame tach{};
    packTachState(snapshot, tach);
    sendFrame(tach);
    lastTachTxMs_ = nowMs;
  }

  if ((nowMs - lastGpsTxMs_) >= 250) {
    can_protocol::CanFrame gps{};
    packGpsState(snapshot, gps);
    sendFrame(gps);
    lastGpsTxMs_ = nowMs;
  }

  if ((nowMs - lastMethConfigTxMs_) >= kMethConfigBroadcastIntervalMs) {
    sendMethConfigBroadcast();
    lastMethConfigTxMs_ = nowMs;
  }
}

void CanManager::updateTimeouts(uint32_t nowMs) {
  state::g_vehicle_state.mutate([&](state::VehicleState& s) {
    s.taillight_online = (nowMs - s.last_taillight_ms) <= kTaillightTimeoutMs;
    s.meth_online = (nowMs - s.last_meth_ms) <= kMethTimeoutMs;
    s.gps_stale = (nowMs - s.last_gps_ms) > kGpsStaleTimeoutMs;

    if (!s.taillight_online || !s.meth_online) {
      s.fault_flags |= 0x0080;
    }

    // Fail-safe local behavior view: if meth module offline, force OFF and zero duty.
    if (!s.meth_online) {
      if (s.meth_can_loss_behavior == state::MethCanLossBehavior::DISARM) {
        s.meth_state = state::MethState::OFF;
        s.meth_pump_duty = 0;
        s.manual_test_running = false;
      }
    }
  });
}

void CanManager::runDemoGenerator(uint32_t nowMs) {
  if ((nowMs - lastDemoMs_) < 50) return;
  lastDemoMs_ = nowMs;

  const float t = nowMs / 1000.0f;
  state::g_vehicle_state.mutate([&](state::VehicleState& s) {
    s.can_online = true;
    s.taillight_online = true;
    s.meth_online = true;

    s.rpm = static_cast<uint16_t>(1800 + 1200 * (0.5f + 0.5f * sinf(t * 1.8f)));
    // generated_tach_hz10 = (RPM / 15) * 10 => preserve scaling in 0.1Hz units.
    s.generated_tach_hz10 = static_cast<uint16_t>((s.rpm * 10U) / 15U);
    s.raw_tach_hz10 = s.generated_tach_hz10;
    s.tach_source = static_cast<uint8_t>(can_protocol::TachSource::DEMO);
    s.tach_status_flags = 0x03;

    s.speed = 45.0f + 20.0f * (0.5f + 0.5f * sinf(t * 0.35f));
    s.gps_satellites = static_cast<uint8_t>(8 + (static_cast<int>(t) % 5));
    s.gps_fix = true;
    s.gps_fix_type = 2;
    s.last_gps_ms = nowMs;
    s.gps_altitude_m = 128;
    s.gps_latitude = 40.7608 + 0.0002 * sinf(t * 0.05f);
    s.gps_longitude = -111.8910 + 0.0002 * cosf(t * 0.05f);

    s.taillight_left_state = static_cast<uint8_t>(static_cast<int>(t * 2) % 4);
    s.taillight_right_state = static_cast<uint8_t>((static_cast<int>(t * 2) + 1) % 4);
    s.taillight_input_flags = static_cast<uint8_t>((static_cast<int>(t) & 0x0F));
    s.taillight_brightness = 180;
    s.taillight_die_temp_c = 44;
    s.taillight_thermal_derate = 10;
    s.last_taillight_ms = nowMs;

    s.meth_state = (static_cast<int>(t) % 10 > 6) ? state::MethState::SPRAYING : state::MethState::ARMED;
    s.meth_pump_duty = (s.meth_state == state::MethState::SPRAYING) ? 140 : 0;
    s.meth_tank_level = static_cast<uint8_t>(60 + 20 * sinf(t * 0.07f));
    s.meth_flow_status = (s.meth_state == state::MethState::SPRAYING) ? 1 : 0;
    s.boost_kpa = static_cast<uint8_t>(95 + 45 * sinf(t * 0.9f));
    s.intake_temp = 26.0f + 4.0f * sinf(t * 0.4f);
    s.engine_bay_temp = 50.0f + 6.0f * sinf(t * 0.25f);
    s.last_meth_ms = nowMs;

    s.cabin_temp = 24.0f + 2.0f * sinf(t * 0.1f);
    s.outside_temp = 21.0f + 1.0f * sinf(t * 0.06f);
    s.coolant_temp = 84.0f + 3.0f * sinf(t * 0.2f);
    s.intercooler_temp = 32.0f + 2.0f * sinf(t * 0.2f);
    s.battery_voltage = 12.8f;
    s.heap_free_bytes = ESP.getFreeHeap();
    s.esp_die_temp_c = static_cast<int8_t>(temperatureRead());
    s.tach_input_frequency_hz = s.raw_tach_hz10 / 10.0f;
    s.tach_generated_frequency_hz = s.generated_tach_hz10 / 10.0f;
  });
}

}  // namespace canbus
