#pragma once

#include "state/VehicleStateData.hpp"
#include "web/WebApiLogic.hpp"

namespace web {

struct StateApiData {
  uint16_t rpm = 0;
  float battery_voltage = 0.0f;
  uint8_t meth_state = 0;
  uint8_t pump_duty = 0;
  bool knock_enabled = false;
  uint16_t analog_sensor_fault_flags = 0;
  const char* meth_manual_test_reject_reason = "none";
};

inline StateApiData buildStateApiData(const state::VehicleState& s) {
  StateApiData data{};
  data.rpm = s.rpm;
  data.battery_voltage = s.battery_voltage;
  data.meth_state = static_cast<uint8_t>(s.meth_state);
  data.pump_duty = s.meth_pump_duty;
  data.knock_enabled = s.knock_enabled;
  data.analog_sensor_fault_flags = s.analog_sensor_fault_flags;
  data.meth_manual_test_reject_reason = manualTestRejectReasonText(s.meth_manual_test_reject_reason);
  return data;
}

}  // namespace web
