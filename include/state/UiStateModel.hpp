#pragma once

#include "state/StateHelpers.hpp"

namespace state {

struct UiStateData {
  bool meth_available = false;
  bool meth_active = false;
  bool meth_faulted = false;
  bool knock_alert = false;
  bool can_fault = false;
  bool analog_fault = false;
  uint8_t pump_duty = 0;
  uint8_t tank_level = 0;
  float intake_temp_c = 0.0f;
  float meth_pressure_psi = 0.0f;
};

inline UiStateData buildUiStateData(const VehicleState& s) {
  UiStateData data{};
  data.meth_available = s.meth_online && methSafetyInputsValid(s);
  data.meth_active = s.meth_state == MethState::SPRAYING || s.manual_test_running;
  data.meth_faulted = hasCriticalMethFault(s);
  data.knock_alert = s.knock_warning_active || s.knock_critical_active;
  data.can_fault = !s.can_online || !s.meth_online || !s.taillight_online;
  data.analog_fault = s.analog_sensor_fault_flags != 0U;
  data.pump_duty = s.meth_pump_duty;
  data.tank_level = s.meth_tank_level;
  data.intake_temp_c = s.intake_temp;
  data.meth_pressure_psi = s.meth_pressure_psi;
  return data;
}

}  // namespace state
