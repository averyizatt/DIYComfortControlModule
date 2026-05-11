#include <Arduino.h>

#include "can/can_manager.h"
#include "state/vehicle_state.h"

namespace {
canbus::CanManager g_can;

void canTask(void*) {
  while (true) {
    g_can.tick();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void heartbeatTask(void*) {
  while (true) {
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.input_flags = 0;  // Button manager will set these bits.
      if (s.fault_flags != 0) {
        s.master_state = 2;  // WARN
      } else {
        s.master_state = 1;  // RUN
      }
    });
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(50);

  state::g_vehicle_state.begin();
  g_can.begin(true);

  xTaskCreatePinnedToCore(canTask, "can_task", 6144, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(heartbeatTask, "hb_task", 3072, nullptr, 1, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
