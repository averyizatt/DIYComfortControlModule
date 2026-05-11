#include <Arduino.h>

#include "core/Application.hpp"

ccm::core::Application app;

void setup() {
  Serial.begin(115200);
  delay(50);
  app.begin();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
