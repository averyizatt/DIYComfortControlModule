#pragma once

#include <Arduino.h>

#include "can/can_protocol.h"
#include "state/vehicle_state.h"

namespace canbus {

class CanManager {
 public:
  bool begin(bool tryHardwareCan = true);
  void tick();

  // Taillight helpers
  bool sendTaillightBrightness(uint8_t brightness);
  bool sendTaillightOverride(uint8_t leftState, uint8_t rightState);
  bool clearTaillightOverride();
  bool sendTaillightCustomAnimation(uint8_t animId, uint16_t durationMs, uint8_t param0, uint8_t param1);

  // Water meth helpers
  bool sendMethArm(bool armed);
  bool sendMethManualTest(uint8_t duty);
  bool sendMethStopManualTest();
  bool sendMethSetBoostThreshold(uint8_t kpa);
  bool sendMethSetIatThreshold(int8_t tempC);
  bool sendMethClearFaults();

 private:
  bool sendFrame(const can_protocol::CanFrame& frame);
  bool receiveFrame(can_protocol::CanFrame& frame);
  void dispatchFrame(const can_protocol::CanFrame& frame, uint32_t nowMs);
  void sendScheduledFrames(uint32_t nowMs);
  void updateTimeouts(uint32_t nowMs);
  void runDemoGenerator(uint32_t nowMs);

  bool hwCanReady_ = false;
  uint32_t lastHeartbeatTxMs_ = 0;
  uint32_t lastTachTxMs_ = 0;
  uint32_t lastGpsTxMs_ = 0;
  uint32_t lastDemoMs_ = 0;
  uint32_t manualTestStartMs_ = 0;
  bool manualTestRunning_ = false;
};

}  // namespace canbus
