#pragma once

#include <Arduino.h>

#include "can/can_protocol.h"
#include "meth/meth_config.h"
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
  bool sendTaillightMode(uint8_t mode);
  bool sendTaillightShowOption(uint8_t option);

  // Water meth helpers
  bool sendMethArm(bool armed);
  bool sendMethManualTest(uint8_t duty);
  bool sendMethStopManualTest();
  bool sendMethClearFaults();
  bool sendMethConfigBroadcast();

 private:
  bool sendFrame(const can_protocol::CanFrame& frame);
  bool receiveFrame(can_protocol::CanFrame& frame);
  void dispatchFrame(const can_protocol::CanFrame& frame, uint32_t nowMs);
  void sendScheduledFrames(uint32_t nowMs);
  void updateTimeouts(uint32_t nowMs);
  void runDemoGenerator(uint32_t nowMs);

  bool hwCanReady_ = false;
  uint32_t canStartMs_ = 0;        // time begin() was called; used for startup grace period
  uint32_t lastHeartbeatTxMs_ = 0;
  uint32_t lastTachTxMs_ = 0;
  uint32_t lastGpsTxMs_ = 0;
  uint32_t lastDemoMs_ = 0;
  uint32_t manualTestStartMs_ = 0;
  uint32_t lastManualTestStopMs_ = 0;
  uint32_t lastMethConfigTxMs_ = 0;
};

}  // namespace canbus
