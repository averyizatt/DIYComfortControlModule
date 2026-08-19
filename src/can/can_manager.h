#pragma once

#include <Arduino.h>

#include "can/can_protocol.h"
#include "can/MicroSquirtProtocol.hpp"
#include "meth/meth_config.h"
#include "state/vehicle_state.h"

namespace storage { class TelemetryRecorder; }

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

  // External knock module helpers. Commands share 0x301 with meth commands but
  // use the reserved 0x40..0x4A command range.
  bool sendKnockCommand(uint8_t command, uint8_t value = 0);
  bool requestKnockConfig();
  void attachTelemetryRecorder(storage::TelemetryRecorder* recorder) { recorder_ = recorder; }

 private:
  bool sendFrame(const can_protocol::CanFrame& frame);
  bool receiveFrame(can_protocol::CanFrame& frame);
  void dispatchFrame(const can_protocol::CanFrame& frame, uint32_t nowMs);
  void sendScheduledFrames(uint32_t nowMs);
  void updateTimeouts(uint32_t nowMs);
  void runDemoGenerator(uint32_t nowMs);
  bool dispatchMicroSquirt(const can_protocol::CanFrame& frame, uint32_t nowMs);

  bool hwCanReady_ = false;
  uint32_t canStartMs_ = 0;        // time begin() was called; used for startup grace period
  uint32_t lastHeartbeatTxMs_ = 0;
  uint32_t lastTachTxMs_ = 0;
  uint32_t lastGpsTxMs_ = 0;
  uint32_t lastDemoMs_ = 0;
  uint32_t manualTestStartMs_ = 0;
  uint32_t lastManualTestStopMs_ = 0;
  uint32_t lastMethConfigTxMs_ = 0;
  uint32_t lastEngineRuntimeTxMs_ = 0;
  uint32_t lastKnockConfigRequestMs_ = 0;
  uint32_t lastCanErrCheckMs_ = 0;   // last MCP2515 EFLG read
  uint32_t lastCanStatusLogMs_ = 0;
  uint16_t lastCanStatusSignature_ = 0xFFFFU;
  uint32_t lastCanRxPollMs_ = 0;     // fallback poll when MCP2515 INT is idle
  uint32_t canNoAckBackoffUntilMs_ = 0;
  uint32_t lastCanNoAckLogMs_ = 0;
  bool     canRxWarnSent_ = false;   // one-shot "silent bus" warning
  bool     canNoAckBackoffLogged_ = false;
  storage::TelemetryRecorder* recorder_ = nullptr;
  microsquirt::LiveData microsquirtData_{};
};

}  // namespace canbus
