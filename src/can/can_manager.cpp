#include "can/can_manager.h"
#include "pin_map.h"

#include "can/CanFrameBuilders.hpp"
#include "hal/SharedSpiBus.hpp"
#include "meth/MethSafetyLogic.hpp"
#include "state/StateHelpers.hpp"
#include "storage/telemetry_recorder.h"

#include <cmath>

#include <SPI.h>

#ifndef CCM_CAN_TRANSPORT_SPI
#define CCM_CAN_TRANSPORT_SPI 0
#endif

#ifndef CCM_CAN_TRANSPORT_TWAI
#define CCM_CAN_TRANSPORT_TWAI 1
#endif

#if CCM_CAN_TRANSPORT_SPI && __has_include(<mcp2515.h>)
#include <mcp2515.h>
#define CCM_HAS_SPI_CAN 1
#else
#define CCM_HAS_SPI_CAN 0
#endif

#if CCM_CAN_TRANSPORT_TWAI && __has_include(<driver/twai.h>)
#include <driver/twai.h>
#define CCM_HAS_TWAI 1
#else
#define CCM_HAS_TWAI 0
#endif

#ifndef CCM_CAN_SPI_HZ
#define CCM_CAN_SPI_HZ 1000000UL
#endif

#ifndef CCM_CAN_MCP_CLOCK_MHZ
#define CCM_CAN_MCP_CLOCK_MHZ 16
#endif

#ifndef CCM_CAN_RX_INT_GATED
#define CCM_CAN_RX_INT_GATED 1
#endif

#ifndef CCM_CAN_RX_FALLBACK_POLL_MS
#define CCM_CAN_RX_FALLBACK_POLL_MS 50
#endif

#ifndef CCM_CAN_RX_MAX_FRAMES_PER_TICK
#define CCM_CAN_RX_MAX_FRAMES_PER_TICK 4
#endif

#ifndef CCM_CAN_HEARTBEAT_TX_MS
#define CCM_CAN_HEARTBEAT_TX_MS 250
#endif

#ifndef CCM_CAN_TACH_TX_MS
#define CCM_CAN_TACH_TX_MS 50
#endif

#ifndef CCM_CAN_GPS_TX_MS
#define CCM_CAN_GPS_TX_MS 500
#endif

#ifndef CCM_CAN_ENGINE_RUNTIME_TX_MS
#define CCM_CAN_ENGINE_RUNTIME_TX_MS 50
#endif

#ifndef CCM_CAN_METH_CONFIG_TX_MS
#define CCM_CAN_METH_CONFIG_TX_MS 1000
#endif

#ifndef CCM_CAN_KNOCK_CONFIG_REQUEST_MS
#define CCM_CAN_KNOCK_CONFIG_REQUEST_MS 1000
#endif

namespace canbus {

namespace {
constexpr float kKpaToPsi = 0.1450377f;
constexpr uint32_t kTaillightTimeoutMs = 500;
constexpr uint32_t kMethTimeoutMs = 250;
constexpr uint32_t kKnockTimeoutMs = 500;
constexpr uint32_t kGpsStaleTimeoutMs = 5000;
constexpr uint32_t kManualTestTimeoutMs = 5000;
// Cooldown to prevent rapid repeated manual pump tests from UI/API retries.
constexpr uint32_t kManualTestCooldownMs = 3000;
constexpr uint32_t kMethConfigBroadcastIntervalMs =
    (CCM_CAN_METH_CONFIG_TX_MS < 500) ? 500U : static_cast<uint32_t>(CCM_CAN_METH_CONFIG_TX_MS);
constexpr uint32_t kCanHeartbeatTxMs =
    (CCM_CAN_HEARTBEAT_TX_MS < 100) ? 100U : static_cast<uint32_t>(CCM_CAN_HEARTBEAT_TX_MS);
constexpr uint32_t kCanTachTxMs =
    (CCM_CAN_TACH_TX_MS < 20) ? 20U : static_cast<uint32_t>(CCM_CAN_TACH_TX_MS);
constexpr uint32_t kCanGpsTxMs =
    (CCM_CAN_GPS_TX_MS < 250) ? 250U : static_cast<uint32_t>(CCM_CAN_GPS_TX_MS);
constexpr uint32_t kCanEngineRuntimeTxMs =
    (CCM_CAN_ENGINE_RUNTIME_TX_MS < 20) ? 20U : static_cast<uint32_t>(CCM_CAN_ENGINE_RUNTIME_TX_MS);
constexpr uint32_t kCanKnockConfigRequestMs =
    (CCM_CAN_KNOCK_CONFIG_REQUEST_MS < 250) ? 250U : static_cast<uint32_t>(CCM_CAN_KNOCK_CONFIG_REQUEST_MS);
constexpr uint32_t kCanNoAckBackoffMs = 10000;
constexpr uint32_t kCanNoAckLogMs = 30000;
constexpr uint32_t kCanTxQuietGraceMs = 5000;
#if CCM_CAN_MCP_CLOCK_MHZ == 8
constexpr auto kCanMcpClock = MCP_8MHZ;
constexpr uint8_t kCanMcpClockMhz = 8;
#elif CCM_CAN_MCP_CLOCK_MHZ == 20
constexpr auto kCanMcpClock = MCP_20MHZ;
constexpr uint8_t kCanMcpClockMhz = 20;
#else
constexpr auto kCanMcpClock = MCP_16MHZ;
constexpr uint8_t kCanMcpClockMhz = 16;
#endif
constexpr uint8_t kCanRxMaxFramesPerTick =
    (CCM_CAN_RX_MAX_FRAMES_PER_TICK < 1) ? 1U : static_cast<uint8_t>(CCM_CAN_RX_MAX_FRAMES_PER_TICK);
constexpr uint32_t kCanRxFallbackPollMs =
    (CCM_CAN_RX_FALLBACK_POLL_MS < 10) ? 10U : static_cast<uint32_t>(CCM_CAN_RX_FALLBACK_POLL_MS);
constexpr bool kCanRxIntGated = CCM_CAN_RX_INT_GATED != 0;
constexpr uint16_t kFaultTaillight = 0x0001;
constexpr uint16_t kFaultMeth = 0x0010;
constexpr uint16_t kFaultModuleOffline = 0x0080;
constexpr uint16_t kFaultKnockCritical = 0x0400;

namespace taillight_animation {
constexpr uint8_t SEQUENTIAL_ID = 1;
constexpr uint16_t SEQUENTIAL_DURATION_MS = 500;
constexpr uint8_t SHOW_ID = 2;
constexpr uint16_t SHOW_DURATION_MS = 800;
constexpr uint8_t DEMO_ID = 3;
constexpr uint16_t DEMO_DURATION_MS = 1200;
}  // namespace taillight_animation

namespace meth_manual_test_reject_reason {
constexpr uint8_t NONE = 0;
constexpr uint8_t OFFLINE = 1;
constexpr uint8_t FAULT = 2;
constexpr uint8_t COOLDOWN = 3;
constexpr uint8_t DUTY_ZERO = 4;
constexpr uint8_t DUTY_OVER_MAX = 5;
}  // namespace meth_manual_test_reject_reason

#if CCM_HAS_SPI_CAN
MCP2515 g_mcp2515(pins::kCanSpiCs, CCM_CAN_SPI_HZ, &SPI);
bool g_spiCanOnline = false;

bool configureSpiCanAtClock(CAN_CLOCK clock, uint8_t clockMhz) {
  Serial.printf("[CAN] try clock=%uMHz\n", static_cast<unsigned>(clockMhz));
  const MCP2515::ERROR bitrateErr = g_mcp2515.setBitrate(CAN_500KBPS, clock);
  if (bitrateErr != MCP2515::ERROR_OK) {
    Serial.printf("[CAN] setBitrate failed clock=%uMHz err=%d\n",
                  static_cast<unsigned>(clockMhz),
                  static_cast<int>(bitrateErr));
    return false;
  }

  const MCP2515::ERROR modeErr = g_mcp2515.setNormalMode();
  if (modeErr != MCP2515::ERROR_OK) {
    Serial.printf("[CAN] setNormalMode failed clock=%uMHz err=%d\n",
                  static_cast<unsigned>(clockMhz),
                  static_cast<int>(modeErr));
    return false;
  }

  Serial.printf("[CAN] MCP2515 online clock=%uMHz normal\n",
                static_cast<unsigned>(clockMhz));
  return true;
}

bool initSpiCan() {
  hal::SharedSpiBusLock spiLock("CAN:init");
  // The shared SPI bus is initialized once in setup(). Re-running SPI.begin()
  // here can disturb Arduino-ESP32 3.x peripheral ownership after the display
  // has attached to the same bus.
  pinMode(pins::kCanSpiInt, INPUT_PULLUP);

  if (pins::kCanSpiRst != 255) {
    pinMode(pins::kCanSpiRst, OUTPUT);
    digitalWrite(pins::kCanSpiRst, LOW);
    delay(2);
    digitalWrite(pins::kCanSpiRst, HIGH);
    delay(2);
  }

  Serial.printf("[CAN] MCP2515 bitrate=500k clock=%uMHz SPI=%luHz int_gated=%u rx_max=%u fallback=%lums\n",
                 static_cast<unsigned>(kCanMcpClockMhz),
                 static_cast<unsigned long>(CCM_CAN_SPI_HZ),
                 static_cast<unsigned>(kCanRxIntGated ? 1U : 0U),
                 static_cast<unsigned>(kCanRxMaxFramesPerTick),
                 static_cast<unsigned long>(kCanRxFallbackPollMs));

  g_mcp2515.reset();
  delay(2);
  if (configureSpiCanAtClock(kCanMcpClock, kCanMcpClockMhz)) {
    g_spiCanOnline = true;
    return true;
  }

#if CCM_CAN_MCP_CLOCK_MHZ != 16
  g_mcp2515.reset();
  delay(2);
  if (configureSpiCanAtClock(MCP_16MHZ, 16)) {
    g_spiCanOnline = true;
    return true;
  }
#endif

#if CCM_CAN_MCP_CLOCK_MHZ != 8
  g_mcp2515.reset();
  delay(2);
  if (configureSpiCanAtClock(MCP_8MHZ, 8)) {
    g_spiCanOnline = true;
    return true;
  }
#endif

  g_spiCanOnline = false;
  return false;
}
#endif

}  // namespace

bool CanManager::begin(bool tryHardwareCan) {
  hwCanReady_ = false;

#if CCM_HAS_SPI_CAN
  if (tryHardwareCan) {
    hwCanReady_ = initSpiCan();
    Serial.printf("[CAN] begin SPI -> %s\n", hwCanReady_ ? "OK" : "FAILED");
  }
#endif

#if CCM_HAS_TWAI
  if (!hwCanReady_ && tryHardwareCan) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(pins::kCanTx),
                                                                 static_cast<gpio_num_t>(pins::kCanRx), TWAI_MODE_NORMAL);
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
  canStartMs_ = millis();
  return hwCanReady_;
}

void CanManager::tick() {
  const uint32_t nowMs = millis();
  bool mustStopManualTest = false;

  can_protocol::CanFrame frame{};
  uint8_t rxFramesThisTick = 0;
  while (rxFramesThisTick < kCanRxMaxFramesPerTick && receiveFrame(frame)) {
    if (recorder_) recorder_->capture(frame, nowMs);
    dispatchFrame(frame, nowMs);
    ++rxFramesThisTick;
  }

#if CCM_HAS_SPI_CAN
  // ── MCP2515 error monitoring & bus-off recovery ───────────────────────────
  // Runs every 3 s. Detects TXBO (bus-off), error-passive, and RX overflow.
  // On bus-off the MCP2515 refuses to TX or RX until it is reset.
  if (hwCanReady_ && g_spiCanOnline && (nowMs - lastCanErrCheckMs_) >= 3000) {
    lastCanErrCheckMs_ = nowMs;
    uint8_t eflg = 0;
    uint8_t rec = 0;
    uint8_t tec = 0;
    uint8_t intf = 0;
    uint8_t stat = 0;
    {
      hal::SharedSpiBusLock spiLock("CAN:eflg");
      eflg = g_mcp2515.getErrorFlags();
      rec = g_mcp2515.errorCountRX();
      tec = g_mcp2515.errorCountTX();
      intf = g_mcp2515.getInterrupts();
      stat = g_mcp2515.getStatus();
    }
    const bool busOff    = (eflg & 0x20) != 0;  // TXBO
    const bool txErrPass = (eflg & 0x10) != 0;  // TXEP
    const bool rxErrPass = (eflg & 0x08) != 0;  // RXEP
    const bool txWarn    = (eflg & 0x04) != 0;  // TXWAR
    const bool rxWarn    = (eflg & 0x02) != 0;  // RXWAR
    const bool errWarn   = (eflg & 0x01) != 0;  // EWARN
    const bool rxOvr     = (eflg & 0xC0) != 0;  // RX0OVR | RX1OVR
    const bool rxPending = (stat & 0x03U) != 0U; // READ_STATUS RX0IF | RX1IF
    const bool txInt     = (intf & 0x1CU) != 0U; // CANINTF TX0IF | TX1IF | TX2IF
    const bool errInt    = (intf & 0x20U) != 0U; // CANINTF ERRIF
    const bool wakeInt   = (intf & 0x40U) != 0U; // CANINTF WAKIF
    const bool msgErrInt = (intf & 0x80U) != 0U; // CANINTF MERRF
    const state::VehicleState snap = state::g_vehicle_state.read();
    const bool emptyBenchBus = snap.can_rx_count == 0 && (txErrPass || busOff || msgErrInt);
    const bool rxErrorNoFrames = snap.can_rx_count == 0 && (rxErrPass || rxWarn);
    const bool noAckBackoffActive =
        canNoAckBackoffUntilMs_ != 0 && nowMs < canNoAckBackoffUntilMs_;
    // TX-complete interrupt bits normally toggle under load and used to print
    // a full status line every three seconds. Log immediately when the actual
    // error state changes, otherwise emit only a slow health heartbeat.
    const uint16_t statusSignature =
        static_cast<uint16_t>((static_cast<uint16_t>(eflg) << 8U) |
                              static_cast<uint16_t>(intf & 0xE0U));
    const bool canErrorActive = eflg != 0U || errInt || msgErrInt || rxOvr;
    const uint32_t statusLogPeriodMs = canErrorActive ? 10000U : 30000U;
    const bool shouldLogCanStatus =
        statusSignature != lastCanStatusSignature_ ||
        (nowMs - lastCanStatusLogMs_) >= statusLogPeriodMs;
    if (shouldLogCanStatus) {
      lastCanStatusSignature_ = statusSignature;
      lastCanStatusLogMs_ = nowMs;
      Serial.printf("[CAN] EFLG=0x%02X INTF=0x%02X STAT=0x%02X REC=%u TEC=%u INT=%d rx=%lu tx=%lu%s%s%s%s%s%s%s%s%s%s%s%s%s\n",
          eflg,
          intf,
          stat,
          static_cast<unsigned>(rec),
          static_cast<unsigned>(tec),
          digitalRead(pins::kCanSpiInt),
          static_cast<unsigned long>(snap.can_rx_count),
          static_cast<unsigned long>(snap.can_tx_count),
          busOff    ? " BUSOFF-REINIT" :
          txErrPass ? " TX-ERR-PASSIVE" : "",
          rxErrPass ? " RX-ERR-PASSIVE" : "",
          txWarn    ? " TX-WARN" : "",
          rxWarn    ? " RX-WARN" : "",
          errWarn   ? " WARN" : "",
          emptyBenchBus ? " NO-ACK-BENCH" : "",
          rxErrorNoFrames ? " RX-BAD-FRAMES" : "",
          rxOvr ? " RX-OVR" : "",
          rxPending ? " RX-PENDING" : "",
          txInt ? " TX-INT" : "",
          errInt ? " ERR-INT" : "",
          wakeInt ? " WAKE-INT" : "",
          msgErrInt ? " MSG-ERR" : "");
      if (rxErrorNoFrames) {
        Serial.println("[CAN] RX errors but no frames decoded - check bitrate, MCP crystal clock, CANH/CANL, ground, and termination");
      }
      if (emptyBenchBus) {
        lastCanNoAckLogMs_ = nowMs;
      }
    }
    if (emptyBenchBus) {
      if (!noAckBackoffActive) {
        canNoAckBackoffUntilMs_ = nowMs + kCanNoAckBackoffMs;
      }
      if (!canNoAckBackoffLogged_) {
        Serial.println("[CAN] no ACK detected; pausing TX probes - check other node power/mode/bitrate/transceiver/CANH-CANL/ground/termination");
        canNoAckBackoffLogged_ = true;
      }
    }
    if (rxOvr) {
      hal::SharedSpiBusLock spiLock("CAN:clear");
      g_mcp2515.clearRXnOVRFlags();
    }
    if (txInt) {
      hal::SharedSpiBusLock spiLock("CAN:txif-clear");
      g_mcp2515.clearTXInterrupts();
    }
    if (msgErrInt) {
      hal::SharedSpiBusLock spiLock("CAN:merr-clear");
      g_mcp2515.clearMERR();
    }
    if (errInt) {
      hal::SharedSpiBusLock spiLock("CAN:errif-clear");
      g_mcp2515.clearERRIF();
    }
    if (busOff) {
      // Full reset + reinit recovers from bus-off.
      g_spiCanOnline = false;
      if (initSpiCan()) {
        Serial.println("[CAN] MCP2515 bus-off recovery OK");
        state::g_vehicle_state.mutate([](state::VehicleState& s) { s.can_online = true; });
      } else {
        Serial.println("[CAN] MCP2515 bus-off recovery FAILED");
        state::g_vehicle_state.mutate([](state::VehicleState& s) { s.can_online = false; });
      }
    }
  }
  // One-shot "silent bus" warning: TX going up but no RX after grace period.
  if (!canRxWarnSent_ && hwCanReady_ && g_spiCanOnline &&
      (nowMs - canStartMs_) > 10000) {
    const state::VehicleState snap = state::g_vehicle_state.read();
    if (snap.can_tx_count > 5 && snap.can_rx_count == 0) {
      Serial.println("[CAN] WARNING: TX active but RX=0 - bus may be ACKing, but no node is transmitting frames this module can read");
      canRxWarnSent_ = true;
    }
  }
#endif

#if defined(DEMO_MODE) && (DEMO_MODE == 1)
  if (!hwCanReady_) {
    runDemoGenerator(nowMs);
  }
#endif

  // Runtime bench test mode — spoofs RPM, speed, and module statuses for
  // bench validation without needing DEMO_MODE=1 or live CAN / GPS hardware.
  if (state::g_vehicle_state.read().bench_test_mode) {
    runDemoGenerator(nowMs);
  }

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

    const uint32_t elapsedSinceManualStop = nowMs - lastManualTestStopMs_;
    if (elapsedSinceManualStop < kManualTestCooldownMs) {
      s.meth_manual_test_cooldown_ms_remaining = static_cast<uint16_t>(kManualTestCooldownMs - elapsedSinceManualStop);
    } else {
      s.meth_manual_test_cooldown_ms_remaining = 0;
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

bool CanManager::sendTaillightMode(uint8_t mode) {
  bool sent = false;
  switch (mode) {
    case can_protocol::taillight_mode::STOCK:
      sent = clearTaillightOverride();
      break;
    case can_protocol::taillight_mode::SEQUENTIAL:
      sent = sendTaillightCustomAnimation(taillight_animation::SEQUENTIAL_ID, taillight_animation::SEQUENTIAL_DURATION_MS, 0, 0);
      break;
    case can_protocol::taillight_mode::SHOW:
      sent = sendTaillightCustomAnimation(taillight_animation::SHOW_ID, taillight_animation::SHOW_DURATION_MS, 0, 0);
      break;
    case can_protocol::taillight_mode::DEMO:
      sent = sendTaillightCustomAnimation(taillight_animation::DEMO_ID, taillight_animation::DEMO_DURATION_MS, 0, 0);
      break;
    default:
      break;
  }
  if (sent) {
    state::g_vehicle_state.mutate([mode](state::VehicleState& s) { s.taillight_mode_commanded = mode; });
  }
  return sent;
}

bool CanManager::sendTaillightShowOption(uint8_t option) {
  const bool sent = sendTaillightCustomAnimation(taillight_animation::SHOW_ID, taillight_animation::SHOW_DURATION_MS, option, 0);
  if (sent) {
    state::g_vehicle_state.mutate([](state::VehicleState& s) { s.taillight_mode_commanded = can_protocol::taillight_mode::SHOW; });
  }
  return sent;
}

bool CanManager::sendMethArm(bool armed) {
  if (armed) {
    const state::VehicleState snapshot = state::g_vehicle_state.read();
    if (!meth::canArm(snapshot)) return false;
  } else {
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.meth_desired_armed = false;
      s.meth_state = state::MethState::OFF;
      s.meth_pump_duty = 0;
      s.manual_test_running = false;
    });
  }
  const bool sent = sendFrame(can_protocol::packMethArm(armed));
  if (sent && armed) {
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.meth_desired_armed = true;
    });
  }
  return sent;
}

bool CanManager::sendMethManualTest(uint8_t duty) {
  const state::VehicleState snapshot = state::g_vehicle_state.read();
  const uint32_t nowMs = millis();
  const meth::ManualTestDecision decision =
      meth::evaluateManualTestRequest(snapshot, duty, true, nowMs, lastManualTestStopMs_, kManualTestCooldownMs);
  if (!decision.allowed) {
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.meth_manual_test_reject_reason = static_cast<uint8_t>(decision.reason);
    });
    return false;
  }

  const bool sent = sendFrame(can_protocol::packMethManualTest(duty));
  if (!sent) {
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.meth_manual_test_reject_reason = meth_manual_test_reject_reason::OFFLINE;
    });
    return false;
  }

  manualTestStartMs_ = nowMs;
  state::g_vehicle_state.mutate([duty](state::VehicleState& s) {
    s.meth_state = state::MethState::TEST;
    s.meth_pump_duty = duty;
    s.manual_test_running = true;
    s.meth_manual_test_reject_reason = meth_manual_test_reject_reason::NONE;
  });
  return true;
}

bool CanManager::sendMethStopManualTest() {
  lastManualTestStopMs_ = millis();
  state::g_vehicle_state.mutate([](state::VehicleState& s) {
    s.meth_pump_duty = 0;
    s.manual_test_running = false;
    if (s.meth_state == state::MethState::TEST) {
      s.meth_state = state::MethState::OFF;
    }
  });
  return sendFrame(can_protocol::packMethStopManualTest());
}

bool CanManager::sendMethClearFaults() {
  return sendFrame(can_protocol::packMethClearFaults());
}

bool CanManager::sendMethConfigBroadcast() {
  state::VehicleState s = state::g_vehicle_state.read();
  s.meth_config_version++;
  const bool sent = sendFrame(packMethConfigState(s));
  if (sent) {
    state::g_vehicle_state.mutate([&](state::VehicleState& live) {
      live.meth_config_version = s.meth_config_version;
    });
  }
  return sent;
}

bool CanManager::sendKnockCommand(uint8_t command, uint8_t value) {
  if (command < can_protocol::knock_command::SET_ENABLE ||
      command > can_protocol::knock_command::CLEAR_EVENTS_AND_FAULTS) {
    return false;
  }
  return sendFrame(can_protocol::packEngineKnockCommand(command, value));
}

bool CanManager::requestKnockConfig() {
  return sendFrame(can_protocol::packEngineKnockConfigRequest());
}

bool CanManager::sendFrame(const can_protocol::CanFrame& frame) {
  // This node is a passive consumer of ECU telemetry. Refuse any frame whose
  // identifier belongs to the MicroSquirt broadcast windows, even if a future
  // caller accidentally constructs one.
  if (microsquirt::isMicroSquirtId(frame.id,
                                  microsquirt::kRecommendedRealtimeBaseId)) {
    return false;
  }

  bool sent = false;
  const uint32_t nowMs = millis();

  if (canNoAckBackoffUntilMs_ != 0 && nowMs < canNoAckBackoffUntilMs_) {
    return false;
  }

#if CCM_HAS_SPI_CAN
  if (hwCanReady_ && g_spiCanOnline) {
    struct can_frame tx{};
    tx.can_id = frame.id & 0x7FFU;
    tx.can_dlc = frame.dlc;
    for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
      tx.data[i] = frame.data[i];
    }
    hal::SharedSpiBusLock spiLock("CAN:tx");
    sent = g_mcp2515.sendMessage(&tx) == MCP2515::ERROR_OK;
  }
#endif

#if CCM_HAS_TWAI
  if (!sent && hwCanReady_) {
    twai_message_t tx{};
    tx.identifier = frame.id;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data_length_code = frame.dlc;
    for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
      tx.data[i] = frame.data[i];
    }
    sent = twai_transmit(&tx, 0) == ESP_OK;
  }
#endif

#if defined(DEMO_MODE) && (DEMO_MODE == 1)
  if (!sent && !hwCanReady_) {
    sent = true;
  }
#endif

  if (sent) {
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.can_tx_count++;
      s.can_last_tx_id = frame.id;
      s.can_last_tx_ms = nowMs;
    });
  }
  return sent;
}

bool CanManager::receiveFrame(can_protocol::CanFrame& frame) {
#if CCM_HAS_SPI_CAN
  if (hwCanReady_ && g_spiCanOnline) {
    const uint32_t nowMs = millis();
    if (kCanRxIntGated && digitalRead(pins::kCanSpiInt) != LOW &&
        (nowMs - lastCanRxPollMs_) < kCanRxFallbackPollMs) {
      return false;
    }
    lastCanRxPollMs_ = nowMs;

    struct can_frame rx{};
    hal::SharedSpiBusLock spiLock("CAN:rx");
    MCP2515::ERROR readErr = g_mcp2515.readMessage(&rx);
    if (readErr != MCP2515::ERROR_OK) {
      const uint8_t eflg = g_mcp2515.getErrorFlags();
      if ((eflg & 0xC0U) != 0U) {
        g_mcp2515.clearRXnOVRFlags();
        g_mcp2515.clearERRIF();
        readErr = g_mcp2515.readMessage(&rx);
      }
      if (readErr != MCP2515::ERROR_OK) {
        return false;
      }
    }
    frame.id = static_cast<uint16_t>(rx.can_id & 0x7FFU);
    frame.dlc = rx.can_dlc;
    for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
      frame.data[i] = rx.data[i];
    }
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.can_rx_count++;
      s.can_last_rx_id = frame.id;
      s.can_last_rx_ms = millis();
      s.can_online = true;
    });
    canNoAckBackoffUntilMs_ = 0;
    lastCanNoAckLogMs_ = 0;
    canNoAckBackoffLogged_ = false;
    return true;
  }
#endif

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
      s.can_online = true;
    });
    canNoAckBackoffUntilMs_ = 0;
    lastCanNoAckLogMs_ = 0;
    canNoAckBackoffLogged_ = false;
    return true;
  }
#endif
  return false;
}

void CanManager::dispatchFrame(const can_protocol::CanFrame& frame, uint32_t nowMs) {
  if (dispatchMicroSquirt(frame, nowMs)) return;
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
      s.fault_flags |= kFaultTaillight;
      s.last_taillight_ms = nowMs;
      s.taillight_online = true;
      s.can_online = true;
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
      // 0x300 byte 4 is already gauge kPa from the meth/knock module.
      // Do not subtract baro here; just expose gauge kPa and gauge psi.
      const bool ecuFresh = s.microsquirt_online &&
                            (nowMs - s.microsquirt_last_ms) <= microsquirt::kFreshTimeoutMs;
      if (!ecuFresh) {
        s.boost_kpa = static_cast<float>(msg.boost_kpa);
        s.boost_psi = s.boost_kpa * kKpaToPsi;
        s.intake_temp = msg.iat_c;
      }
      s.engine_bay_temp = msg.engine_bay_c;
      s.intake_temp_valid = true;
      s.engine_bay_temp_valid = true;
      s.meth_fault_flags = msg.fault_flags;
      if (msg.fault_flags != 0U || s.meth_state == state::MethState::FAULT) {
        s.fault_flags |= kFaultMeth;
      } else {
        s.fault_flags &= static_cast<uint16_t>(~kFaultMeth);
      }
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
      s.fault_flags |= kFaultMeth;
      s.last_meth_ms = nowMs;
      s.meth_online = true;
      s.can_online = true;
      if (msg.severity >= static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL)) {
        s.master_state = static_cast<uint8_t>(can_protocol::MasterState::FAULT);
        s.meth_state = state::MethState::FAULT;
        s.meth_pump_duty = 0;
        s.meth_desired_armed = false;
        s.meth_fault_flags = msg.code != 0U ? msg.code : 0xFFU;
        s.manual_test_running = false;
      }
    });
    return;
  }

  if (frame.id == can_protocol::ID_ENGINE_SENSOR_EXT) {
    can_protocol::EngineSensorExt msg{};
    if (!can_protocol::unpackEngineSensorExt(frame, msg)) return;
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.oil_pressure_psi = msg.oil_pressure_psi;
      s.fuel_pressure_psi = msg.fuel_pressure_psi;
      s.meth_pressure_psi = msg.meth_pressure_psi;
      s.boost_ref_pressure_psi = msg.boost_ref_pressure_psi;
      s.outside_temp = msg.ambient_temp_c;
      s.cabin_temp = msg.cabin_temp_c;
      s.analog_sensor_fault_flags = msg.analog_fault_flags;
      s.oil_pressure_valid = true;
      s.fuel_pressure_valid = true;
      s.meth_pressure_valid = true;
      s.boost_ref_pressure_valid = true;
      s.outside_temp_valid = true;
      s.cabin_temp_valid = true;
      s.last_analog_sensor_ms = nowMs;
      s.last_meth_ms = nowMs;
      s.meth_online = true;
      s.can_online = true;
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
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.last_meth_ms = nowMs;
      s.meth_online = true;
      s.can_online = true;
    });
    lastMethConfigTxMs_ = 0;  // Force immediate rebroadcast on next scheduler tick.
    return;
  }

  if (frame.id == can_protocol::ID_METH_CONFIG_ACK) {
    can_protocol::EngineKnockConfigAck knockAck{};
    if (can_protocol::unpackEngineKnockConfigAck(frame, knockAck)) {
      state::g_vehicle_state.mutate([&](state::VehicleState& s) {
        s.can_online = true;
        switch (knockAck.command) {
          case can_protocol::knock_command::SET_ENABLE:
            s.knock_enabled = knockAck.applied_value != 0;
            break;
          case can_protocol::knock_command::SET_THRESHOLD_OFFSET:
            s.knock_threshold_offset = static_cast<float>(knockAck.applied_value);
            break;
          case can_protocol::knock_command::SET_ADAPTIVE_MULTIPLIER:
            s.knock_threshold_multiplier = static_cast<float>(knockAck.applied_value) / 10.0f;
            break;
          case can_protocol::knock_command::SET_MIN_RPM:
            s.knock_rpm_enable_min = static_cast<uint16_t>(knockAck.applied_value) * 100U;
            break;
          case can_protocol::knock_command::SET_MIN_MAP_KPA:
            s.knock_boost_enable_kpa = static_cast<float>(knockAck.applied_value);
            break;
          case can_protocol::knock_command::SET_DEBOUNCE:
            s.knock_event_cooldown_ms = static_cast<uint16_t>(knockAck.applied_value) * 10U;
            break;
          case can_protocol::knock_command::SET_GAIN:
            s.knock_gain = static_cast<float>(knockAck.applied_value) / 10.0f;
            break;
          case can_protocol::knock_command::SET_CENTER_FREQUENCY:
            s.knock_center_frequency_hz = static_cast<uint16_t>(knockAck.applied_value) * 100U;
            break;
          case can_protocol::knock_command::SET_BANDWIDTH:
            s.knock_bandwidth_hz = static_cast<uint16_t>(knockAck.applied_value) * 100U;
            break;
          case can_protocol::knock_command::SET_AUTO_FREQUENCY_FROM_BORE:
            s.knock_auto_frequency_from_bore = knockAck.applied_value != 0;
            break;
          case can_protocol::knock_command::CLEAR_EVENTS_AND_FAULTS:
            s.knock_event_count = 0;
            s.knock_warning_active = false;
            s.knock_critical_active = false;
            s.knock_sensor_fault = false;
            s.knock_clipping_detected = false;
            break;
          default:
            break;
        }
      });
      return;
    }

    can_protocol::MethConfigAck ack{};
    if (!can_protocol::unpackMethConfigAck(frame, ack)) return;
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.meth_config_version = ack.accepted_version;
      if (ack.active_ratio_percent <= 100 || ack.active_ratio_percent == 255) {
        s.meth_selected_ratio_percent = ack.active_ratio_percent;
      }
      s.last_meth_ms = nowMs;
      s.meth_online = true;
      s.can_online = true;
      if (ack.status == 0) {
        s.meth_online = true;
      } else if (ack.status == 3) {
        s.meth_state = state::MethState::FAULT;
        s.meth_pump_duty = 0;
        s.meth_desired_armed = false;
        s.meth_fault_flags = ack.reject_reason != 0U ? ack.reject_reason : 0xFFU;
        s.fault_flags |= kFaultMeth;
        s.manual_test_running = false;
      }
    });
    return;
  }

  // 0x307: Knock state from the external knock controller — parse all fields into VehicleState.
  if (frame.id == can_protocol::ID_ENGINE_KNOCK_CONFIG_PAGE1) {
    can_protocol::EngineKnockConfigPage1 cfg{};
    if (!can_protocol::unpackEngineKnockConfigPage1(frame, cfg)) return;
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.knock_enabled = (cfg.config_flags & 0x01U) != 0;
      s.knock_auto_frequency_from_bore = (cfg.config_flags & 0x02U) != 0;
      s.knock_threshold_offset = static_cast<float>(cfg.threshold_offset);
      s.knock_threshold_multiplier = static_cast<float>(cfg.adaptive_multiplier_x10) / 10.0f;
      s.knock_rpm_enable_min = static_cast<uint16_t>(cfg.min_rpm_div100) * 100U;
      s.knock_boost_enable_kpa = static_cast<float>(cfg.min_map_kpa);
      s.knock_event_cooldown_ms = static_cast<uint16_t>(cfg.debounce_ms_div10) * 10U;
      s.knock_gain = static_cast<float>(cfg.gain_x10) / 10.0f;
      s.knock_center_frequency_hz = static_cast<uint16_t>(cfg.center_frequency_div100) * 100U;
      s.can_online = true;
    });
    return;
  }

  if (frame.id == can_protocol::ID_ENGINE_KNOCK_CONFIG_PAGE2) {
    can_protocol::EngineKnockConfigPage2 cfg{};
    if (!can_protocol::unpackEngineKnockConfigPage2(frame, cfg)) return;
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.knock_bandwidth_hz = static_cast<uint16_t>(cfg.bandwidth_div100) * 100U;
      s.knock_sample_rate_hz = static_cast<uint16_t>(cfg.sample_rate_div100) * 100U;
      s.knock_samples_per_update = cfg.samples_per_update;
      s.knock_bias_alpha = static_cast<float>(cfg.bias_alpha_x1000) / 1000.0f;
      s.knock_rms_alpha = static_cast<float>(cfg.rms_alpha_x100) / 100.0f;
      s.knock_envelope_alpha = static_cast<float>(cfg.envelope_alpha_x100) / 100.0f;
      s.knock_bore_mm = cfg.bore_mm;
      s.can_online = true;
    });
    return;
  }

  if (frame.id == can_protocol::ID_ENGINE_KNOCK_STATE) {
    can_protocol::EngineKnockState ks{};
    if (!can_protocol::unpackEngineKnockState(frame, ks)) return;
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.knock_enabled         = (ks.status_flags & can_protocol::knock_status_flag::ENABLED) != 0;
      s.knock_signal_valid    = (ks.status_flags & can_protocol::knock_status_flag::SIGNAL_VALID) != 0;
      s.knock_warning_active  = (ks.status_flags & can_protocol::knock_status_flag::WARNING_ACTIVE) != 0;
      s.knock_critical_active = (ks.status_flags & can_protocol::knock_status_flag::CRITICAL_ACTIVE) != 0;
      s.knock_baseline_learned = (ks.status_flags & can_protocol::knock_status_flag::BASELINE_LEARNED) != 0;
      s.knock_sensor_fault    = (ks.status_flags & can_protocol::knock_status_flag::SENSOR_FAULT) != 0;
      s.knock_clipping_detected = (ks.status_flags & can_protocol::knock_status_flag::CLIPPING_DETECTED) != 0;
      s.knock_energy          = static_cast<float>(ks.energy);
      s.knock_baseline        = static_cast<float>(ks.baseline);
      s.knock_threshold       = static_cast<float>(ks.threshold);
      s.knock_event_count     = ks.event_count;
      s.knock_last_event_rpm  = static_cast<uint16_t>(ks.last_event_rpm_div100) * 100U;
      s.knock_last_event_boost_kpa = ks.last_event_boost_kpa;
      s.last_knock_ms         = nowMs;
      s.knock_online          = true;
      s.can_online            = true;
    });
    return;
  }

  // 0x308: Knock fault from an external module — update critical/sensor fault flags.
  if (frame.id == can_protocol::ID_ENGINE_KNOCK_FAULT) {
    can_protocol::EngineKnockFault msg{};
    if (!can_protocol::unpackEngineKnockFault(frame, msg)) return;
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.last_knock_ms = nowMs;
      s.knock_online = true;
      s.can_online = true;
      const bool isSensorFault = (msg.code == can_protocol::knock_fault_code::SENSOR_DISCONNECTED ||
                                  msg.code == can_protocol::knock_fault_code::ADC_FAULT);
      if (isSensorFault) s.knock_sensor_fault = true;
      if (msg.severity >= static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL)) {
        s.knock_critical_active = true;
        s.fault_flags |= kFaultKnockCritical;
      }
    });
    return;
  }
}

void CanManager::sendScheduledFrames(uint32_t nowMs) {
  state::VehicleState snapshot = state::g_vehicle_state.read();
  const bool rxSilent = snapshot.can_rx_count == 0 && (nowMs - canStartMs_) > kCanTxQuietGraceMs;

  if (rxSilent) {
    if ((nowMs - lastKnockConfigRequestMs_) >= kCanKnockConfigRequestMs) {
      requestKnockConfig();
      lastKnockConfigRequestMs_ = nowMs;
    }
    return;
  }

  if ((nowMs - lastHeartbeatTxMs_) >= kCanHeartbeatTxMs) {
    can_protocol::CanFrame hb{};
    packMasterHeartbeat(snapshot, hb);
    sendFrame(hb);
    lastHeartbeatTxMs_ = nowMs;
  }

  if ((nowMs - lastTachTxMs_) >= kCanTachTxMs) {
    can_protocol::CanFrame tach{};
    packTachState(snapshot, tach);
    sendFrame(tach);
    lastTachTxMs_ = nowMs;
  }

  if ((nowMs - lastGpsTxMs_) >= kCanGpsTxMs) {
    can_protocol::CanFrame gps{};
    packGpsState(snapshot, gps);
    sendFrame(gps);
    lastGpsTxMs_ = nowMs;
  }

  if ((nowMs - lastMethConfigTxMs_) >= kMethConfigBroadcastIntervalMs) {
    sendMethConfigBroadcast();
    lastMethConfigTxMs_ = nowMs;
  }

  if (snapshot.can_rx_count == 0 && (nowMs - lastKnockConfigRequestMs_) >= kCanKnockConfigRequestMs) {
    requestKnockConfig();
    lastKnockConfigRequestMs_ = nowMs;
  }

  if ((nowMs - lastEngineRuntimeTxMs_) >= kCanEngineRuntimeTxMs) {
    can_protocol::EngineRuntime runtime{};
    runtime.rpm = snapshot.rpm;
    runtime.map_kpa = 0;
    runtime.valid_flags = runtime.rpm > 0 ? 0x01 : 0x00;
    sendFrame(can_protocol::packEngineRuntime(runtime));
    lastEngineRuntimeTxMs_ = nowMs;
  }
}

bool CanManager::dispatchMicroSquirt(const can_protocol::CanFrame& frame, uint32_t nowMs) {
  bool decoded = microsquirt::decodeDash(frame, microsquirtData_, nowMs);
  if (!decoded) {
    decoded = microsquirt::decodeRealtime(
        frame, microsquirt::kRecommendedRealtimeBaseId, microsquirtData_, nowMs);
  }
  if (!decoded) {
    decoded = microsquirt::decodeRealtime(
        frame, microsquirt::kDefaultRealtimeBaseId, microsquirtData_, nowMs);
  }
  if (!decoded) return false;

  const auto& ms = microsquirtData_;
  state::g_vehicle_state.mutate([&](state::VehicleState& s) {
    s.microsquirt_online = true;
    s.microsquirt_last_id = frame.id;
    s.microsquirt_last_ms = nowMs;
    s.microsquirt_frame_count = ms.frame_count;
    s.microsquirt_invalid_count = ms.invalid_count;
    s.can_online = true;

    if ((ms.valid & microsquirt::VALID_RPM) != 0U) {
      s.rpm = ms.rpm;
      s.tach_source = static_cast<uint8_t>(can_protocol::TachSource::CAN);
    }
    if ((ms.valid & microsquirt::VALID_MAP) != 0U) s.microsquirt_map_kpa = ms.map_kpa;
    if ((ms.valid & microsquirt::VALID_BARO) != 0U) s.microsquirt_baro_kpa = ms.baro_kpa;
    if ((ms.valid & microsquirt::VALID_TPS) != 0U) s.microsquirt_tps_percent = ms.tps_percent;
    if ((ms.valid & microsquirt::VALID_SPARK) != 0U) s.microsquirt_spark_deg = ms.spark_advance_deg;
    if ((ms.valid & microsquirt::VALID_PW1) != 0U) s.microsquirt_pw1_ms = ms.pulse_width_1_ms;
    if ((ms.valid & microsquirt::VALID_PW2) != 0U) s.microsquirt_pw2_ms = ms.pulse_width_2_ms;
    if ((ms.valid & microsquirt::VALID_AFR_TARGET1) != 0U) s.microsquirt_afr_target = ms.afr_target1;
    if ((ms.valid & microsquirt::VALID_KNOCK) != 0U) s.microsquirt_knock_percent = ms.knock_percent;
    if ((ms.valid & microsquirt::VALID_BATTERY) != 0U) s.battery_voltage = ms.battery_v;
    if ((ms.valid & microsquirt::VALID_AFR1) != 0U && ms.afr1 >= 5.0f && ms.afr1 <= 30.0f) {
      s.afr = ms.afr1;
    }
    if ((ms.valid & microsquirt::VALID_CLT) != 0U) {
      s.coolant_temp = microsquirt::fahrenheitToCelsius(ms.coolant_f);
    }
    if ((ms.valid & microsquirt::VALID_MAT) != 0U) {
      s.intake_temp = microsquirt::fahrenheitToCelsius(ms.mat_f);
      s.intake_temp_valid = true;
    }
    if ((ms.valid & (microsquirt::VALID_MAP | microsquirt::VALID_BARO)) ==
        (microsquirt::VALID_MAP | microsquirt::VALID_BARO)) {
      s.boost_kpa = microsquirt::gaugeBoostKpa(ms);
      s.boost_psi = s.boost_kpa * kKpaToPsi;
    }
  });
  return true;
}

void CanManager::updateTimeouts(uint32_t nowMs) {
  // Allow 5 seconds at startup for modules to announce themselves before
  // treating their absence as a fault.
  constexpr uint32_t kStartupGraceMs = 5000;
  const bool inStartupGrace = (nowMs - canStartMs_) < kStartupGraceMs;

  state::g_vehicle_state.mutate([&](state::VehicleState& s) {
    s.taillight_online = (nowMs - s.last_taillight_ms) <= kTaillightTimeoutMs;
    s.meth_online = !state::nodeTimedOut(nowMs, s.last_meth_ms, kMethTimeoutMs);
    s.gps_stale = (nowMs - s.last_gps_ms) > kGpsStaleTimeoutMs;
    s.microsquirt_online = s.microsquirt_last_ms != 0U &&
                           (nowMs - s.microsquirt_last_ms) <= microsquirt::kFreshTimeoutMs;

    // Knock online: driven entirely by 0x307 frames from the external knock module.
    if (!inStartupGrace) {
      s.knock_online = (nowMs - s.last_knock_ms) <= kKnockTimeoutMs;
    }

    if (!inStartupGrace && (!s.taillight_online || !s.meth_online)) {
      s.fault_flags |= kFaultModuleOffline;
    } else {
      s.fault_flags &= static_cast<uint16_t>(~kFaultModuleOffline);
    }

    // Fail-safe local behavior: if meth module offline, force OFF and zero duty.
    if (state::methCanLossDisarms(s, nowMs, kMethTimeoutMs)) {
      s.meth_state = state::MethState::OFF;
      s.meth_pump_duty = 0;
      s.meth_desired_armed = false;
      s.meth_fault_flags = 0;
      s.manual_test_running = false;
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

    const float rpmWave = 0.5f + 0.5f * sinf(t * 1.8f);
    s.rpm = static_cast<uint16_t>(1200 + 5800 * rpmWave);
    // generated_tach_hz10 = (RPM / 15) * 10 => preserve scaling in 0.1Hz units.
    s.generated_tach_hz10 = static_cast<uint16_t>((s.rpm * 10U) / 15U);
    s.raw_tach_hz10 = s.generated_tach_hz10;
    s.tach_source = static_cast<uint8_t>(can_protocol::TachSource::DEMO);
    s.tach_status_flags = 0x03;

    s.speed = 45.0f + 20.0f * (0.5f + 0.5f * sinf(t * 0.35f));
    s.gps_satellites = static_cast<uint8_t>(8 + (static_cast<int>(t) % 5));
    s.gps_satellites_in_view = static_cast<uint8_t>(s.gps_satellites + 2U);
    s.gps_fix = true;
    s.gps_fix_quality = 1;
    s.gps_fix_mode = 3;
    s.gps_hdop_x10 = 9;
    s.gps_fix_type = 1;
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

    const bool demoMethSpraying = (static_cast<int>(t) % 10 > 6);
    const float methDutyWave = 0.5f + 0.5f * sinf(t * 1.1f);
    s.meth_state = demoMethSpraying ? state::MethState::SPRAYING : state::MethState::ARMED;
    s.meth_pump_duty = demoMethSpraying ? static_cast<uint8_t>(25 + 70 * methDutyWave) : 0;
    s.meth_tank_level = static_cast<uint8_t>(60 + 20 * sinf(t * 0.07f));
    s.meth_flow_status = (s.meth_state == state::MethState::SPRAYING) ? 1 : 0;
    s.boost_kpa = static_cast<uint8_t>(95 + 45 * sinf(t * 0.9f));
    s.boost_psi = s.boost_kpa * kKpaToPsi;
    s.intake_temp = 26.0f + 4.0f * sinf(t * 0.4f);
    s.engine_bay_temp = 50.0f + 6.0f * sinf(t * 0.25f);
    s.last_meth_ms = nowMs;

    s.cabin_temp = 24.0f + 2.0f * sinf(t * 0.1f);
    s.outside_temp = 21.0f + 1.0f * sinf(t * 0.06f);
    s.coolant_temp = 84.0f + 3.0f * sinf(t * 0.2f);
    s.intercooler_temp = 32.0f + 2.0f * sinf(t * 0.2f);
    s.oil_pressure_psi = 62.0f + 8.0f * sinf(t * 0.28f);
    s.fuel_pressure_psi = 49.0f + 4.0f * sinf(t * 0.33f);
    s.meth_pressure_psi = 115.0f + 9.0f * sinf(t * 0.44f);
    s.boost_ref_pressure_psi = 6.0f + 3.0f * sinf(t * 0.52f);
    s.spare_pressure_1_psi = 30.0f + 2.0f * sinf(t * 0.17f);
    s.spare_pressure_2_psi = 35.0f + 2.0f * sinf(t * 0.19f);
    s.intake_temp_valid = true;
    s.engine_bay_temp_valid = true;
    s.cabin_temp_valid = true;
    s.outside_temp_valid = true;
    s.oil_pressure_valid = true;
    s.fuel_pressure_valid = true;
    s.meth_pressure_valid = true;
    s.boost_ref_pressure_valid = true;
    s.spare_pressure_1_valid = true;
    s.spare_pressure_2_valid = true;
    s.analog_sensor_fault_flags = 0;
    s.battery_voltage = 12.8f;

    const float knockPhase = fmodf(t, 18.0f);
    const float knockNoise = 0.5f + 0.5f * sinf(t * 5.2f);
    const bool knockWarning = knockPhase > 9.0f && knockPhase <= 13.5f;
    const bool knockCritical = knockPhase > 13.5f && knockPhase <= 15.5f;
    s.knock_enabled = true;
    s.knock_online = true;
    s.last_knock_ms = nowMs;
    s.knock_signal_valid = true;
    s.knock_baseline_learned = true;
    s.knock_sensor_fault = false;
    s.knock_clipping_detected = knockPhase > 15.0f && knockPhase <= 15.5f;
    s.knock_signal_clip_high_count = s.knock_clipping_detected ? static_cast<uint16_t>(12 + static_cast<int>(t) % 8) : 0;
    s.knock_signal_clip_low_count = s.knock_clipping_detected ? static_cast<uint16_t>(4 + static_cast<int>(t) % 5) : 0;
    s.knock_baseline = 18.0f + 4.0f * sinf(t * 0.42f);
    s.knock_threshold = s.knock_baseline * s.knock_threshold_multiplier + s.knock_threshold_offset;
    float knockTarget = s.knock_baseline + 6.0f * knockNoise;
    if (knockWarning) knockTarget = s.knock_threshold * (0.72f + 0.18f * knockNoise);
    if (knockCritical) knockTarget = s.knock_threshold * (1.04f + 0.18f * knockNoise);
    s.knock_energy = knockTarget;
    s.knock_warning_active = knockWarning || knockCritical;
    s.knock_critical_active = knockCritical;
    s.knock_event_count = static_cast<uint8_t>((static_cast<uint32_t>(t) / 6U) % 100U);
    if (knockWarning || knockCritical) {
      s.knock_last_event_rpm = s.rpm;
      s.knock_last_event_boost_kpa = static_cast<uint8_t>(min<float>(255.0f, max<float>(0.0f, s.boost_kpa)));
      s.knock_last_event_iat_c = static_cast<int8_t>(s.intake_temp);
      s.knock_last_event_time_ms = nowMs;
    }
    s.knock_logging_active = s.sd_mounted;

    s.heap_free_bytes = ESP.getFreeHeap();
    s.esp_die_temp_c = static_cast<int8_t>(temperatureRead());
    s.tach_input_frequency_hz = s.raw_tach_hz10 / 10.0f;
    s.tach_generated_frequency_hz = s.generated_tach_hz10 / 10.0f;
  });
}

}  // namespace canbus
