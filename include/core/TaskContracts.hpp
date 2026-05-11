#pragma once

#include <Arduino.h>

#include "can/CanProtocol.hpp"
#include "core/SharedTypes.hpp"

namespace ccm::core {

enum class UiActionType : uint8_t {
  None,
  ChangeMethMix,
  ToggleMethEnable,
  SetTaillightMode,
  AckPopup,
};

struct UiAction {
  UiActionType type = UiActionType::None;
  uint8_t value = 0;
};

struct SensorFrame {
  uint32_t timestampMs = 0;
  EnvironmentData environment;
  PowerData power;
};

struct RpmFrame {
  uint32_t timestampMs = 0;
  uint16_t rpm = 0;
  RpmSource source = RpmSource::CanBus;
};

struct HealthFrame {
  uint32_t timestampMs = 0;
  bool canOnline = false;
  bool gpsOnline = false;
  uint32_t nodeBitmask = 0;
};

struct CanCommand {
  can::CanId id = can::CanId::MasterHeartbeat;
  uint8_t len = 0;
  uint8_t data[8]{};
};

}  // namespace ccm::core
