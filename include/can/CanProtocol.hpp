#pragma once

#include <Arduino.h>
#include "can_contract/can_protocol.h"

namespace ccm::can {

constexpr uint8_t kProtocolVersion = CCM_CAN_PROTOCOL_VERSION;
static_assert(kProtocolVersion == can_protocol::CAN_PROTOCOL_SCHEMA_VERSION,
              "Build flags and shared CAN schema must agree");

enum class ModuleId : uint8_t {
  Master = 0x01,
  EngineBay = 0x02,
  Lighting = 0x03,
  RearBody = 0x04,
  SensorNode = 0x05,
  MethController = 0x06,
  TailLightController = 0x07,
};

enum class CanId : uint16_t {
  MasterHeartbeat = can_protocol::ID_MASTER_HEARTBEAT,
  MethCommand = can_protocol::ID_ENGINE_METH_COMMAND,
  MethConfig = can_protocol::ID_METH_CONFIG_BROADCAST,
  MethStatus = can_protocol::ID_ENGINE_METH_STATE,
  TailLightCommand = can_protocol::ID_TAILLIGHT_COMMAND,
  TailLightStatus = can_protocol::ID_TAILLIGHT_STATE,
  RpmTelemetry = can_protocol::ID_TACH_RPM_STATE,
  GpsTelemetry = can_protocol::ID_GPS_STATE,
  EnvironmentTelemetry = can_protocol::ID_ENGINE_SENSOR_EXT,
};

enum class MethMode : uint8_t {
  Disabled = 0,
  Mix25 = 25,
  Mix50 = 50,
  Mix75 = 75,
  Mix100 = 100,
};

enum class TailLightMode : uint8_t {
  Stock = 0,
  Sequential = 1,
  Show = 2,
  Demo = 3,
};

using CanFrame = can_protocol::CanFrame;

struct NodeHealth {
  ModuleId owner = ModuleId::Master;
  uint32_t lastSeenMs = 0;
  bool online = false;
};

class CanScheduler {
 public:
  void begin();
  bool shouldSendHeartbeat(uint32_t nowMs);
  bool isNodeTimedOut(uint32_t nowMs, uint32_t lastSeenMs, uint32_t timeoutMs) const;

 private:
  uint32_t heartbeatLastMs_ = 0;
};

}  // namespace ccm::can
