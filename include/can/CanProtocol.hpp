#pragma once

#include <Arduino.h>

namespace ccm::can {

constexpr uint8_t kProtocolVersion = CCM_CAN_PROTOCOL_VERSION;

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
  MasterHeartbeat = 0x100,
  DashboardBroadcast = 0x101,
  NodeStatus = 0x120,
  Diagnostic = 0x130,

  MethCommand = 0x200,
  MethStatus = 0x201,

  TailLightCommand = 0x210,
  TailLightStatus = 0x211,

  RpmTelemetry = 0x300,
  GpsTelemetry = 0x301,
  EnvironmentTelemetry = 0x302,
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

struct CanFrame {
  uint16_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8]{};
};

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
