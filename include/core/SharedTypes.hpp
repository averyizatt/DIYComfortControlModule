#pragma once

#include <Arduino.h>

namespace ccm::core {

enum class RpmSource : uint8_t {
  TachInput,
  CanBus,
};

enum class TachScaleMode : uint8_t {
  RpmDiv15,
  RpmDiv30,
};

enum class SystemFault : uint32_t {
  None = 0,
  CanOffline = 1 << 0,
  GpsOffline = 1 << 1,
  Undervoltage = 1 << 2,
  SensorFault = 1 << 3,
};

inline constexpr SystemFault operator|(SystemFault a, SystemFault b) {
  return static_cast<SystemFault>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline constexpr bool hasFault(SystemFault faults, SystemFault fault) {
  return (static_cast<uint32_t>(faults) & static_cast<uint32_t>(fault)) != 0;
}

struct EnvironmentData {
  float cabinC = NAN;
  float engineBayC = NAN;
  float outsideC = NAN;
  float intakeC = NAN;
  float humidity = NAN;
};

struct GpsData {
  bool validFix = false;
  float speedKph = 0.0f;
  double latitude = 0.0;
  double longitude = 0.0;
  float altitudeM = 0.0f;
  uint32_t satellites = 0;
  uint32_t satellitesInView = 0;
  uint8_t fixQuality = 0;
  uint8_t fixMode = 0;
  uint32_t baud = 0;
  uint32_t charsProcessed = 0;
  uint32_t passedChecksum = 0;
  uint32_t failedChecksum = 0;
  uint32_t sentencesWithFix = 0;
  uint32_t lastRxMs = 0;
  uint32_t lastFixMs = 0;
};

struct PowerData {
  float batteryV = 0.0f;
  bool undervoltage = false;
};

struct DashboardData {
  uint16_t rpm = 0;
  uint16_t speedKph = 0;
  bool canOnline = false;
  bool gpsOnline = false;
  EnvironmentData environment;
  PowerData power;
};

}  // namespace ccm::core
