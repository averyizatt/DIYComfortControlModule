#pragma once

#include <Wire.h>

namespace imu {

/// Reads MPU-6050 accelerometer on the shared I2C bus and writes
/// lateral/longitudinal G-force data to VehicleState.
class ImuService {
 public:
  ImuService() = default;

  /// Call once from setup() after Wire.begin().
  /// Returns true if the MPU-6050 was found and configured.
  bool begin(TwoWire& wire);

  /// Call from the IMU task (~20 Hz). Reads a sample and updates VehicleState.
  void update();

  bool online() const { return online_; }

 private:
  TwoWire* wire_        = nullptr;
  bool     online_      = false;
  uint8_t  activeAddr_  = 0;
  uint8_t  failCount_   = 0;
  uint32_t lastSampleMs_ = 0; // keep IMU reads from crowding touch I2C
  uint32_t lastRetryMs_ = 0;

  // Low-pass filter state (alpha = 0.25 — smooths jitter without much lag)
  float filtLat_ = 0.0f;
  float filtLon_ = 0.0f;

  static constexpr uint8_t  kMaxFails       = 5;
  static constexpr float    kFilterAlpha    = 0.25f;
  static constexpr uint32_t kSamplePeriodMs = 100;
  static constexpr uint32_t kRetryIntervalMs = 2000;

  bool readAccel(float& ax, float& ay, float& az);
};

}  // namespace imu
