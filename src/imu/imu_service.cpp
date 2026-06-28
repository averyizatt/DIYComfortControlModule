#include "imu_service.h"

#include <cmath>
#include <cstdint>

#include "pin_map.h"
#include "state/vehicle_state.h"

// MPU-6050 register map (minimal subset needed)
namespace {
  constexpr uint8_t kRegPwrMgmt1    = 0x6B;
  constexpr uint8_t kRegAccelConfig  = 0x1C;
  constexpr uint8_t kRegAccelXoutH   = 0x3B;
  constexpr uint8_t kRegWhoAmI       = 0x75;

  // ±8 G range → sensitivity = 4096 LSB/G
  constexpr float kLsbPerG = 4096.0f;
  // ACCEL_FS_SEL = 2 (±8 G)
  constexpr uint8_t kAccelFs8G = 0x10;
}

namespace imu {

bool ImuService::begin(TwoWire& wire) {
  wire_ = &wire;

  activeAddr_ = 0;
  const uint8_t candidates[] = {
      pins::kGyroI2cAddrPrimary,
      pins::kGyroI2cAddrSecondary,
  };
  for (uint8_t addr : candidates) {
    // Check WHO_AM_I — should return 0x68 for MPU-6050.
    // Use sendStop=true so a STOP condition is always sent, even on NAK.
    // Without it, a missing device leaves the bus held without a STOP,
    // which can corrupt the next touch-controller transaction.
    wire_->beginTransmission(addr);
    wire_->write(kRegWhoAmI);
    if (wire_->endTransmission(true) != 0) {
      continue;
    }
    if (wire_->requestFrom(static_cast<uint8_t>(addr), static_cast<uint8_t>(1)) != 1 ||
        !wire_->available()) {
      continue;
    }
    const uint8_t whoAmI = static_cast<uint8_t>(wire_->read());
    // MPU-6050 = 0x68, MPU-6500 = 0x70 — both accepted
    if (whoAmI == 0x68 || whoAmI == 0x70) {
      activeAddr_ = addr;
      break;
    }
  }

  if (activeAddr_ == 0) {
    online_ = false;
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.imu_online = false;
    });
    return false;
  }

  // Wake the device (clear SLEEP bit in PWR_MGMT_1)
  wire_->beginTransmission(activeAddr_);
  wire_->write(kRegPwrMgmt1);
  wire_->write(0x00);
  if (wire_->endTransmission() != 0) {
    online_ = false;
    activeAddr_ = 0;
    return false;
  }

  // Set accelerometer range to ±8 G
  wire_->beginTransmission(activeAddr_);
  wire_->write(kRegAccelConfig);
  wire_->write(kAccelFs8G);
  if (wire_->endTransmission() != 0) {
    online_ = false;
    activeAddr_ = 0;
    return false;
  }

  online_    = true;
  failCount_ = 0;
  filtLat_   = 0.0f;
  filtLon_   = 0.0f;
  return true;
}

bool ImuService::readAccel(float& ax, float& ay, float& az) {
  if (!wire_ || activeAddr_ == 0) return false;

  wire_->beginTransmission(activeAddr_);
  wire_->write(kRegAccelXoutH);
  // sendStop=true: always terminate cleanly so a failed read
  // cannot leave the bus in a bad state for the touch controller.
  if (wire_->endTransmission(true) != 0) return false;

  if (wire_->requestFrom(static_cast<uint8_t>(activeAddr_), static_cast<uint8_t>(6)) != 6) return false;

  const int16_t rawX = static_cast<int16_t>((wire_->read() << 8) | wire_->read());
  const int16_t rawY = static_cast<int16_t>((wire_->read() << 8) | wire_->read());
  const int16_t rawZ = static_cast<int16_t>((wire_->read() << 8) | wire_->read());

  ax = static_cast<float>(rawX) / kLsbPerG;
  ay = static_cast<float>(rawY) / kLsbPerG;
  az = static_cast<float>(rawZ) / kLsbPerG;
  return true;
}

void ImuService::update() {
  if (!wire_) return;
  const uint32_t nowMs = static_cast<uint32_t>(millis());

  if (!online_) {
    return;
  }

  if ((nowMs - lastSampleMs_) < kSamplePeriodMs) {
    return;
  }
  lastSampleMs_ = nowMs;

  float ax = 0.0f, ay = 0.0f, az = 0.0f;
  if (!readAccel(ax, ay, az)) {
    failCount_++;
    if (failCount_ >= kMaxFails) {
      online_ = false;
      activeAddr_ = 0;
      state::g_vehicle_state.mutate([](state::VehicleState& s) {
        s.imu_online = false;
      });
    }
    return;
  }
  failCount_ = 0;

  // GY-521 standard mounting in a car (flat on dashboard/shelf):
  //   X = longitudinal (forward = +), Y = lateral (right = +),
  //   Z ≈ +1 G at rest (gravity). We subtract gravity from Z,
  //   but for horizontal G-force only X and Y matter.
  const float gLat = ay;   // lateral: right is positive
  const float gLon = ax;   // longitudinal: forward/accel is positive

  // Low-pass filter
  filtLat_ = filtLat_ + kFilterAlpha * (gLat - filtLat_);
  filtLon_ = filtLon_ + kFilterAlpha * (gLon - filtLon_);

  const float gTotal = sqrtf(filtLat_ * filtLat_ + filtLon_ * filtLon_);

  state::g_vehicle_state.mutate([&](state::VehicleState& s) {
    s.imu_g_lateral      = filtLat_;
    s.imu_g_longitudinal = filtLon_;
    s.imu_g_total        = gTotal;
    if (gTotal > s.imu_g_peak) s.imu_g_peak = gTotal;
    s.imu_online         = true;
  });
}

}  // namespace imu
