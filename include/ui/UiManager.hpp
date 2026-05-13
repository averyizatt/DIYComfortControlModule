#pragma once

#include "core/TaskContracts.hpp"
#include "hal/DisplayHal.hpp"
#include "hal/TouchHal.hpp"

namespace ccm::ui {

class UiManager {
 public:
  UiManager(hal::DisplayHal& display, hal::TouchHal& touch) : display_(display), touch_(touch) {}

  bool begin();
  void tick(const core::DashboardData& dashboard);
  bool pollAction(core::UiAction& action);

 private:
  static constexpr uint8_t kTaillightModeCount = 4;
  static constexpr uint8_t kMethMixMin = 25;
  static constexpr uint8_t kMethMixMax = 100;
  static constexpr uint8_t kMethMixStep = 25;
  static constexpr uint16_t kTouchZoneToggleMethMaxY = 160;
  static constexpr uint16_t kTouchZoneChangeMixMaxY = 320;

  hal::DisplayHal& display_;
  hal::TouchHal& touch_;
  uint8_t taillightMode_ = 0;
  uint8_t methMix_ = 50;
  bool touchActive_ = false;
  uint32_t lastTouchActionMs_ = 0;
  static constexpr uint32_t kTouchActionDebounceMs = 250;
};

}  // namespace ccm::ui
