#pragma once

#include "core/TaskContracts.hpp"
#include "hal/ButtonHal.hpp"
#include "hal/DisplayHal.hpp"
#include "hal/TouchHal.hpp"

namespace ccm::ui {

class UiManager {
 public:
  UiManager(hal::DisplayHal& display, hal::TouchHal& touch, hal::ButtonHal& buttons)
      : display_(display), touch_(touch), buttons_(buttons) {}

  bool begin();
  void tick(const core::DashboardData& dashboard);
  bool pollAction(core::UiAction& action);

 private:
  hal::DisplayHal& display_;
  hal::TouchHal& touch_;
  hal::ButtonHal& buttons_;
  uint8_t taillightMode_ = 0;
  uint8_t methMix_ = 50;
};

}  // namespace ccm::ui
