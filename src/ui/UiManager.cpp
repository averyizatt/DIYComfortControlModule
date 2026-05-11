#include "ui/UiManager.hpp"

namespace ccm::ui {

bool UiManager::begin() {
  const bool displayOk = display_.begin();
  const bool touchOk = touch_.begin();
  const bool buttonsOk = buttons_.begin();
  display_.setBrightness(180);
  return displayOk && touchOk && buttonsOk;
}

void UiManager::tick(const core::DashboardData& dashboard) {
  display_.renderDashboard(dashboard);
  const auto point = touch_.readPoint();
  (void)point;
}

bool UiManager::pollAction(core::UiAction& action) {
  action = {};
  hal::ButtonEvent event;
  if (!buttons_.poll(event)) {
    return false;
  }

  switch (event.type) {
    case hal::ButtonEventType::Up:
      taillightMode_ = static_cast<uint8_t>((taillightMode_ + 1) % kTaillightModeCount);
      action.type = core::UiActionType::SetTaillightMode;
      action.value = taillightMode_;
      return true;
    case hal::ButtonEventType::Down:
      methMix_ = (methMix_ >= kMethMixMax) ? kMethMixMin : static_cast<uint8_t>(methMix_ + kMethMixStep);
      action.type = core::UiActionType::ChangeMethMix;
      action.value = methMix_;
      return true;
    case hal::ButtonEventType::Select:
      action.type = core::UiActionType::ToggleMethEnable;
      action.value = 1;
      return true;
    default:
      return false;
  }
}

}  // namespace ccm::ui
