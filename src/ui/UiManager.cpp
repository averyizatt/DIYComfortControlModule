#include "ui/UiManager.hpp"

namespace ccm::ui {

bool UiManager::begin() {
  const bool displayOk = display_.begin();
  const bool touchOk = touch_.begin();
  display_.setBrightness(180);
  return displayOk && touchOk;
}

void UiManager::tick(const core::DashboardData& dashboard) {
  display_.renderDashboard(dashboard);
}

bool UiManager::pollAction(core::UiAction& action) {
  action = {};
  const hal::TouchPoint point = touch_.readPoint();
  if (!point.touched) {
    touchActive_ = false;
    return false;
  }

  const uint32_t nowMs = millis();
  if (touchActive_ || (nowMs - lastTouchActionMs_) < kTouchActionDebounceMs) {
    return false;
  }
  touchActive_ = true;
  lastTouchActionMs_ = nowMs;

  if (point.y < kTouchZoneToggleMethMaxY) {
    action.type = core::UiActionType::ToggleMethEnable;
    action.value = 1;
    return true;
  }

  if (point.y < kTouchZoneChangeMixMaxY) {
    methMix_ = (methMix_ >= kMethMixMax) ? kMethMixMin : static_cast<uint8_t>(methMix_ + kMethMixStep);
    action.type = core::UiActionType::ChangeMethMix;
    action.value = methMix_;
    return true;
  }

  taillightMode_ = static_cast<uint8_t>((taillightMode_ + 1) % kTaillightModeCount);
  action.type = core::UiActionType::SetTaillightMode;
  action.value = taillightMode_;
  return true;
}

}  // namespace ccm::ui
