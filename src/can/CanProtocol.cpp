#include "can/CanProtocol.hpp"

namespace ccm::can {

void CanScheduler::begin() {
  heartbeatLastMs_ = 0;
}

bool CanScheduler::shouldSendHeartbeat(uint32_t nowMs) {
  constexpr uint32_t kIntervalMs = 250;
  if ((nowMs - heartbeatLastMs_) >= kIntervalMs) {
    heartbeatLastMs_ = nowMs;
    return true;
  }
  return false;
}

bool CanScheduler::isNodeTimedOut(uint32_t nowMs, uint32_t lastSeenMs, uint32_t timeoutMs) const {
  return (nowMs - lastSeenMs) > timeoutMs;
}

}  // namespace ccm::can
