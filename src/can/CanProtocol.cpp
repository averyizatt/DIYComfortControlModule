#include "can/CanProtocol.hpp"

namespace ccm::can {

void CanScheduler::begin() {
  heartbeatLastMs_ = 0;
}

bool CanScheduler::shouldSendHeartbeat(uint32_t nowMs) const {
  constexpr uint32_t kIntervalMs = 250;
  return (nowMs - heartbeatLastMs_) >= kIntervalMs;
}

bool CanScheduler::isNodeTimedOut(uint32_t nowMs, uint32_t lastSeenMs, uint32_t timeoutMs) const {
  return (nowMs - lastSeenMs) > timeoutMs;
}

}  // namespace ccm::can
