#pragma once
#include <cstdint>

namespace gateway {
struct CommandLease {
  uint32_t received = 0;
  bool active = false;
  void refresh(uint32_t now) { received = now; active = true; }
  bool expire(uint32_t now, uint32_t timeout) {
    if (!active || uint32_t(now - received) < timeout) return false;
    active = false; return true;
  }
};
struct Pulse {
  uint32_t last = 0, period = 0;
  bool seen = false, ready = false;
  void edge(uint32_t now, uint32_t minimum, uint32_t timeout) volatile {
    const uint32_t elapsed = now - last;
    if (!seen || elapsed >= minimum) {
      ready = seen && elapsed <= timeout;
      period = ready ? elapsed : 0;
      last = now; seen = true;
    }
  }
  uint32_t periodAt(uint32_t now, uint32_t timeout) volatile {
    if (seen && uint32_t(now - last) > timeout) {
      seen = false; ready = false; period = 0;
    }
    return ready ? period : 0;
  }
};
// All elapsed comparisons use subtraction so millis()/micros() rollover is safe.
struct DebouncedButton {
  bool stable = false, candidate = false;
  uint32_t since = 0;
  bool update(bool raw, uint32_t now) {
    if (raw != candidate) { candidate = raw; since = now; }
    if (stable != candidate && uint32_t(now - since) >= 25) {
      stable = candidate; return true;
    }
    return false;
  }
};
inline uint16_t bounded16(uint64_t n) { return n > 65535 ? 65535 : uint16_t(n); }
inline uint16_t rpm(uint32_t periodUs, uint16_t pulsesPerRev10) {
  return !periodUs || !pulsesPerRev10 ? 0 :
      bounded16(600000000ULL / (uint64_t(periodUs) * pulsesPerRev10));
}
inline uint16_t speedKph10(uint32_t periodUs, uint32_t pulsesPerMile) {
  return !periodUs || !pulsesPerMile ? 0 :
      bounded16(57936384000ULL / (uint64_t(periodUs) * pulsesPerMile));
}
inline uint8_t fuelPercent(float resistance, float empty, float full) {
  if (empty == full) return 255;
  float percent = (resistance - empty) * 100 / (full - empty);
  return percent < 0 ? 0 : percent > 100 ? 100 : uint8_t(percent + 0.5f);
}
inline bool fuelResistance(uint32_t millivolts, float supplyMv, float pullupOhms,
                           float& ohms) {
  if (millivolts <= 50 || millivolts >= supplyMv - 200 || pullupOhms <= 0) return false;
  ohms = pullupOhms * millivolts / (supplyMv - millivolts);
  return true;
}
} // namespace gateway
