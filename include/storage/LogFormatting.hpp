#pragma once

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <string>

namespace storage::logfmt {

inline std::string csvHeader() {
  return "timestamp_ms,category,payload";
}

inline std::string clampPayload(std::string payload, size_t maxLen = 512U) {
  if (payload.size() > maxLen) payload.resize(maxLen);
  return payload;
}

inline std::string formatCsvLine(uint32_t nowMs, const std::string& category, const std::string& payload) {
  std::ostringstream out;
  out << nowMs << ',' << (category.empty() ? "misc" : category) << ',' << clampPayload(payload);
  return out.str();
}

inline std::string formatFaultLine(uint32_t nowMs, const std::string& code, const std::string& detail, bool critical) {
  std::ostringstream out;
  out << nowMs << ",faults," << (critical ? "CRITICAL" : "WARN") << ':' << code << ':' << clampPayload(detail);
  return out.str();
}

inline bool shouldFlushImmediately(bool criticalFault) {
  return criticalFault;
}

}  // namespace storage::logfmt
