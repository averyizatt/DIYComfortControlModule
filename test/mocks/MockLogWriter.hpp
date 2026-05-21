#pragma once

#include <string>
#include <utility>
#include <vector>

#include "hal/TestInterfaces.hpp"

struct MockLogWriter : public ccm::hal::ILogWriter {
  bool allow_writes = true;
  std::vector<std::pair<std::string, std::string>> writes;
  std::vector<std::string> flushed;

  bool appendLine(const std::string& path, const std::string& line) override {
    if (!allow_writes) return false;
    writes.emplace_back(path, line);
    return true;
  }

  bool flush(const std::string& path) override {
    flushed.push_back(path);
    return allow_writes;
  }
};
