#pragma once

#include <string>
#include <unordered_map>

#include "hal/TestInterfaces.hpp"

struct MockSettingsStore : public ccm::hal::ISettingsStore {
  std::unordered_map<std::string, std::string> values;

  bool put(const std::string& key, const std::string& value) override {
    values[key] = value;
    return true;
  }

  std::string get(const std::string& key, const std::string& fallback = {}) const override {
    auto it = values.find(key);
    return it == values.end() ? fallback : it->second;
  }
};
