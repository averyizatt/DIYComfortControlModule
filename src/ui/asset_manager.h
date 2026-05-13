#pragma once

#include <Arduino.h>

#include "storage/sd_manager.h"

namespace ui {

class AssetManager {
 public:
  bool begin(storage::SdManager* sd);
  bool hasAsset(const char* path) const;
  String resolveOrFallback(const char* path, const char* fallback) const;

 private:
  storage::SdManager* sd_ = nullptr;
};

}  // namespace ui
