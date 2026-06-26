#include "ui/asset_manager.h"

namespace ui {

bool AssetManager::begin(storage::SdManager* sd) {
  sd_ = sd;
  return sd_ != nullptr;
}

bool AssetManager::hasAsset(const char* path) const {
  if (!sd_ || !sd_->mounted() || !path) return false;
  return sd_->exists(path);
}

String AssetManager::resolveOrFallback(const char* path, const char* fallback) const {
  if (hasAsset(path)) return String(path);
  return String(fallback ? fallback : "");
}

}  // namespace ui
