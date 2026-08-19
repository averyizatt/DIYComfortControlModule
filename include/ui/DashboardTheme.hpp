#pragma once

#include <cstdint>

namespace ui::dashboard_theme {

inline constexpr uint16_t width = 480;
inline constexpr uint16_t height = 320;
// The reference cockpit uses a 26 px title bar plus an 18 px system strip.
inline constexpr uint16_t headerHeight = 44;
inline constexpr uint16_t navigationHeight = 52;
inline constexpr uint16_t contentHeight = height - headerHeight - navigationHeight;

inline constexpr uint32_t background = 0x02070A;
inline constexpr uint32_t panel = 0x071014;
inline constexpr uint32_t heroPanel = 0x081318;
inline constexpr uint32_t button = 0x071014;
inline constexpr uint32_t buttonActive = 0x08262D;
inline constexpr uint32_t buttonBorder = 0x34434A;
inline constexpr uint32_t divider = 0x26343A;
inline constexpr uint32_t row = 0x050C10;
inline constexpr uint32_t meterTrack = 0x182329;
inline constexpr uint32_t text = 0xF2F3F3;
inline constexpr uint32_t textMuted = 0x7F8B94;
inline constexpr uint32_t good = 0x42CB54;
inline constexpr uint32_t warning = 0xF6B80B;
inline constexpr uint32_t bad = 0xEF352F;
inline constexpr uint32_t cyan = 0x20D6E4;
inline constexpr uint32_t blue = 0x2798E5;
inline constexpr uint32_t purple = 0x9850DD;

}  // namespace ui::dashboard_theme
