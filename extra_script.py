Import("env")
import os, shutil

env.Append(CPPPATH=[os.path.join(env["PROJECT_DIR"], "shared", "can_contract", "include")])

# LVGL 8.3 uses `#include "../../lv_conf.h"` (relative to its src/ dir) when
# neither LV_CONF_PATH nor LV_CONF_INCLUDE_SIMPLE is defined.  That resolves to
# .pio/libdeps/<env>/lv_conf.h — one level above the lvgl/ library folder.
# Copy the project lv_conf.h there so every library translation unit finds it
# via the reliable relative-path include (avoids GCC bug #80753 that causes
# __has_include-triggered #include "lv_conf.h" to silently fail).
_lvgl_libdeps = os.path.join(env["PROJECT_LIBDEPS_DIR"], env["PIOENV"])
_conf_src = os.path.join(env["PROJECT_DIR"], "include", "lv_conf.h")
_conf_dst = os.path.join(_lvgl_libdeps, "lv_conf.h")
if os.path.isdir(_lvgl_libdeps) and os.path.isfile(_conf_src):
    shutil.copy2(_conf_src, _conf_dst)

# ESPAsyncWebServer's AsyncJson.h guards its content with __has_include("ArduinoJson.h"),
# but ArduinoJson is not listed in its library.json dependencies. Without the include path
# present during library compilation PlatformIO's LDF never adds it, __has_include fails,
# ASYNC_JSON_SUPPORT stays 0, and AsyncCallbackJsonWebHandler's constructor is never
# compiled into the library object -- causing an undefined-reference at link time.
# Force ArduinoJson into the global CPPPATH so every compilation unit (including library
# sources) can find it.
arduinojson_src = os.path.join(
    env["PROJECT_LIBDEPS_DIR"], env["PIOENV"], "ArduinoJson", "src"
)
# Add unconditionally — directory exists by the time the compiler runs even if the
# library manager hasn't installed it yet when this script is first evaluated.
env.Append(CPPPATH=[arduinojson_src])

# Patch GFX Library for Arduino: spiFrequencyToClockDiv() gained a spi_t* first
# parameter in newer Arduino ESP32 framework (pioarduino 55.03.38+). The parameter
# is unused on non-P4 targets so nullptr is safe where _spi is not yet available.
def _patch_file(path, old, new):
    if not os.path.isfile(path):
        return
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()
    if old in content:
        with open(path, "w", encoding="utf-8") as f:
            f.write(content.replace(old, new))

_gfx_databus = os.path.join(
    env["PROJECT_LIBDEPS_DIR"], env["PIOENV"],
    "GFX Library for Arduino", "src", "databus"
)
_gfx_src = os.path.join(
    env["PROJECT_LIBDEPS_DIR"], env["PIOENV"],
    "GFX Library for Arduino", "src"
)
# This project uses LCD DC on GPIO46. On ESP32-S3 + Arduino 3.x, avoid direct
# GPIO register writes for GFX CS/DC if CCM_GFX_DISABLE_FAST_PINIO is enabled.
# A missed DC edge turns pixel bytes into commands and shows up as white blocks.
_patch_file(
    os.path.join(_gfx_src, "Arduino_DataBus.h"),
    "#elif defined(ESP32)\n#define USE_FAST_PINIO   ///< Use direct PORT register access\n#define HAS_PORT_SET_CLR ///< PORTs have set & clear registers\ntypedef uint32_t ARDUINOGFX_PORT_t;",
    "#elif defined(ESP32)\n#if !defined(CCM_GFX_DISABLE_FAST_PINIO) || (CCM_GFX_DISABLE_FAST_PINIO == 0)\n#define USE_FAST_PINIO   ///< Use direct PORT register access\n#define HAS_PORT_SET_CLR ///< PORTs have set & clear registers\n#endif\ntypedef uint32_t ARDUINOGFX_PORT_t;\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\ntypedef volatile ARDUINOGFX_PORT_t *PORTreg_t;\n#endif",
)
_patch_file(
    os.path.join(_gfx_src, "Arduino_DataBus.h"),
    "#elif defined(ESP32)\n#if !defined(CCM_GFX_DISABLE_FAST_PINIO) || (CCM_GFX_DISABLE_FAST_PINIO == 0)\n#define USE_FAST_PINIO   ///< Use direct PORT register access\n#define HAS_PORT_SET_CLR ///< PORTs have set & clear registers\n#endif\ntypedef uint32_t ARDUINOGFX_PORT_t;\n#elif defined(ESP8266)",
    "#elif defined(ESP32)\n#if !defined(CCM_GFX_DISABLE_FAST_PINIO) || (CCM_GFX_DISABLE_FAST_PINIO == 0)\n#define USE_FAST_PINIO   ///< Use direct PORT register access\n#define HAS_PORT_SET_CLR ///< PORTs have set & clear registers\n#endif\ntypedef uint32_t ARDUINOGFX_PORT_t;\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\ntypedef volatile ARDUINOGFX_PORT_t *PORTreg_t;\n#endif\n#elif defined(ESP8266)",
)
# _on_apb_change: local spi_t* _spi is already in scope
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPI.cpp"),
    "spiFrequencyToClockDiv(old_apb /",
    "spiFrequencyToClockDiv(_spi, old_apb /",
)
# begin(): _spi member not yet initialised at this point; nullptr is safe
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPI.cpp"),
    "_div = spiFrequencyToClockDiv(_speed);",
    "_div = spiFrequencyToClockDiv(nullptr, _speed);",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPI.cpp"),
    "GFX_INLINE void Arduino_ESP32SPI::DC_HIGH(void)\n{\n  *_dcPortSet = _dcPinMask;\n}",
    "GFX_INLINE void Arduino_ESP32SPI::DC_HIGH(void)\n{\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\n  digitalWrite(_dc, HIGH);\n#else\n  *_dcPortSet = _dcPinMask;\n#endif\n}",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPI.cpp"),
    "GFX_INLINE void Arduino_ESP32SPI::DC_LOW(void)\n{\n  *_dcPortClr = _dcPinMask;\n}",
    "GFX_INLINE void Arduino_ESP32SPI::DC_LOW(void)\n{\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\n  digitalWrite(_dc, LOW);\n#else\n  *_dcPortClr = _dcPinMask;\n#endif\n}",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPI.cpp"),
    "GFX_INLINE void Arduino_ESP32SPI::CS_HIGH(void)\n{\n  if (_cs != GFX_NOT_DEFINED)\n  {\n    *_csPortSet = _csPinMask;\n  }\n}",
    "GFX_INLINE void Arduino_ESP32SPI::CS_HIGH(void)\n{\n  if (_cs != GFX_NOT_DEFINED)\n  {\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\n    digitalWrite(_cs, HIGH);\n#else\n    *_csPortSet = _csPinMask;\n#endif\n  }\n}",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPI.cpp"),
    "GFX_INLINE void Arduino_ESP32SPI::CS_LOW(void)\n{\n  if (_cs != GFX_NOT_DEFINED)\n  {\n    *_csPortClr = _csPinMask;\n  }\n}",
    "GFX_INLINE void Arduino_ESP32SPI::CS_LOW(void)\n{\n  if (_cs != GFX_NOT_DEFINED)\n  {\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\n    digitalWrite(_cs, LOW);\n#else\n    *_csPortClr = _csPinMask;\n#endif\n  }\n}",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPIDMA.cpp"),
    "_div = spiFrequencyToClockDiv(_speed);",
    "_div = spiFrequencyToClockDiv(nullptr, _speed);",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPIDMA.cpp"),
    "GFX_INLINE void Arduino_ESP32SPIDMA::DC_HIGH(void)\n{\n  *_dcPortSet = _dcPinMask;\n}",
    "GFX_INLINE void Arduino_ESP32SPIDMA::DC_HIGH(void)\n{\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\n  digitalWrite(_dc, HIGH);\n#else\n  *_dcPortSet = _dcPinMask;\n#endif\n}",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPIDMA.cpp"),
    "GFX_INLINE void Arduino_ESP32SPIDMA::DC_LOW(void)\n{\n  *_dcPortClr = _dcPinMask;\n}",
    "GFX_INLINE void Arduino_ESP32SPIDMA::DC_LOW(void)\n{\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\n  digitalWrite(_dc, LOW);\n#else\n  *_dcPortClr = _dcPinMask;\n#endif\n}",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPIDMA.cpp"),
    "GFX_INLINE void Arduino_ESP32SPIDMA::CS_HIGH(void)\n{\n  if (_cs != GFX_NOT_DEFINED)\n  {\n    *_csPortSet = _csPinMask;\n  }\n}",
    "GFX_INLINE void Arduino_ESP32SPIDMA::CS_HIGH(void)\n{\n  if (_cs != GFX_NOT_DEFINED)\n  {\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\n    digitalWrite(_cs, HIGH);\n#else\n    *_csPortSet = _csPinMask;\n#endif\n  }\n}",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32SPIDMA.cpp"),
    "GFX_INLINE void Arduino_ESP32SPIDMA::CS_LOW(void)\n{\n  if (_cs != GFX_NOT_DEFINED)\n  {\n    *_csPortClr = _csPinMask;\n  }\n}",
    "GFX_INLINE void Arduino_ESP32SPIDMA::CS_LOW(void)\n{\n  if (_cs != GFX_NOT_DEFINED)\n  {\n#if defined(CCM_GFX_DISABLE_FAST_PINIO) && (CCM_GFX_DISABLE_FAST_PINIO != 0)\n    digitalWrite(_cs, LOW);\n#else\n    *_csPortClr = _csPinMask;\n#endif\n  }\n}",
)
# pioarduino + newer toolchains can treat narrowing in designated initializers
# as an error for this file. Clamp/cast _speed for pclk_hz field.
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32LCD8.cpp"),
    ".pclk_hz = _speed,",
    ".pclk_hz = static_cast<uint32_t>(_speed > 0 ? _speed : 0),",
)
_patch_file(
    os.path.join(_gfx_databus, "Arduino_ESP32LCD8.cpp"),
    "      .pclk_hz = _speed,",
    "      .pclk_hz = static_cast<uint32_t>(_speed > 0 ? _speed : 0),",
)
