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
