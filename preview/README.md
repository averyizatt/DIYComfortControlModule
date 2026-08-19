# LVGL Live Preview

The desktop preview is configured for the **LVGL Live Preview** VS Code extension.
It is isolated from the ESP32 display, touch, CAN, and storage drivers, so preview
builds do not affect the PlatformIO firmware targets.

1. Install the recommended `themastercoder007.lvgl-live-preview` extension.
2. Open the extension's preview settings (gear icon) and select display width
   `480` and display height `320`. The adapter supports both the firmware's LVGL
   `8.3.11` and the extension's default LVGL `9.4.0`.
3. Open `preview/lvgl_live_preview.cpp` and run **LVGL: Start Live Preview**
   (`Ctrl+Shift+L`). The first run installs Emscripten and SDL and can take a few
   minutes.

The preview is a host-safe copy of the current `ScreenDashboard` LVGL UI. It uses
the same 480x320 geometry, header/content/navigation dimensions, theme colors,
page layouts, controls, and enabled Montserrat fonts. Hardware-only display,
touch, SPI, CAN, and SD operations are omitted.

A built-in simulated CAN feed automatically cycles RPM, speed, boost, meth duty,
GPS, and knock values so the copied data-driven screens remain active without
vehicle modules.
