#include "ui/screen_dashboard.h"

#include "ui/assets/ui_background.h"
#include "led/led_manager.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>

#include <SPI.h>
#include <esp_heap_caps.h>

#ifndef CCM_DISPLAY_BACKEND_ARDUINO_GFX
#define CCM_DISPLAY_BACKEND_ARDUINO_GFX 1
#endif

#ifndef CCM_DISPLAY_BACKEND_LOVYAN_GFX
#define CCM_DISPLAY_BACKEND_LOVYAN_GFX 2
#endif

#ifndef CCM_DISPLAY_BACKEND
#define CCM_DISPLAY_BACKEND CCM_DISPLAY_BACKEND_LOVYAN_GFX
#endif

#if CCM_DISPLAY_BACKEND == CCM_DISPLAY_BACKEND_LOVYAN_GFX
#define LGFX_USE_V1
#if __has_include(<LovyanGFX.hpp>)
#include <LovyanGFX.hpp>
#define CCM_HAS_LOVYAN_GFX 1
#else
#error "CCM_DISPLAY_BACKEND_LOVYAN_GFX selected, but LovyanGFX is not installed"
#endif
#else
#define CCM_HAS_LOVYAN_GFX 0
#endif

#if CCM_DISPLAY_BACKEND == CCM_DISPLAY_BACKEND_ARDUINO_GFX
#if __has_include(<Arduino_GFX_Library.h>)
#include <Arduino_GFX_Library.h>
#define CCM_HAS_ARDUINO_GFX 1
#else
#error "CCM_DISPLAY_BACKEND_ARDUINO_GFX selected, but GFX Library for Arduino is not installed"
#endif
#else
#define CCM_HAS_ARDUINO_GFX 0
#endif

#include "can/can_protocol.h"
#include "hal/SharedSpiBus.hpp"
#include "state/vehicle_state.h"

namespace ui {

// ---------------------------------------------------------------------------
// Static module-level storage
// ---------------------------------------------------------------------------

constexpr uint16_t kDisplayWidth = 480;
constexpr uint16_t kDisplayHeight = 320;

#ifndef CCM_DISPLAY_SPI_HZ
#define CCM_DISPLAY_SPI_HZ 8000000UL
#endif

#ifndef CCM_DISPLAY_USE_GFX_DMA
#define CCM_DISPLAY_USE_GFX_DMA 0
#endif

#ifndef CCM_DISPLAY_USE_LGFX_DMA
#define CCM_DISPLAY_USE_LGFX_DMA 0
#endif

#ifndef CCM_DISPLAY_PIXEL_MODE
#define CCM_DISPLAY_PIXEL_MODE 16
#endif

#if (CCM_DISPLAY_PIXEL_MODE != 16) && (CCM_DISPLAY_PIXEL_MODE != 18)
#error "CCM_DISPLAY_PIXEL_MODE must be 16 or 18"
#endif

#ifndef CCM_LVGL_DRAW_BUFFER_ROWS
#define CCM_LVGL_DRAW_BUFFER_ROWS 8
#endif

#ifndef CCM_LVGL_FLUSH_CHUNK_ROWS
#define CCM_LVGL_FLUSH_CHUNK_ROWS 1
#endif

#ifndef CCM_LGFX_FLUSH_CHUNK_ROWS
#define CCM_LGFX_FLUSH_CHUNK_ROWS 8
#endif

#ifndef CCM_DISPLAY_DIAG_BOOT
#define CCM_DISPLAY_DIAG_BOOT 0
#endif

#ifndef CCM_SCREEN_SERIAL_LOGS
#define CCM_SCREEN_SERIAL_LOGS 1
#endif

#ifndef CCM_DISPLAY_COMMAND_SETTLE_US
#define CCM_DISPLAY_COMMAND_SETTLE_US 1
#endif

#ifndef CCM_DISPLAY_STRIPE_GAP_US
#define CCM_DISPLAY_STRIPE_GAP_US 2
#endif

#ifndef CCM_LVGL_FULL_REFRESH
#define CCM_LVGL_FULL_REFRESH 0
#endif

#ifndef CCM_LVGL_FULL_WIDTH_DIRTY_BANDS
#define CCM_LVGL_FULL_WIDTH_DIRTY_BANDS 1
#endif

#ifndef CCM_LVGL_DIRTY_BAND_ROWS
#define CCM_LVGL_DIRTY_BAND_ROWS 8
#endif

#ifndef CCM_ILI9488_INVCTR
#define CCM_ILI9488_INVCTR 0x00
#endif

#ifndef CCM_UI_HEADER_UPDATE_MS
#define CCM_UI_HEADER_UPDATE_MS 250
#endif

#ifndef CCM_UI_DASH_UPDATE_MS
#define CCM_UI_DASH_UPDATE_MS 33
#endif

#ifndef CCM_UI_LIVE_UPDATE_MS
#define CCM_UI_LIVE_UPDATE_MS 100
#endif

#ifndef CCM_UI_HEAVY_UPDATE_MS
#define CCM_UI_HEAVY_UPDATE_MS 250
#endif

#ifndef CCM_UI_SCREEN_STATS_MS
#define CCM_UI_SCREEN_STATS_MS 5000
#endif

#ifndef CCM_TOUCH_STALE_RELEASE_MS
#define CCM_TOUCH_STALE_RELEASE_MS 260
#endif

#ifndef CCM_TOUCH_MAX_PRESS_MS
#define CCM_TOUCH_MAX_PRESS_MS 1200
#endif

#ifndef CCM_LVGL_TOUCH_MIN_FREE_BYTES
#define CCM_LVGL_TOUCH_MIN_FREE_BYTES 2048
#endif

#ifndef CCM_TOUCH_PRESS_STABLE_SAMPLES
#define CCM_TOUCH_PRESS_STABLE_SAMPLES 1
#endif

#ifndef CCM_TOUCH_RELEASE_STABLE_SAMPLES
#define CCM_TOUCH_RELEASE_STABLE_SAMPLES 2
#endif

#ifndef CCM_GPS_TIMEZONE_OFFSET_MINUTES
#define CCM_GPS_TIMEZONE_OFFSET_MINUTES -360
#endif

#ifndef CCM_UI_TAP_REPEAT_GUARD_MS
#define CCM_UI_TAP_REPEAT_GUARD_MS 60
#endif

#ifndef CCM_UI_ACTIONS_PER_TICK
#define CCM_UI_ACTIONS_PER_TICK 4
#endif

#ifndef CCM_UI_CONFIRM_TIMEOUT_MS
#define CCM_UI_CONFIRM_TIMEOUT_MS 3000
#endif

#ifndef CCM_UI_SWIPE_MIN_PX
#define CCM_UI_SWIPE_MIN_PX 90
#endif

#ifndef CCM_UI_SWIPE_MAX_OFF_AXIS_PX
#define CCM_UI_SWIPE_MAX_OFF_AXIS_PX 70
#endif

#ifndef CCM_UI_SWIPE_MAX_MS
#define CCM_UI_SWIPE_MAX_MS 800
#endif

#ifndef CCM_UI_DOUBLE_TAP_MS
#define CCM_UI_DOUBLE_TAP_MS 350
#endif

#ifndef CCM_DISPLAY_DIAG_VERBOSE
#define CCM_DISPLAY_DIAG_VERBOSE 0
#endif

#ifndef CCM_DISPLAY_DIAG_FLUSH_SAMPLE
#define CCM_DISPLAY_DIAG_FLUSH_SAMPLE 32
#endif

#ifndef CCM_DISPLAY_FULL_REPAINT_ON_PAGE_SWITCH
#define CCM_DISPLAY_FULL_REPAINT_ON_PAGE_SWITCH 0
#endif

#ifndef CCM_DISPLAY_18BIT_CHUNK_TRANSACTION
#define CCM_DISPLAY_18BIT_CHUNK_TRANSACTION 1
#endif

#ifndef CCM_DISPLAY_18BIT_HOLD_CS_PER_FLUSH
#define CCM_DISPLAY_18BIT_HOLD_CS_PER_FLUSH 1
#endif

#ifndef CCM_DISPLAY_CRITICAL_SPI_BURSTS
#define CCM_DISPLAY_CRITICAL_SPI_BURSTS 1
#endif

#ifndef CCM_DISPLAY_PAGE_STRESS_TEST
#define CCM_DISPLAY_PAGE_STRESS_TEST 0
#endif

#ifndef CCM_DISPLAY_PAGE_STRESS_MS
#define CCM_DISPLAY_PAGE_STRESS_MS 1500
#endif

#ifndef CCM_SD_BROWSER_AUTO_REFRESH_MS
#define CCM_SD_BROWSER_AUTO_REFRESH_MS 0
#endif

#ifndef CCM_SETTINGS_SAVE_DEBOUNCE_MS
#define CCM_SETTINGS_SAVE_DEBOUNCE_MS 1800
#endif

#ifndef CCM_LCD_FLUSH_SPI_WAIT_MS
#define CCM_LCD_FLUSH_SPI_WAIT_MS 2
#endif

// Stability-first defaults: bounded draw and flush strips prevent a single
// noisy SPI burst from corrupting a large continuous ILI9488 transfer.
constexpr uint16_t kDrawBufferRows =
    (CCM_LVGL_DRAW_BUFFER_ROWS < 2) ? 2 : CCM_LVGL_DRAW_BUFFER_ROWS;
constexpr uint16_t kFlushChunkRows =
    (CCM_LVGL_FLUSH_CHUNK_ROWS < 1) ? 1 : CCM_LVGL_FLUSH_CHUNK_ROWS;
constexpr uint16_t kCommandSettleUs = CCM_DISPLAY_COMMAND_SETTLE_US;
constexpr uint16_t kStripeGapUs = CCM_DISPLAY_STRIPE_GAP_US;
constexpr bool kLvglFullRefresh = CCM_LVGL_FULL_REFRESH != 0;
constexpr bool kFullWidthDirtyBands = CCM_LVGL_FULL_WIDTH_DIRTY_BANDS != 0;
constexpr uint16_t kDirtyBandRows =
    (CCM_LVGL_DIRTY_BAND_ROWS < 1) ? 1 : CCM_LVGL_DIRTY_BAND_ROWS;
constexpr size_t kDrawBufferPixels = static_cast<size_t>(kDisplayWidth) * kDrawBufferRows;
constexpr size_t kDrawBufferBytes = kDrawBufferPixels * sizeof(lv_color_t);
constexpr size_t kFlushChunkBytes = static_cast<size_t>(kDisplayWidth) * kFlushChunkRows * 3U;
static lv_color_t* s_buf1 = nullptr;
static lv_color_t* s_buf2 = nullptr;
#if CCM_HAS_ARDUINO_GFX && (CCM_DISPLAY_PIXEL_MODE == 18)
static uint8_t* s_flushChunks[2] = {};
static uint8_t s_nextFlushChunk = 0;
#endif

// Context structs for callbacks that need both self-pointer and a small value.
// Static storage – one ScreenDashboard instance only, set during buildXxxPage().
struct NavCtx     { ScreenDashboard* self; uint8_t page; };
struct LedModeCtx { ScreenDashboard* self; uint8_t channel; state::LedMode mode; };
struct LedShowCtx { ScreenDashboard* self; state::LedMode mode; };
struct LedColorCtx { ScreenDashboard* self; uint32_t color; };
struct SdFileRowCtx { ScreenDashboard* self; uint8_t row; };

static NavCtx     s_navCtxs[8];
static LedModeCtx s_ledModeCtxs[5][2];
static LedShowCtx s_ledShowCtxs[5];
static LedColorCtx s_ledColorCtxs[6];
static SdFileRowCtx s_sdFileRowCtxs[5];

#ifndef CCM_LED_LOW_WHITE_BRIGHTNESS
#define CCM_LED_LOW_WHITE_BRIGHTNESS 35
#endif

#ifndef CCM_LED_HIGH_WHITE_BRIGHTNESS
#define CCM_LED_HIGH_WHITE_BRIGHTNESS 180
#endif

const char* ledZoneName(uint8_t row) {
  switch (row) {
    case 0: return "ALL";
    case 1: return "DRIVER";
    case 2: return "PASS";
    case 3: return "TOP";
    case 4: return "BOTTOM";
    default: return "?";
  }
}

const char* ledUiModeName(led::LedUiMode mode) {
  switch (mode) {
    case led::LedUiMode::Off: return "OFF";
    case led::LedUiMode::LowWhite: return "LOW_WHITE";
    case led::LedUiMode::HighWhite: return "HIGH_WHITE";
    default: return "UNKNOWN";
  }
}

uint8_t ledUiModeBrightness(led::LedUiMode mode) {
  switch (mode) {
    case led::LedUiMode::LowWhite: return CCM_LED_LOW_WHITE_BRIGHTNESS;
    case led::LedUiMode::HighWhite: return CCM_LED_HIGH_WHITE_BRIGHTNESS;
    case led::LedUiMode::Off:
    default: return 0;
  }
}

bool ledModeToUiMode(state::LedMode mode, led::LedUiMode& out) {
  switch (mode) {
    case state::LedMode::OFF:
      out = led::LedUiMode::Off;
      return true;
    case state::LedMode::LOW_LIGHT:
      out = led::LedUiMode::LowWhite;
      return true;
    case state::LedMode::HIGH_LIGHT:
      out = led::LedUiMode::HighWhite;
      return true;
    default:
      return false;
  }
}

void applyLedUiModeToState(state::VehicleState& vs, led::LedUiMode mode) {
  const bool enabled = mode != led::LedUiMode::Off;
  const uint8_t brightness = ledUiModeBrightness(mode);
  const state::LedMode stateMode =
      (mode == led::LedUiMode::HighWhite)
          ? state::LedMode::HIGH_LIGHT
          : ((mode == led::LedUiMode::LowWhite) ? state::LedMode::LOW_LIGHT : state::LedMode::OFF);

  vs.led_channel_1_enabled = true;
  vs.led_channel_2_enabled = enabled;
  vs.led_channel_3_enabled = enabled;
  vs.led_channel_1_mode = state::LedMode::RPM_GAUGE;
  vs.led_channel_2_mode = stateMode;
  vs.led_channel_3_mode = stateMode;
  vs.led_channel_1_brightness = 180;
  vs.led_channel_2_brightness = brightness;
  vs.led_channel_3_brightness = brightness;
  for (uint8_t i = 0; i < state::kLedZoneCount; ++i) {
    vs.led_zone_enabled[i] = enabled;
    vs.led_zone_mode[i] = stateMode;
    vs.led_zone_brightness[i] = brightness;
  }
  vs.led_startup_preview = false;
}

uint8_t brightnessForLedMode(state::LedMode mode) {
  switch (mode) {
    case state::LedMode::STATIC_COLOR: return 160;
    case state::LedMode::LOW_LIGHT: return ledUiModeBrightness(led::LedUiMode::LowWhite);
    case state::LedMode::HIGH_LIGHT: return ledUiModeBrightness(led::LedUiMode::HighWhite);
    case state::LedMode::RAINBOW:
    case state::LedMode::BREATHING:
    case state::LedMode::RPM_REACTIVE:
    case state::LedMode::WARNING_FLASH:
      return 140;
    case state::LedMode::OFF:
    default:
      return 0;
  }
}

void applyLedModeToZone(state::VehicleState& vs, uint8_t zone, state::LedMode mode) {
  if (zone >= state::kLedZoneCount) return;
  const uint8_t brightness = brightnessForLedMode(mode);
  const bool enabled = mode != state::LedMode::OFF;
  vs.led_zone_enabled[zone] = enabled;
  vs.led_zone_mode[zone] = mode;
  vs.led_zone_brightness[zone] = brightness;
  vs.led_startup_preview = false;

  vs.led_channel_1_enabled = true;
  vs.led_channel_1_mode = state::LedMode::RPM_GAUGE;
  vs.led_channel_1_brightness = 180;
  vs.led_channel_2_enabled = false;
  vs.led_channel_2_mode = state::LedMode::OFF;
  vs.led_channel_2_brightness = 0;
  vs.led_channel_3_enabled = false;
  vs.led_channel_3_mode = state::LedMode::OFF;
  vs.led_channel_3_brightness = 0;
  for (uint8_t i = 0; i < state::kLedZoneCount; ++i) {
    if (!vs.led_zone_enabled[i] || vs.led_zone_mode[i] == state::LedMode::OFF) continue;
    if (i < 2U) {
      vs.led_channel_3_enabled = true;
      vs.led_channel_3_mode = vs.led_zone_mode[i];
      vs.led_channel_3_brightness = vs.led_zone_brightness[i];
    } else {
      vs.led_channel_2_enabled = true;
      vs.led_channel_2_mode = vs.led_zone_mode[i];
      vs.led_channel_2_brightness = vs.led_zone_brightness[i];
    }
  }
}

const char* ledModeButtonName(state::LedMode mode) {
  switch (mode) {
    case state::LedMode::LOW_LIGHT: return "LOW";
    case state::LedMode::HIGH_LIGHT: return "HIGH";
    case state::LedMode::STATIC_COLOR: return "COLOR";
    case state::LedMode::RAINBOW: return "RAINBOW";
    case state::LedMode::BREATHING: return "BREATHE";
    case state::LedMode::RPM_REACTIVE: return "CHASE";
    case state::LedMode::WARNING_FLASH: return "SPARKLE";
    case state::LedMode::OFF: return "OFF";
    default: return "MODE";
  }
}

void applyLedModeToAllZones(state::VehicleState& vs, state::LedMode mode) {
  for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
    applyLedModeToZone(vs, zone, mode);
  }
}

void applyLedColorToAllZones(state::VehicleState& vs, uint32_t color) {
  for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
    vs.led_zone_color[zone] = color;
    applyLedModeToZone(vs, zone, state::LedMode::STATIC_COLOR);
  }
}

constexpr uint32_t kDisplaySpiHz = CCM_DISPLAY_SPI_HZ;
constexpr bool kDisplayUseGfxDma = CCM_DISPLAY_USE_GFX_DMA != 0;
constexpr bool kDisplayUseLgfxDma = CCM_DISPLAY_USE_LGFX_DMA != 0;
constexpr uint8_t kDisplayPixelMode = CCM_DISPLAY_PIXEL_MODE;
constexpr uint16_t kLovyanFlushChunkRows =
    (CCM_LGFX_FLUSH_CHUNK_ROWS < 1) ? 1 : CCM_LGFX_FLUSH_CHUNK_ROWS;
constexpr bool kDisplayDiagBoot = CCM_DISPLAY_DIAG_BOOT != 0;
constexpr bool kDisplayDiagVerbose = CCM_DISPLAY_DIAG_VERBOSE != 0;
constexpr uint16_t kDisplayDiagFlushSample =
    (CCM_DISPLAY_DIAG_FLUSH_SAMPLE < 1) ? 1 : CCM_DISPLAY_DIAG_FLUSH_SAMPLE;
constexpr bool kFullRepaintOnPageSwitch = CCM_DISPLAY_FULL_REPAINT_ON_PAGE_SWITCH != 0;
constexpr bool kDisplay18BitChunkTransaction = CCM_DISPLAY_18BIT_CHUNK_TRANSACTION != 0;
constexpr bool kDisplay18BitHoldCsPerFlush = CCM_DISPLAY_18BIT_HOLD_CS_PER_FLUSH != 0;
constexpr bool kDisplayCriticalSpiBursts = CCM_DISPLAY_CRITICAL_SPI_BURSTS != 0;
constexpr bool kPageStressTest = CCM_DISPLAY_PAGE_STRESS_TEST != 0;
constexpr uint32_t kPageStressPeriodMs =
    (CCM_DISPLAY_PAGE_STRESS_MS < 250) ? 250U : static_cast<uint32_t>(CCM_DISPLAY_PAGE_STRESS_MS);
constexpr uint32_t kSdBrowserAutoRefreshMs = CCM_SD_BROWSER_AUTO_REFRESH_MS;
constexpr const char* kDisplayBackendName =
#if CCM_DISPLAY_BACKEND == CCM_DISPLAY_BACKEND_LOVYAN_GFX
    "LovyanGFX";
#elif CCM_DISPLAY_BACKEND == CCM_DISPLAY_BACKEND_ARDUINO_GFX
    "Arduino_GFX";
#else
    "Unknown";
#endif
constexpr uint32_t kHeaderUpdatePeriodMs = CCM_UI_HEADER_UPDATE_MS;
constexpr uint32_t kDashUpdatePeriodMs = CCM_UI_DASH_UPDATE_MS;
constexpr uint32_t kLivePageUpdatePeriodMs = CCM_UI_LIVE_UPDATE_MS;
constexpr uint32_t kHeavyPageUpdatePeriodMs = CCM_UI_HEAVY_UPDATE_MS;
constexpr uint32_t kScreenStatsPeriodMs = CCM_UI_SCREEN_STATS_MS;
constexpr uint8_t kIli9488Caset = 0x2A;
constexpr uint8_t kIli9488Paset = 0x2B;
constexpr uint8_t kIli9488Ramwr = 0x2C;
constexpr uint8_t kIli9488Invctr = 0xB4;
constexpr uint8_t kIli9488InvctrValue = static_cast<uint8_t>(CCM_ILI9488_INVCTR);
constexpr const char* kStatusColorOn  = "#00C853";
constexpr const char* kStatusColorOff = "#FF3B30";
constexpr uint32_t kUiColorBg = 0x080202;
constexpr uint32_t kUiColorPanel = 0x160707;
constexpr uint32_t kUiColorButton = 0x260A0A;
constexpr uint32_t kUiColorButtonActive = 0xC62828;
constexpr uint32_t kUiColorButtonBorder = 0x6D1A1A;
constexpr uint32_t kUiColorDivider = 0x351010;
constexpr uint32_t kUiColorRow = 0x120505;
constexpr uint32_t kUiColorRowSelected = 0x5B1515;
constexpr uint32_t kUiColorMeterTrack = 0x220C0C;
constexpr uint32_t kUiColorTextMuted = 0xB08686;
constexpr uint32_t kUiColorText = 0xF6EAEA;
constexpr uint32_t kUiColorHeroPanel = 0x1E0808;
constexpr uint32_t kUiColorGood = 0x00C853;
constexpr uint32_t kUiColorWarn = 0xFF9500;
constexpr uint32_t kUiColorBad = 0xFF3B30;
constexpr float kGpsZeroClampMph         = 5.0f;
constexpr uint8_t kGpsStatusDeadReckoned = 0x20;
constexpr uint8_t kGpsStatusSpikeRejected = 0x40;
constexpr uint8_t kGpsStatusZeroClamped = 0x80;
constexpr uint32_t kMethSensorStaleMs    = 1000;
constexpr TickType_t kLcdFlushSpiWaitTicks =
  pdMS_TO_TICKS((CCM_LCD_FLUSH_SPI_WAIT_MS < 0) ? 0 : CCM_LCD_FLUSH_SPI_WAIT_MS);

uint32_t s_flushCount = 0;
uint32_t s_flushPixels = 0;
uint32_t s_flushBytes = 0;
uint32_t s_flushMaxAreaPixels = 0;
uint32_t s_flushFullWidthBands = 0;
uint32_t s_flushWriteCalls = 0;
uint32_t s_flushMaxUs = 0;
uint32_t s_flushTotalUs = 0;
uint32_t s_flushClipped = 0;
uint32_t s_flushSkipped = 0;
uint32_t s_flushSeq = 0;
uint16_t s_flushMinX = kDisplayWidth;
uint16_t s_flushMinY = kDisplayHeight;
uint16_t s_flushMaxX = 0;
uint16_t s_flushMaxY = 0;
uint32_t s_frameCount = 0;
uint32_t s_frameFlushCurrent = 0;
uint32_t s_frameFlushMax = 0;
uint32_t s_frameFlushLast = 0;
uint32_t s_frameFlushSum = 0;
uint32_t s_statsLastMs = 0;
uint32_t s_lvHandlerMaxMs = 0;
uint32_t s_lvHandlerSlowCount = 0;

uint32_t lvglFreeBytes() {
  lv_mem_monitor_t mon{};
  lv_mem_monitor(&mon);
  return mon.free_size;
}

#if CCM_HAS_LOVYAN_GFX
class CabinLgfxDisplay : public lgfx::LGFX_Device {
 public:
  CabinLgfxDisplay(uint8_t lcdCs, uint8_t lcdRst, uint8_t lcdDc,
                   uint8_t spiSck, uint8_t spiMosi, uint8_t spiMiso) {
    {
      auto cfg = bus_.config();
#if defined(SPI2_HOST)
      cfg.spi_host = SPI2_HOST;
#endif
      cfg.spi_mode = 0;
      cfg.freq_write = kDisplaySpiHz;
      cfg.freq_read = 8000000;
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = kDisplayUseLgfxDma ? SPI_DMA_CH_AUTO : 0;
      cfg.pin_sclk = spiSck;
      cfg.pin_mosi = spiMosi;
      cfg.pin_miso = spiMiso;
      cfg.pin_dc = lcdDc;
      bus_.config(cfg);
      panel_.setBus(&bus_);
    }

    {
      auto cfg = panel_.config();
      cfg.pin_cs = lcdCs;
      cfg.pin_rst = lcdRst;
      cfg.pin_busy = -1;
      cfg.panel_width = 320;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.readable = false;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;
      panel_.config(cfg);
    }

    setPanel(&panel_);
  }

 private:
  lgfx::Bus_SPI bus_;
  lgfx::Panel_ILI9488 panel_;
};

static CabinLgfxDisplay* s_lgfx = nullptr;
#endif

#if CCM_HAS_ARDUINO_GFX
static Arduino_DataBus* s_bus = nullptr;
static Arduino_TFT* s_gfx = nullptr;
#endif

void* allocDmaBuffer(size_t bytes, const char* name) {
  void* ptr = heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  if (!ptr) {
    ptr = heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_DMA);
  }
  if (!ptr) {
    Serial.printf("[SCREEN] DMA buffer alloc FAILED name=%s bytes=%lu heap=%lu dma_free=%lu dma_big=%lu\n",
                   name,
                   static_cast<unsigned long>(bytes),
                   static_cast<unsigned long>(ESP.getFreeHeap()),
                   static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_DMA)),
                   static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)));
    return nullptr;
  }
  memset(ptr, 0, bytes);
  return ptr;
}

void releaseDisplayBuffers() {
  if (s_buf1) {
    heap_caps_free(s_buf1);
    s_buf1 = nullptr;
  }
  if (s_buf2) {
    heap_caps_free(s_buf2);
    s_buf2 = nullptr;
  }
#if CCM_HAS_ARDUINO_GFX && (CCM_DISPLAY_PIXEL_MODE == 18)
  for (uint8_t i = 0; i < 2; ++i) {
    if (s_flushChunks[i]) {
      heap_caps_free(s_flushChunks[i]);
      s_flushChunks[i] = nullptr;
    }
  }
#endif
}

bool ensureDisplayBuffers() {
  if (s_buf1 && s_buf2
#if CCM_HAS_ARDUINO_GFX && (CCM_DISPLAY_PIXEL_MODE == 18)
      && s_flushChunks[0] && s_flushChunks[1]
#endif
  ) {
    return true;
  }

  releaseDisplayBuffers();
  s_buf1 = static_cast<lv_color_t*>(allocDmaBuffer(kDrawBufferBytes, "lvgl_draw_a"));
  s_buf2 = static_cast<lv_color_t*>(allocDmaBuffer(kDrawBufferBytes, "lvgl_draw_b"));
#if CCM_HAS_ARDUINO_GFX && (CCM_DISPLAY_PIXEL_MODE == 18)
  s_flushChunks[0] = static_cast<uint8_t*>(allocDmaBuffer(kFlushChunkBytes, "flush_rgb888_a"));
  s_flushChunks[1] = static_cast<uint8_t*>(allocDmaBuffer(kFlushChunkBytes, "flush_rgb888_b"));
#endif

  if (!s_buf1 || !s_buf2
#if CCM_HAS_ARDUINO_GFX && (CCM_DISPLAY_PIXEL_MODE == 18)
      || !s_flushChunks[0] || !s_flushChunks[1]
#endif
  ) {
    releaseDisplayBuffers();
    return false;
  }

  Serial.printf("[SCREEN] buffers draw=%lux%u rows (%luB each) flush=%u rows (%luB x2) dma_free=%lu dma_big=%lu\n",
                 static_cast<unsigned long>(kDisplayWidth),
                 static_cast<unsigned>(kDrawBufferRows),
                 static_cast<unsigned long>(kDrawBufferBytes),
#if CCM_HAS_ARDUINO_GFX
#if CCM_DISPLAY_PIXEL_MODE == 18
                 static_cast<unsigned>(kFlushChunkRows),
                 static_cast<unsigned long>(kFlushChunkBytes),
#else
                 static_cast<unsigned>(kFlushChunkRows),
                 static_cast<unsigned long>(0),
#endif
#else
                 static_cast<unsigned>(kLovyanFlushChunkRows),
                 static_cast<unsigned long>(0),
#endif
                 static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_DMA)),
                 static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)));
  return true;
}

uint16_t rgb888To565(uint32_t rgb) {
  const uint8_t r = static_cast<uint8_t>((rgb >> 16) & 0xFFU);
  const uint8_t g = static_cast<uint8_t>((rgb >> 8) & 0xFFU);
  const uint8_t b = static_cast<uint8_t>(rgb & 0xFFU);
  return static_cast<uint16_t>(((r & 0xF8U) << 8) |
                               ((g & 0xFCU) << 3) |
                               (b >> 3));
}

void displayFillScreen(uint32_t rgb) {
  hal::SharedSpiBusLock spiLock("LCD:fill");
#if CCM_HAS_LOVYAN_GFX
  if (s_lgfx) {
    s_lgfx->fillScreen(rgb);
  }
#elif CCM_HAS_ARDUINO_GFX
  if (s_gfx) {
    s_gfx->fillScreen(rgb888To565(rgb));
  }
#else
  (void)rgb;
#endif
}

void displayFillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t rgb) {
  hal::SharedSpiBusLock spiLock("LCD:rect");
#if CCM_HAS_LOVYAN_GFX
  if (s_lgfx) {
    s_lgfx->fillRect(x, y, w, h, rgb);
  }
#elif CCM_HAS_ARDUINO_GFX
  if (s_gfx) {
    s_gfx->fillRect(x, y, w, h, rgb888To565(rgb));
  }
#else
  (void)x; (void)y; (void)w; (void)h; (void)rgb;
#endif
}

void runDisplayDiagnostic() {
  if (!kDisplayDiagBoot) {
    return;
  }

  Serial.println("[SCREEN] boot diagnostic start");
  const uint32_t fills[] = {
      0x000000U, 0xFF0000U, 0x00FF00U, 0x0000FFU, 0xFFFFFFU, 0x000000U};
  for (uint32_t color : fills) {
    displayFillScreen(color);
    delay(120);
  }

  const uint32_t bars[] = {
      0xFF0000U, 0xFFFF00U, 0x00FF00U, 0x00FFFFU, 0x0000FFU, 0xFF00FFU};
  const int32_t barW = kDisplayWidth / static_cast<int32_t>(sizeof(bars) / sizeof(bars[0]));
  for (uint8_t i = 0; i < sizeof(bars) / sizeof(bars[0]); ++i) {
    displayFillRect(static_cast<int32_t>(i) * barW, 0, barW, kDisplayHeight, bars[i]);
  }
  delay(250);

  displayFillScreen(0x000000U);
  for (int32_t y = kDisplayHeight - 32; y < kDisplayHeight; y += 2) {
    displayFillRect(0, y, kDisplayWidth, 1, (y & 2) ? 0xFFFFFFU : 0x00FFFFU);
  }
  delay(400);
  displayFillScreen(0x000000U);
  Serial.println("[SCREEN] boot diagnostic OK");
}

const char* methFlowName(uint8_t flow) {
  switch (flow) {
    case 1: return "OK";
    case 2: return "LOW";
    case 3: return "NO";
    default: return "UNK";
  }
}

const char* tailModeName(uint8_t mode) {
  switch (mode) {
    case 0: return "STOCK";
    case 1: return "SEQ";
    case 2: return "DEMO";
    default: return "SHOW";
  }
}

void setLabelText(lv_obj_t* label, const char* text) {
  if (!label) return;
  if (!text) text = "";

  const char* current = lv_label_get_text(label);
  if (current && strcmp(current, text) == 0) {
    return;
  }

  lv_label_set_text(label, text);
}

void setLabelTextStatic(lv_obj_t* label, char* storage, size_t storageLen, const char* text) {
  if (!label || !storage || storageLen == 0) return;
  if (!text) text = "";

  if (strncmp(storage, text, storageLen) == 0) {
    return;
  }

  snprintf(storage, storageLen, "%s", text);
  storage[storageLen - 1] = '\0';
  lv_label_set_text_static(label, storage);
}

void setObjHidden(lv_obj_t* obj, bool hidden) {
  if (!obj) return;
  const bool isHidden = lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN);
  if (hidden == isHidden) return;
  if (hidden) lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

void setBgColor(lv_obj_t* obj, lv_color_t color, uint32_t part) {
  if (!obj) return;
  if ((part & LV_STATE_ANY) == 0 &&
      lv_color_to32(lv_obj_get_style_bg_color(obj, part)) == lv_color_to32(color)) {
    return;
  }
  lv_obj_set_style_bg_color(obj, color, part);
}

void setTextColor(lv_obj_t* obj, lv_color_t color, uint32_t part) {
  if (!obj) return;
  if ((part & LV_STATE_ANY) == 0 &&
      lv_color_to32(lv_obj_get_style_text_color(obj, part)) == lv_color_to32(color)) {
    return;
  }
  lv_obj_set_style_text_color(obj, color, part);
}

lv_style_selector_t mainSelector(lv_state_t state = LV_STATE_DEFAULT) {
  return static_cast<lv_style_selector_t>(LV_PART_MAIN) | static_cast<lv_style_selector_t>(state);
}

void setNavButtonBg(lv_obj_t* btn, lv_color_t color) {
  if (!btn) return;
  setBgColor(btn, color, LV_PART_MAIN);
  setBgColor(btn, color, mainSelector(LV_STATE_PRESSED));
  setBgColor(btn, color, mainSelector(LV_STATE_FOCUSED));
  setBgColor(btn, color, mainSelector(LV_STATE_FOCUS_KEY));
  setBgColor(btn, color, mainSelector(LV_STATE_CHECKED));
}

void flattenNavButtonState(lv_obj_t* btn, lv_state_t state) {
  if (!btn) return;
  const lv_style_selector_t selector = mainSelector(state);
  lv_obj_set_style_radius(btn, 0, selector);
  lv_obj_set_style_border_width(btn, 0, selector);
  lv_obj_set_style_shadow_width(btn, 0, selector);
  lv_obj_set_style_outline_width(btn, 0, selector);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, selector);
}

void styleChip(lv_obj_t* obj) {
  if (!obj) return;
  lv_obj_set_style_radius(obj, 8, LV_PART_MAIN);
  setNavButtonBg(obj, lv_color_hex(kUiColorPanel));
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(obj, lv_color_hex(kUiColorButtonBorder), LV_PART_MAIN);
  lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN);
  lv_obj_set_style_pad_left(obj, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_right(obj, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_top(obj, 3, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(obj, 3, LV_PART_MAIN);
}

void styleActionButton(lv_obj_t* obj) {
  if (!obj) return;
  lv_obj_set_style_radius(obj, 8, LV_PART_MAIN);
  lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(obj, lv_color_hex(kUiColorButtonBorder), LV_PART_MAIN);
  setBgColor(obj, lv_color_hex(kUiColorButton), LV_PART_MAIN);
  setBgColor(obj, lv_color_hex(kUiColorButtonActive), mainSelector(LV_STATE_PRESSED));
  setBgColor(obj, lv_color_hex(kUiColorButtonActive), mainSelector(LV_STATE_FOCUSED));
  setBgColor(obj, lv_color_hex(kUiColorButtonActive), mainSelector(LV_STATE_FOCUS_KEY));
  setBgColor(obj, lv_color_hex(kUiColorButtonActive), mainSelector(LV_STATE_CHECKED));
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, mainSelector(LV_STATE_PRESSED));
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, mainSelector(LV_STATE_FOCUSED));
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, mainSelector(LV_STATE_FOCUS_KEY));
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, mainSelector(LV_STATE_CHECKED));
  lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_shadow_color(obj, lv_color_hex(kUiColorBg), LV_PART_MAIN);
}

static lv_obj_t* makeLabel(lv_obj_t* parent, lv_coord_t x, lv_coord_t y,
                            lv_coord_t w, const char* text,
                            const lv_font_t* font);

void stylePanel(lv_obj_t* obj, uint32_t color = kUiColorPanel, lv_opa_t opa = LV_OPA_COVER) {
  if (!obj) return;
  const lv_color_t panelColor = lv_color_hex(color);
  lv_obj_set_style_radius(obj, 8, LV_PART_MAIN);
  lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(obj, lv_color_hex(kUiColorDivider), LV_PART_MAIN);
  setBgColor(obj, panelColor, LV_PART_MAIN);
  setBgColor(obj, panelColor, mainSelector(LV_STATE_PRESSED));
  setBgColor(obj, panelColor, mainSelector(LV_STATE_FOCUSED));
  setBgColor(obj, panelColor, mainSelector(LV_STATE_FOCUS_KEY));
  setBgColor(obj, panelColor, mainSelector(LV_STATE_CHECKED));
  lv_obj_set_style_bg_opa(obj, opa, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(obj, opa, mainSelector(LV_STATE_PRESSED));
  lv_obj_set_style_bg_opa(obj, opa, mainSelector(LV_STATE_FOCUSED));
  lv_obj_set_style_bg_opa(obj, opa, mainSelector(LV_STATE_FOCUS_KEY));
  lv_obj_set_style_bg_opa(obj, opa, mainSelector(LV_STATE_CHECKED));
  lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(obj, 6, LV_PART_MAIN);
  lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

lv_obj_t* makePanel(lv_obj_t* parent, lv_coord_t x, lv_coord_t y,
                    lv_coord_t w, lv_coord_t h,
                    uint32_t color = kUiColorPanel,
                    lv_opa_t opa = LV_OPA_COVER) {
  lv_obj_t* panel = lv_obj_create(parent);
  lv_obj_set_pos(panel, x, y);
  lv_obj_set_size(panel, w, h);
  stylePanel(panel, color, opa);
  return panel;
}

lv_obj_t* makeSectionTitle(lv_obj_t* parent, lv_coord_t x, lv_coord_t y,
                           lv_coord_t w, const char* text) {
  lv_obj_t* label = makeLabel(parent, x, y, w, text, &lv_font_montserrat_16);
  setTextColor(label, lv_color_hex(kUiColorText), 0);
  lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
  return label;
}

lv_obj_t* makeStatusPill(lv_obj_t* parent, lv_coord_t x, lv_coord_t y,
                         lv_coord_t w, const char* text, lv_color_t stateColor) {
  lv_obj_t* pill = makeLabel(parent, x, y, w, text, &lv_font_montserrat_12);
  lv_obj_set_height(pill, 24);
  lv_obj_set_style_text_align(pill, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(pill, LV_LABEL_LONG_DOT);
  stylePanel(pill, kUiColorRow);
  lv_obj_set_style_border_color(pill, stateColor, LV_PART_MAIN);
  setTextColor(pill, stateColor, 0);
  return pill;
}

lv_obj_t* makeMetricTile(lv_obj_t* parent, lv_coord_t x, lv_coord_t y,
                         lv_coord_t w, lv_coord_t h, const char* text,
                         const lv_font_t* font = &lv_font_montserrat_20) {
  lv_obj_t* tile = makeLabel(parent, x, y, w, text, font);
  lv_obj_set_height(tile, h);
  lv_obj_set_style_text_align(tile, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(tile, LV_LABEL_LONG_WRAP);
  stylePanel(tile);
  return tile;
}

void stylePrimaryButton(lv_obj_t* obj) {
  styleActionButton(obj);
  lv_obj_set_style_border_color(obj, lv_color_hex(kUiColorButtonActive), LV_PART_MAIN);
}

void styleSecondaryButton(lv_obj_t* obj) {
  styleActionButton(obj);
  lv_obj_set_style_border_color(obj, lv_color_hex(kUiColorButtonBorder), LV_PART_MAIN);
}

void styleDangerButton(lv_obj_t* obj) {
  styleActionButton(obj);
  setBgColor(obj, lv_color_hex(0x2A1111), LV_PART_MAIN);
  lv_obj_set_style_border_color(obj, lv_color_hex(kUiColorBad), LV_PART_MAIN);
}

void animSetOpa(void* obj, int32_t v) {
  lv_obj_set_style_opa(static_cast<lv_obj_t*>(obj), static_cast<lv_opa_t>(v), LV_PART_MAIN);
}

void animSetY(void* obj, int32_t v) {
  lv_obj_set_y(static_cast<lv_obj_t*>(obj), static_cast<lv_coord_t>(v));
}

void animatePageEnter(lv_obj_t* page) {
  if (!page) return;
  lv_anim_del(page, animSetOpa);
  lv_anim_del(page, animSetY);
  lv_obj_set_style_opa(page, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_y(page, 0);
  lv_obj_invalidate(page);
}

const char* methStateName(state::MethState st) {
  switch (st) {
    case state::MethState::OFF: return "OFF";
    case state::MethState::ARMED: return "ARM";
    case state::MethState::SPRAYING: return "SPRAY";
    case state::MethState::FAULT: return "FAULT";
    case state::MethState::TEST: return "TEST";
    default: return "?";
  }
}

// ---------------------------------------------------------------------------
// LVGL driver callbacks (static)
// ---------------------------------------------------------------------------

void ScreenDashboard::lvglFlushCb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* colors) {
  const uint32_t flushStartUs = micros();
  const int32_t reqX1 = area ? area->x1 : 0;
  const int32_t reqY1 = area ? area->y1 : 0;
  const int32_t reqX2 = area ? area->x2 : -1;
  const int32_t reqY2 = area ? area->y2 : -1;
  bool transferred = false;
  uint32_t transferBytes = 0;
  uint32_t transferPixels = 0;
#if CCM_HAS_LOVYAN_GFX
  if (s_lgfx && area && colors) {
    const int32_t srcW = area->x2 - area->x1 + 1;
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;

    if (srcW > 0 && x1 < kDisplayWidth && y1 < kDisplayHeight && x2 >= 0 && y2 >= 0) {
      const bool clipped = x1 < 0 || y1 < 0 || x2 >= kDisplayWidth || y2 >= kDisplayHeight;
      if (x1 < 0) x1 = 0;
      if (y1 < 0) y1 = 0;
      if (x2 >= kDisplayWidth) x2 = kDisplayWidth - 1;
      if (y2 >= kDisplayHeight) y2 = kDisplayHeight - 1;

      const int32_t w = x2 - x1 + 1;
      const int32_t h = y2 - y1 + 1;
      if (w > 0 && h > 0 && w <= kDisplayWidth) {
        const uint32_t areaPixels = static_cast<uint32_t>(w * h);
        ++s_flushCount;
        ++s_frameFlushCurrent;
        s_flushPixels += areaPixels;
        s_flushBytes += areaPixels * 2U;
        transferPixels = areaPixels;
        transferBytes = areaPixels * 2U;
        transferred = true;
        if (clipped) {
          ++s_flushClipped;
        }
        if (x1 < s_flushMinX) s_flushMinX = static_cast<uint16_t>(x1);
        if (y1 < s_flushMinY) s_flushMinY = static_cast<uint16_t>(y1);
        if (x2 > s_flushMaxX) s_flushMaxX = static_cast<uint16_t>(x2);
        if (y2 > s_flushMaxY) s_flushMaxY = static_cast<uint16_t>(y2);
        if (areaPixels > s_flushMaxAreaPixels) {
          s_flushMaxAreaPixels = areaPixels;
        }
        if (w == kDisplayWidth) {
          ++s_flushFullWidthBands;
        }

        const uint16_t* srcBase = reinterpret_cast<const uint16_t*>(colors) +
                                  ((y1 - area->y1) * srcW) +
                                  (x1 - area->x1);

        hal::SharedSpiBusLock spiLock("LCD:flush", kLcdFlushSpiWaitTicks);
        if (!spiLock.locked()) {
          lv_disp_flush_ready(drv);
          taskYIELD();
          return;
        }
        s_lgfx->startWrite();
        int32_t rowBase = 0;
        while (rowBase < h) {
          const bool contiguousRows = (w == srcW);
          const int32_t maxRowsThis = contiguousRows
              ? static_cast<int32_t>(kLovyanFlushChunkRows)
              : 1;
          const int32_t rowsThis = ((h - rowBase) > maxRowsThis)
              ? maxRowsThis
              : (h - rowBase);
          const auto* pixels =
              reinterpret_cast<const lgfx::rgb565_t*>(srcBase + rowBase * srcW);
          const int32_t pixelCount = w * rowsThis;

          s_lgfx->setAddrWindow(x1, y1 + rowBase, w, rowsThis);
          if (kCommandSettleUs > 0) {
            delayMicroseconds(kCommandSettleUs);
          }
          if (kDisplayUseLgfxDma) {
            s_lgfx->writePixelsDMA(pixels, pixelCount);
            s_lgfx->waitDMA();
          } else {
            s_lgfx->writePixels(pixels, pixelCount);
          }
          if (kStripeGapUs > 0) {
            delayMicroseconds(kStripeGapUs);
          }
          ++s_flushWriteCalls;
          rowBase += rowsThis;
        }
        s_lgfx->endWrite();
      }
    }
  }
#elif CCM_HAS_ARDUINO_GFX
  // Drive the ILI9488 in native 18-bit SPI mode. Convert LVGL RGB565 into
  // RGB888 and send small address-windowed chunks. It is slower than one large
  // RAMWR burst, but far more tolerant of a shared, hand-wired SPI display.
  if (s_gfx && s_bus && area && colors) {
    const int32_t srcW = area->x2 - area->x1 + 1;
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;

    if (srcW > 0 && x1 < kDisplayWidth && y1 < kDisplayHeight && x2 >= 0 && y2 >= 0) {
      const bool clipped = x1 < 0 || y1 < 0 || x2 >= kDisplayWidth || y2 >= kDisplayHeight;
      if (x1 < 0) x1 = 0;
      if (y1 < 0) y1 = 0;
      if (x2 >= kDisplayWidth) x2 = kDisplayWidth - 1;
      if (y2 >= kDisplayHeight) y2 = kDisplayHeight - 1;

      const int32_t w = x2 - x1 + 1;
      const int32_t h = y2 - y1 + 1;
      if (w > 0 && h > 0 && w <= kDisplayWidth) {
        const uint32_t areaPixels = static_cast<uint32_t>(w * h);
        ++s_flushCount;
        ++s_frameFlushCurrent;
        s_flushPixels += areaPixels;
        transferPixels = areaPixels;
#if CCM_DISPLAY_PIXEL_MODE == 16
        transferBytes = areaPixels * 2U;
#else
        transferBytes = areaPixels * 3U;
#endif
        s_flushBytes += transferBytes;
        transferred = true;
        if (clipped) {
          ++s_flushClipped;
        }
        if (x1 < s_flushMinX) s_flushMinX = static_cast<uint16_t>(x1);
        if (y1 < s_flushMinY) s_flushMinY = static_cast<uint16_t>(y1);
        if (x2 > s_flushMaxX) s_flushMaxX = static_cast<uint16_t>(x2);
        if (y2 > s_flushMaxY) s_flushMaxY = static_cast<uint16_t>(y2);
        if (areaPixels > s_flushMaxAreaPixels) {
          s_flushMaxAreaPixels = areaPixels;
        }
        if (w == kDisplayWidth) {
          ++s_flushFullWidthBands;
        }

        const uint16_t* srcBase = reinterpret_cast<const uint16_t*>(colors) +
                                  ((y1 - area->y1) * srcW) +
                                  (x1 - area->x1);

#if CCM_DISPLAY_PIXEL_MODE == 16
        hal::SharedSpiBusLock spiLock("LCD:flush", kLcdFlushSpiWaitTicks);
        if (!spiLock.locked()) {
          lv_disp_flush_ready(drv);
          taskYIELD();
          return;
        }
        s_gfx->startWrite();
        int32_t rowBase = 0;
        while (rowBase < h) {
          const bool contiguousRows = (w == srcW);
          const int32_t maxRowsThis = contiguousRows
              ? static_cast<int32_t>(kFlushChunkRows)
              : 1;
          const int32_t rowsThis = ((h - rowBase) > maxRowsThis)
              ? maxRowsThis
              : (h - rowBase);
          uint16_t* const pixels = const_cast<uint16_t*>(srcBase + rowBase * srcW);
          const int32_t pixelCount = w * rowsThis;

          s_gfx->writeAddrWindow(static_cast<int16_t>(x1),
                                 static_cast<int16_t>(y1 + rowBase),
                                 static_cast<uint16_t>(w),
                                 static_cast<uint16_t>(rowsThis));
          if (kCommandSettleUs > 0) {
            delayMicroseconds(kCommandSettleUs);
          }
          s_gfx->writePixels(pixels, static_cast<uint32_t>(pixelCount));
          if (kStripeGapUs > 0) {
            delayMicroseconds(kStripeGapUs);
          }
          ++s_flushWriteCalls;
          rowBase += rowsThis;
        }
        s_gfx->endWrite();
#else
        hal::SharedSpiBusLock spiLock("LCD:flush", kLcdFlushSpiWaitTicks);
        if (!spiLock.locked()) {
          lv_disp_flush_ready(drv);
          taskYIELD();
          return;
        }
        const bool holdCsForChunkedFlush =
            kDisplay18BitChunkTransaction && kDisplay18BitHoldCsPerFlush;
        if (!kDisplay18BitChunkTransaction || holdCsForChunkedFlush) {
          s_gfx->startWrite();
        }
        if (!kDisplay18BitChunkTransaction) {
          s_gfx->writeAddrWindow(static_cast<int16_t>(x1),
                                 static_cast<int16_t>(y1),
                                 static_cast<uint16_t>(w),
                                 static_cast<uint16_t>(h));
          if (kCommandSettleUs > 0) {
            delayMicroseconds(kCommandSettleUs);
          }
        }
        int32_t rowBase = 0;
        while (rowBase < h) {
          const int32_t rowsThis = ((h - rowBase) > kFlushChunkRows)
              ? kFlushChunkRows
              : (h - rowBase);
          uint8_t* const chunk = s_flushChunks[s_nextFlushChunk++ & 1U];
          if (!chunk) {
            break;
          }
          uint8_t* dst = chunk;
          for (int32_t row = 0; row < rowsThis; ++row) {
            const uint16_t* src = srcBase + (rowBase + row) * srcW;
            for (int32_t col = 0; col < w; ++col) {
              const uint16_t px = *src++;
              *dst++ = static_cast<uint8_t>((px & 0xF800U) >> 8);
              *dst++ = static_cast<uint8_t>((px & 0x07E0U) >> 3);
              *dst++ = static_cast<uint8_t>((px & 0x001FU) << 3);
            }
          }

          const bool criticalAddressAndData =
              kDisplayCriticalSpiBursts && kDisplay18BitChunkTransaction;
          const bool criticalDataOnly =
              kDisplayCriticalSpiBursts && !kDisplay18BitChunkTransaction;

          if (criticalAddressAndData) {
            noInterrupts();
          }
          if (kDisplay18BitChunkTransaction) {
            if (!holdCsForChunkedFlush) {
              s_gfx->startWrite();
            }
            s_gfx->writeAddrWindow(static_cast<int16_t>(x1),
                                   static_cast<int16_t>(y1 + rowBase),
                                   static_cast<uint16_t>(w),
                                   static_cast<uint16_t>(rowsThis));
            if (kCommandSettleUs > 0) {
              delayMicroseconds(kCommandSettleUs);
            }
          }
          if (criticalDataOnly) {
            noInterrupts();
          }
          s_bus->writeBytes(chunk, static_cast<uint32_t>(dst - chunk));
          if (criticalDataOnly) {
            interrupts();
          }
          if (kDisplay18BitChunkTransaction) {
            if (!holdCsForChunkedFlush) {
              s_gfx->endWrite();
            }
          }
          if (criticalAddressAndData) {
            interrupts();
          }
          if (kStripeGapUs > 0) {
            delayMicroseconds(kStripeGapUs);
          }
          ++s_flushWriteCalls;
          rowBase += rowsThis;
        }
        if (!kDisplay18BitChunkTransaction || holdCsForChunkedFlush) {
          s_gfx->endWrite();
        }
#endif
      }
    }
  }
#else
  (void)area;
  (void)colors;
#endif
  if (!transferred) {
    ++s_flushSkipped;
  }
  const uint32_t flushUs = micros() - flushStartUs;
  s_flushTotalUs += flushUs;
  if (flushUs > s_flushMaxUs) {
    s_flushMaxUs = flushUs;
  }
  ++s_flushSeq;
  if (kDisplayDiagVerbose &&
      (s_flushSeq <= 8U || (s_flushSeq % kDisplayDiagFlushSample) == 0U)) {
    Serial.printf("[SCREEN:FLUSH] #%lu req=(%ld,%ld)-(%ld,%ld) px=%lu bytes=%lu us=%lu ok=%u\n",
                   static_cast<unsigned long>(s_flushSeq),
                   static_cast<long>(reqX1),
                   static_cast<long>(reqY1),
                   static_cast<long>(reqX2),
                   static_cast<long>(reqY2),
                   static_cast<unsigned long>(transferPixels),
                   static_cast<unsigned long>(transferBytes),
                   static_cast<unsigned long>(flushUs),
                   transferred ? 1U : 0U);
  }
  lv_disp_flush_ready(drv);
  taskYIELD();
}

void ScreenDashboard::lvglRounderCb(lv_disp_drv_t* /*drv*/, lv_area_t* area) {
  if (!area) {
    return;
  }

  if (kFullWidthDirtyBands) {
    area->x1 = 0;
    area->x2 = static_cast<lv_coord_t>(kDisplayWidth - 1U);
  }

  if (kDirtyBandRows > 1U) {
    int32_t y1 = area->y1;
    int32_t y2 = area->y2;
    if (y1 < 0) y1 = 0;
    if (y2 < 0) y2 = 0;
    y1 = (y1 / static_cast<int32_t>(kDirtyBandRows)) *
         static_cast<int32_t>(kDirtyBandRows);
    y2 = (((y2 + static_cast<int32_t>(kDirtyBandRows)) /
           static_cast<int32_t>(kDirtyBandRows)) *
          static_cast<int32_t>(kDirtyBandRows)) - 1;
    if (y2 >= kDisplayHeight) y2 = kDisplayHeight - 1;
    area->y1 = static_cast<lv_coord_t>(y1);
    area->y2 = static_cast<lv_coord_t>(y2);
  }
}

void logScreenStats(uint32_t nowMs, uint32_t handlerMs, float fps) {
  if (handlerMs > s_lvHandlerMaxMs) {
    s_lvHandlerMaxMs = handlerMs;
  }
  if (handlerMs > 40U) {
    ++s_lvHandlerSlowCount;
  }

  if (s_statsLastMs == 0) {
    s_statsLastMs = nowMs;
    return;
  }

  const uint32_t elapsedMs = nowMs - s_statsLastMs;
  if (elapsedMs < kScreenStatsPeriodMs) {
    return;
  }

  if (!CCM_SCREEN_SERIAL_LOGS) {
    s_statsLastMs = nowMs;
    s_flushCount = 0;
    s_flushPixels = 0;
    s_flushBytes = 0;
    s_flushMaxAreaPixels = 0;
    s_flushFullWidthBands = 0;
    s_flushWriteCalls = 0;
    s_flushMaxUs = 0;
    s_flushTotalUs = 0;
    s_flushClipped = 0;
    s_flushSkipped = 0;
    s_flushMinX = kDisplayWidth;
    s_flushMinY = kDisplayHeight;
    s_flushMaxX = 0;
    s_flushMaxY = 0;
    s_frameCount = 0;
    s_frameFlushMax = 0;
    s_frameFlushLast = 0;
    s_frameFlushSum = 0;
    s_lvHandlerMaxMs = 0;
    s_lvHandlerSlowCount = 0;
    return;
  }

  lv_mem_monitor_t lvMon{};
  lv_mem_monitor(&lvMon);

  const uint32_t pxPerSec = elapsedMs == 0
      ? 0
      : static_cast<uint32_t>((static_cast<uint64_t>(s_flushPixels) * 1000ULL) / elapsedMs);
  const uint32_t bytesPerSec = elapsedMs == 0
      ? 0
      : static_cast<uint32_t>((static_cast<uint64_t>(s_flushBytes) * 1000ULL) / elapsedMs);
  const uint32_t avgFlushUs = s_flushCount == 0
      ? 0
      : static_cast<uint32_t>(s_flushTotalUs / s_flushCount);
  const uint32_t avgFlushesPerFrameX10 = s_frameCount == 0
      ? 0
      : static_cast<uint32_t>((static_cast<uint64_t>(s_frameFlushSum) * 10ULL) / s_frameCount);
  const uint32_t psramTotal = ESP.getPsramSize();
  const uint32_t psramFree = psramTotal == 0 ? 0 : ESP.getFreePsram();
  const uint32_t dmaFree = heap_caps_get_free_size(MALLOC_CAP_DMA);
  const uint32_t dmaBig = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);

  Serial.printf(
      "[SCREEN] fps=%.1f frames=%lu handler_max=%lums slow=%lu flush=%lu writes=%lu "
      "flush/frame=%lu.%lu last=%lu max=%lu px/s=%lu B/s=%lu max_area=%lu bounds=(%u,%u)-(%u,%u) "
      "fullw=%lu clip=%lu skip=%lu avg_us=%lu max_us=%lu "
      "heap=%lu min=%lu max_block=%lu dma_free=%lu dma_big=%lu psram=%lu/%lu "
      "lv_free=%lu lv_big=%lu lv_frag=%u%% lv_used=%u%% backend=%s pixel=%ubit spi=%luHz "
      "display_dma=%u rows=%u/%u dirty=%u/%u chunk_tx=%u hold_cs=%u crit=%u settle=%uus gap=%uus full_refresh=%u\n",
      static_cast<double>(fps),
      static_cast<unsigned long>(s_frameCount),
      static_cast<unsigned long>(s_lvHandlerMaxMs),
      static_cast<unsigned long>(s_lvHandlerSlowCount),
      static_cast<unsigned long>(s_flushCount),
      static_cast<unsigned long>(s_flushWriteCalls),
      static_cast<unsigned long>(avgFlushesPerFrameX10 / 10U),
      static_cast<unsigned long>(avgFlushesPerFrameX10 % 10U),
      static_cast<unsigned long>(s_frameFlushLast),
      static_cast<unsigned long>(s_frameFlushMax),
      static_cast<unsigned long>(pxPerSec),
      static_cast<unsigned long>(bytesPerSec),
      static_cast<unsigned long>(s_flushMaxAreaPixels),
      static_cast<unsigned>(s_flushMinX == kDisplayWidth ? 0 : s_flushMinX),
      static_cast<unsigned>(s_flushMinY == kDisplayHeight ? 0 : s_flushMinY),
      static_cast<unsigned>(s_flushMaxX),
      static_cast<unsigned>(s_flushMaxY),
      static_cast<unsigned long>(s_flushFullWidthBands),
      static_cast<unsigned long>(s_flushClipped),
      static_cast<unsigned long>(s_flushSkipped),
      static_cast<unsigned long>(avgFlushUs),
      static_cast<unsigned long>(s_flushMaxUs),
      static_cast<unsigned long>(ESP.getFreeHeap()),
      static_cast<unsigned long>(ESP.getMinFreeHeap()),
      static_cast<unsigned long>(ESP.getMaxAllocHeap()),
      static_cast<unsigned long>(dmaFree),
      static_cast<unsigned long>(dmaBig),
      static_cast<unsigned long>(psramFree),
      static_cast<unsigned long>(psramTotal),
      static_cast<unsigned long>(lvMon.free_size),
      static_cast<unsigned long>(lvMon.free_biggest_size),
      static_cast<unsigned>(lvMon.frag_pct),
      static_cast<unsigned>(lvMon.used_pct),
      kDisplayBackendName,
      static_cast<unsigned>(kDisplayPixelMode),
      static_cast<unsigned long>(kDisplaySpiHz),
      static_cast<unsigned>((kDisplayUseLgfxDma || kDisplayUseGfxDma) ? 1 : 0),
      static_cast<unsigned>(kDrawBufferRows),
      static_cast<unsigned>(
#if CCM_HAS_LOVYAN_GFX
          kLovyanFlushChunkRows
#else
          kFlushChunkRows
#endif
      ),
      static_cast<unsigned>(kFullWidthDirtyBands ? 1 : 0),
      static_cast<unsigned>(kDirtyBandRows),
      static_cast<unsigned>(kDisplay18BitChunkTransaction ? 1 : 0),
      static_cast<unsigned>(kDisplay18BitHoldCsPerFlush ? 1 : 0),
      static_cast<unsigned>(kDisplayCriticalSpiBursts ? 1 : 0),
      static_cast<unsigned>(kCommandSettleUs),
      static_cast<unsigned>(kStripeGapUs),
      static_cast<unsigned>(kLvglFullRefresh ? 1 : 0));

  s_statsLastMs = nowMs;
  s_flushCount = 0;
  s_flushPixels = 0;
  s_flushBytes = 0;
  s_flushMaxAreaPixels = 0;
  s_flushFullWidthBands = 0;
  s_flushWriteCalls = 0;
  s_flushMaxUs = 0;
  s_flushTotalUs = 0;
  s_flushClipped = 0;
  s_flushSkipped = 0;
  s_flushMinX = kDisplayWidth;
  s_flushMinY = kDisplayHeight;
  s_flushMaxX = 0;
  s_flushMaxY = 0;
  s_frameCount = 0;
  s_frameFlushMax = 0;
  s_frameFlushLast = 0;
  s_frameFlushSum = 0;
  s_lvHandlerMaxMs = 0;
  s_lvHandlerSlowCount = 0;
}

void ScreenDashboard::lvglTouchReadCb(lv_indev_drv_t* drv, lv_indev_data_t* data) {
  auto* self = static_cast<ScreenDashboard*>(drv->user_data);
  // lastTouch_ is written by touchTask and read here from screenTask — use a spinlock
  // to prevent a torn read if preemption occurs mid-write on the same core.
  portENTER_CRITICAL(&self->touchMux_);
  const touch::TouchSample t = self->filteredTouch_;
  const uint32_t sampleMs = self->filteredTouchSampleMs_;
  const uint32_t pressStartMs = self->filteredTouchPressStartMs_;
  portEXIT_CRITICAL(&self->touchMux_);
  const uint32_t ageMs = millis() - sampleMs;
  const uint32_t heldMs = millis() - pressStartMs;
  bool lowLvglMem = false;
  size_t lvFreeBytes = 0;
  {
    lv_mem_monitor_t mon{};
    lv_mem_monitor(&mon);
    lvFreeBytes = mon.free_size;
    lowLvglMem = lvFreeBytes < static_cast<size_t>(CCM_LVGL_TOUCH_MIN_FREE_BYTES);
  }
  if (t.touched && ageMs <= CCM_TOUCH_STALE_RELEASE_MS && heldMs <= CCM_TOUCH_MAX_PRESS_MS) {
    if (lowLvglMem) {
      static uint32_t s_lastTouchLowMemWarnMs = 0;
      const uint32_t nowMs = millis();
      if (nowMs - s_lastTouchLowMemWarnMs >= 2000U) {
        s_lastTouchLowMemWarnMs = nowMs;
        Serial.printf("[SCREEN] touch release forced: lv_free=%lu < %u bytes\n",
                      static_cast<unsigned long>(lvFreeBytes),
                      static_cast<unsigned>(CCM_LVGL_TOUCH_MIN_FREE_BYTES));
      }
      data->state = LV_INDEV_STATE_RELEASED;
      return;
    }
    data->point.x = static_cast<lv_coord_t>(t.x);
    data->point.y = static_cast<lv_coord_t>(t.y);
    data->state   = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void ScreenDashboard::attach(canbus::CanManager* canMgr, race::RacePerformanceManager* raceMgr,
                              settings::SettingsManager* settingsMgr, storage::SdManager* sdMgr) {
  canMgr_      = canMgr;
  raceMgr_     = raceMgr;
  settingsMgr_ = settingsMgr;
  sdMgr_       = sdMgr;
}

bool ScreenDashboard::begin(uint8_t lcdCs, uint8_t lcdRst, uint8_t lcdDc,
                             uint8_t spiSck, uint8_t spiMosi, uint8_t spiMiso) {
#if CCM_HAS_LOVYAN_GFX
  Serial.printf("[SCREEN] backend=%s spi=%luHz dma=%u chunk_rows=%u\n",
                 kDisplayBackendName,
                 static_cast<unsigned long>(kDisplaySpiHz),
                 static_cast<unsigned>(kDisplayUseLgfxDma ? 1 : 0),
                 static_cast<unsigned>(kLovyanFlushChunkRows));
  Serial.println("[SCREEN] creating LovyanGFX ILI9488 driver");
  s_lgfx = new (std::nothrow) CabinLgfxDisplay(lcdCs, lcdRst, lcdDc, spiSck, spiMosi, spiMiso);
  if (!s_lgfx) {
    Serial.println("[SCREEN] LovyanGFX allocation FAILED");
    return false;
  }

  {
    hal::SharedSpiBusLock spiLock("LCD:init");
    if (!s_lgfx->init()) {
      Serial.println("[SCREEN] LovyanGFX init FAILED");
      return false;
    }
    s_lgfx->setRotation(1);
    s_lgfx->setColorDepth(16);
    s_lgfx->setSwapBytes(false);
    s_lgfx->startWrite();
    s_lgfx->writeCommand(kIli9488Invctr);
    s_lgfx->writeData(kIli9488InvctrValue);
    s_lgfx->endWrite();
    Serial.printf("[SCREEN] LovyanGFX init OK size=%dx%d depth=%u\n",
                   s_lgfx->width(),
                   s_lgfx->height(),
                   static_cast<unsigned>(static_cast<int>(s_lgfx->getColorDepth()) &
                                         static_cast<int>(lgfx::color_depth_t::bit_mask)));
    Serial.printf("[SCREEN] ILI9488 INVCTR=0x%02X\n",
                   static_cast<unsigned>(kIli9488InvctrValue));
    runDisplayDiagnostic();
    Serial.println("[SCREEN] clear start");
    s_lgfx->fillScreen(0x000000U);
    Serial.println("[SCREEN] clear OK");
  }
#elif CCM_HAS_ARDUINO_GFX
#if CCM_DISPLAY_USE_GFX_DMA
  // Dedicated ESP-IDF DMA bus path. Only enable this when LCD is allowed to own
  // the SPI host; the normal build keeps shared Arduino SPI for LCD/CAN/SD.
  Serial.println("[SCREEN] creating ESP32SPIDMA bus");
  s_bus = new (std::nothrow) Arduino_ESP32SPIDMA(lcdDc, lcdCs, spiSck, spiMosi, spiMiso, FSPI, true);
  if (!s_bus) {
    Serial.println("[SCREEN] ESP32SPIDMA allocation FAILED");
    return false;
  }
#else
  // CAN, SD, and LCD share the same physical SPI pins. Use Arduino's shared SPI
  // peripheral here; Arduino_ESP32SPI can wedge during ILI9488 init on Arduino
  // ESP32 3.x / IDF 5 with this ESP32-S3 setup.
  Serial.println("[SCREEN] creating HWSPI bus");
  s_bus = new (std::nothrow) Arduino_HWSPI(lcdDc, lcdCs, spiSck, spiMosi, spiMiso, &SPI, true);
  if (!s_bus) {
    Serial.println("[SCREEN] HWSPI allocation FAILED");
    return false;
  }
#endif
  // rotation=1: 90° CW landscape (480×320).
#if CCM_DISPLAY_PIXEL_MODE == 16
  Serial.println("[SCREEN] creating ILI9488 16-bit driver");
  s_gfx = new Arduino_ILI9488(s_bus, lcdRst, 1 /*rotation 90°CW*/, false /*ips*/);
#else
  Serial.println("[SCREEN] creating ILI9488 18-bit driver");
  s_gfx = new Arduino_ILI9488_18bit(s_bus, lcdRst, 1 /*rotation 90°CW*/, false /*ips*/);
#endif
  if (!s_gfx) {
    Serial.println("[SCREEN] ILI9488 allocation FAILED");
    return false;
  }
  Serial.printf("[SCREEN] GFX begin @ %lu Hz bus=%s pixel=%ubit\n",
                 static_cast<unsigned long>(kDisplaySpiHz),
                 kDisplayUseGfxDma ? "ESP32SPIDMA" : "HWSPI",
                 static_cast<unsigned>(kDisplayPixelMode));
  {
    hal::SharedSpiBusLock spiLock("LCD:init");
    if (!s_gfx || !s_gfx->begin(kDisplaySpiHz)) {
      Serial.println("[SCREEN] GFX begin FAILED");
      return false;
    }
    Serial.println("[SCREEN] GFX begin OK");
    Serial.println("[SCREEN] clear start");
    s_gfx->fillScreen(0x0000U);  // clear to black before LVGL builds first frame
    Serial.println("[SCREEN] clear OK");
  }
#else
  (void)lcdCs; (void)lcdRst; (void)lcdDc;
  (void)spiSck; (void)spiMosi; (void)spiMiso;
#endif

  if (!ensureDisplayBuffers()) {
    return false;
  }

  // ---- LVGL init ----
  lv_init();

  // Display driver
  static lv_disp_draw_buf_t drawBuf;
  lv_disp_draw_buf_init(&drawBuf, s_buf1, s_buf2, kDrawBufferPixels);

  static lv_disp_drv_t dispDrv;
  lv_disp_drv_init(&dispDrv);
  dispDrv.hor_res     = static_cast<lv_coord_t>(kWidth);
  dispDrv.ver_res     = static_cast<lv_coord_t>(kHeight);
  dispDrv.flush_cb    = lvglFlushCb;
  dispDrv.rounder_cb  = lvglRounderCb;
  dispDrv.draw_buf    = &drawBuf;
  dispDrv.full_refresh = kLvglFullRefresh ? 1 : 0;
  lv_disp_t* disp = lv_disp_drv_register(&dispDrv);
  if (!disp) {
    Serial.println("[SCREEN] LVGL display registration FAILED");
    return false;
  }

  // Touch input device
  static lv_indev_drv_t indevDrv;
  lv_indev_drv_init(&indevDrv);
  indevDrv.type      = LV_INDEV_TYPE_POINTER;
  indevDrv.read_cb   = lvglTouchReadCb;
  indevDrv.user_data = this;
  if (!lv_indev_drv_register(&indevDrv)) {
    Serial.println("[SCREEN] LVGL touch registration FAILED");
    return false;
  }

  // Dark theme
  lv_theme_t* theme = lv_theme_default_init(
      disp,
      lv_palette_main(LV_PALETTE_RED),
      lv_palette_main(LV_PALETTE_ORANGE),
      true  /* dark mode */,
      &lv_font_montserrat_16);
  if (!theme) {
    Serial.println("[SCREEN] LVGL theme allocation FAILED");
    return false;
  }
  lv_disp_set_theme(disp, theme);

  buildUi();
  online_ = true;
  {
    const uint32_t nowMs = millis();
    const state::VehicleState initialState = state::g_vehicle_state.read();
    updateHeader(initialState, nowMs);
    updateDashPage(initialState);
    Serial.println("[SCREEN] first frame deferred to screen task");
    lastHeaderUpdateMs_ = nowMs;
    lastPageUpdateMs_ = 0;
  }
  return true;
}

void ScreenDashboard::tick(const state::VehicleState& s, uint32_t nowMs) {
  if (!online_) return;

#if !LV_TICK_CUSTOM
  if (lastLvTickMs_ == 0) {
    lastLvTickMs_ = nowMs;
  } else {
    const uint32_t elapsedMs = nowMs - lastLvTickMs_;
    if (elapsedMs != 0U) {
      lv_tick_inc(elapsedMs);
      lastLvTickMs_ = nowMs;
    }
  }
#endif

  if (kPageStressTest &&
      (lastStressPageSwitchMs_ == 0 ||
       static_cast<uint32_t>(nowMs - lastStressPageSwitchMs_) >= kPageStressPeriodMs)) {
    lastStressPageSwitchMs_ = nowMs;
    const uint8_t nextPage = static_cast<uint8_t>((activePage_ + 1U) % kPageCount);
    Serial.printf("[SCREEN:STRESS] auto page %u->%u period=%lums flush_seq=%lu\n",
                   static_cast<unsigned>(activePage_),
                   static_cast<unsigned>(nextPage),
                   static_cast<unsigned long>(kPageStressPeriodMs),
                   static_cast<unsigned long>(s_flushSeq));
    showPage(nextPage);
  }

  serviceTouchGestures(nowMs);

  const bool feedbackActive = actionFeedback_[0] != '\0';
  if (lastHeaderUpdateMs_ == 0 ||
      (nowMs - lastHeaderUpdateMs_) >= kHeaderUpdatePeriodMs ||
      feedbackActive) {
    updateHeader(s, nowMs);
    lastHeaderUpdateMs_ = nowMs;
  }

  uint32_t pageUpdatePeriodMs = kLivePageUpdatePeriodMs;
  if (activePage_ == 0) {
    pageUpdatePeriodMs = kDashUpdatePeriodMs;
  } else if (activePage_ == 5 || activePage_ == 6) {
    pageUpdatePeriodMs = kHeavyPageUpdatePeriodMs;
  }

  if (pageSwitchPending_) {
    updateActivePage(s, nowMs);
    forceContentRepaint();
    pageSwitchPending_ = false;
    lastPageUpdateMs_ = nowMs;
  } else if (lastPageUpdateMs_ == 0 || (nowMs - lastPageUpdateMs_) >= pageUpdatePeriodMs) {
    updateActivePage(s, nowMs);
    lastPageUpdateMs_ = nowMs;
  }

  s_frameFlushCurrent = 0;
  const uint32_t handlerStartMs = millis();
  lv_task_handler();
  const uint32_t handlerEndMs = millis();
  serviceUiActions(handlerEndMs);
  serviceQueuedSettingsSave(handlerEndMs);
  s_frameFlushLast = s_frameFlushCurrent;
  s_frameFlushSum += s_frameFlushCurrent;
  if (s_frameFlushCurrent > s_frameFlushMax) {
    s_frameFlushMax = s_frameFlushCurrent;
  }
  ++s_frameCount;
  logScreenStats(handlerEndMs, handlerEndMs - handlerStartMs, s.ui_fps);
}

void ScreenDashboard::handleTouch(const touch::TouchSample& sample, uint32_t nowMs) {
  // Normalize raw coordinates and buffer for the LVGL indev driver.
  const touch::TouchSample normalized = normalizeRaw(sample);
  portENTER_CRITICAL(&touchMux_);
  rawTouch_ = normalized;
  if (normalized.touched) {
    if (touchPressStableCount_ < 255U) ++touchPressStableCount_;
    touchReleaseStableCount_ = 0;
    if (!filteredTouchDown_ && touchPressStableCount_ >= CCM_TOUCH_PRESS_STABLE_SAMPLES) {
      filteredTouchDown_ = true;
      filteredTouchPressStartMs_ = nowMs;
    }
  } else {
    if (touchReleaseStableCount_ < 255U) ++touchReleaseStableCount_;
    touchPressStableCount_ = 0;
    if (filteredTouchDown_ && touchReleaseStableCount_ >= CCM_TOUCH_RELEASE_STABLE_SAMPLES) {
      filteredTouchDown_ = false;
    }
  }
  if (normalized.touched) {
    filteredTouch_ = normalized;
  }
  filteredTouch_.touched = filteredTouchDown_;
  filteredTouchSampleMs_ = nowMs;
  if (!filteredTouchDown_) {
    filteredTouchPressStartMs_ = nowMs;
  }
  portEXIT_CRITICAL(&touchMux_);

  // Mark touch event in shared vehicle state so CAN telemetry can observe it.
  if (normalized.touched) {
    state::g_vehicle_state.mutate([](state::VehicleState& vs) {
      vs.input_flags |= can_protocol::input_flag::TOUCH;
    });
  }
}

// ---------------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------------

static lv_obj_t* makeLabel(lv_obj_t* parent, lv_coord_t x, lv_coord_t y,
                            lv_coord_t w, const char* text,
                            const lv_font_t* font = nullptr) {
  lv_obj_t* lbl = lv_label_create(parent);
  setLabelText(lbl, text);
  lv_label_set_long_mode(lbl, LV_LABEL_LONG_CLIP);
  lv_obj_set_pos(lbl, x, y);
  lv_obj_set_width(lbl, w);
  if (font) lv_obj_set_style_text_font(lbl, font, LV_PART_MAIN);
  return lbl;
}

static lv_obj_t* makeBtn(lv_obj_t* parent, const char* text,
                          lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                          lv_event_cb_t cb, void* userData) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  styleActionButton(btn);
  lv_obj_t* lbl = lv_label_create(btn);
  setLabelText(lbl, text);
  lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
  lv_obj_set_width(lbl, (w > 12) ? static_cast<lv_coord_t>(w - 12) : w);
  lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_center(lbl);
  if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_RELEASED, userData);
  return btn;
}

// Returns the label child of a button (first child).
static lv_obj_t* btnLabel(lv_obj_t* btn) {
  return lv_obj_get_child(btn, 0);
}

void formatFileSize(uint32_t bytes, char* out, size_t outLen) {
  if (!out || outLen == 0) return;
  if (bytes >= 1024UL * 1024UL) {
    snprintf(out, outLen, "%luM", static_cast<unsigned long>(bytes / (1024UL * 1024UL)));
  } else if (bytes >= 1024UL) {
    snprintf(out, outLen, "%luK", static_cast<unsigned long>(bytes / 1024UL));
  } else {
    snprintf(out, outLen, "%luB", static_cast<unsigned long>(bytes));
  }
}

// ---------------------------------------------------------------------------
// buildUi – entry point: header + content panels + nav bar
// ---------------------------------------------------------------------------

void ScreenDashboard::buildUi() {
  lv_obj_t* scr = lv_scr_act();
  setBgColor(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* bg = lv_img_create(scr);
  lv_img_set_src(bg, &ui_background);
  lv_obj_set_pos(bg, 0, 0);
  lv_obj_clear_flag(bg, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_move_background(bg);

  buildHeader(scr);
  buildContentArea(scr);
  buildNavBar(scr);
  buildTouchCalibrationOverlay(scr);
  buildFaultOverlay(scr);

  // Start on DASH page (index 0) without animation.
  showPage(0);
}

// ---------------------------------------------------------------------------
// buildHeader – fixed 34 px bar at y=0
// ---------------------------------------------------------------------------

void ScreenDashboard::buildHeader(lv_obj_t* scr) {
  lv_obj_t* hdr = lv_obj_create(scr);
  lv_obj_set_pos(hdr, 0, 0);
  lv_obj_set_size(hdr, kWidth, kHdrH);
  lv_obj_set_style_radius(hdr, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(hdr, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(hdr, 0, LV_PART_MAIN);
  setBgColor(hdr, lv_color_hex(kUiColorBg), LV_PART_MAIN);
  lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

  // Left spacer
  hdrBatLabel_ = lv_label_create(hdr);
  setLabelTextStatic(hdrBatLabel_, hdrTimeText_, sizeof(hdrTimeText_), "--:--:--Z");
  lv_obj_set_style_text_font(hdrBatLabel_, &lv_font_montserrat_16, 0);
  setTextColor(hdrBatLabel_, lv_color_hex(kUiColorText), 0);
  lv_obj_align(hdrBatLabel_, LV_ALIGN_LEFT_MID, 8, 0);

  // Center: active page name
  hdrTitleLabel_ = lv_label_create(hdr);
  setLabelText(hdrTitleLabel_, "DASH");
  lv_obj_set_style_text_font(hdrTitleLabel_, &lv_font_montserrat_18, 0);
  setTextColor(hdrTitleLabel_, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_align(hdrTitleLabel_, LV_ALIGN_CENTER, 0, 0);

  // Right: action feedback (green)
  hdrFeedbackLabel_ = lv_label_create(hdr);
  setLabelText(hdrFeedbackLabel_, "");
  lv_obj_set_style_text_font(hdrFeedbackLabel_, &lv_font_montserrat_14, 0);
  setTextColor(hdrFeedbackLabel_, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_align(hdrFeedbackLabel_, LV_ALIGN_RIGHT_MID, -30, 0);

  // Far right: fault status dot (circle label)
  hdrFaultDot_ = lv_label_create(hdr);
  setLabelText(hdrFaultDot_, LV_SYMBOL_STOP);
  setTextColor(hdrFaultDot_, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_set_style_text_font(hdrFaultDot_, &lv_font_montserrat_14, 0);
  lv_obj_align(hdrFaultDot_, LV_ALIGN_RIGHT_MID, -8, 0);
}

void ScreenDashboard::buildTouchCalibrationOverlay(lv_obj_t* scr) {
  touchCalOverlay_ = lv_obj_create(scr);
  lv_obj_set_pos(touchCalOverlay_, 0, 0);
  lv_obj_set_size(touchCalOverlay_, kWidth, kHeight);
  setBgColor(touchCalOverlay_, lv_color_hex(kUiColorBg), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(touchCalOverlay_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(touchCalOverlay_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(touchCalOverlay_, 0, LV_PART_MAIN);
  lv_obj_add_flag(touchCalOverlay_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(touchCalOverlay_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(touchCalOverlay_, onTouchCalStartClicked, LV_EVENT_RELEASED, this);

  touchCalPromptLabel_ = makeLabel(touchCalOverlay_, 12, 10, 330, "Touch Calibration", &lv_font_montserrat_18);
  touchCalCloseBtn_ = makeBtn(touchCalOverlay_, "CLOSE", 370, 8, 96, 34, onTouchCalCloseClicked, this);
  styleSecondaryButton(touchCalCloseBtn_);

  touchCalTargetLabel_ = makeLabel(touchCalOverlay_, 20, 38, 80, "+", &lv_font_montserrat_48);
  lv_obj_set_style_text_align(touchCalTargetLabel_, LV_TEXT_ALIGN_CENTER, 0);
  setTextColor(touchCalTargetLabel_, lv_palette_main(LV_PALETTE_RED), 0);

  lv_obj_add_flag(touchCalOverlay_, LV_OBJ_FLAG_HIDDEN);
}

void ScreenDashboard::buildFaultOverlay(lv_obj_t* scr) {
  faultOverlay_ = lv_obj_create(scr);
  lv_obj_set_pos(faultOverlay_, 0, 0);
  lv_obj_set_size(faultOverlay_, kWidth, kHeight);
  setBgColor(faultOverlay_, lv_color_hex(kUiColorBg), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(faultOverlay_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(faultOverlay_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(faultOverlay_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(faultOverlay_, LV_OBJ_FLAG_SCROLLABLE);

  makeLabel(faultOverlay_, 12, 10, 240, "Fault Details", &lv_font_montserrat_18);
  faultCloseBtn_ = makeBtn(faultOverlay_, "CLOSE", 370, 8, 96, 34, onFaultCloseClicked, this);
  styleSecondaryButton(faultCloseBtn_);

  faultLabel_ = makeMetricTile(faultOverlay_, 8, 50, 464, 246, "No fault data", &lv_font_montserrat_14);
  lv_obj_set_style_text_align(faultLabel_, LV_TEXT_ALIGN_LEFT, 0);
  lv_label_set_recolor(faultLabel_, true);

  lv_obj_add_flag(faultOverlay_, LV_OBJ_FLAG_HIDDEN);
}

// ---------------------------------------------------------------------------
// buildContentArea – creates 7 page panels at (0, kHdrH), size 320 × kContentH
// ---------------------------------------------------------------------------

void ScreenDashboard::buildContentArea(lv_obj_t* scr) {
  // Container for all page panels (transparent pass-through)
  lv_obj_t* cont = lv_obj_create(scr);
  contentArea_ = cont;
  lv_obj_set_pos(cont, 0, kHdrH);
  lv_obj_set_size(cont, kWidth, kContentH);
  setBgColor(cont, lv_color_hex(kUiColorBg), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(cont, 0, LV_PART_MAIN);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  for (uint8_t i = 0; i < kPageCount; i++) {
    lv_obj_t* pg = lv_obj_create(cont);
    lv_obj_set_pos(pg, 0, 0);
    lv_obj_set_size(pg, kWidth, kContentH);
    setBgColor(pg, lv_color_hex(kUiColorBg), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(pg, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(pg, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(pg, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pg, 4, LV_PART_MAIN);
    lv_obj_add_flag(pg, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(pg, LV_OBJ_FLAG_SCROLLABLE);
    pages_[i] = pg;
  }

  buildDashPage(pages_[0]);
  buildMethPage(pages_[1]);
  buildTailPage(pages_[2]);
  buildLedsPage(pages_[3]);
  buildGpsPage(pages_[4]);
  buildTempsPage(pages_[5]);
  buildDiagPage(pages_[6]);
  buildKnockPage(pages_[7]);
}

// ---------------------------------------------------------------------------
// buildNavBar – 7 icon buttons at y = kHdrH + kContentH = 396, height = 84
// ---------------------------------------------------------------------------

void ScreenDashboard::buildNavBar(lv_obj_t* scr) {
  static const char* const kSymbols[8] = {
    LV_SYMBOL_BARS,      // 0 DASH
    LV_SYMBOL_TINT,      // 1 METH
    LV_SYMBOL_LOOP,      // 2 TAIL
    LV_SYMBOL_CHARGE,    // 3 LEDS
    LV_SYMBOL_GPS,       // 4 GPS
    LV_SYMBOL_WARNING,   // 5 TEMPS
    LV_SYMBOL_LIST,      // 6 DIAG
    LV_SYMBOL_WARNING,   // 7 KNOCK
  };
  static const char* const kNames[8] = {
    "DASH", "METH", "TAIL", "LEDS", "GPS", "TEMPS", "DIAG", "KNOCK"
  };

  topContentEdgeGuard_ = lv_obj_create(scr);
  lv_obj_set_pos(topContentEdgeGuard_, 0, static_cast<lv_coord_t>(kHdrH - 1U));
  lv_obj_set_size(topContentEdgeGuard_, kWidth, 2);
  setBgColor(topContentEdgeGuard_, lv_color_hex(kUiColorBg), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(topContentEdgeGuard_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(topContentEdgeGuard_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(topContentEdgeGuard_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(topContentEdgeGuard_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(topContentEdgeGuard_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(topContentEdgeGuard_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_move_foreground(topContentEdgeGuard_);

  lv_obj_t* bar = lv_obj_create(scr);
  lv_obj_set_pos(bar, 0, static_cast<lv_coord_t>(kHdrH + kContentH));
  lv_obj_set_size(bar, kWidth, kNavH);
  setBgColor(bar, lv_color_hex(kUiColorBg), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bar, static_cast<lv_opa_t>(150), LV_PART_MAIN);
  lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(bar, 0, LV_PART_MAIN);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  // Divider line between content and nav bar
  lv_obj_t* sep = lv_obj_create(scr);
  lv_obj_set_pos(sep, 0, static_cast<lv_coord_t>(kHdrH + kContentH));
  lv_obj_set_size(sep, kWidth, 1);
  setBgColor(sep, lv_color_hex(kUiColorDivider), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(sep, static_cast<lv_opa_t>(180), LV_PART_MAIN);
  lv_obj_set_style_border_width(sep, 0, LV_PART_MAIN);

  // Eight equal buttons span the full nav bar.
  constexpr lv_coord_t btnW = static_cast<lv_coord_t>(kWidth / kPageCount);
  for (uint8_t i = 0; i < kPageCount; i++) {
    s_navCtxs[i] = {this, i};

    const lv_coord_t bx = static_cast<lv_coord_t>(i * btnW);
    const lv_coord_t bw = (i == kPageCount - 1)
                          ? static_cast<lv_coord_t>(kWidth - bx)
                          : btnW;

    lv_obj_t* btn = lv_btn_create(bar);
    lv_obj_set_pos(btn, bx, 0);
    lv_obj_set_size(btn, bw, kNavH);
    flattenNavButtonState(btn, LV_STATE_DEFAULT);
    flattenNavButtonState(btn, LV_STATE_PRESSED);
    flattenNavButtonState(btn, LV_STATE_FOCUSED);
    flattenNavButtonState(btn, LV_STATE_FOCUS_KEY);
    flattenNavButtonState(btn, LV_STATE_CHECKED);
    setNavButtonBg(btn, lv_color_hex(kUiColorButton));
    lv_obj_add_event_cb(btn, onNavClicked, LV_EVENT_RELEASED, &s_navCtxs[i]);
    navBtns_[i] = btn;

    // Symbol icon (top half of button)
    lv_obj_t* icon = lv_label_create(btn);
    setLabelText(icon, kSymbols[i]);
    lv_obj_set_width(icon, bw);
    lv_obj_set_style_text_align(icon, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_18, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -9);
    navBtnIcons_[i] = icon;

    // Text label (bottom half of button)
    lv_obj_t* txt = lv_label_create(btn);
    setLabelText(txt, kNames[i]);
    lv_label_set_long_mode(txt, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(txt, bw);
    lv_obj_set_style_text_align(txt, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(txt, &lv_font_montserrat_12, 0);
    lv_obj_align(txt, LV_ALIGN_CENTER, 0, +14);
  }

  bottomEdgeGuard_ = lv_obj_create(scr);
  lv_obj_set_pos(bottomEdgeGuard_, 0, static_cast<lv_coord_t>(kHeight - 1U));
  lv_obj_set_size(bottomEdgeGuard_, kWidth, 1);
  setBgColor(bottomEdgeGuard_, lv_color_hex(kUiColorBg), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bottomEdgeGuard_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(bottomEdgeGuard_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(bottomEdgeGuard_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(bottomEdgeGuard_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(bottomEdgeGuard_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(bottomEdgeGuard_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_move_foreground(bottomEdgeGuard_);
}

// ---------------------------------------------------------------------------
// showPage – hide all pages, reveal the requested one, highlight nav button
// ---------------------------------------------------------------------------

void ScreenDashboard::showPage(uint8_t idx) {
  if (idx >= kPageCount) return;
  static const char* const kNames[8] = {
    "DASH", "METH", "TAIL", "LED", "GPS", "TEMP", "DIAG", "KNOCK"
  };

  if (idx == activePage_ && pages_[idx] && !lv_obj_has_flag(pages_[idx], LV_OBJ_FLAG_HIDDEN)) {
    return;
  }

  const uint8_t oldPage = activePage_;
  if (kDisplayDiagVerbose) {
    Serial.printf("[SCREEN:PAGE] %u->%u heap=%lu max_block=%lu dma_free=%lu dma_big=%lu lv_free=%lu\n",
                   static_cast<unsigned>(oldPage),
                   static_cast<unsigned>(idx),
                   static_cast<unsigned long>(ESP.getFreeHeap()),
                   static_cast<unsigned long>(ESP.getMaxAllocHeap()),
                   static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_DMA)),
                   static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)),
                   static_cast<unsigned long>(lvglFreeBytes()));
  }

  if (oldPage < kPageCount) {
    setObjHidden(pages_[oldPage], true);
    if (navBtns_[oldPage]) {
      lv_obj_clear_state(navBtns_[oldPage], LV_STATE_PRESSED | LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
      setNavButtonBg(navBtns_[oldPage], lv_color_hex(kUiColorButton));
    }
  }

  setObjHidden(pages_[idx], false);
  lv_obj_scroll_to_y(pages_[idx], 0, LV_ANIM_OFF);
  lv_obj_move_foreground(pages_[idx]);
  animatePageEnter(pages_[idx]);
  if (navBtns_[idx]) {
    lv_obj_clear_state(navBtns_[idx], LV_STATE_PRESSED | LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
    setNavButtonBg(navBtns_[idx], lv_color_hex(kUiColorButtonActive));
  }

  activePage_ = idx;
  pageSwitchPending_ = true;
  lastHeaderUpdateMs_ = 0;
  lastPageUpdateMs_ = 0;
  if (hdrTitleLabel_) setLabelText(hdrTitleLabel_, kNames[idx]);
  if (idx == 6) {
    refreshSdBrowser(millis(), true);
  }
  if (kDisplayDiagVerbose) {
    Serial.printf("[SCREEN:PAGE] active=%u heap=%lu max_block=%lu dma_free=%lu dma_big=%lu lv_free=%lu\n",
                   static_cast<unsigned>(activePage_),
                   static_cast<unsigned long>(ESP.getFreeHeap()),
                   static_cast<unsigned long>(ESP.getMaxAllocHeap()),
                   static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_DMA)),
                   static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)),
                   static_cast<unsigned long>(lvglFreeBytes()));
  }
}

// ---------------------------------------------------------------------------
// buildDashPage – arc RPM gauge + boost bar + status rows + race controls
// ---------------------------------------------------------------------------

void ScreenDashboard::buildDashPage(lv_obj_t* parent) {
  spdValLabel_ = makeLabel(parent, 8, 38, 214, "0\nMPH", &lv_font_montserrat_48);
  lv_obj_set_height(spdValLabel_, 128);
  lv_obj_set_style_text_align(spdValLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(spdValLabel_, LV_LABEL_LONG_WRAP);
  stylePanel(spdValLabel_, kUiColorHeroPanel);

  boostValLabel_ = makeMetricTile(parent, 236, 20, 112, 74, "BOOST\n0.0 PSI");
  dashStatusLabel_ = makeMetricTile(parent, 356, 20, 112, 74, "METH\nOFFLINE");
  dashEnvLabel_ = makeMetricTile(parent, 236, 102, 112, 74, "TANK\n100%");
  dashRaceLabel_ = makeMetricTile(parent, 356, 102, 112, 74, "KNOCK\nOK");
  lv_label_set_recolor(dashStatusLabel_, true);
  lv_label_set_recolor(dashRaceLabel_, true);

  rpmValLabel_ = makeMetricTile(parent, 8, 182, 104, 44, "RPM --", &lv_font_montserrat_16);
  gLiveLabel_ = makeMetricTile(parent, 120, 182, 104, 44, "FUEL --", &lv_font_montserrat_16);
  gPeakLabel_ = makeMetricTile(parent, 236, 182, 112, 44, "OIL --", &lv_font_montserrat_16);
  raceAccelBtn_ = makeMetricTile(parent, 356, 182, 112, 44, "CAN --", &lv_font_montserrat_16);
}


// ---------------------------------------------------------------------------
// buildMethPage
// ---------------------------------------------------------------------------

void ScreenDashboard::buildMethPage(lv_obj_t* parent) {
  methBadgeLabel_ = makeStatusPill(parent, 8, 8, 142, "WATER-METH", lv_color_hex(kUiColorBad));
  methStateLabel_ = makeStatusPill(parent, 158, 8, 94, "OFFLINE", lv_color_hex(kUiColorBad));
  methSensorLabel_ = makeStatusPill(parent, 260, 8, 96, "EXT OFF", lv_color_hex(kUiColorBad));
  methParamLabel_ = makeStatusPill(parent, 364, 8, 104, "ARM OFF", lv_color_hex(kUiColorTextMuted));

  methDutyLabel_ = makeMetricTile(parent, 8, 48, 110, 70, "DUTY\n0%", &lv_font_montserrat_20);
  methTankLabel_ = makeMetricTile(parent, 126, 48, 110, 70, "TANK\n100%", &lv_font_montserrat_20);
  methMapLabel_ = makeMetricTile(parent, 244, 48, 110, 70, "MAP\n0 kPa", &lv_font_montserrat_20);
  methPressureLabel_ = makeMetricTile(parent, 362, 48, 106, 70, "METH\n0 psi", &lv_font_montserrat_20);

  methIatLabel_ = makeMetricTile(parent, 8, 128, 226, 34, "IAT -- C", &lv_font_montserrat_16);
  methBayLabel_ = makeMetricTile(parent, 242, 128, 226, 34, "BAY -- C", &lv_font_montserrat_16);

  methArmBtn_ = makeBtn(parent, "ARM", 8, 178, 226, 44, onMethArmClicked, this);
  methArmBtnLabel_ = btnLabel(methArmBtn_);
  methRatioBtn_ = makeBtn(parent, "RATIO 55%", 242, 178, 226, 44, onMethRatioClicked, this);
  methRatioBtnLabel_ = btnLabel(methRatioBtn_);
  stylePrimaryButton(methArmBtn_);
  styleSecondaryButton(methRatioBtn_);
}


// ---------------------------------------------------------------------------
// buildTailPage
// ---------------------------------------------------------------------------

void ScreenDashboard::buildTailPage(lv_obj_t* parent) {
  tailStatusLabel_ = makeStatusPill(parent, 8, 8, 300, "TAILS  OFFLINE  STOCK  BRI 0", lv_color_hex(kUiColorBad));
  tailOnlineLed_ = lv_led_create(parent);
  lv_obj_set_pos(tailOnlineLed_, 438, 11);
  lv_obj_set_size(tailOnlineLed_, 18, 18);
  lv_led_set_brightness(tailOnlineLed_, 180);
  lv_led_off(tailOnlineLed_);

  tailModePanel_ = makePanel(parent, 8, 48, 464, 166, kUiColorBg);
  lv_obj_set_style_pad_all(tailModePanel_, 0, LV_PART_MAIN);
  constexpr lv_coord_t tailBtnW = 224;
  constexpr lv_coord_t tailBtnH = 72;
  constexpr lv_coord_t tailGap = 8;
  tailStockBtn_ = makeBtn(tailModePanel_, "STOCK", 0, 0, tailBtnW, tailBtnH, onTailStockClicked, this);
  tailSeqBtn_ = makeBtn(tailModePanel_, "SEQ", tailBtnW + tailGap, 0, tailBtnW, tailBtnH, onTailSeqClicked, this);
  tailShowMenuBtn_ = makeBtn(tailModePanel_, "SHOW", 0, tailBtnH + tailGap, tailBtnW, tailBtnH, onTailShowMenuClicked, this);
  tailDemoBtn_ = makeBtn(tailModePanel_, "DEMO", tailBtnW + tailGap, tailBtnH + tailGap, tailBtnW, tailBtnH, onTailDemoClicked, this);
  lv_obj_set_style_text_font(btnLabel(tailStockBtn_), &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_font(btnLabel(tailSeqBtn_), &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_font(btnLabel(tailShowMenuBtn_), &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_font(btnLabel(tailDemoBtn_), &lv_font_montserrat_20, 0);
  styleSecondaryButton(tailStockBtn_);
  styleSecondaryButton(tailSeqBtn_);
  styleSecondaryButton(tailShowMenuBtn_);
  styleSecondaryButton(tailDemoBtn_);

  tailShowPanel_ = makePanel(parent, 8, 48, 464, 166);
  lv_obj_add_flag(tailShowPanel_, LV_OBJ_FLAG_HIDDEN);
  tailShowPageLabel_ = makeSectionTitle(tailShowPanel_, 0, 0, 120, "SHOW 1/4");
  tailShowBackBtn_ = makeBtn(tailShowPanel_, "BACK", 128, 0, 104, 34, onTailShowBackClicked, this);
  tailShowPrevBtn_ = makeBtn(tailShowPanel_, "PREV", 240, 0, 104, 34, onTailShowPrevClicked, this);
  tailShowNextBtn_ = makeBtn(tailShowPanel_, "NEXT", 352, 0, 104, 34, onTailShowNextClicked, this);
  styleSecondaryButton(tailShowBackBtn_);
  styleSecondaryButton(tailShowPrevBtn_);
  styleSecondaryButton(tailShowNextBtn_);
  constexpr lv_coord_t optW = 142;
  constexpr lv_coord_t optH = 38;
  constexpr lv_coord_t optGap = 10;
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    const lv_coord_t col = static_cast<lv_coord_t>(i % 3);
    const lv_coord_t row = static_cast<lv_coord_t>(i / 3);
    tailShowOptBtns_[i] = makeBtn(tailShowPanel_, "N/A",
        static_cast<lv_coord_t>(col * (optW + optGap)),
        static_cast<lv_coord_t>(46 + row * (optH + optGap)),
        optW, optH, onTailShowOptClicked, this);
    styleSecondaryButton(tailShowOptBtns_[i]);
  }
}


// ---------------------------------------------------------------------------
// buildLedsPage – LED mode selector + cabin zone status
// ---------------------------------------------------------------------------

void ScreenDashboard::buildLedsPage(lv_obj_t* parent) {
  lv_obj_set_style_bg_opa(parent, LV_OPA_TRANSP, LV_PART_MAIN);

  makeLabel(parent, 0, 2, 146, "Interior LEDs", &lv_font_montserrat_20);

  ledMasterSwitch_ = lv_switch_create(parent);
  lv_obj_set_pos(ledMasterSwitch_, 378, 2);
  lv_obj_set_size(ledMasterSwitch_, 84, 32);
  setBgColor(ledMasterSwitch_, lv_color_hex(kUiColorButton), LV_PART_MAIN);
  setBgColor(ledMasterSwitch_, lv_color_hex(kUiColorButtonActive),
             static_cast<lv_style_selector_t>(LV_PART_INDICATOR) | LV_STATE_CHECKED);
  setBgColor(ledMasterSwitch_, lv_color_hex(kUiColorRow), LV_PART_INDICATOR);
  setBgColor(ledMasterSwitch_, lv_color_hex(kUiColorText), LV_PART_KNOB);
  lv_obj_set_style_bg_opa(ledMasterSwitch_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ledMasterSwitch_, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_border_color(ledMasterSwitch_, lv_color_hex(kUiColorButtonBorder), LV_PART_MAIN);
  lv_obj_set_style_border_width(ledMasterSwitch_, 1, LV_PART_MAIN);
  lv_obj_add_event_cb(ledMasterSwitch_, onLedMasterSwitchChanged, LV_EVENT_VALUE_CHANGED, this);

  ledMasterLabel_ = makeLabel(parent, 286, 10, 86, "MASTER OFF", &lv_font_montserrat_12);

  ledShowBtn_ = makeBtn(parent, "SHOW MODE", 156, 2, 118, 30, onLedShowMenuClicked, this);
  styleActionButton(ledShowBtn_);

  ledMainPanel_ = lv_obj_create(parent);
  lv_obj_set_pos(ledMainPanel_, 0, 38);
  lv_obj_set_size(ledMainPanel_, 472, 224);
  lv_obj_set_style_pad_all(ledMainPanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(ledMainPanel_, 0, LV_PART_MAIN);
  setBgColor(ledMainPanel_, lv_color_hex(kUiColorPanel), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ledMainPanel_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_clear_flag(ledMainPanel_, LV_OBJ_FLAG_SCROLLABLE);

  static const char* const kModeNames[2] = { "LOW", "HIGH" };
  static const state::LedMode kModeValues[2] = {
    state::LedMode::LOW_LIGHT,
    state::LedMode::HIGH_LIGHT,
  };

  lv_obj_t* simpleZonesPanel = makePanel(ledMainPanel_, 236, 8, 232, 148, kUiColorPanel,
                                         static_cast<lv_opa_t>(145));
  makeSectionTitle(simpleZonesPanel, 8, 8, 216, "TOP / BOTTOM");

  const lv_coord_t labelX[kInteriorLedUiCount] = { 8, 8, 8, 248, 248 };
  const lv_coord_t labelY[kInteriorLedUiCount] = { 16, 80, 144, 58, 122 };
  const lv_coord_t btnX[kInteriorLedUiCount] = { 92, 92, 92, 336, 336 };
  const lv_coord_t btnY[kInteriorLedUiCount] = { 8, 72, 136, 48, 112 };
  constexpr lv_coord_t btnW = 60;
  constexpr lv_coord_t btnH = 42;
  constexpr lv_coord_t gapX = 8;
  for (uint8_t ch = 0; ch < kInteriorLedUiCount; ++ch) {
    ledStripLabels_[ch] = makeLabel(ledMainPanel_, labelX[ch], labelY[ch], 82, ledZoneName(ch), &lv_font_montserrat_14);
    for (uint8_t i = 0; i < kLedModeButtonCount; i++) {
      s_ledModeCtxs[ch][i] = {this, ch, kModeValues[i]};
      const lv_coord_t bx = static_cast<lv_coord_t>(btnX[ch] + i * (btnW + gapX));
      ledModeBtns_[ch][i] = makeBtn(ledMainPanel_, kModeNames[i], bx, btnY[ch], btnW, btnH,
                                    onLedModeClicked, &s_ledModeCtxs[ch][i]);
      lv_obj_set_style_text_font(btnLabel(ledModeBtns_[ch][i]), &lv_font_montserrat_12, 0);
      styleActionButton(ledModeBtns_[ch][i]);
    }
  }

  ledShowPanel_ = lv_obj_create(parent);
  lv_obj_set_pos(ledShowPanel_, 0, 38);
  lv_obj_set_size(ledShowPanel_, 472, 224);
  lv_obj_set_style_pad_all(ledShowPanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(ledShowPanel_, 0, LV_PART_MAIN);
  setBgColor(ledShowPanel_, lv_color_hex(kUiColorPanel), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ledShowPanel_, static_cast<lv_opa_t>(90), LV_PART_MAIN);
  lv_obj_clear_flag(ledShowPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(ledShowPanel_, LV_OBJ_FLAG_HIDDEN);

  makeLabel(ledShowPanel_, 0, 0, 140, "SHOW MODE", &lv_font_montserrat_14);
  ledShowBackBtn_ = makeBtn(ledShowPanel_, "BACK", 362, 0, 110, 28, onLedShowBackClicked, this);
  ledShowOnBtn_ = makeBtn(ledShowPanel_, "SHOW ON", 0, 34, 112, 32, onLedShowOnClicked, this);
  ledShowOffBtn_ = makeBtn(ledShowPanel_, "SHOW OFF", 120, 34, 112, 32, onLedShowOffClicked, this);
  ledShowClearBtn_ = makeBtn(ledShowPanel_, "CLEAR", 240, 34, 112, 32, onLedShowClearClicked, this);
  styleActionButton(ledShowBackBtn_);
  styleActionButton(ledShowOnBtn_);
  styleActionButton(ledShowOffBtn_);
  styleActionButton(ledShowClearBtn_);

  static const char* const kShowNames[kLedShowModeButtonCount] = {
    "RAINBOW", "BREATHE", "COLOR", "CHASE", "SPARKLE"
  };
  static const state::LedMode kShowModes[kLedShowModeButtonCount] = {
    state::LedMode::RAINBOW,
    state::LedMode::BREATHING,
    state::LedMode::STATIC_COLOR,
    state::LedMode::RPM_REACTIVE,
    state::LedMode::WARNING_FLASH,
  };
  for (uint8_t i = 0; i < kLedShowModeButtonCount; ++i) {
    s_ledShowCtxs[i] = {this, kShowModes[i]};
    const lv_coord_t bx = static_cast<lv_coord_t>((i % 3) * 118);
    const lv_coord_t by = static_cast<lv_coord_t>(74 + (i / 3) * 34);
    ledShowModeBtns_[i] = makeBtn(ledShowPanel_, kShowNames[i], bx, by, 110, 28,
                                  onLedShowModeClicked, &s_ledShowCtxs[i]);
    styleActionButton(ledShowModeBtns_[i]);
  }

  static const uint32_t kColors[kLedColorButtonCount] = {
    0xFFFFFF, 0xFF3030, 0xFFB000, 0x20E060, 0x2080FF, 0xB040FF
  };
  static const char* const kColorNames[kLedColorButtonCount] = {
    "WHITE", "RED", "AMBER", "GREEN", "BLUE", "VIOLET"
  };
  makeLabel(ledShowPanel_, 362, 38, 100, "COLOR", &lv_font_montserrat_12);
  for (uint8_t i = 0; i < kLedColorButtonCount; ++i) {
    s_ledColorCtxs[i] = {this, kColors[i]};
    const lv_coord_t bx = static_cast<lv_coord_t>(356 + (i % 2) * 58);
    const lv_coord_t by = static_cast<lv_coord_t>(58 + (i / 2) * 32);
    ledColorBtns_[i] = makeBtn(ledShowPanel_, kColorNames[i], bx, by, 56, 28,
                               onLedColorClicked, &s_ledColorCtxs[i]);
    lv_obj_set_style_text_font(btnLabel(ledColorBtns_[i]), &lv_font_montserrat_12, 0);
    styleActionButton(ledColorBtns_[i]);
    setBgColor(ledColorBtns_[i], lv_color_hex(kColors[i]), LV_PART_MAIN);
    setTextColor(btnLabel(ledColorBtns_[i]), (i == 0U || i == 2U) ? lv_color_black() : lv_color_white(), 0);
  }
}

// ---------------------------------------------------------------------------
// buildGpsPage – GPS speed, fix, coordinates
// ---------------------------------------------------------------------------

void ScreenDashboard::buildGpsPage(lv_obj_t* parent) {
  gpsSpdLabel_ = makeLabel(parent, 8, 20, 464, "-- mph", &lv_font_montserrat_48);
  lv_obj_set_height(gpsSpdLabel_, 78);
  lv_obj_set_style_text_align(gpsSpdLabel_, LV_TEXT_ALIGN_CENTER, 0);
  stylePanel(gpsSpdLabel_, kUiColorHeroPanel);

  gpsInfoLabel_ = makeMetricTile(parent, 8, 110, 464, 110,
      "FIX:NO   USED:0   VIEW:0   HDOP:--\nLAT: --\nLON: --",
      &lv_font_montserrat_20);
  lv_label_set_recolor(gpsInfoLabel_, true);
}


// ---------------------------------------------------------------------------
// buildTempsPage – all temperature channels
// ---------------------------------------------------------------------------

void ScreenDashboard::buildTempsPage(lv_obj_t* parent) {
  tempsLabel_ = makeStatusPill(parent, 8, 8, 464, "SENSOR BOARD  OK", lv_color_hex(kUiColorGood));

  tempsTable_ = lv_table_create(parent);
  lv_obj_set_pos(tempsTable_, 8, 44);
  lv_obj_set_size(tempsTable_, 464, 178);
  lv_table_set_col_cnt(tempsTable_, 3);
  lv_table_set_row_cnt(tempsTable_, 7);
  lv_table_set_col_width(tempsTable_, 0, 138);
  lv_table_set_col_width(tempsTable_, 1, 168);
  lv_table_set_col_width(tempsTable_, 2, 126);
  stylePanel(tempsTable_, kUiColorPanel);
  lv_obj_set_style_bg_color(tempsTable_, lv_color_hex(kUiColorRow), LV_PART_ITEMS);
  lv_obj_set_style_bg_opa(tempsTable_, LV_OPA_COVER, LV_PART_ITEMS);
  lv_obj_set_style_border_color(tempsTable_, lv_color_hex(kUiColorDivider), LV_PART_ITEMS);
  lv_obj_set_style_text_color(tempsTable_, lv_color_hex(kUiColorText), LV_PART_ITEMS);
  lv_obj_set_style_text_font(tempsTable_, &lv_font_montserrat_16, LV_PART_ITEMS);
  lv_obj_set_style_pad_top(tempsTable_, 5, LV_PART_ITEMS);
  lv_obj_set_style_pad_bottom(tempsTable_, 5, LV_PART_ITEMS);
  lv_table_set_cell_value(tempsTable_, 0, 0, "SENSOR");
  lv_table_set_cell_value(tempsTable_, 0, 1, "VALUE");
  lv_table_set_cell_value(tempsTable_, 0, 2, "STATUS");
}


// ---------------------------------------------------------------------------
// buildDiagPage
// ---------------------------------------------------------------------------

void ScreenDashboard::buildDiagPage(lv_obj_t* parent) {
  lv_obj_add_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir(parent, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_AUTO);
  lv_obj_set_style_pad_right(parent, 4, LV_PART_MAIN);

  diagInfoBtn_ = makeBtn(parent, "INFO", 8, 6, 108, 32, onDiagInfoClicked, this);
  diagToolsBtn_ = makeBtn(parent, "TOOLS", 122, 6, 108, 32, onDiagToolsClicked, this);
  stylePrimaryButton(diagInfoBtn_);
  styleSecondaryButton(diagToolsBtn_);

  diagInfoPanel_ = lv_obj_create(parent);
  lv_obj_set_pos(diagInfoPanel_, 0, 44);
  lv_obj_set_size(diagInfoPanel_, 472, 270);
  lv_obj_set_style_bg_opa(diagInfoPanel_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(diagInfoPanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(diagInfoPanel_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(diagInfoPanel_, LV_OBJ_FLAG_SCROLLABLE);

  diagToolsPanel_ = lv_obj_create(parent);
  lv_obj_set_pos(diagToolsPanel_, 0, 44);
  lv_obj_set_size(diagToolsPanel_, 472, 310);
  lv_obj_set_style_bg_opa(diagToolsPanel_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(diagToolsPanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(diagToolsPanel_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(diagToolsPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(diagToolsPanel_, LV_OBJ_FLAG_HIDDEN);

  diagLabel_ = makeMetricTile(diagInfoPanel_, 0, 0, 472, 236,
      "CAN --     GPS --\nMETH --    TAIL --\nSD --      TOUCH --\nIMU --\n\nSYS --",
      &lv_font_montserrat_14);
  lv_obj_set_style_text_align(diagLabel_, LV_TEXT_ALIGN_LEFT, 0);
  lv_label_set_recolor(diagLabel_, true);

  makeSectionTitle(diagToolsPanel_, 8, 0, 200, "DIAG TOOLS");
  faultPageBtn_ = makeBtn(diagToolsPanel_, "FAULTS", 8, 28, 108, 34,
                          onFaultPageClicked, this);
  canPingBtn_ = makeBtn(diagToolsPanel_, "CAN PING", 122, 28, 108, 34,
                        onCanPingClicked, this);
  ledOutputTestBtn_ = makeBtn(diagToolsPanel_, "LED TEST", 236, 28, 108, 34,
                              onLedOutputTestClicked, this);
  restartBtn_ = makeBtn(diagToolsPanel_, "RESTART", 350, 28, 108, 34,
                        onRestartClicked, this);
  touchCalBtn_ = makeBtn(diagToolsPanel_, "TOUCH CAL", 8, 70, 138, 34,
                         onTouchCalStartClicked, this);
  styleSecondaryButton(touchCalBtn_);
  benchTestBtn_ = makeBtn(diagToolsPanel_, "SIM: OFF", 154, 70, 138, 34,
                          onBenchTestClicked, this);
  makeSectionTitle(diagToolsPanel_, 8, 110, 180, "RACE TIMING");
  diagRaceStatusLabel_ = makeLabel(diagToolsPanel_, 170, 110, 292, "RACE OFF", &lv_font_montserrat_12);
  diagRaceAccelBtn_ = makeBtn(diagToolsPanel_, "ACCEL", 8, 134, 72, 30, onRaceAccelClicked, this);
  diagRaceLapBtn_ = makeBtn(diagToolsPanel_, "LAP", 86, 134, 72, 30, onRaceLapClicked, this);
  diagRaceMarkBtn_ = makeBtn(diagToolsPanel_, "MARK", 164, 134, 72, 30, onRaceMarkLapClicked, this);
  diagRaceStopBtn_ = makeBtn(diagToolsPanel_, "STOP", 242, 134, 72, 30, onRaceStopClicked, this);
  diagRaceResetBtn_ = makeBtn(diagToolsPanel_, "RESET", 320, 134, 72, 30, onRaceResetClicked, this);
  diagRaceSetStartBtn_ = makeBtn(diagToolsPanel_, "SET S/F", 398, 134, 72, 30, onRaceSetStartClicked, this);
  styleSecondaryButton(faultPageBtn_);
  styleSecondaryButton(canPingBtn_);
  styleSecondaryButton(ledOutputTestBtn_);
  styleSecondaryButton(diagRaceAccelBtn_);
  styleSecondaryButton(diagRaceLapBtn_);
  styleSecondaryButton(diagRaceMarkBtn_);
  styleDangerButton(diagRaceStopBtn_);
  styleDangerButton(diagRaceResetBtn_);
  styleSecondaryButton(diagRaceSetStartBtn_);
  styleDangerButton(restartBtn_);
  styleDangerButton(benchTestBtn_);
  buildSdBrowser(diagToolsPanel_);
}


void ScreenDashboard::buildSdBrowser(lv_obj_t* parent) {
  constexpr lv_coord_t kSdTop = 176;
  sdPathLabel_ = makeLabel(parent, 0, kSdTop, 296, "SD: not mounted", &lv_font_montserrat_12);
  lv_label_set_long_mode(sdPathLabel_, LV_LABEL_LONG_DOT);

  sdUpBtn_ = makeBtn(parent, "UP", 300, kSdTop - 2, 38, 26, onSdUpClicked, this);
  sdPrevBtn_ = makeBtn(parent, "PREV", 340, kSdTop - 2, 42, 26, onSdPrevClicked, this);
  sdNextBtn_ = makeBtn(parent, "NEXT", 384, kSdTop - 2, 42, 26, onSdNextClicked, this);
  sdTestBtn_ = makeBtn(parent, "READ", 428, kSdTop - 2, 44, 26, onSdTestClicked, this);
  styleActionButton(sdUpBtn_);
  styleActionButton(sdPrevBtn_);
  styleActionButton(sdNextBtn_);
  styleActionButton(sdTestBtn_);

  for (uint8_t i = 0; i < kSdFileRowCount; ++i) {
    s_sdFileRowCtxs[i] = {this, i};
    lv_obj_t* row = lv_btn_create(parent);
    lv_obj_set_pos(row, 0, static_cast<lv_coord_t>(kSdTop + 28 + i * 22));
    lv_obj_set_size(row, 472, 18);
    lv_obj_set_style_radius(row, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(row, lv_color_hex(kUiColorRow), LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(row, lv_color_hex(kUiColorButtonBorder), LV_PART_MAIN);
    lv_obj_set_style_pad_all(row, 2, LV_PART_MAIN);
    lv_obj_add_event_cb(row, onSdFileRowClicked, LV_EVENT_CLICKED, &s_sdFileRowCtxs[i]);

    lv_obj_t* label = lv_label_create(row);
    lv_obj_set_width(label, 456);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 4, 0);
    setLabelText(label, "--");

    sdFileRows_[i] = row;
    sdFileRowLabels_[i] = label;
  }
}

// ---------------------------------------------------------------------------
// buildKnockPage – motorsport-style knock monitoring view
// Layout (480×232 content, 4px page padding → 472×224 usable):
//   y  0: state header label  (enabled / response mode)
//   y 20: sensor status label (signal valid / fault / clipping / learned / online)
//   y 40: energy label + value
//   y 56: energy bar (full width)
//   y 74: baseline label + value
//   y 90: baseline bar
//   y108: threshold label + value
//   y124: threshold bar
//   y142: event / warning / critical counts label
//   y158: last event details label
//   y178: 4 control buttons (en/dis, reset baseline, clear events, simulate)
//   y220: logging / note label
// ---------------------------------------------------------------------------

void ScreenDashboard::buildKnockPage(lv_obj_t* parent) {
  knockStateLabel_ = makeStatusPill(parent, 8, 8, 274, "KNOCK  ONLINE  WARN_ONLY", lv_color_hex(kUiColorGood));
  knockSensorLabel_ = makeMetricTile(parent, 8, 40, 274, 28, "Sensor OK  Learn YES", &lv_font_montserrat_12);
  lv_obj_set_style_text_align(knockSensorLabel_, LV_TEXT_ALIGN_LEFT, 0);

  constexpr lv_coord_t knockBarXOem = 8;
  constexpr lv_coord_t knockBarWOem = 274;
  constexpr lv_coord_t knockBarHOem = 12;
  knockEnergyLabel_ = makeLabel(parent, knockBarXOem, 78, knockBarWOem, "ENERGY 0%", &lv_font_montserrat_12);
  knockEnergyBar_ = lv_bar_create(parent);
  lv_obj_set_pos(knockEnergyBar_, knockBarXOem, 96);
  lv_obj_set_size(knockEnergyBar_, knockBarWOem, knockBarHOem);
  lv_bar_set_range(knockEnergyBar_, 0, 100);
  setBgColor(knockEnergyBar_, lv_color_hex(kUiColorMeterTrack), LV_PART_MAIN);
  setBgColor(knockEnergyBar_, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);

  knockBaselineLabel_ = makeLabel(parent, knockBarXOem, 118, knockBarWOem, "BASELINE 0%", &lv_font_montserrat_12);
  knockBaselineBar_ = lv_bar_create(parent);
  lv_obj_set_pos(knockBaselineBar_, knockBarXOem, 136);
  lv_obj_set_size(knockBaselineBar_, knockBarWOem, knockBarHOem);
  lv_bar_set_range(knockBaselineBar_, 0, 100);
  setBgColor(knockBaselineBar_, lv_color_hex(kUiColorMeterTrack), LV_PART_MAIN);
  setBgColor(knockBaselineBar_, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);

  knockThresholdLabel_ = makeLabel(parent, knockBarXOem, 158, knockBarWOem, "THRESHOLD 0.0", &lv_font_montserrat_12);
  knockThresholdBar_ = lv_bar_create(parent);
  lv_obj_set_pos(knockThresholdBar_, knockBarXOem, 176);
  lv_obj_set_size(knockThresholdBar_, knockBarWOem, knockBarHOem);
  lv_bar_set_range(knockThresholdBar_, 0, 100);
  lv_bar_set_value(knockThresholdBar_, 100, LV_ANIM_OFF);
  setBgColor(knockThresholdBar_, lv_color_hex(kUiColorMeterTrack), LV_PART_MAIN);
  setBgColor(knockThresholdBar_, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);

  knockGraphLabel_ = makeSectionTitle(parent, 300, 8, 168, LV_SYMBOL_WARNING " TREND");
  knockGraphChart_ = lv_chart_create(parent);
  lv_obj_set_pos(knockGraphChart_, 300, 30);
  lv_obj_set_size(knockGraphChart_, 168, 82);
  lv_chart_set_type(knockGraphChart_, LV_CHART_TYPE_LINE);
  lv_chart_set_range(knockGraphChart_, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
  lv_chart_set_point_count(knockGraphChart_, 32);
  lv_chart_set_div_line_count(knockGraphChart_, 3, 5);
  stylePanel(knockGraphChart_, kUiColorRow);
  lv_obj_set_style_line_width(knockGraphChart_, 1, LV_PART_ITEMS);
  knockGraphEnergySeries_ = lv_chart_add_series(knockGraphChart_, lv_palette_main(LV_PALETTE_RED),
                                                LV_CHART_AXIS_PRIMARY_Y);
  knockGraphBaselineSeries_ = lv_chart_add_series(knockGraphChart_, lv_palette_main(LV_PALETTE_GREEN),
                                                  LV_CHART_AXIS_PRIMARY_Y);
  knockGraphThresholdSeries_ = lv_chart_add_series(knockGraphChart_, lv_palette_main(LV_PALETTE_ORANGE),
                                                   LV_CHART_AXIS_PRIMARY_Y);
  for (uint8_t i = 0; i < 32; ++i) {
    lv_chart_set_next_value(knockGraphChart_, knockGraphEnergySeries_, 0);
    lv_chart_set_next_value(knockGraphChart_, knockGraphBaselineSeries_, 0);
    lv_chart_set_next_value(knockGraphChart_, knockGraphThresholdSeries_, 100);
  }

  knockEventLabel_ = makeMetricTile(parent, 300, 124, 168, 54, "CLEAN", &lv_font_montserrat_24);
  lv_label_set_recolor(knockEventLabel_, true);
  knockLastLabel_ = makeMetricTile(parent, 8, 198, 274, 24, "GAIN 1.00   MULT 2.5", &lv_font_montserrat_12);

  constexpr lv_coord_t btnY = 198;
  knockEnableBtn_ = makeBtn(parent, "DISABLE", 300, btnY, 80, 32, onKnockEnableClicked, this);
  knockResetBlBtn_ = makeBtn(parent, "RESET", 388, btnY, 80, 32, onKnockResetBaselineClicked, this);
  knockGainDownBtn_ = makeBtn(parent, "GAIN-", 8, 230, 72, 32, onKnockGainDownClicked, this);
  knockGainUpBtn_ = makeBtn(parent, "GAIN+", 86, 230, 72, 32, onKnockGainUpClicked, this);
  knockMultDownBtn_ = makeBtn(parent, "MULT-", 164, 230, 72, 32, onKnockMultiplierDownClicked, this);
  knockMultUpBtn_ = makeBtn(parent, "MULT+", 242, 230, 72, 32, onKnockMultiplierUpClicked, this);
  knockEnableBtnLabel_ = btnLabel(knockEnableBtn_);
  styleSecondaryButton(knockEnableBtn_);
  styleSecondaryButton(knockResetBlBtn_);
  styleSecondaryButton(knockGainDownBtn_);
  styleSecondaryButton(knockGainUpBtn_);
  styleSecondaryButton(knockMultDownBtn_);
  styleSecondaryButton(knockMultUpBtn_);
  knockLearningSpinner_ = makeLabel(parent, 392, 184, 64, "LEARN", &lv_font_montserrat_12);
}

// ---------------------------------------------------------------------------
// Per-tick update helpers
// ---------------------------------------------------------------------------

void ScreenDashboard::updateHeader(const state::VehicleState& s, uint32_t nowMs) {
  char timeText[16];
  if (s.gps_time_valid) {
    const uint32_t elapsedSec = (nowMs - s.gps_time_sync_ms) / 1000UL;
    const int32_t utcSecOfDay = static_cast<int32_t>((s.gps_utc_seconds_of_day + elapsedSec) % 86400UL);
    int32_t localSecOfDay = utcSecOfDay + static_cast<int32_t>(CCM_GPS_TIMEZONE_OFFSET_MINUTES) * 60;
    localSecOfDay %= 86400L;
    if (localSecOfDay < 0) {
      localSecOfDay += 86400L;
    }
    const uint32_t secOfDay = static_cast<uint32_t>(localSecOfDay);
    const uint8_t hour = static_cast<uint8_t>(secOfDay / 3600UL);
    const uint8_t minute = static_cast<uint8_t>((secOfDay / 60UL) % 60UL);
    const uint8_t second = static_cast<uint8_t>(secOfDay % 60UL);
    const bool pm = hour >= 12U;
    uint8_t hour12 = static_cast<uint8_t>(hour % 12U);
    if (hour12 == 0U) {
      hour12 = 12U;
    }
    snprintf(timeText, sizeof(timeText), "%u:%02u:%02u %s",
             static_cast<unsigned>(hour12),
             static_cast<unsigned>(minute),
             static_cast<unsigned>(second),
             pm ? "PM" : "AM");
  } else {
    snprintf(timeText, sizeof(timeText), "--:--:-- --");
  }
  setLabelTextStatic(hdrBatLabel_, hdrTimeText_, sizeof(hdrTimeText_), timeText);
  setTextColor(hdrBatLabel_,
      lv_color_hex(s.gps_time_valid ? kUiColorText : kUiColorTextMuted),
      0);

  // Fault indicator (right dot)
  setTextColor(hdrFaultDot_,
      (s.fault_flags != 0)
          ? lv_palette_main(LV_PALETTE_RED)
          : lv_palette_main(LV_PALETTE_GREEN),
      0);

  // Action feedback
  if (actionFeedback_[0] != '\0' &&
      nowMs < actionFeedbackUntilMs_ &&
      (actionFeedbackUntilMs_ - nowMs) <= kActionFeedbackMs) {
    setLabelText(hdrFeedbackLabel_, actionFeedback_);
    setTextColor(hdrFeedbackLabel_, lv_palette_main(LV_PALETTE_GREEN), 0);
  } else {
    actionFeedback_[0] = '\0';
    const char* alert = "";
    lv_color_t alertColor = lv_color_hex(kUiColorTextMuted);
    if (s.fault_flags != 0) {
      alert = "FAULT";
      alertColor = lv_palette_main(LV_PALETTE_RED);
    } else if (!s.can_online) {
      alert = "CAN OFF";
      alertColor = lv_palette_main(LV_PALETTE_RED);
    } else if (!s.touch_online) {
      alert = "TOUCH OFF";
      alertColor = lv_palette_main(LV_PALETTE_RED);
    } else if (!s.meth_online) {
      alert = "METH OFF";
      alertColor = lv_palette_main(LV_PALETTE_ORANGE);
    } else if (!s.taillight_online) {
      alert = "TAIL OFF";
      alertColor = lv_palette_main(LV_PALETTE_ORANGE);
    } else if (!s.sd_mounted) {
      alert = "SD";
      alertColor = lv_color_hex(kUiColorTextMuted);
    }
    setLabelText(hdrFeedbackLabel_, alert);
    setTextColor(hdrFeedbackLabel_, alertColor, 0);
  }
}

void ScreenDashboard::updateDashPage(const state::VehicleState& s) {
  char buf[64];
  const float spdMph = s.speed * 0.621371f;
  snprintf(buf, sizeof(buf), "%.0f\nMPH", static_cast<double>(spdMph));
  setLabelTextStatic(spdValLabel_, spdText_, sizeof(spdText_), buf);

  const float boostPsi = s.boost_kpa * 0.145038f;
  lv_color_t boostColor = lv_color_hex(kUiColorText);
  if (boostPsi >= 20.0f) boostColor = lv_color_hex(kUiColorBad);
  else if (boostPsi >= 10.0f) boostColor = lv_color_hex(kUiColorWarn);
  snprintf(buf, sizeof(buf), "BOOST\n%.1f PSI", static_cast<double>(boostPsi));
  setLabelTextStatic(boostValLabel_, boostText_, sizeof(boostText_), buf);
  setTextColor(boostValLabel_, boostColor, 0);

  const uint8_t methDuty = static_cast<uint8_t>(s.meth_pump_duty > 100U ? 100U : s.meth_pump_duty);
  snprintf(buf, sizeof(buf), "TANK\n%u%% / %u%%",
           static_cast<unsigned>(s.meth_tank_level),
           static_cast<unsigned>(methDuty));
  setLabelTextStatic(dashEnvLabel_, dashEnvText_, sizeof(dashEnvText_), buf);

  const bool methActive = (s.meth_state == state::MethState::SPRAYING) || s.manual_test_running;
  const char* methColor = !s.meth_online ? kStatusColorOff : (methActive ? "#FF9500" : "#00C853");
  const char* methText = !s.meth_online ? "OFFLINE" : (methActive ? "ACTIVE" : (s.meth_desired_armed ? "ARMED" : "READY"));
  snprintf(buf, sizeof(buf), "METH\n%s %s#", methColor, methText);
  setLabelTextStatic(dashStatusLabel_, dashStatusText_, sizeof(dashStatusText_), buf);

  const char* knockDash =
      s.knock_critical_active ? "#FF3B30 CRIT#" :
      (s.knock_warning_active ? "#FF9500 WARN#" : "#00C853 OK#");
  snprintf(buf, sizeof(buf), "KNOCK\n%s", knockDash);
  setLabelTextStatic(dashRaceLabel_, dashRaceText_, sizeof(dashRaceText_), buf);

  snprintf(buf, sizeof(buf), "RPM %u", static_cast<unsigned>(s.rpm));
  setLabelTextStatic(rpmValLabel_, rpmText_, sizeof(rpmText_), buf);
  snprintf(buf, sizeof(buf), "FUEL %.0f psi", static_cast<double>(s.fuel_pressure_psi));
  setLabelTextStatic(gLiveLabel_, gLiveText_, sizeof(gLiveText_), buf);
  setTextColor(gLiveLabel_, lv_color_hex(s.fuel_pressure_valid ? kUiColorText : kUiColorBad), 0);
  snprintf(buf, sizeof(buf), "OIL %.0f psi", static_cast<double>(s.oil_pressure_psi));
  setLabelTextStatic(gPeakLabel_, gPeakText_, sizeof(gPeakText_), buf);
  setTextColor(gPeakLabel_, lv_color_hex(s.oil_pressure_valid ? kUiColorText : kUiColorBad), 0);
  snprintf(buf, sizeof(buf), "CAN %s", s.can_online ? "ON" : "OFF");
  setLabelText(raceAccelBtn_, buf);
}


void ScreenDashboard::updateMethPage(const state::VehicleState& s, uint32_t nowMs) {
  char buf[160];
  const bool extFresh = (s.last_analog_sensor_ms != 0U) &&
                        ((nowMs - s.last_analog_sensor_ms) <= kMethSensorStaleMs);
  const bool methActive = (s.meth_state == state::MethState::SPRAYING) || s.manual_test_running;
  lv_color_t moduleColor = lv_color_hex(s.meth_online ? kUiColorGood : kUiColorBad);
  lv_color_t extColor = lv_color_hex(extFresh ? kUiColorGood : kUiColorBad);
  lv_color_t armColor = lv_color_hex((s.meth_desired_armed || methActive) ? kUiColorButtonActive : kUiColorTextMuted);

  setLabelText(methBadgeLabel_, "WATER-METH");
  lv_obj_set_style_border_color(methBadgeLabel_, moduleColor, LV_PART_MAIN);
  setTextColor(methBadgeLabel_, moduleColor, 0);
  snprintf(buf, sizeof(buf), "%s", s.meth_online ? (methActive ? "ACTIVE" : "ONLINE") : "OFFLINE");
  setLabelText(methStateLabel_, buf);
  lv_obj_set_style_border_color(methStateLabel_, moduleColor, LV_PART_MAIN);
  setTextColor(methStateLabel_, moduleColor, 0);
  snprintf(buf, sizeof(buf), "EXT %s", extFresh ? "ON" : "OFF");
  setLabelText(methSensorLabel_, buf);
  lv_obj_set_style_border_color(methSensorLabel_, extColor, LV_PART_MAIN);
  setTextColor(methSensorLabel_, extColor, 0);
  snprintf(buf, sizeof(buf), "%s", (s.meth_desired_armed || methActive) ? "ARMED" : "ARM OFF");
  setLabelText(methParamLabel_, buf);
  lv_obj_set_style_border_color(methParamLabel_, armColor, LV_PART_MAIN);
  setTextColor(methParamLabel_, armColor, 0);

  snprintf(buf, sizeof(buf), "DUTY\n%u%%",
           static_cast<unsigned>(s.meth_pump_duty > 100U ? 100U : s.meth_pump_duty));
  setLabelText(methDutyLabel_, buf);
  snprintf(buf, sizeof(buf), "TANK\n%u%%", static_cast<unsigned>(s.meth_tank_level));
  setLabelText(methTankLabel_, buf);
  snprintf(buf, sizeof(buf), "MAP\n%.0f kPa", static_cast<double>(s.boost_kpa));
  setLabelText(methMapLabel_, buf);
  snprintf(buf, sizeof(buf), "METH\n%.0f psi", static_cast<double>(s.meth_pressure_psi));
  setLabelText(methPressureLabel_, buf);
  snprintf(buf, sizeof(buf), "IAT %.1f C", static_cast<double>(s.intake_temp));
  setLabelText(methIatLabel_, buf);
  snprintf(buf, sizeof(buf), "BAY %.1f C", static_cast<double>(s.engine_bay_temp));
  setLabelText(methBayLabel_, buf);

  setLabelText(methArmBtnLabel_, s.meth_desired_armed ? "DISARM" : "ARM");
  snprintf(buf, sizeof(buf), "RATIO %u%%", static_cast<unsigned>(s.meth_selected_ratio_percent));
  setLabelText(methRatioBtnLabel_, buf);
  setBgColor(methArmBtn_,
      methActive ? lv_color_hex(kUiColorWarn) :
      (s.meth_desired_armed ? lv_color_hex(kUiColorButtonActive) : lv_color_hex(kUiColorButton)),
      LV_PART_MAIN);
}


void ScreenDashboard::updateTailPage(const state::VehicleState& s) {
  char buf[96];
  snprintf(buf, sizeof(buf), "TAILS  %s  %s  BRI %u",
           s.taillight_online ? "ONLINE" : "OFFLINE",
           tailModeName(s.taillight_mode_commanded),
           static_cast<unsigned>(s.taillight_brightness));
  setLabelText(tailStatusLabel_, buf);
  lv_obj_set_style_border_color(tailStatusLabel_,
      lv_color_hex(s.taillight_online ? kUiColorGood : kUiColorBad), LV_PART_MAIN);
  setTextColor(tailStatusLabel_,
      lv_color_hex(s.taillight_online ? kUiColorGood : kUiColorBad), 0);

  if (tailOnlineLed_) {
    if (s.taillight_online) lv_led_on(tailOnlineLed_);
    else lv_led_off(tailOnlineLed_);
  }

  setBgColor(tailStockBtn_, lv_color_hex(s.taillight_mode_commanded == 0 ? kUiColorButtonActive : kUiColorButton), LV_PART_MAIN);
  setBgColor(tailSeqBtn_, lv_color_hex(s.taillight_mode_commanded == 1 ? kUiColorButtonActive : kUiColorButton), LV_PART_MAIN);
  setBgColor(tailDemoBtn_, lv_color_hex(s.taillight_mode_commanded == 2 ? kUiColorButtonActive : kUiColorButton), LV_PART_MAIN);
  setBgColor(tailShowMenuBtn_, lv_color_hex(s.taillight_mode_commanded > 2 ? kUiColorButtonActive : kUiColorButton), LV_PART_MAIN);

  static constexpr const char* kTailShowNames[kTaillightShowOptionCount] = {
    "Rainbow", "Chase", "Theater", "Fire", "Meteor", "Police",
    "Night Rider", "Color Cycle", "Sparkle", "Plasma", "Matrix", "Juggle",
    "BPM", "Confetti", "Ocean", "Lightning", "Heartbeat", "Ripple",
    "Sunrise", "Text Scroll", "Colorwaves", "TwinkleFox", "Bounce", "Fireworks",
    "Drip", "Cylon", "V8", "Drag Launch", "Neon", "Streaks", "Radar", "Aurora", "Glitch"
  };
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    const uint16_t optVal = static_cast<uint16_t>(tailShowPage_) * kTaillightShowOptionsPerPage + i;
    setLabelText(btnLabel(tailShowOptBtns_[i]), optVal < kTaillightShowOptionCount ? kTailShowNames[optVal] : "N/A");
  }
  snprintf(buf, sizeof(buf), "SHOW %u/%u",
           static_cast<unsigned>(tailShowPage_ + 1U),
           static_cast<unsigned>(kTaillightShowPageCount));
  setLabelText(tailShowPageLabel_, buf);
}


void ScreenDashboard::updateLedsPage(const state::VehicleState& s) {
  bool allEnabled = true;
  for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
    const bool enabled = s.led_zone_enabled[zone] && s.led_zone_mode[zone] != state::LedMode::OFF;
    allEnabled = allEnabled && enabled;
  }
  if (ledMasterSwitch_) {
    suppressLedMasterEvent_ = true;
    if (allEnabled) lv_obj_add_state(ledMasterSwitch_, LV_STATE_CHECKED);
    else lv_obj_clear_state(ledMasterSwitch_, LV_STATE_CHECKED);
    suppressLedMasterEvent_ = false;
  }
  if (ledMasterLabel_) {
    setLabelText(ledMasterLabel_, allEnabled ? "MASTER ON" : "MASTER OFF");
  }

  for (uint8_t ch = 0; ch < kInteriorLedUiCount; ++ch) {
    if (ledStripLabels_[ch]) {
      setLabelText(ledStripLabels_[ch], ledZoneName(ch));
    }
    for (uint8_t i = 0; i < kLedModeButtonCount; i++) {
      const state::LedMode btnMode = s_ledModeCtxs[ch][i].mode;
      bool active = false;
      if (ch == 0U) {
        active = true;
        for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
          active = active &&
                   s.led_zone_enabled[zone] &&
                   s.led_zone_mode[zone] == btnMode;
        }
      } else {
        const uint8_t zone = ch - 1U;
        const bool enabled = s.led_zone_enabled[zone] &&
                             s.led_zone_mode[zone] != state::LedMode::OFF;
        active = enabled && s.led_zone_mode[zone] == btnMode;
      }
      setBgColor(ledModeBtns_[ch][i],
          active ? lv_color_hex(kUiColorButtonActive) : lv_color_hex(kUiColorButton),
          LV_PART_MAIN);
    }
  }

  for (uint8_t i = 0; i < kLedShowModeButtonCount; ++i) {
    const state::LedMode btnMode = s_ledShowCtxs[i].mode;
    bool active = true;
    for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
      active = active && s.led_zone_enabled[zone] && s.led_zone_mode[zone] == btnMode;
    }
    if (ledShowModeBtns_[i]) {
      setBgColor(ledShowModeBtns_[i],
          active ? lv_color_hex(kUiColorButtonActive) : lv_color_hex(kUiColorButton),
          LV_PART_MAIN);
    }
  }

  for (uint8_t i = 0; i < kLedColorButtonCount; ++i) {
    const uint32_t color = s_ledColorCtxs[i].color;
    bool active = true;
    for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
      active = active &&
               s.led_zone_enabled[zone] &&
               s.led_zone_mode[zone] == state::LedMode::STATIC_COLOR &&
               s.led_zone_color[zone] == color;
    }
    if (ledColorBtns_[i]) {
      lv_obj_set_style_border_width(ledColorBtns_[i], active ? 3 : 1, LV_PART_MAIN);
      lv_obj_set_style_border_color(ledColorBtns_[i],
          active ? lv_color_white() : lv_color_hex(kUiColorButtonBorder),
          LV_PART_MAIN);
    }
  }
}

void ScreenDashboard::updateGpsPage(const state::VehicleState& s) {
  char spd[24];
  const bool gpsDeadReckoned = (s.gps_status_flags & kGpsStatusDeadReckoned) != 0;
  const bool gpsSpeedUsable = s.gps_fix || gpsDeadReckoned;
  if (gpsSpeedUsable) {
    float displayMph = (s.speed > 0.0f) ? (s.speed * 0.621371f) : 0.0f;
    if (displayMph < kGpsZeroClampMph) {
      displayMph = 0.0f;
    }
    snprintf(spd, sizeof(spd), "%.0f mph", static_cast<double>(displayMph));
  } else {
    snprintf(spd, sizeof(spd), "0 mph");
  }
  setLabelText(gpsSpdLabel_, spd);

  char info[160];
  const char* fixColor = s.gps_fix ? "00C853" : "FF3B30";
  snprintf(info, sizeof(info),
           "#%s FIX:%s#   USED:%u   VIEW:%u   HDOP:%.1f\nLAT: %.6f\nLON: %.6f",
           fixColor,
           s.gps_fix ? "YES" : "NO",
           static_cast<unsigned>(s.gps_satellites),
           static_cast<unsigned>(s.gps_satellites_in_view),
           static_cast<double>(s.gps_hdop_x10 / 10.0f),
           s.gps_latitude,
           s.gps_longitude);
  setLabelText(gpsInfoLabel_, info);
}


void ScreenDashboard::updateTempsPage(const state::VehicleState& s) {
  char v[32];
  const bool sensorsOk = s.analog_sensor_fault_flags == 0 &&
                         s.oil_pressure_valid &&
                         s.fuel_pressure_valid &&
                         s.intake_temp_valid &&
                         s.engine_bay_temp_valid &&
                         s.cabin_temp_valid &&
                         s.outside_temp_valid;
  if (tempsLabel_) {
    if (s.analog_sensor_fault_flags != 0) {
      snprintf(v, sizeof(v), "SENSOR BOARD  FAULT 0x%04X",
               static_cast<unsigned>(s.analog_sensor_fault_flags));
    } else {
      snprintf(v, sizeof(v), "SENSOR BOARD  %s", sensorsOk ? "OK" : "CHECK");
    }
    setLabelText(tempsLabel_, v);
    lv_obj_set_style_border_color(tempsLabel_,
        lv_color_hex(sensorsOk ? kUiColorGood : kUiColorBad), LV_PART_MAIN);
    setTextColor(tempsLabel_, lv_color_hex(sensorsOk ? kUiColorGood : kUiColorBad), 0);
  }
  if (!tempsTable_) return;

  auto setRow = [&](uint16_t row, const char* name, float value, bool ok, const char* unit) {
    char valueBuf[24];
    snprintf(valueBuf, sizeof(valueBuf), "%.1f %s", static_cast<double>(value), unit);
    lv_table_set_cell_value(tempsTable_, row, 0, name);
    lv_table_set_cell_value(tempsTable_, row, 1, valueBuf);
    lv_table_set_cell_value(tempsTable_, row, 2, ok ? "OK" : "BAD");
  };
  setRow(1, "OIL", s.oil_pressure_psi, s.oil_pressure_valid, "psi");
  setRow(2, "FUEL", s.fuel_pressure_psi, s.fuel_pressure_valid, "psi");
  setRow(3, "IAT", s.intake_temp, s.intake_temp_valid, "C");
  setRow(4, "BAY", s.engine_bay_temp, s.engine_bay_temp_valid, "C");
  setRow(5, "CABIN", s.cabin_temp, s.cabin_temp_valid, "C");
  setRow(6, "AMBIENT", s.outside_temp, s.outside_temp_valid, "C");
}


void ScreenDashboard::updateDiagPage(const state::VehicleState& s) {
  if (!diagLabel_) return;

  const uint32_t now = s.uptime_ms;
  auto msAgo = [now](uint32_t ts) -> uint32_t {
    return (ts == 0 || now < ts) ? 9999UL : (now - ts) / 1000UL;
  };

  const char* resetStr;
  switch (s.reset_reason) {
    case 1:  resetStr = "POR"; break;
    case 3:  resetStr = "SW";  break;
    case 5:  resetStr = "WDT"; break;
    case 7:  resetStr = "SLP"; break;
    case 12: resetStr = "BOR"; break;
    case 14: resetStr = "EXT"; break;
    default: resetStr = "UNK"; break;
  }

  auto col = [](bool ok) -> const char* {
    return ok ? "#00C853" : "#FF3B30";
  };

  const bool gpsActive = msAgo(s.last_gps_ms) < 5;
  const char* gpsCol = s.gps_fix ? "#00C853" : (gpsActive ? "#FFD600" : "#FF3B30");
  char gpsStat[12];
  if (s.gps_fix)      snprintf(gpsStat, sizeof(gpsStat), "FIX %u/%u", s.gps_satellites, s.gps_satellites_in_view);
  else if (gpsActive) snprintf(gpsStat, sizeof(gpsStat), "NO FIX");
  else                snprintf(gpsStat, sizeof(gpsStat), "OFFLINE");

  const bool canRxActive = s.can_rx_count > 0 && msAgo(s.can_last_rx_ms) < 10;
  const char* canCol = s.can_online ? (canRxActive ? "#00C853" : "#FFD600") : "#FF3B30";
  const char* canStat = s.can_online ? (canRxActive ? "RX OK" : "TX ONLY") : "OFFLINE";
  const uint32_t heapMin = (s.heap_min_free_bytes == 0xFFFFFFFFUL)
      ? s.heap_free_bytes : s.heap_min_free_bytes;

  char compact[900];
  snprintf(compact, sizeof(compact),
      "#FFFFFF SYSTEM#  Up %lus  Heap %luK/%luK  Die %dC  Rst %s  FPS %.1f\n"
      "#FFFFFF NETWORK# CAN %s %s#  RX %lu TX %lu CRC %lu  Last 0x%03X %lus\n"
      "#FFFFFF GPS#     %s %s#  Used:%u View:%u HDOP:%.1f\n"
      "#FFFFFF MODULES# METH %s %s#  TAIL %s %s#  SD %s %s#\n"
      "#FFFFFF INPUT#   TOUCH %s %s#  IMU %s %s#  G %.2f / %.2f\n"
      "#FFFFFF TACH#    RPM %u  In %.1fHz  Out %.1fHz  PPR %.1f  ST 0x%02X\n"
      "#FFFFFF METH#    Duty %u%%  Flow %s  Tank %s  Ratio %u%%",
      static_cast<unsigned long>(s.uptime_ms / 1000UL),
      static_cast<unsigned long>(s.heap_free_bytes / 1024),
      static_cast<unsigned long>(heapMin / 1024),
      static_cast<int>(s.esp_die_temp_c),
      resetStr,
      static_cast<double>(s.ui_fps),
      canCol, canStat,
      static_cast<unsigned long>(s.can_rx_count),
      static_cast<unsigned long>(s.can_tx_count),
      static_cast<unsigned long>(s.can_bad_checksum_count),
      static_cast<unsigned>(s.can_last_rx_id),
      static_cast<unsigned long>(msAgo(s.can_last_rx_ms)),
      gpsCol, gpsStat,
      static_cast<unsigned>(s.gps_satellites),
      static_cast<unsigned>(s.gps_satellites_in_view),
      static_cast<double>(s.gps_hdop_x10 / 10.0f),
      col(s.meth_online), s.meth_online ? "ONLINE" : "OFFLINE",
      col(s.taillight_online), s.taillight_online ? "ONLINE" : "OFFLINE",
      col(s.sd_mounted), s.sd_mounted ? "MOUNTED" : "NO CARD",
      col(s.touch_online), s.touch_online ? "OK" : "OFFLINE",
      col(s.imu_online), s.imu_online ? "OK" : "OFFLINE",
      static_cast<double>(s.imu_g_lateral),
      static_cast<double>(s.imu_g_longitudinal),
      static_cast<unsigned>(s.rpm),
      static_cast<double>(s.tach_input_frequency_hz),
      static_cast<double>(s.tach_generated_frequency_hz),
      static_cast<double>(s.pulses_per_rev10 / 10.0f),
      static_cast<unsigned>(s.tach_status_flags),
      static_cast<unsigned>(s.meth_pump_duty),
      methFlowName(s.meth_flow_status),
      (s.meth_tank_level == 0) ? "EMPTY" : "OK",
      static_cast<unsigned>(s.meth_selected_ratio_percent));
  setLabelText(diagLabel_, compact);
  const bool toolsVisible = !diagToolsPanel_ || !lv_obj_has_flag(diagToolsPanel_, LV_OBJ_FLAG_HIDDEN);
  if (benchTestBtn_ && toolsVisible) {
    setBgColor(benchTestBtn_,
        s.bench_test_mode ? lv_color_hex(0xFF8C00) : lv_color_hex(kUiColorButton),
        LV_PART_MAIN);
    setLabelText(btnLabel(benchTestBtn_),
        s.bench_test_mode ? "SIM: ON" : "SIM: OFF");
  }
  if (diagRaceStatusLabel_ && toolsVisible) {
    char raceBuf[160];
    const char* mode =
        s.race_mode == state::RaceMode::ACCEL ? "ACCEL" :
        s.race_mode == state::RaceMode::LAP ? "LAP" : "OFF";
    snprintf(raceBuf, sizeof(raceBuf),
             "%s %s Q%u  0-60 %.2f  1/4 %.2f@%.0f  LAP %.2f BEST %.2f",
             mode,
             s.race_running ? "RUN" : "STOP",
             static_cast<unsigned>(s.race_quality_percent),
             static_cast<double>(s.race_0_60_s),
             static_cast<double>(s.race_quarter_mile_et_s),
             static_cast<double>(s.race_quarter_mile_trap_mph),
             static_cast<double>(s.race_last_lap_s),
             static_cast<double>(s.race_best_lap_s));
    setLabelText(diagRaceStatusLabel_, raceBuf);
  }
  if (toolsVisible) {
    refreshSdBrowser(now, false);
  }
}


void ScreenDashboard::updateKnockPage(const state::VehicleState& s, uint32_t nowMs) {
  (void)nowMs;
  if (!knockStateLabel_) return;

  // Determine sensor status string
  const char* sensorStr;
  if (!s.knock_enabled) {
    sensorStr = "DISABLED";
  } else if (s.knock_sensor_fault) {
    sensorStr = "DISCONNECTED";
  } else if (s.knock_clipping_detected) {
    sensorStr = "CLIPPING";
  } else if (!s.knock_baseline_learned) {
    sensorStr = "LEARNING";
  } else if (!s.knock_signal_valid) {
    sensorStr = "NOISY";
  } else {
    sensorStr = "OK";
  }

  // Response mode strings
  static const char* const kRespMode[] = { "LOG_ONLY", "WARN_ONLY", "METH_ENABLE", "SAFE_SHTDWN" };
  const char* respStr = (s.knock_response_mode < 4) ? kRespMode[s.knock_response_mode] : "?";

  // Header with warning/critical colour
  char buf[128];
  if (s.knock_critical_active) {
    snprintf(buf, sizeof(buf), "#FF3B30 KNOCK# | %s | %s",
             s.knock_online ? "ONLINE" : "STALE", respStr);
  } else if (s.knock_warning_active) {
    snprintf(buf, sizeof(buf), "#FF9500 KNOCK# | %s | %s",
             s.knock_online ? "ONLINE" : "STALE", respStr);
  } else {
    snprintf(buf, sizeof(buf), "#00C853 KNOCK# | %s | %s",
             s.knock_online ? "ONLINE" : "STALE", respStr);
  }
  setLabelText(knockStateLabel_, buf);

  // Sensor status row
  snprintf(buf, sizeof(buf), "Sensor:%s  Learn:%s  Clip:%u/%u",
           sensorStr,
           s.knock_baseline_learned ? "YES" : "NO",
           static_cast<unsigned>(s.knock_signal_clip_high_count & 0xFFU),
           static_cast<unsigned>(s.knock_signal_clip_low_count  & 0xFFU));
  setLabelText(knockSensorLabel_, buf);

  // Energy bar: 0..100 where 100 = threshold level
  const float thresh = (s.knock_threshold > 0.1f) ? s.knock_threshold : 1.0f;
  const int energyPct = static_cast<int>((s.knock_energy / thresh) * 100.0f);
  const int clampedE  = (energyPct > 100) ? 100 : (energyPct < 0 ? 0 : energyPct);
  snprintf(buf, sizeof(buf), "Knock Energy:  %.1f  (%d%% of threshold)",
           static_cast<double>(s.knock_energy), clampedE);
  setLabelText(knockEnergyLabel_, buf);
  lv_bar_set_value(knockEnergyBar_, clampedE, LV_ANIM_OFF);

  // Energy bar colour: green → yellow → red based on level
  lv_color_t eCo;
  if (clampedE >= 90)       eCo = lv_palette_main(LV_PALETTE_RED);
  else if (clampedE >= 60)  eCo = lv_palette_main(LV_PALETTE_ORANGE);
  else                      eCo = lv_palette_main(LV_PALETTE_GREEN);
  setBgColor(knockEnergyBar_, eCo, LV_PART_INDICATOR);

  // Baseline bar
  const int baselinePct = static_cast<int>((s.knock_baseline / thresh) * 100.0f);
  const int clampedB    = (baselinePct > 100) ? 100 : (baselinePct < 0 ? 0 : baselinePct);
  snprintf(buf, sizeof(buf), "Baseline:  %.1f  (%d%% of threshold)",
           static_cast<double>(s.knock_baseline), clampedB);
  setLabelText(knockBaselineLabel_, buf);
  lv_bar_set_value(knockBaselineBar_, clampedB, LV_ANIM_OFF);

  // Threshold label (threshold bar is always 100% — it marks the limit)
  snprintf(buf, sizeof(buf), "Threshold:  %.1f  (×%.1f baseline + %.1f offset)",
           static_cast<double>(s.knock_threshold),
           static_cast<double>(s.knock_threshold_multiplier),
           static_cast<double>(s.knock_threshold_offset));
  setLabelText(knockThresholdLabel_, buf);

  // Push current normalized values into live chart trend.
  if (knockGraphChart_ && knockGraphEnergySeries_ && knockGraphBaselineSeries_ &&
      knockGraphThresholdSeries_) {
    lv_chart_set_next_value(knockGraphChart_, knockGraphEnergySeries_, clampedE);
    lv_chart_set_next_value(knockGraphChart_, knockGraphBaselineSeries_, clampedB);
    lv_chart_set_next_value(knockGraphChart_, knockGraphThresholdSeries_, 100);
    lv_chart_refresh(knockGraphChart_);
  }
  if (knockLearningSpinner_) {
    setObjHidden(knockLearningSpinner_, !(s.knock_enabled && !s.knock_baseline_learned));
  }

  // Prominent warning banner.
  if (s.knock_critical_active) {
    snprintf(buf, sizeof(buf), "#FFFFFF KNOCK#\n#FF3B30 CRITICAL#");
    setBgColor(knockEventLabel_, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
  } else if (s.knock_warning_active) {
    snprintf(buf, sizeof(buf), "#000000 KNOCK#\n#FF9500 WARNING#");
    setBgColor(knockEventLabel_, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);
  } else if (!s.knock_baseline_learned && s.knock_enabled) {
    snprintf(buf, sizeof(buf), "#FF9500 LEARNING#");
    setBgColor(knockEventLabel_, lv_color_hex(kUiColorRow), LV_PART_MAIN);
  } else {
    snprintf(buf, sizeof(buf), "#00C853 CLEAN#");
    setBgColor(knockEventLabel_, lv_color_hex(kUiColorRow), LV_PART_MAIN);
  }
  setLabelText(knockEventLabel_, buf);

  snprintf(buf, sizeof(buf), "GAIN %.2f    MULT %.1f",
           static_cast<double>(s.knock_gain),
           static_cast<double>(s.knock_threshold_multiplier));
  setLabelText(knockLastLabel_, buf);

  // Enable button label
  setLabelText(knockEnableBtnLabel_, s.knock_enabled ? "DISABLE" : "ENABLE");
}

void ScreenDashboard::updateActivePage(const state::VehicleState& s, uint32_t nowMs) {
  // Only update the visible page. Updating hidden objects still dirties LVGL's
  // region tracker and can produce stale partial redraws on tab switches.
  switch (activePage_) {
    case 0: updateDashPage(s);          break;
    case 1: updateMethPage(s, nowMs);   break;
    case 2: updateTailPage(s);          break;
    case 3: updateLedsPage(s);          break;
    case 4: updateGpsPage(s);           break;
    case 5: updateTempsPage(s);         break;
    case 6: updateDiagPage(s);          break;
    case 7: updateKnockPage(s, nowMs);  break;
    default: break;
  }
}

void ScreenDashboard::forceContentRepaint() {
  if (topContentEdgeGuard_) {
    lv_obj_move_foreground(topContentEdgeGuard_);
    lv_obj_invalidate(topContentEdgeGuard_);
  }
  if (bottomEdgeGuard_) {
    lv_obj_move_foreground(bottomEdgeGuard_);
    lv_obj_invalidate(bottomEdgeGuard_);
  }
  if (kFullRepaintOnPageSwitch) {
    lv_obj_invalidate(lv_scr_act());
    return;
  }
  if (activePage_ < kPageCount && pages_[activePage_]) {
    lv_obj_invalidate(pages_[activePage_]);
  }
}

void ScreenDashboard::setActionFeedback(const char* text, uint32_t nowMs) {
  if (!text) return;
  strncpy(actionFeedback_, text, sizeof(actionFeedback_) - 1);
  actionFeedback_[sizeof(actionFeedback_) - 1] = '\0';
  actionFeedbackUntilMs_ = nowMs + kActionFeedbackMs;
}

void ScreenDashboard::queueSettingsSave(uint32_t nowMs) {
  if (!settingsMgr_) return;
  settingsMgr_->updateFromState(state::g_vehicle_state.read());
  settingsSaveQueued_ = true;
  settingsSaveDueMs_ = nowMs + CCM_SETTINGS_SAVE_DEBOUNCE_MS;
}

void ScreenDashboard::serviceQueuedSettingsSave(uint32_t nowMs) {
  if (!settingsSaveQueued_ || !settingsMgr_) return;
  if (static_cast<int32_t>(nowMs - settingsSaveDueMs_) < 0) return;

  touch::TouchSample touch = {};
  uint32_t sampleMs = 0;
  uint32_t pressStartMs = 0;
  portENTER_CRITICAL(&touchMux_);
  touch = filteredTouch_;
  sampleMs = filteredTouchSampleMs_;
  pressStartMs = filteredTouchPressStartMs_;
  portEXIT_CRITICAL(&touchMux_);
  if (touch.touched &&
      (nowMs - sampleMs) <= CCM_TOUCH_STALE_RELEASE_MS &&
      (nowMs - pressStartMs) <= CCM_TOUCH_MAX_PRESS_MS) {
    settingsSaveDueMs_ = nowMs + CCM_SETTINGS_SAVE_DEBOUNCE_MS;
    return;
  }

  settingsMgr_->save();
  settingsSaveQueued_ = false;
}

bool ScreenDashboard::enqueueAction(const UiAction& action, uint32_t nowMs) {
  if (action.type == UiActionType::None) return false;
  if (pendingConfirmType_ != UiActionType::None &&
      (pendingConfirmType_ != action.type ||
       pendingConfirmAction_.arg0 != action.arg0 ||
       pendingConfirmAction_.arg1 != action.arg1 ||
       pendingConfirmAction_.value != action.value)) {
    pendingConfirmType_ = UiActionType::None;
    pendingConfirmAction_ = {};
  }
  if (uiActionCount_ >= kUiActionQueueSize) {
    setActionFeedback("UI BUSY", nowMs);
    return false;
  }
  uiActions_[uiActionTail_] = action;
  uiActionTail_ = static_cast<uint8_t>((uiActionTail_ + 1U) % kUiActionQueueSize);
  ++uiActionCount_;
  return true;
}

bool ScreenDashboard::shouldAcceptUiTap(lv_obj_t* target, uint32_t nowMs) {
  if (target && target == lastAcceptedTapTarget_ &&
      static_cast<uint32_t>(nowMs - lastAcceptedTapMs_) < CCM_UI_TAP_REPEAT_GUARD_MS) {
    return false;
  }
  lastAcceptedTapTarget_ = target;
  lastAcceptedTapMs_ = nowMs;
  return true;
}

bool ScreenDashboard::confirmOrEnqueue(const UiAction& action, const char* prompt, uint32_t nowMs) {
  const bool samePending =
      pendingConfirmType_ == action.type &&
      pendingConfirmAction_.arg0 == action.arg0 &&
      pendingConfirmAction_.arg1 == action.arg1 &&
      pendingConfirmAction_.value == action.value &&
      static_cast<int32_t>(nowMs - pendingConfirmUntilMs_) <= 0;
  if (samePending) {
    pendingConfirmType_ = UiActionType::None;
    pendingConfirmAction_ = {};
    return enqueueAction(action, nowMs);
  }

  pendingConfirmType_ = action.type;
  pendingConfirmAction_ = action;
  pendingConfirmUntilMs_ = nowMs + CCM_UI_CONFIRM_TIMEOUT_MS;
  setActionFeedback(prompt ? prompt : "TAP AGAIN", nowMs);
  return false;
}

void ScreenDashboard::serviceTouchGestures(uint32_t nowMs) {
  if (touchCalOverlay_ && !lv_obj_has_flag(touchCalOverlay_, LV_OBJ_FLAG_HIDDEN)) {
    gestureTouchWasDown_ = false;
    return;
  }
  if (faultOverlay_ && !lv_obj_has_flag(faultOverlay_, LV_OBJ_FLAG_HIDDEN)) {
    gestureTouchWasDown_ = false;
    return;
  }

  touch::TouchSample touch = {};
  portENTER_CRITICAL(&touchMux_);
  touch = filteredTouch_;
  portEXIT_CRITICAL(&touchMux_);

  if (touch.touched && !gestureTouchWasDown_) {
    gestureTouchWasDown_ = true;
    gestureStartMs_ = nowMs;
    gestureStartX_ = touch.x;
    gestureStartY_ = touch.y;
    return;
  }
  if (touch.touched || !gestureTouchWasDown_) {
    return;
  }

  gestureTouchWasDown_ = false;
  const int16_t dx = static_cast<int16_t>(touch.x) - static_cast<int16_t>(gestureStartX_);
  const int16_t dy = static_cast<int16_t>(touch.y) - static_cast<int16_t>(gestureStartY_);
  const uint32_t durationMs = nowMs - gestureStartMs_;
  const uint16_t absDx = static_cast<uint16_t>(dx < 0 ? -dx : dx);
  const uint16_t absDy = static_cast<uint16_t>(dy < 0 ? -dy : dy);

  if (gestureStartY_ < kHdrH && durationMs <= CCM_UI_DOUBLE_TAP_MS) {
    if (lastHeaderTapMs_ != 0 && (nowMs - lastHeaderTapMs_) <= CCM_UI_DOUBLE_TAP_MS) {
      lastHeaderTapMs_ = 0;
      enqueueAction({UiActionType::Nav, 0, 0, 0}, nowMs);
      setActionFeedback("HOME", nowMs);
    } else {
      lastHeaderTapMs_ = nowMs;
    }
    return;
  }

  const bool fromContent = gestureStartY_ >= kHdrH &&
                           gestureStartY_ < static_cast<uint16_t>(kHdrH + kContentH);
  if (fromContent &&
      durationMs <= CCM_UI_SWIPE_MAX_MS &&
      absDx >= CCM_UI_SWIPE_MIN_PX &&
      absDy <= CCM_UI_SWIPE_MAX_OFF_AXIS_PX) {
    const uint8_t nextPage = (dx < 0)
        ? static_cast<uint8_t>((activePage_ + 1U) % kPageCount)
        : static_cast<uint8_t>((activePage_ + kPageCount - 1U) % kPageCount);
    enqueueAction({UiActionType::Nav, nextPage, 0, 0}, nowMs);
  }
}

void ScreenDashboard::serviceUiActions(uint32_t nowMs) {
  uint8_t serviced = 0;
  while (uiActionCount_ > 0 && serviced < CCM_UI_ACTIONS_PER_TICK) {
    const UiAction action = uiActions_[uiActionHead_];
    uiActionHead_ = static_cast<uint8_t>((uiActionHead_ + 1U) % kUiActionQueueSize);
    --uiActionCount_;
    performUiAction(action, nowMs);
    ++serviced;
  }
}

void ScreenDashboard::performUiAction(const UiAction& action, uint32_t nowMs) {
  switch (action.type) {
    case UiActionType::Nav:
      showPage(action.arg0);
      break;

    case UiActionType::MethArm: {
      const bool arm = action.arg0 != 0;
      if (canMgr_ && canMgr_->sendMethArm(arm)) {
        state::g_vehicle_state.mutate([arm](state::VehicleState& vs) { vs.meth_desired_armed = arm; });
        setActionFeedback(arm ? "METH ON" : "METH OFF", nowMs);
      } else {
        setActionFeedback("METH CMD REJECTED", nowMs);
      }
      break;
    }

    case UiActionType::MethRatio: {
      const uint8_t ratio = action.arg0;
      state::g_vehicle_state.mutate([ratio](state::VehicleState& vs) { vs.meth_selected_ratio_percent = ratio; });
      queueSettingsSave(nowMs);
      if (canMgr_) canMgr_->sendMethConfigBroadcast();
      setActionFeedback("METH RATIO UPDATED", nowMs);
      break;
    }

    case UiActionType::TailMode: {
      const uint8_t mode = action.arg0;
      const bool sent = canMgr_ && canMgr_->sendTaillightMode(mode);
      if (!sent) {
        setActionFeedback("TAIL CMD REJECTED", nowMs);
      } else if (mode == can_protocol::taillight_mode::STOCK) {
        setActionFeedback("TAIL STOCK", nowMs);
      } else if (mode == can_protocol::taillight_mode::SEQUENTIAL) {
        setActionFeedback("TAIL SEQUENTIAL", nowMs);
      } else {
        setActionFeedback("TAIL DEMO", nowMs);
      }
      break;
    }

    case UiActionType::TailShowMenu:
      tailShowPage_ = 0;
      if (tailModePanel_) lv_obj_add_flag(tailModePanel_, LV_OBJ_FLAG_HIDDEN);
      if (tailShowPanel_) lv_obj_clear_flag(tailShowPanel_, LV_OBJ_FLAG_HIDDEN);
      if (pages_[2]) lv_obj_invalidate(pages_[2]);
      setActionFeedback("SHOW MENU", nowMs);
      break;

    case UiActionType::TailShowPrev:
      tailShowPage_ = (tailShowPage_ == 0)
          ? static_cast<uint8_t>(kTaillightShowPageCount - 1U)
          : static_cast<uint8_t>(tailShowPage_ - 1U);
      if (tailShowPanel_) lv_obj_invalidate(tailShowPanel_);
      setActionFeedback("SHOW PAGE", nowMs);
      break;

    case UiActionType::TailShowNext:
      tailShowPage_ = static_cast<uint8_t>((tailShowPage_ + 1U) % kTaillightShowPageCount);
      if (tailShowPanel_) lv_obj_invalidate(tailShowPanel_);
      setActionFeedback("SHOW PAGE", nowMs);
      break;

    case UiActionType::TailShowBack:
      if (tailModePanel_) lv_obj_clear_flag(tailModePanel_, LV_OBJ_FLAG_HIDDEN);
      if (tailShowPanel_) lv_obj_add_flag(tailShowPanel_, LV_OBJ_FLAG_HIDDEN);
      if (pages_[2]) lv_obj_invalidate(pages_[2]);
      setActionFeedback("SHOW MENU EXIT", nowMs);
      break;

    case UiActionType::TailShowOption: {
      const uint8_t option = action.arg0;
      static constexpr const char* kTailShowFbNames[] = {
        "Rainbow", "Chase", "Theater", "Fire", "Meteor", "Police",
        "Night Rider", "Color Cycle", "Sparkle", "Plasma", "Matrix",
        "Juggle", "BPM", "Confetti", "Ocean", "Lightning", "Heartbeat",
        "Ripple", "Sunrise", "Text Scroll", "Colorwaves", "TwinkleFox",
        "Bounce", "Fireworks", "Drip", "Cylon", "V8", "Drag Launch",
        "Neon", "Streaks", "Radar", "Aurora", "Glitch",
      };
      if (option >= kTaillightShowOptionCount) {
        setActionFeedback("SHOW SLOT EMPTY", nowMs);
      } else if (canMgr_ && canMgr_->sendTaillightShowOption(option)) {
        setActionFeedback(kTailShowFbNames[option], nowMs);
      } else {
        setActionFeedback("SHOW CMD REJECTED", nowMs);
      }
      break;
    }

    case UiActionType::LedShowMenu:
      if (ledMainPanel_) lv_obj_add_flag(ledMainPanel_, LV_OBJ_FLAG_HIDDEN);
      if (ledShowPanel_) lv_obj_clear_flag(ledShowPanel_, LV_OBJ_FLAG_HIDDEN);
      setActionFeedback("LED SHOW MENU", nowMs);
      break;

    case UiActionType::LedShowBack:
      if (ledMainPanel_) lv_obj_clear_flag(ledMainPanel_, LV_OBJ_FLAG_HIDDEN);
      if (ledShowPanel_) lv_obj_add_flag(ledShowPanel_, LV_OBJ_FLAG_HIDDEN);
      setActionFeedback("LED ZONES", nowMs);
      break;

    case UiActionType::LedShowMode: {
      const state::LedMode requestedMode = static_cast<state::LedMode>(action.value);
      const state::VehicleState before = state::g_vehicle_state.read();
      bool alreadyActive = true;
      for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
        alreadyActive = alreadyActive &&
                        before.led_zone_enabled[zone] &&
                        before.led_zone_mode[zone] == requestedMode;
      }
      const state::LedMode mode = alreadyActive ? state::LedMode::OFF : requestedMode;
      state::g_vehicle_state.mutate([mode](state::VehicleState& vs) {
        applyLedModeToAllZones(vs, mode);
      });
      queueSettingsSave(nowMs);
      if (kDisplayDiagVerbose) Serial.printf("[LED_UI] show mode=%s\n", ledModeButtonName(mode));
      setActionFeedback(ledModeButtonName(mode), nowMs);
      break;
    }

    case UiActionType::LedShowOn: {
      const state::VehicleState before = state::g_vehicle_state.read();
      state::LedMode mode = state::LedMode::RAINBOW;
      for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
        const state::LedMode candidate = before.led_zone_mode[zone];
        if (candidate == state::LedMode::STATIC_COLOR ||
            candidate == state::LedMode::RAINBOW ||
            candidate == state::LedMode::BREATHING ||
            candidate == state::LedMode::RPM_REACTIVE ||
            candidate == state::LedMode::WARNING_FLASH) {
          mode = candidate;
          break;
        }
      }
      state::g_vehicle_state.mutate([mode](state::VehicleState& vs) {
        applyLedModeToAllZones(vs, mode);
      });
      queueSettingsSave(nowMs);
      setActionFeedback("SHOW ON", nowMs);
      break;
    }

    case UiActionType::LedShowOff:
      state::g_vehicle_state.mutate([](state::VehicleState& vs) {
        applyLedModeToAllZones(vs, state::LedMode::OFF);
      });
      queueSettingsSave(nowMs);
      setActionFeedback("SHOW OFF", nowMs);
      break;

    case UiActionType::LedShowClear:
      state::g_vehicle_state.mutate([](state::VehicleState& vs) {
        for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
          vs.led_zone_color[zone] = 0xFFFFFF;
        }
        applyLedModeToAllZones(vs, state::LedMode::OFF);
      });
      queueSettingsSave(nowMs);
      setActionFeedback("SHOW CLEARED", nowMs);
      break;

    case UiActionType::LedColor: {
      const uint32_t color = action.value;
      state::g_vehicle_state.mutate([color](state::VehicleState& vs) {
        applyLedColorToAllZones(vs, color);
      });
      queueSettingsSave(nowMs);
      if (kDisplayDiagVerbose) Serial.printf("[LED_UI] color=#%06lX\n", static_cast<unsigned long>(color));
      setActionFeedback("COLOR SET", nowMs);
      break;
    }

    case UiActionType::LedMode: {
      const uint8_t row = action.arg0;
      const state::LedMode requestedMode = static_cast<state::LedMode>(action.value);
      led::LedUiMode uiMode = led::LedUiMode::Off;
      const bool isGlobalUiMode = ledModeToUiMode(requestedMode, uiMode);
      const state::VehicleState before = state::g_vehicle_state.read();
      bool alreadyActive = false;
      if (row == 0U) {
        alreadyActive = true;
        for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
          alreadyActive = alreadyActive &&
                          before.led_zone_enabled[zone] &&
                          before.led_zone_mode[zone] == requestedMode;
        }
      } else {
        const uint8_t zone = row - 1U;
        alreadyActive = zone < state::kLedZoneCount &&
                        before.led_zone_enabled[zone] &&
                        before.led_zone_mode[zone] == requestedMode;
      }
      const state::LedMode selectedMode = alreadyActive ? state::LedMode::OFF : requestedMode;

      if (row == 0U && isGlobalUiMode && selectedMode != state::LedMode::OFF) {
        state::g_vehicle_state.mutate([uiMode](state::VehicleState& vs) {
          applyLedUiModeToState(vs, uiMode);
        });
        queueSettingsSave(nowMs);
        if (kDisplayDiagVerbose) {
          Serial.printf("[LED_UI] diag mode=%s brightness=%u animations=0 show_called=queued_for_led_task\n",
                        ledUiModeName(uiMode),
                        static_cast<unsigned>(ledUiModeBrightness(uiMode)));
        }
        char msg[24];
        snprintf(msg, sizeof(msg), "LEDS %s", ledUiModeName(uiMode));
        setActionFeedback(msg, nowMs);
        break;
      }

      const bool animation = selectedMode == state::LedMode::RAINBOW ||
                             selectedMode == state::LedMode::BREATHING;
      const uint8_t brightness = brightnessForLedMode(selectedMode);
      const char* selectedName =
          selectedMode == state::LedMode::OFF ? "OFF" :
          selectedMode == state::LedMode::LOW_LIGHT ? "LOW" :
          selectedMode == state::LedMode::HIGH_LIGHT ? "HIGH" :
          selectedMode == state::LedMode::RAINBOW ? "RAINBOW" :
          selectedMode == state::LedMode::BREATHING ? "BREATHE" : "MODE";

      if (row == 0U) {
        state::g_vehicle_state.mutate([selectedMode](state::VehicleState& vs) {
          for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
            applyLedModeToZone(vs, zone, selectedMode);
          }
        });
        queueSettingsSave(nowMs);
        if (kDisplayDiagVerbose) {
          Serial.printf("[LED_UI] diag mode=%s zone=ALL brightness=%u animations=%u show_called=queued_for_led_task\n",
                        selectedName, static_cast<unsigned>(brightness), animation ? 1U : 0U);
        }
        char msg[24];
        snprintf(msg, sizeof(msg), "ALL %s", selectedName);
        setActionFeedback(msg, nowMs);
        break;
      }

      const uint8_t zone = row - 1U;
      state::g_vehicle_state.mutate([zone, selectedMode](state::VehicleState& vs) {
        applyLedModeToZone(vs, zone, selectedMode);
      });
      queueSettingsSave(nowMs);
      if (kDisplayDiagVerbose) {
        Serial.printf("[LED_UI] diag mode=%s zone=%s brightness=%u animations=%u show_called=queued_for_led_task\n",
                      selectedName, ledZoneName(row), static_cast<unsigned>(brightness), animation ? 1U : 0U);
      }
      char msg[24];
      snprintf(msg, sizeof(msg), "%s %s", ledZoneName(row), selectedName);
      setActionFeedback(msg, nowMs);
      break;
    }

    case UiActionType::LedMaster: {
      const bool enabled = action.arg0 != 0;
      const led::LedUiMode uiMode = enabled ? led::LedUiMode::HighWhite : led::LedUiMode::Off;
      state::g_vehicle_state.mutate([uiMode](state::VehicleState& vs) {
        applyLedUiModeToState(vs, uiMode);
      });
      queueSettingsSave(nowMs);
      if (kDisplayDiagVerbose) {
        Serial.printf("[LED_UI] diag mode=%s brightness=%u animations=0 show_called=queued_for_led_task\n",
                      ledUiModeName(uiMode),
                      static_cast<unsigned>(ledUiModeBrightness(uiMode)));
      }
      setActionFeedback(enabled ? "LED MASTER ON" : "LED MASTER OFF", nowMs);
      break;
    }

    case UiActionType::KnockEnable: {
      const bool en = !state::g_vehicle_state.read().knock_enabled;
      state::g_vehicle_state.mutate([en](state::VehicleState& vs) { vs.knock_enabled = en; });
      queueSettingsSave(nowMs);
      setActionFeedback(en ? "KNOCK ENABLED" : "KNOCK DISABLED", nowMs);
      break;
    }

    case UiActionType::KnockResetBaseline:
      state::g_vehicle_state.mutate([](state::VehicleState& vs) {
        vs.knock_reset_baseline_request = true;
      });
      setActionFeedback("KNOCK BL RESET", nowMs);
      break;

    case UiActionType::KnockGainDown:
      state::g_vehicle_state.mutate([](state::VehicleState& vs) {
        vs.knock_gain = (vs.knock_gain <= 0.35f) ? 0.25f : (vs.knock_gain - 0.25f);
      });
      queueSettingsSave(nowMs);
      setActionFeedback("KNOCK GAIN DOWN", nowMs);
      break;

    case UiActionType::KnockGainUp:
      state::g_vehicle_state.mutate([](state::VehicleState& vs) {
        vs.knock_gain = (vs.knock_gain >= 7.75f) ? 8.0f : (vs.knock_gain + 0.25f);
      });
      queueSettingsSave(nowMs);
      setActionFeedback("KNOCK GAIN UP", nowMs);
      break;

    case UiActionType::KnockMultiplierDown:
      state::g_vehicle_state.mutate([](state::VehicleState& vs) {
        vs.knock_threshold_multiplier =
            (vs.knock_threshold_multiplier <= 1.1f) ? 1.0f : (vs.knock_threshold_multiplier - 0.1f);
      });
      queueSettingsSave(nowMs);
      setActionFeedback("KNOCK MULT DOWN", nowMs);
      break;

    case UiActionType::KnockMultiplierUp:
      state::g_vehicle_state.mutate([](state::VehicleState& vs) {
        vs.knock_threshold_multiplier =
            (vs.knock_threshold_multiplier >= 7.9f) ? 8.0f : (vs.knock_threshold_multiplier + 0.1f);
      });
      queueSettingsSave(nowMs);
      setActionFeedback("KNOCK MULT UP", nowMs);
      break;

    case UiActionType::BenchTest: {
      const bool on = !state::g_vehicle_state.read().bench_test_mode;
      state::g_vehicle_state.mutate([on](state::VehicleState& vs) {
        vs.bench_test_mode = on;
      });
      setActionFeedback(on ? "SIM MODE ON" : "SIM MODE OFF", nowMs);
      break;
    }

    case UiActionType::DiagInfo:
      if (diagInfoPanel_) lv_obj_clear_flag(diagInfoPanel_, LV_OBJ_FLAG_HIDDEN);
      if (diagToolsPanel_) lv_obj_add_flag(diagToolsPanel_, LV_OBJ_FLAG_HIDDEN);
      if (diagInfoBtn_) setBgColor(diagInfoBtn_, lv_color_hex(kUiColorButtonActive), LV_PART_MAIN);
      if (diagToolsBtn_) setBgColor(diagToolsBtn_, lv_color_hex(kUiColorButton), LV_PART_MAIN);
      setActionFeedback("DIAG INFO", nowMs);
      break;

    case UiActionType::DiagTools:
      if (diagInfoPanel_) lv_obj_add_flag(diagInfoPanel_, LV_OBJ_FLAG_HIDDEN);
      if (diagToolsPanel_) lv_obj_clear_flag(diagToolsPanel_, LV_OBJ_FLAG_HIDDEN);
      if (diagInfoBtn_) setBgColor(diagInfoBtn_, lv_color_hex(kUiColorButton), LV_PART_MAIN);
      if (diagToolsBtn_) setBgColor(diagToolsBtn_, lv_color_hex(kUiColorButtonActive), LV_PART_MAIN);
      refreshSdBrowser(nowMs, true);
      setActionFeedback("DIAG TOOLS", nowMs);
      break;

    case UiActionType::FaultPage:
      showFaultOverlay(true);
      break;

    case UiActionType::FaultClose:
      showFaultOverlay(false);
      break;

    case UiActionType::LedOutputTest:
      runLedOutputTest(nowMs);
      break;

    case UiActionType::CanPing: {
      const state::VehicleState before = state::g_vehicle_state.read();
      const bool sent = canMgr_ && canMgr_->sendMethConfigBroadcast();
      const state::VehicleState after = state::g_vehicle_state.read();
      if (!sent) {
        setActionFeedback("CAN PING FAIL", nowMs);
      } else if (after.can_rx_count != before.can_rx_count || after.can_online) {
        setActionFeedback("CAN PING OK", nowMs);
      } else {
        setActionFeedback("CAN TX ONLY", nowMs);
      }
      break;
    }

    case UiActionType::Restart:
      setActionFeedback("RESTARTING", nowMs);
      ESP.restart();
      break;

    case UiActionType::RaceStartAccel:
      if (raceMgr_) {
        raceMgr_->startRun(state::RaceMode::ACCEL);
        setActionFeedback("RACE ACCEL", nowMs);
      } else {
        setActionFeedback("RACE OFFLINE", nowMs);
      }
      break;

    case UiActionType::RaceStartLap:
      if (raceMgr_) {
        raceMgr_->startRun(state::RaceMode::LAP);
        setActionFeedback("RACE LAP", nowMs);
      } else {
        setActionFeedback("RACE OFFLINE", nowMs);
      }
      break;

    case UiActionType::RaceStop:
      if (raceMgr_) {
        raceMgr_->stopRun();
        setActionFeedback("RACE STOP", nowMs);
      } else {
        setActionFeedback("RACE OFFLINE", nowMs);
      }
      break;

    case UiActionType::RaceReset:
      if (raceMgr_) {
        raceMgr_->resetSession();
        setActionFeedback("RACE RESET", nowMs);
      } else {
        setActionFeedback("RACE OFFLINE", nowMs);
      }
      break;

    case UiActionType::RaceSetStartFinish:
      if (raceMgr_) {
        raceMgr_->setStartFinishPointFromCurrentFix();
        setActionFeedback("RACE S/F SET", nowMs);
      } else {
        setActionFeedback("RACE OFFLINE", nowMs);
      }
      break;

    case UiActionType::RaceMarkLap:
      if (raceMgr_) {
        raceMgr_->markLap();
        setActionFeedback("RACE MARK", nowMs);
      } else {
        setActionFeedback("RACE OFFLINE", nowMs);
      }
      break;

    case UiActionType::SdFileRow: {
      if (action.arg0 >= sdEntryCount_) break;
      const storage::SdFileEntry& entry = sdEntries_[action.arg0];
      if (entry.isDirectory) {
        enterSdDirectory(entry.name, nowMs);
        break;
      }
      char msg[48];
      snprintf(msg, sizeof(msg), "SD FILE: %.36s", entry.name);
      setActionFeedback(msg, nowMs);
      break;
    }

    case UiActionType::SdUp:
      setSdPathParent();
      refreshSdBrowser(nowMs, true);
      setActionFeedback("SD UP", nowMs);
      break;

    case UiActionType::SdPrev:
      if (sdListOffset_ >= kSdFileRowCount) {
        sdListOffset_ = static_cast<uint16_t>(sdListOffset_ - kSdFileRowCount);
      } else {
        sdListOffset_ = 0;
      }
      refreshSdBrowser(nowMs, true);
      setActionFeedback("SD PREV", nowMs);
      break;

    case UiActionType::SdNext:
      if (sdListOffset_ + kSdFileRowCount < sdTotalEntries_) {
        sdListOffset_ = static_cast<uint16_t>(sdListOffset_ + kSdFileRowCount);
      }
      refreshSdBrowser(nowMs, true);
      setActionFeedback("SD NEXT", nowMs);
      break;

    case UiActionType::SdTest: {
      if (!sdMgr_ || !sdMgr_->mounted()) {
        setActionFeedback("SD NOT MOUNTED", nowMs);
        refreshSdBrowser(nowMs, true);
        break;
      }
      refreshSdBrowser(nowMs, true);
      setActionFeedback("SD READ OK", nowMs);
      break;
    }

    case UiActionType::TouchCalStart:
      showTouchCalibration(true);
      break;

    case UiActionType::TouchCalClose:
      showTouchCalibration(false);
      break;

    case UiActionType::TouchCalSample:
      recordTouchCalibrationSample(nowMs);
      break;

    case UiActionType::None:
    default:
      break;
  }
}

uint8_t ScreenDashboard::nextMethRatio(uint8_t current) const {
  // Cycle in 5 % steps: 5 → 10 → 15 → … → 100 → 5
  const uint8_t rounded = static_cast<uint8_t>((current / 5U) * 5U);
  const uint8_t next    = static_cast<uint8_t>(rounded + 5U);
  return (next > 100U) ? 5U : next;
}

void ScreenDashboard::showTouchCalibration(bool show) {
  if (!touchCalOverlay_) return;
  if (show) {
    touchCalIndex_ = 0;
    touchCalAccumX_ = 0;
    touchCalAccumY_ = 0;
    touchCalOffsetX_ = 0;
    touchCalOffsetY_ = 0;
    updateTouchCalibrationPrompt();
    lv_obj_clear_flag(touchCalOverlay_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(touchCalOverlay_);
    setActionFeedback("TOUCH CAL", millis());
  } else {
    lv_obj_add_flag(touchCalOverlay_, LV_OBJ_FLAG_HIDDEN);
    setActionFeedback("CAL CLOSED", millis());
  }
}

void ScreenDashboard::showFaultOverlay(bool show) {
  if (!faultOverlay_ || !faultLabel_) return;
  if (!show) {
    lv_obj_add_flag(faultOverlay_, LV_OBJ_FLAG_HIDDEN);
    setActionFeedback("FAULTS CLOSED", millis());
    return;
  }

  const state::VehicleState s = state::g_vehicle_state.read();
  char buf[900];
  int n = snprintf(buf, sizeof(buf),
      "#FFFFFF MASTER# flags 0x%04X\n"
      "%s%s%s%s%s%s%s\n"
      "#FFFFFF METH# flags 0x%02X  state %u  online %s\n"
      "#FFFFFF KNOCK# warn %u  crit %u  sensor %u  clip %u  pending %u code 0x%02X\n"
      "#FFFFFF ANALOG# flags 0x%04X\n"
      "IAT:%s BAY:%s CAB:%s AMB:%s OIL:%s FUEL:%s METH:%s BOOST:%s\n"
      "#FFFFFF MODULES# CAN:%s GPS:%s SD:%s TOUCH:%s IMU:%s TAIL:%s\n",
      static_cast<unsigned>(s.fault_flags),
      (s.fault_flags == 0U) ? "#00C853 NONE# " : "",
      (s.fault_flags & 0x0001U) ? "#FF3B30 TAIL# " : "",
      (s.fault_flags & 0x0010U) ? "#FF3B30 METH# " : "",
      (s.fault_flags & 0x0080U) ? "#FF9500 MODULE_OFF# " : "",
      (s.fault_flags & 0x0200U) ? "#FF9500 KNOCK_WARN# " : "",
      (s.fault_flags & 0x0400U) ? "#FF3B30 KNOCK_CRIT# " : "",
      (s.fault_flags & 0x0800U) ? "#FF9500 TEMP# " : "",
      static_cast<unsigned>(s.meth_fault_flags),
      static_cast<unsigned>(s.meth_state),
      s.meth_online ? "YES" : "NO",
      s.knock_warning_active ? 1U : 0U,
      s.knock_critical_active ? 1U : 0U,
      s.knock_sensor_fault ? 1U : 0U,
      s.knock_clipping_detected ? 1U : 0U,
      s.knock_fault_pending ? 1U : 0U,
      static_cast<unsigned>(s.knock_fault_code_pending),
      static_cast<unsigned>(s.analog_sensor_fault_flags),
      (s.analog_sensor_fault_flags & (1U << 0)) ? "BAD" : "OK",
      (s.analog_sensor_fault_flags & (1U << 1)) ? "BAD" : "OK",
      (s.analog_sensor_fault_flags & (1U << 2)) ? "BAD" : "OK",
      (s.analog_sensor_fault_flags & (1U << 3)) ? "BAD" : "OK",
      (s.analog_sensor_fault_flags & (1U << 4)) ? "BAD" : "OK",
      (s.analog_sensor_fault_flags & (1U << 5)) ? "BAD" : "OK",
      (s.analog_sensor_fault_flags & (1U << 6)) ? "BAD" : "OK",
      (s.analog_sensor_fault_flags & (1U << 7)) ? "BAD" : "OK",
      s.can_online ? "OK" : "OFF",
      s.gps_stale ? "STALE" : "OK",
      s.sd_mounted ? "OK" : "OFF",
      s.touch_online ? "OK" : "OFF",
      s.imu_online ? "OK" : "OFF",
      s.taillight_online ? "OK" : "OFF");
  if ((s.fault_flags & 0x1000U) && n > 0 && static_cast<size_t>(n) < sizeof(buf)) {
    snprintf(buf + n, sizeof(buf) - static_cast<size_t>(n), "#FF9500 PRESSURE#");
  }

  setLabelText(faultLabel_, buf);
  lv_obj_clear_flag(faultOverlay_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(faultOverlay_);
  setActionFeedback("FAULTS", millis());
}

void ScreenDashboard::runLedOutputTest(uint32_t nowMs) {
  const uint8_t step = ledOutputTestStep_;
  state::g_vehicle_state.mutate([step](state::VehicleState& vs) {
    applyLedModeToAllZones(vs, state::LedMode::OFF);
    if (step < state::kLedZoneCount) {
      applyLedModeToZone(vs, step, state::LedMode::HIGH_LIGHT);
    }
  });

  if (step < state::kLedZoneCount) {
    char msg[28];
    snprintf(msg, sizeof(msg), "LED TEST %s", ledZoneName(static_cast<uint8_t>(step + 1U)));
    setActionFeedback(msg, nowMs);
    ledOutputTestStep_ = static_cast<uint8_t>(step + 1U);
  } else {
    setActionFeedback("LED TEST OFF", nowMs);
    ledOutputTestStep_ = 0;
  }
}

void ScreenDashboard::updateTouchCalibrationPrompt() {
  if (!touchCalOverlay_ || !touchCalPromptLabel_ || !touchCalTargetLabel_) return;
  static constexpr uint16_t kTargetX[kTouchCalPointCount] = {
    96, 240, 384,
    96, 240, 384,
    96, 240, 384
  };
  static constexpr uint16_t kTargetY[kTouchCalPointCount] = {
    76, 76, 76,
    160, 160, 160,
    244, 244, 244
  };

  if (touchCalIndex_ >= kTouchCalPointCount) {
    setLabelText(touchCalPromptLabel_, "Calibration complete");
    return;
  }

  char prompt[64];
  snprintf(prompt, sizeof(prompt), "Tap target %u/%u", static_cast<unsigned>(touchCalIndex_ + 1U),
           static_cast<unsigned>(kTouchCalPointCount));
  setLabelText(touchCalPromptLabel_, prompt);
  lv_obj_set_pos(touchCalTargetLabel_,
                 static_cast<lv_coord_t>(kTargetX[touchCalIndex_] - 24U),
                 static_cast<lv_coord_t>(kTargetY[touchCalIndex_] - 32U));
}

void ScreenDashboard::recordTouchCalibrationSample(uint32_t nowMs) {
  if (touchCalIndex_ >= kTouchCalPointCount) return;

  static constexpr uint16_t kTargetX[kTouchCalPointCount] = {
    96, 240, 384,
    96, 240, 384,
    96, 240, 384
  };
  static constexpr uint16_t kTargetY[kTouchCalPointCount] = {
    76, 76, 76,
    160, 160, 160,
    244, 244, 244
  };

  lv_point_t p{};
  lv_indev_t* indev = lv_indev_get_act();
  if (indev) {
    lv_indev_get_point(indev, &p);
  } else {
    portENTER_CRITICAL(&touchMux_);
    p.x = static_cast<lv_coord_t>(filteredTouch_.x);
    p.y = static_cast<lv_coord_t>(filteredTouch_.y);
    portEXIT_CRITICAL(&touchMux_);
  }

  touchCalAccumX_ = static_cast<int16_t>(touchCalAccumX_ + static_cast<int16_t>(kTargetX[touchCalIndex_]) - p.x);
  touchCalAccumY_ = static_cast<int16_t>(touchCalAccumY_ + static_cast<int16_t>(kTargetY[touchCalIndex_]) - p.y);
  ++touchCalIndex_;

  if (touchCalIndex_ >= kTouchCalPointCount) {
    touchCalOffsetX_ = static_cast<int16_t>(touchCalAccumX_ / static_cast<int16_t>(kTouchCalPointCount));
    touchCalOffsetY_ = static_cast<int16_t>(touchCalAccumY_ / static_cast<int16_t>(kTouchCalPointCount));
    showTouchCalibration(false);
    setActionFeedback("CAL SAVED", nowMs);
    return;
  }

  updateTouchCalibrationPrompt();
  setActionFeedback("CAL POINT OK", nowMs);
}

touch::TouchSample ScreenDashboard::normalizeRaw(const touch::TouchSample& raw) const {
  if (!raw.touched) return raw;
  touch::TouchSample t = raw;
  auto applyCalibration = [this](touch::TouchSample& sample) {
    int32_t x = static_cast<int32_t>(sample.x) + touchCalOffsetX_;
    int32_t y = static_cast<int32_t>(sample.y) + touchCalOffsetY_;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= kWidth) x = kWidth - 1;
    if (y >= kHeight) y = kHeight - 1;
    sample.x = static_cast<uint16_t>(x);
    sample.y = static_cast<uint16_t>(y);
  };

  // This panel's FT touch controller reports 320x480 portrait coordinates while
  // the display renders 480x320 landscape.
  if (t.x <= kHeight && t.y <= kWidth) {
    const uint16_t x = t.x;
    t.x = t.y;
    t.y = static_cast<uint16_t>(kHeight > x ? (kHeight - x) : 0U);
    applyCalibration(t);
    return t;
  }

  if (t.x <= kWidth && t.y <= kHeight) {
    applyCalibration(t);
    return t;
  }

  // Fallback: scale raw 12-bit (0..4095) ranges down to pixel coordinates.
  if (t.x > kWidth)  t.x = static_cast<uint16_t>((static_cast<uint32_t>(t.x) * kWidth)  / 4095U);
  if (t.y > kHeight) t.y = static_cast<uint16_t>((static_cast<uint32_t>(t.y) * kHeight) / 4095U);
  applyCalibration(t);
  return t;
}

void ScreenDashboard::setSdPathRoot() {
  snprintf(sdCurrentPath_, sizeof(sdCurrentPath_), "/");
  sdListOffset_ = 0;
  sdBrowserLastRefreshMs_ = 0;
}

void ScreenDashboard::setSdPathParent() {
  if (strcmp(sdCurrentPath_, "/") == 0) {
    return;
  }

  char* slash = strrchr(sdCurrentPath_, '/');
  if (!slash || slash == sdCurrentPath_) {
    setSdPathRoot();
    return;
  }

  *slash = '\0';
  sdListOffset_ = 0;
  sdBrowserLastRefreshMs_ = 0;
}

bool ScreenDashboard::enterSdDirectory(const char* name, uint32_t nowMs) {
  if (!name || !name[0]) {
    return false;
  }

  char next[sizeof(sdCurrentPath_)];
  const int written = (strcmp(sdCurrentPath_, "/") == 0)
      ? snprintf(next, sizeof(next), "/%s", name)
      : snprintf(next, sizeof(next), "%s/%s", sdCurrentPath_, name);
  if (written < 0 || static_cast<size_t>(written) >= sizeof(next)) {
    setActionFeedback("SD PATH TOO LONG", nowMs);
    return false;
  }

  snprintf(sdCurrentPath_, sizeof(sdCurrentPath_), "%s", next);
  sdListOffset_ = 0;
  refreshSdBrowser(nowMs, true);
  return true;
}

void ScreenDashboard::refreshSdBrowser(uint32_t nowMs, bool force) {
  if (!sdPathLabel_) {
    return;
  }
  if (!force && kSdBrowserAutoRefreshMs == 0U) {
    return;
  }
  if (!force && sdBrowserLastRefreshMs_ != 0 &&
      static_cast<uint32_t>(nowMs - sdBrowserLastRefreshMs_) < kSdBrowserAutoRefreshMs) {
    return;
  }
  sdBrowserLastRefreshMs_ = nowMs;

  auto clearRows = [this](const char* text) {
    sdEntryCount_ = 0;
    sdTotalEntries_ = 0;
    for (uint8_t i = 0; i < kSdFileRowCount; ++i) {
      sdEntries_[i] = {};
      if (sdFileRowLabels_[i]) {
        setLabelText(sdFileRowLabels_[i], text ? text : "--");
      }
      if (sdFileRows_[i]) {
        setBgColor(sdFileRows_[i], lv_color_hex(kUiColorRow), LV_PART_MAIN);
      }
    }
  };

  if (!sdMgr_ || !sdMgr_->mounted()) {
    char status[64];
    snprintf(status, sizeof(status), "SD: %s",
             sdMgr_ ? sdMgr_->lastStatus() : "not_ready");
    setLabelText(sdPathLabel_, status);
    clearRows("--");
    return;
  }

  size_t total = 0;
  bool ok = sdMgr_->listDirectory(sdCurrentPath_, sdEntries_, kSdFileRowCount,
                                  sdListOffset_, total);
  if (ok && total > 0 && sdListOffset_ >= total) {
    sdListOffset_ = static_cast<uint16_t>(((total - 1U) / kSdFileRowCount) * kSdFileRowCount);
    ok = sdMgr_->listDirectory(sdCurrentPath_, sdEntries_, kSdFileRowCount,
                               sdListOffset_, total);
  }

  if (!ok) {
    setLabelText(sdPathLabel_, "SD: open failed");
    clearRows("--");
    return;
  }

  sdTotalEntries_ = (total > 65535U) ? static_cast<uint16_t>(65535U) : static_cast<uint16_t>(total);
  sdEntryCount_ = 0;
  for (uint8_t i = 0; i < kSdFileRowCount; ++i) {
    if (sdEntries_[i].name[0]) {
      ++sdEntryCount_;
    }
  }

  const uint16_t first = (sdTotalEntries_ == 0) ? 0 : static_cast<uint16_t>(sdListOffset_ + 1U);
  const uint16_t last = static_cast<uint16_t>(sdListOffset_ + sdEntryCount_);
  char pathBuf[96];
  snprintf(pathBuf, sizeof(pathBuf), "SD %s  %u-%u/%u",
           sdCurrentPath_,
           static_cast<unsigned>(first),
           static_cast<unsigned>(last),
           static_cast<unsigned>(sdTotalEntries_));
  setLabelText(sdPathLabel_, pathBuf);

  for (uint8_t i = 0; i < kSdFileRowCount; ++i) {
    if (!sdFileRowLabels_[i]) {
      continue;
    }

    if (i >= sdEntryCount_) {
      setLabelText(sdFileRowLabels_[i], "--");
      if (sdFileRows_[i]) {
        setBgColor(sdFileRows_[i], lv_color_hex(kUiColorRow), LV_PART_MAIN);
      }
      continue;
    }

    char line[72];
    if (sdEntries_[i].isDirectory) {
      snprintf(line, sizeof(line), "[DIR] %s", sdEntries_[i].name);
      if (sdFileRows_[i]) {
        setBgColor(sdFileRows_[i], lv_color_hex(kUiColorRowSelected), LV_PART_MAIN);
      }
    } else {
      char sizeBuf[12];
      formatFileSize(sdEntries_[i].sizeBytes, sizeBuf, sizeof(sizeBuf));
      snprintf(line, sizeof(line), "[FILE] %-36s %s", sdEntries_[i].name, sizeBuf);
      if (sdFileRows_[i]) {
        setBgColor(sdFileRows_[i], lv_color_hex(kUiColorRow), LV_PART_MAIN);
      }
    }
    setLabelText(sdFileRowLabels_[i], line);
  }
}

// ---------------------------------------------------------------------------
// Button event handlers
// ---------------------------------------------------------------------------

void ScreenDashboard::onMethArmClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  const bool arm = !state::g_vehicle_state.read().meth_desired_armed;
  self->setActionFeedback(arm ? "METH ON" : "METH OFF", now);
  self->enqueueAction({UiActionType::MethArm, static_cast<uint8_t>(arm ? 1U : 0U), 0, 0}, now);
}

void ScreenDashboard::onMethRatioClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  const uint8_t ratio = self->nextMethRatio(state::g_vehicle_state.read().meth_selected_ratio_percent);
  self->setActionFeedback("METH RATIO UPDATED", now);
  self->enqueueAction({UiActionType::MethRatio, ratio, 0, 0}, now);
}

void ScreenDashboard::onTailStockClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("TAIL STOCK", now);
  self->enqueueAction({UiActionType::TailMode, can_protocol::taillight_mode::STOCK, 0, 0}, now);
}

void ScreenDashboard::onTailSeqClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("TAIL SEQUENTIAL", now);
  self->enqueueAction({UiActionType::TailMode, can_protocol::taillight_mode::SEQUENTIAL, 0, 0}, now);
}

void ScreenDashboard::onTailDemoClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("TAIL DEMO", now);
  self->enqueueAction({UiActionType::TailMode, can_protocol::taillight_mode::DEMO, 0, 0}, now);
}

void ScreenDashboard::onTailShowMenuClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SHOW MENU", now);
  self->enqueueAction({UiActionType::TailShowMenu, 0, 0, 0}, now);
}

void ScreenDashboard::onTailShowPrevClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SHOW PAGE", now);
  self->enqueueAction({UiActionType::TailShowPrev, 0, 0, 0}, now);
}

void ScreenDashboard::onTailShowNextClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SHOW PAGE", now);
  self->enqueueAction({UiActionType::TailShowNext, 0, 0, 0}, now);
}

void ScreenDashboard::onTailShowBackClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SHOW MENU EXIT", now);
  self->enqueueAction({UiActionType::TailShowBack, 0, 0, 0}, now);
}

void ScreenDashboard::onTailShowOptClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  lv_obj_t* btn = lv_event_get_target(e);

  uint8_t idx = kTaillightShowOptionsPerPage;
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    if (self->tailShowOptBtns_[i] == btn) { idx = i; break; }
  }
  if (idx >= kTaillightShowOptionsPerPage) return;

  const uint16_t optVal = static_cast<uint16_t>(self->tailShowPage_) * kTaillightShowOptionsPerPage + idx;
  if (optVal >= kTaillightShowOptionCount) {
    self->setActionFeedback("SHOW SLOT EMPTY", now);
    return;
  }
  self->setActionFeedback("SHOW OPTION", now);
  self->enqueueAction({UiActionType::TailShowOption, static_cast<uint8_t>(optVal), 0, 0}, now);
}

void ScreenDashboard::onNavClicked(lv_event_t* e) {
  auto* ctx = static_cast<NavCtx*>(lv_event_get_user_data(e));
  if (!ctx || !ctx->self) return;
  const uint32_t now = millis();
  if (!ctx->self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  ctx->self->enqueueAction({UiActionType::Nav, ctx->page, 0, 0}, now);
}

void ScreenDashboard::onLedShowMenuClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("LED SHOW MENU", now);
  self->enqueueAction({UiActionType::LedShowMenu, 0, 0, 0}, now);
}

void ScreenDashboard::onLedShowBackClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("LED ZONES", now);
  self->enqueueAction({UiActionType::LedShowBack, 0, 0, 0}, now);
}

void ScreenDashboard::onLedShowModeClicked(lv_event_t* e) {
  auto* ctx = static_cast<LedShowCtx*>(lv_event_get_user_data(e));
  if (!ctx || !ctx->self) return;
  const uint32_t now = millis();
  if (!ctx->self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  ctx->self->setActionFeedback("LED SHOW", now);
  ctx->self->enqueueAction({UiActionType::LedShowMode, 0, 0, static_cast<uint32_t>(ctx->mode)}, now);
}

void ScreenDashboard::onLedShowOnClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SHOW ON", now);
  self->enqueueAction({UiActionType::LedShowOn, 0, 0, 0}, now);
}

void ScreenDashboard::onLedShowOffClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SHOW OFF", now);
  self->enqueueAction({UiActionType::LedShowOff, 0, 0, 0}, now);
}

void ScreenDashboard::onLedShowClearClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->confirmOrEnqueue({UiActionType::LedShowClear, 0, 0, 0}, "TAP AGAIN CLEAR", now);
}

void ScreenDashboard::onLedColorClicked(lv_event_t* e) {
  auto* ctx = static_cast<LedColorCtx*>(lv_event_get_user_data(e));
  if (!ctx || !ctx->self) return;
  const uint32_t now = millis();
  if (!ctx->self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  ctx->self->setActionFeedback("COLOR SET", now);
  ctx->self->enqueueAction({UiActionType::LedColor, 0, 0, ctx->color}, now);
}

void ScreenDashboard::onLedModeClicked(lv_event_t* e) {
  auto* ctx = static_cast<LedModeCtx*>(lv_event_get_user_data(e));
  if (!ctx || !ctx->self) return;
  const uint32_t now = millis();
  if (!ctx->self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  ctx->self->setActionFeedback("LED MODE", now);
  ctx->self->enqueueAction({UiActionType::LedMode, ctx->channel, 0, static_cast<uint32_t>(ctx->mode)}, now);
}

void ScreenDashboard::onLedMasterSwitchChanged(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  if (self->suppressLedMasterEvent_) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  lv_obj_t* sw = lv_event_get_target(e);
  const bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
  self->setActionFeedback(enabled ? "LED MASTER ON" : "LED MASTER OFF", now);
  self->enqueueAction({UiActionType::LedMaster, static_cast<uint8_t>(enabled ? 1U : 0U), 0, 0}, now);
}

void ScreenDashboard::onKnockEnableClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("KNOCK ENABLE", now);
  self->enqueueAction({UiActionType::KnockEnable, 0, 0, 0}, now);
}

void ScreenDashboard::onKnockResetBaselineClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->confirmOrEnqueue({UiActionType::KnockResetBaseline, 0, 0, 0}, "TAP AGAIN RESET", now);
}

void ScreenDashboard::onKnockGainDownClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("KNOCK GAIN DOWN", now);
  self->enqueueAction({UiActionType::KnockGainDown, 0, 0, 0}, now);
}

void ScreenDashboard::onKnockGainUpClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("KNOCK GAIN UP", now);
  self->enqueueAction({UiActionType::KnockGainUp, 0, 0, 0}, now);
}

void ScreenDashboard::onKnockMultiplierDownClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("KNOCK MULT DOWN", now);
  self->enqueueAction({UiActionType::KnockMultiplierDown, 0, 0, 0}, now);
}

void ScreenDashboard::onKnockMultiplierUpClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("KNOCK MULT UP", now);
  self->enqueueAction({UiActionType::KnockMultiplierUp, 0, 0, 0}, now);
}

void ScreenDashboard::onBenchTestClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->confirmOrEnqueue({UiActionType::BenchTest, 0, 0, 0}, "TAP AGAIN SIM", now);
}

void ScreenDashboard::onDiagInfoClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::DiagInfo, 0, 0, 0}, now);
}

void ScreenDashboard::onDiagToolsClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::DiagTools, 0, 0, 0}, now);
}

void ScreenDashboard::onFaultPageClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::FaultPage, 0, 0, 0}, now);
}

void ScreenDashboard::onFaultCloseClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::FaultClose, 0, 0, 0}, now);
}

void ScreenDashboard::onLedOutputTestClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::LedOutputTest, 0, 0, 0}, now);
}

void ScreenDashboard::onCanPingClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("CAN PING", now);
  self->enqueueAction({UiActionType::CanPing, 0, 0, 0}, now);
}

void ScreenDashboard::onRestartClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->confirmOrEnqueue({UiActionType::Restart, 0, 0, 0}, "TAP AGAIN RESTART", now);
}

void ScreenDashboard::onRaceAccelClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::RaceStartAccel, 0, 0, 0}, now);
}

void ScreenDashboard::onRaceLapClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::RaceStartLap, 0, 0, 0}, now);
}

void ScreenDashboard::onRaceStopClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::RaceStop, 0, 0, 0}, now);
}

void ScreenDashboard::onRaceResetClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->confirmOrEnqueue({UiActionType::RaceReset, 0, 0, 0}, "TAP AGAIN RST", now);
}

void ScreenDashboard::onRaceSetStartClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->confirmOrEnqueue({UiActionType::RaceSetStartFinish, 0, 0, 0}, "TAP AGAIN S/F", now);
}

void ScreenDashboard::onRaceMarkLapClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::RaceMarkLap, 0, 0, 0}, now);
}

void ScreenDashboard::onSdFileRowClicked(lv_event_t* e) {
  auto* ctx = static_cast<SdFileRowCtx*>(lv_event_get_user_data(e));
  if (!ctx || !ctx->self || ctx->row >= ctx->self->sdEntryCount_) {
    return;
  }

  ScreenDashboard* self = ctx->self;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SD OPEN", now);
  self->enqueueAction({UiActionType::SdFileRow, ctx->row, 0, 0}, now);
}

void ScreenDashboard::onSdUpClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) {
    return;
  }
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SD UP", now);
  self->enqueueAction({UiActionType::SdUp, 0, 0, 0}, now);
}

void ScreenDashboard::onSdPrevClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) {
    return;
  }
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SD PREV", now);
  self->enqueueAction({UiActionType::SdPrev, 0, 0, 0}, now);
}

void ScreenDashboard::onSdNextClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) {
    return;
  }
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SD NEXT", now);
  self->enqueueAction({UiActionType::SdNext, 0, 0, 0}, now);
}

void ScreenDashboard::onSdTestClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) {
    return;
  }

  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->setActionFeedback("SD READ", now);
  self->enqueueAction({UiActionType::SdTest, 0, 0, 0}, now);
}

void ScreenDashboard::onTouchCalStartClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  lv_obj_t* target = lv_event_get_target(e);
  if (!self->shouldAcceptUiTap(target, now)) return;

  if (target == self->touchCalOverlay_) {
    self->enqueueAction({UiActionType::TouchCalSample, 0, 0, 0}, now);
  } else {
    self->enqueueAction({UiActionType::TouchCalStart, 0, 0, 0}, now);
  }
}

void ScreenDashboard::onTouchCalCloseClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  const uint32_t now = millis();
  if (!self->shouldAcceptUiTap(lv_event_get_target(e), now)) return;
  self->enqueueAction({UiActionType::TouchCalClose, 0, 0, 0}, now);
}

}  // namespace ui
