#include "ui/screen_dashboard.h"

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

#ifndef CCM_DISPLAY_DIAG_VERBOSE
#define CCM_DISPLAY_DIAG_VERBOSE 0
#endif

#ifndef CCM_DISPLAY_DIAG_FLUSH_SAMPLE
#define CCM_DISPLAY_DIAG_FLUSH_SAMPLE 32
#endif

#ifndef CCM_DISPLAY_FULL_REPAINT_ON_PAGE_SWITCH
#define CCM_DISPLAY_FULL_REPAINT_ON_PAGE_SWITCH 1
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
struct LedModeCtx { ScreenDashboard* self; state::LedMode mode; };
struct SdFileRowCtx { ScreenDashboard* self; uint8_t row; };

static NavCtx     s_navCtxs[8];
static LedModeCtx s_ledModeCtxs[5];
static SdFileRowCtx s_sdFileRowCtxs[5];

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
constexpr float kGpsLowSpeedThresholdMph = 12.0f;
constexpr float kGpsLowSpeedFilterAlpha  = 0.18f;
constexpr float kGpsHighSpeedFilterAlpha = 0.35f;
constexpr float kGpsZeroClampMph         = 1.0f;
constexpr uint32_t kMethSensorStaleMs    = 1000;

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
    Serial0.printf("[SCREEN] DMA buffer alloc FAILED name=%s bytes=%lu heap=%lu dma_free=%lu dma_big=%lu\n",
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

  Serial0.printf("[SCREEN] buffers draw=%lux%u rows (%luB each) flush=%u rows (%luB x2) dma_free=%lu dma_big=%lu\n",
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

  Serial0.println("[SCREEN] boot diagnostic start");
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
  Serial0.println("[SCREEN] boot diagnostic OK");
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

void setArcColor(lv_obj_t* obj, lv_color_t color, uint32_t part) {
  if (!obj) return;
  if ((part & LV_STATE_ANY) == 0 &&
      lv_color_to32(lv_obj_get_style_arc_color(obj, part)) == lv_color_to32(color)) {
    return;
  }
  lv_obj_set_style_arc_color(obj, color, part);
}

void setArcValue(lv_obj_t* obj, int32_t& cached, int32_t value) {
  if (!obj || cached == value) return;
  cached = value;
  lv_arc_set_value(obj, static_cast<int16_t>(value));
}

void setBarValue(lv_obj_t* obj, int32_t& cached, int32_t value) {
  if (!obj || cached == value) return;
  cached = value;
  lv_bar_set_value(obj, value, LV_ANIM_OFF);
}

void setMeterValue(lv_obj_t* obj, lv_meter_indicator_t* indicator,
                   int32_t& cached, int32_t value) {
  if (!obj || !indicator || cached == value) return;
  cached = value;
  lv_meter_set_indicator_value(obj, indicator, value);
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
  setBgColor(obj, lv_color_hex(0x1a2538), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(obj, lv_color_hex(0x2b4b6c), LV_PART_MAIN);
  lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN);
  lv_obj_set_style_pad_left(obj, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_right(obj, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_top(obj, 3, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(obj, 3, LV_PART_MAIN);
}

void styleActionButton(lv_obj_t* obj) {
  if (!obj) return;
  lv_obj_set_style_radius(obj, 10, LV_PART_MAIN);
  lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(obj, lv_color_hex(0x35557a), LV_PART_MAIN);
  lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_shadow_color(obj, lv_color_hex(0x050c16), LV_PART_MAIN);
}

void styleOpaqueRedrawSurface(lv_obj_t* obj, lv_color_t color = lv_color_hex(0x0f1724)) {
  if (!obj) return;
  setBgColor(obj, color, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
  lv_obj_set_style_outline_width(obj, 0, LV_PART_MAIN);
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

        hal::SharedSpiBusLock spiLock("LCD:flush");
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
        hal::SharedSpiBusLock spiLock("LCD:flush");
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
        hal::SharedSpiBusLock spiLock("LCD:flush");
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
    Serial0.printf("[SCREEN:FLUSH] #%lu req=(%ld,%ld)-(%ld,%ld) px=%lu bytes=%lu us=%lu ok=%u\n",
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

  Serial0.printf(
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
  const touch::TouchSample t = self->lastTouch_;
  portEXIT_CRITICAL(&self->touchMux_);
  if (t.touched) {
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
  Serial0.printf("[SCREEN] backend=%s spi=%luHz dma=%u chunk_rows=%u\n",
                 kDisplayBackendName,
                 static_cast<unsigned long>(kDisplaySpiHz),
                 static_cast<unsigned>(kDisplayUseLgfxDma ? 1 : 0),
                 static_cast<unsigned>(kLovyanFlushChunkRows));
  Serial0.println("[SCREEN] creating LovyanGFX ILI9488 driver");
  s_lgfx = new (std::nothrow) CabinLgfxDisplay(lcdCs, lcdRst, lcdDc, spiSck, spiMosi, spiMiso);
  if (!s_lgfx) {
    Serial0.println("[SCREEN] LovyanGFX allocation FAILED");
    return false;
  }

  {
    hal::SharedSpiBusLock spiLock("LCD:init");
    if (!s_lgfx->init()) {
      Serial0.println("[SCREEN] LovyanGFX init FAILED");
      return false;
    }
    s_lgfx->setRotation(1);
    s_lgfx->setColorDepth(16);
    s_lgfx->setSwapBytes(false);
    s_lgfx->startWrite();
    s_lgfx->writeCommand(kIli9488Invctr);
    s_lgfx->writeData(kIli9488InvctrValue);
    s_lgfx->endWrite();
    Serial0.printf("[SCREEN] LovyanGFX init OK size=%dx%d depth=%u\n",
                   s_lgfx->width(),
                   s_lgfx->height(),
                   static_cast<unsigned>(static_cast<int>(s_lgfx->getColorDepth()) &
                                         static_cast<int>(lgfx::color_depth_t::bit_mask)));
    Serial0.printf("[SCREEN] ILI9488 INVCTR=0x%02X\n",
                   static_cast<unsigned>(kIli9488InvctrValue));
    runDisplayDiagnostic();
    Serial0.println("[SCREEN] clear start");
    s_lgfx->fillScreen(0x000000U);
    Serial0.println("[SCREEN] clear OK");
  }
#elif CCM_HAS_ARDUINO_GFX
#if CCM_DISPLAY_USE_GFX_DMA
  // Dedicated ESP-IDF DMA bus path. Only enable this when LCD is allowed to own
  // the SPI host; the normal build keeps shared Arduino SPI for LCD/CAN/SD.
  Serial0.println("[SCREEN] creating ESP32SPIDMA bus");
  s_bus = new (std::nothrow) Arduino_ESP32SPIDMA(lcdDc, lcdCs, spiSck, spiMosi, spiMiso, FSPI, true);
  if (!s_bus) {
    Serial0.println("[SCREEN] ESP32SPIDMA allocation FAILED");
    return false;
  }
#else
  // CAN, SD, and LCD share the same physical SPI pins. Use Arduino's shared SPI
  // peripheral here; Arduino_ESP32SPI can wedge during ILI9488 init on Arduino
  // ESP32 3.x / IDF 5 with this ESP32-S3 setup.
  Serial0.println("[SCREEN] creating HWSPI bus");
  s_bus = new (std::nothrow) Arduino_HWSPI(lcdDc, lcdCs, spiSck, spiMosi, spiMiso, &SPI, true);
  if (!s_bus) {
    Serial0.println("[SCREEN] HWSPI allocation FAILED");
    return false;
  }
#endif
  // rotation=1: 90° CW landscape (480×320).
#if CCM_DISPLAY_PIXEL_MODE == 16
  Serial0.println("[SCREEN] creating ILI9488 16-bit driver");
  s_gfx = new Arduino_ILI9488(s_bus, lcdRst, 1 /*rotation 90°CW*/, false /*ips*/);
#else
  Serial0.println("[SCREEN] creating ILI9488 18-bit driver");
  s_gfx = new Arduino_ILI9488_18bit(s_bus, lcdRst, 1 /*rotation 90°CW*/, false /*ips*/);
#endif
  if (!s_gfx) {
    Serial0.println("[SCREEN] ILI9488 allocation FAILED");
    return false;
  }
  Serial0.printf("[SCREEN] GFX begin @ %lu Hz bus=%s pixel=%ubit\n",
                 static_cast<unsigned long>(kDisplaySpiHz),
                 kDisplayUseGfxDma ? "ESP32SPIDMA" : "HWSPI",
                 static_cast<unsigned>(kDisplayPixelMode));
  if (!s_gfx || !s_gfx->begin(kDisplaySpiHz)) {
    Serial0.println("[SCREEN] GFX begin FAILED");
    return false;
  }
  Serial0.println("[SCREEN] GFX begin OK");
  Serial0.println("[SCREEN] clear start");
  s_gfx->fillScreen(0x0000U);  // clear to black before LVGL builds first frame
  Serial0.println("[SCREEN] clear OK");
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
    Serial0.println("[SCREEN] LVGL display registration FAILED");
    return false;
  }

  // Touch input device
  static lv_indev_drv_t indevDrv;
  lv_indev_drv_init(&indevDrv);
  indevDrv.type      = LV_INDEV_TYPE_POINTER;
  indevDrv.read_cb   = lvglTouchReadCb;
  indevDrv.user_data = this;
  if (!lv_indev_drv_register(&indevDrv)) {
    Serial0.println("[SCREEN] LVGL touch registration FAILED");
    return false;
  }

  // Dark theme
  lv_theme_t* theme = lv_theme_default_init(
      disp,
      lv_palette_main(LV_PALETTE_BLUE),
      lv_palette_main(LV_PALETTE_CYAN),
      true  /* dark mode */,
      &lv_font_montserrat_16);
  if (!theme) {
    Serial0.println("[SCREEN] LVGL theme allocation FAILED");
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
    lv_obj_invalidate(lv_scr_act());
    Serial0.println("[SCREEN] first frame flush start");
    {
      hal::SharedSpiBusLock spiLock("LCD:frame");
      lv_refr_now(disp);
    }
    Serial0.println("[SCREEN] first frame flush OK");
    lastHeaderUpdateMs_ = nowMs;
    lastPageUpdateMs_ = nowMs;
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
    Serial0.printf("[SCREEN:STRESS] auto page %u->%u period=%lums flush_seq=%lu\n",
                   static_cast<unsigned>(activePage_),
                   static_cast<unsigned>(nextPage),
                   static_cast<unsigned long>(kPageStressPeriodMs),
                   static_cast<unsigned long>(s_flushSeq));
    showPage(nextPage);
  }

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
  {
    hal::SharedSpiBusLock spiLock("LCD:frame");
    lv_task_handler();
  }
  const uint32_t handlerEndMs = millis();
  s_frameFlushLast = s_frameFlushCurrent;
  s_frameFlushSum += s_frameFlushCurrent;
  if (s_frameFlushCurrent > s_frameFlushMax) {
    s_frameFlushMax = s_frameFlushCurrent;
  }
  ++s_frameCount;
  logScreenStats(handlerEndMs, handlerEndMs - handlerStartMs, s.ui_fps);
}

void ScreenDashboard::handleTouch(const touch::TouchSample& sample, uint32_t /*nowMs*/) {
  // Normalize raw coordinates and buffer for the LVGL indev driver.
  const touch::TouchSample normalized = normalizeRaw(sample);
  portENTER_CRITICAL(&touchMux_);
  lastTouch_ = normalized;
  portEXIT_CRITICAL(&touchMux_);

  // Mark touch event in shared vehicle state so CAN telemetry can observe it.
  if (lastTouch_.touched) {
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
  lv_obj_t* lbl = lv_label_create(btn);
  setLabelText(lbl, text);
  lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
  lv_obj_set_width(lbl, (w > 12) ? static_cast<lv_coord_t>(w - 12) : w);
  lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_center(lbl);
  if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, userData);
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

  buildHeader(scr);
  buildContentArea(scr);
  buildNavBar(scr);

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
  setBgColor(hdr, lv_color_hex(0x0d1520), LV_PART_MAIN);
  lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

  // Left: battery voltage
  hdrBatLabel_ = lv_label_create(hdr);
  setLabelTextStatic(hdrBatLabel_, hdrBatText_, sizeof(hdrBatText_), "12.0V");
  lv_obj_set_style_text_font(hdrBatLabel_, &lv_font_montserrat_16, 0);
  lv_obj_align(hdrBatLabel_, LV_ALIGN_LEFT_MID, 8, 0);

  // Center: active page name (cyan accent)
  hdrTitleLabel_ = lv_label_create(hdr);
  setLabelText(hdrTitleLabel_, "DASH");
  lv_obj_set_style_text_font(hdrTitleLabel_, &lv_font_montserrat_18, 0);
  setTextColor(hdrTitleLabel_, lv_palette_main(LV_PALETTE_CYAN), 0);
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

// ---------------------------------------------------------------------------
// buildContentArea – creates 7 page panels at (0, kHdrH), size 320 × kContentH
// ---------------------------------------------------------------------------

void ScreenDashboard::buildContentArea(lv_obj_t* scr) {
  // Container for all page panels (transparent pass-through)
  lv_obj_t* cont = lv_obj_create(scr);
  contentArea_ = cont;
  lv_obj_set_pos(cont, 0, kHdrH);
  lv_obj_set_size(cont, kWidth, kContentH);
  setBgColor(cont, lv_color_hex(0x0f1724), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(cont, 0, LV_PART_MAIN);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  for (uint8_t i = 0; i < kPageCount; i++) {
    lv_obj_t* pg = lv_obj_create(cont);
    lv_obj_set_pos(pg, 0, 0);
    lv_obj_set_size(pg, kWidth, kContentH);
    setBgColor(pg, lv_color_hex(0x0f1724), LV_PART_MAIN);
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

  lv_obj_t* bar = lv_obj_create(scr);
  lv_obj_set_pos(bar, 0, static_cast<lv_coord_t>(kHdrH + kContentH));
  lv_obj_set_size(bar, kWidth, kNavH);
  setBgColor(bar, lv_color_hex(0x0d1520), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(bar, 0, LV_PART_MAIN);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  // Divider line between content and nav bar
  lv_obj_t* sep = lv_obj_create(scr);
  lv_obj_set_pos(sep, 0, static_cast<lv_coord_t>(kHdrH + kContentH));
  lv_obj_set_size(sep, kWidth, 1);
  setBgColor(sep, lv_color_hex(0x2a3a50), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, LV_PART_MAIN);
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
    setNavButtonBg(btn, lv_color_hex(0x1a2540));
    lv_obj_add_event_cb(btn, onNavClicked, LV_EVENT_CLICKED, &s_navCtxs[i]);
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
  setBgColor(bottomEdgeGuard_, lv_color_hex(0x0d1520), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bottomEdgeGuard_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(bottomEdgeGuard_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(bottomEdgeGuard_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(bottomEdgeGuard_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(bottomEdgeGuard_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_move_foreground(bottomEdgeGuard_);
}

// ---------------------------------------------------------------------------
// showPage – hide all pages, reveal the requested one, highlight nav button
// ---------------------------------------------------------------------------

void ScreenDashboard::showPage(uint8_t idx) {
  if (idx >= kPageCount) return;
  static const char* const kNames[8] = {
    "DASH", "METH", "TAIL", "LEDS", "GPS", "TEMPS", "DIAG", "KNOCK"
  };

  if (idx == activePage_ && pages_[idx] && !lv_obj_has_flag(pages_[idx], LV_OBJ_FLAG_HIDDEN)) {
    return;
  }

  const uint8_t oldPage = activePage_;
  Serial0.printf("[SCREEN:PAGE] %u->%u heap=%lu max_block=%lu dma_free=%lu dma_big=%lu lv_free=%lu\n",
                 static_cast<unsigned>(oldPage),
                 static_cast<unsigned>(idx),
                 static_cast<unsigned long>(ESP.getFreeHeap()),
                 static_cast<unsigned long>(ESP.getMaxAllocHeap()),
                 static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_DMA)),
                 static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)),
                 static_cast<unsigned long>(lvglFreeBytes()));

  for (uint8_t i = 0; i < kPageCount; i++) {
    if (i == idx) {
      setObjHidden(pages_[i], false);
      lv_obj_scroll_to_y(pages_[i], 0, LV_ANIM_OFF);
      lv_obj_move_foreground(pages_[i]);
      animatePageEnter(pages_[i]);
      if (navBtns_[i]) {
        lv_obj_clear_state(navBtns_[i], LV_STATE_PRESSED | LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
        setNavButtonBg(navBtns_[i], lv_palette_main(LV_PALETTE_BLUE));
      }
    } else {
      setObjHidden(pages_[i], true);
      if (navBtns_[i]) {
        lv_obj_clear_state(navBtns_[i], LV_STATE_PRESSED | LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY | LV_STATE_CHECKED);
        setNavButtonBg(navBtns_[i], lv_color_hex(0x1a2540));
      }
    }
  }
  activePage_ = idx;
  pageSwitchPending_ = true;
  lastHeaderUpdateMs_ = 0;
  lastPageUpdateMs_ = 0;
  if (hdrTitleLabel_) setLabelText(hdrTitleLabel_, kNames[idx]);
  if (idx == 6) {
    refreshSdBrowser(millis(), true);
  }
  forceContentRepaint();
  Serial0.printf("[SCREEN:PAGE] active=%u heap=%lu max_block=%lu dma_free=%lu dma_big=%lu lv_free=%lu\n",
                 static_cast<unsigned>(activePage_),
                 static_cast<unsigned long>(ESP.getFreeHeap()),
                 static_cast<unsigned long>(ESP.getMaxAllocHeap()),
                 static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_DMA)),
                 static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)),
                 static_cast<unsigned long>(lvglFreeBytes()));
}

// ---------------------------------------------------------------------------
// buildDashPage – arc RPM gauge + boost bar + status rows + race controls
// ---------------------------------------------------------------------------

void ScreenDashboard::buildDashPage(lv_obj_t* parent) {
  // Landscape dual-gauge layout (472×224 inner after 4 px page padding):
  //   LEFT  (x 0–163):   RPM arc 160×160
  //   CENTRE(x 168–303): boost bar, env, status, race stats, race buttons
  //   RIGHT (x 308–471): Speed arc 160×160
  //
  // Arc Y: (224-160)/2 = 32  →  vertically centred

  // ---- Helper lambda: build a generic arc gauge ----
  auto makeArc = [](lv_obj_t* par, lv_coord_t x, lv_coord_t y,
                    lv_coord_t size, int32_t rangeMax) -> lv_obj_t* {
    lv_obj_t* arc = lv_arc_create(par);
    lv_obj_set_size(arc, size, size);
    lv_obj_set_pos(arc, x, y);
    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_arc_set_range(arc, 0, rangeMax);
    lv_arc_set_value(arc, 0);
    setArcColor(arc, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 10, LV_PART_INDICATOR);
    setArcColor(arc, lv_color_hex(0x1a2540), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_pad_all(arc, 0, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    return arc;
  };

  constexpr lv_coord_t arcSz = 220;
  constexpr lv_coord_t arcY  = 6;
  constexpr lv_coord_t leftX = 6;
  constexpr lv_coord_t cx    = 244;
  constexpr lv_coord_t cw    = 228;
  constexpr lv_coord_t rightX = 6;

  // ---- LEFT: RPM arc ----
  rpmArc_ = makeArc(parent, leftX, arcY, arcSz, 8000);

  rpmValLabel_ = lv_label_create(parent);
  setLabelTextStatic(rpmValLabel_, rpmText_, sizeof(rpmText_), "0");
  lv_obj_set_width(rpmValLabel_, arcSz);
  lv_obj_set_style_text_font(rpmValLabel_, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_align(rpmValLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(rpmValLabel_, leftX, arcY + 50);

  lv_obj_t* rpmUnit = lv_label_create(parent);
  setLabelText(rpmUnit, "RPM");
  lv_obj_set_width(rpmUnit, arcSz);
  lv_obj_set_style_text_align(rpmUnit, LV_TEXT_ALIGN_CENTER, 0);
  setTextColor(rpmUnit, lv_color_hex(0x7090a0), 0);
  lv_obj_set_style_text_font(rpmUnit, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(rpmUnit, leftX, arcY + 82);
  setObjHidden(rpmArc_, true);
  setObjHidden(rpmValLabel_, true);
  setObjHidden(rpmUnit, true);
  rpmArc_ = nullptr;
  rpmValLabel_ = nullptr;

  // ---- RIGHT: Speed arc ----
  spdArc_ = makeArc(parent, rightX, arcY, arcSz, 160);

  spdValLabel_ = lv_label_create(parent);
  setLabelTextStatic(spdValLabel_, spdText_, sizeof(spdText_), "0");
  lv_obj_set_width(spdValLabel_, arcSz);
  lv_obj_set_style_text_font(spdValLabel_, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_align(spdValLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(spdValLabel_, rightX, arcY + 66);

  lv_obj_t* spdUnit = lv_label_create(parent);
  setLabelText(spdUnit, "MPH");
  lv_obj_set_width(spdUnit, arcSz);
  lv_obj_set_style_text_align(spdUnit, LV_TEXT_ALIGN_CENTER, 0);
  setTextColor(spdUnit, lv_color_hex(0x7090a0), 0);
  lv_obj_set_style_text_font(spdUnit, &lv_font_montserrat_20, 0);
  lv_obj_set_pos(spdUnit, rightX, arcY + 128);

  // ---- RIGHT: G-force widgets (below speed arc, y=130..196) ----
  gLiveLabel_ = lv_label_create(parent);
  setLabelTextStatic(gLiveLabel_, gLiveText_, sizeof(gLiveText_), "-- G");
  lv_obj_set_width(gLiveLabel_, arcSz);
  lv_obj_set_style_text_font(gLiveLabel_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_align(gLiveLabel_, LV_TEXT_ALIGN_CENTER, 0);
  setTextColor(gLiveLabel_, lv_color_hex(0x7090a0), 0);
  lv_obj_set_pos(gLiveLabel_, rightX, arcY + 96);

  gPeakLabel_ = lv_label_create(parent);
  setLabelTextStatic(gPeakLabel_, gPeakText_, sizeof(gPeakText_), "PK --");
  lv_obj_set_width(gPeakLabel_, arcSz);
  lv_obj_set_style_text_font(gPeakLabel_, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_align(gPeakLabel_, LV_TEXT_ALIGN_CENTER, 0);
  setTextColor(gPeakLabel_, lv_color_hex(0x506070), 0);
  lv_obj_set_pos(gPeakLabel_, rightX, arcY + 120);

  // Lateral bar: centred under label, range -100..+100 (maps ±1.5 G → ±100)
  gLatBar_ = lv_bar_create(parent);
  lv_obj_set_pos(gLatBar_, rightX + 12, arcY + 138);
  lv_obj_set_size(gLatBar_, arcSz - 24, 7);
  lv_bar_set_mode(gLatBar_, LV_BAR_MODE_SYMMETRICAL);  // negative values grow left from centre
  lv_bar_set_range(gLatBar_, -100, 100);
  lv_bar_set_value(gLatBar_, 0, LV_ANIM_OFF);
  setBgColor(gLatBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  setBgColor(gLatBar_, lv_palette_main(LV_PALETTE_CYAN), LV_PART_INDICATOR);
  setObjHidden(gLiveLabel_, true);
  setObjHidden(gPeakLabel_, true);
  setObjHidden(gLatBar_, true);
  gLiveLabel_ = nullptr;
  gPeakLabel_ = nullptr;
  gLatBar_ = nullptr;

  // Boost bar
  boostBar_ = lv_bar_create(parent);
  lv_obj_set_pos(boostBar_, cx, 76);
  lv_obj_set_size(boostBar_, cw, 18);
  lv_bar_set_range(boostBar_, 0, 40);
  lv_bar_set_value(boostBar_, 0, LV_ANIM_OFF);
  setBgColor(boostBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  setBgColor(boostBar_, lv_palette_main(LV_PALETTE_CYAN), LV_PART_INDICATOR);

  boostValLabel_ = makeLabel(parent, cx, 8, cw, "", &lv_font_montserrat_24);
  setLabelTextStatic(boostValLabel_, boostText_, sizeof(boostText_), "BOOST\n0.0 PSI");
  lv_obj_set_height(boostValLabel_, 64);
  lv_obj_set_style_text_align(boostValLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(boostValLabel_, LV_LABEL_LONG_WRAP);

  dashEnvLabel_ = makeLabel(parent, cx, 174, cw, "", &lv_font_montserrat_18);
  setLabelTextStatic(dashEnvLabel_, dashEnvText_, sizeof(dashEnvText_), "DUTY 0%  TANK 0%");
  lv_obj_set_height(dashEnvLabel_, 28);
  lv_obj_set_style_text_align(dashEnvLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(dashEnvLabel_, LV_LABEL_LONG_CLIP);

  dashStatusLabel_ = makeLabel(parent, cx, 108, cw, "", &lv_font_montserrat_24);
  setLabelTextStatic(dashStatusLabel_, dashStatusText_, sizeof(dashStatusText_), "METH\nOFFLINE");
  lv_obj_set_height(dashStatusLabel_, 62);
  lv_obj_set_style_text_align(dashStatusLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(dashStatusLabel_, LV_LABEL_LONG_WRAP);
  lv_label_set_recolor(dashStatusLabel_, true);

  dashRaceLabel_ = makeLabel(parent, cx, 208, cw, "", &lv_font_montserrat_14);
  setLabelTextStatic(dashRaceLabel_, dashRaceText_, sizeof(dashRaceText_), "12.5V  GPS 0/0  IAT 0C");
  lv_obj_set_height(dashRaceLabel_, 22);
  lv_obj_set_style_text_align(dashRaceLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(dashRaceLabel_, LV_LABEL_LONG_DOT);

  // Race control buttons – 2×2 grid inside centre strip
  constexpr lv_coord_t bgap = 6;
  constexpr lv_coord_t bw = static_cast<lv_coord_t>((cw - bgap) / 2);
  constexpr lv_coord_t bh = 36;
  constexpr lv_coord_t by = 122;
  raceAccelBtn_ = makeBtn(parent, "ACCEL", cx,          by,          bw, bh, onRaceAccelClicked, this);
  raceLapBtn_   = makeBtn(parent, "LAP",   cx+bw+bgap,  by,          bw, bh, onRaceLapClicked,   this);
  raceStopBtn_  = makeBtn(parent, "STOP",  cx,          by+bh+bgap,  bw, bh, onRaceStopClicked,  this);
  raceResetBtn_ = makeBtn(parent, "RESET", cx+bw+bgap,  by+bh+bgap,  bw, bh, onRaceResetClicked, this);
  styleChip(boostValLabel_);
  styleChip(dashEnvLabel_);
  styleChip(dashStatusLabel_);
  styleChip(dashRaceLabel_);
  styleActionButton(raceAccelBtn_);
  styleActionButton(raceLapBtn_);
  styleActionButton(raceStopBtn_);
  styleActionButton(raceResetBtn_);
  setObjHidden(raceAccelBtn_, true);
  setObjHidden(raceLapBtn_, true);
  setObjHidden(raceStopBtn_, true);
  setObjHidden(raceResetBtn_, true);
  raceAccelBtn_ = nullptr;
  raceLapBtn_ = nullptr;
  raceStopBtn_ = nullptr;
  raceResetBtn_ = nullptr;
}

// ---------------------------------------------------------------------------
// buildMethPage
// ---------------------------------------------------------------------------

void ScreenDashboard::buildMethPage(lv_obj_t* parent) {
  methBadgeLabel_ = makeLabel(parent, 0, 0, 308,
      LV_SYMBOL_TINT "  WATER-METH  |  #FF3B30 OFFLINE#", &lv_font_montserrat_16);
  lv_label_set_recolor(methBadgeLabel_, true);
  styleChip(methBadgeLabel_);

  methStateLabel_  = makeLabel(parent, 0, 24, 308, "LINK:OFF  EXT:OFF  ST:OFF  D:0%  T:0%", &lv_font_montserrat_16);
  methSensorLabel_ = makeLabel(parent, 0, 50, 308, "MAP 0  IAT 0.0  BAY 0.0  MP 0", &lv_font_montserrat_16);

  methOnlineLed_ = lv_led_create(parent);
  lv_obj_set_pos(methOnlineLed_, 318, 4);
  lv_obj_set_size(methOnlineLed_, 16, 16);
  lv_led_set_brightness(methOnlineLed_, 180);
  lv_led_off(methOnlineLed_);

  methOfflineSpinner_ = lv_label_create(parent);
  setLabelText(methOfflineSpinner_, LV_SYMBOL_WARNING);
  lv_obj_set_pos(methOfflineSpinner_, 340, 2);
  lv_obj_set_size(methOfflineSpinner_, 22, 22);
  lv_obj_set_style_text_font(methOfflineSpinner_, &lv_font_montserrat_20, 0);
  setTextColor(methOfflineSpinner_, lv_palette_main(LV_PALETTE_ORANGE), 0);

  methDutyMeter_ = lv_meter_create(parent);
  lv_obj_set_pos(methDutyMeter_, 314, 26);
  lv_obj_set_size(methDutyMeter_, 70, 70);
  lv_obj_set_style_bg_opa(methDutyMeter_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_meter_scale_t* dutyScale = lv_meter_add_scale(methDutyMeter_);
  lv_meter_set_scale_ticks(methDutyMeter_, dutyScale, 21, 1, 8, lv_color_hex(0x3a4a66));
  lv_meter_set_scale_major_ticks(methDutyMeter_, dutyScale, 5, 2, 10, lv_color_hex(0x8aa0c8), 8);
  lv_meter_set_scale_range(methDutyMeter_, dutyScale, 0, 100, 270, 135);
  methDutyNeedle_ = lv_meter_add_needle_line(methDutyMeter_, dutyScale, 3,
                                             lv_palette_main(LV_PALETTE_BLUE), -8);
  makeLabel(parent, 312, 96, 78, "DUTY", &lv_font_montserrat_12);

  methTankMeter_ = lv_meter_create(parent);
  lv_obj_set_pos(methTankMeter_, 392, 26);
  lv_obj_set_size(methTankMeter_, 70, 70);
  lv_obj_set_style_bg_opa(methTankMeter_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_meter_scale_t* tankScale = lv_meter_add_scale(methTankMeter_);
  lv_meter_set_scale_ticks(methTankMeter_, tankScale, 21, 1, 8, lv_color_hex(0x3a4a66));
  lv_meter_set_scale_major_ticks(methTankMeter_, tankScale, 5, 2, 10, lv_color_hex(0x8aa0c8), 8);
  lv_meter_set_scale_range(methTankMeter_, tankScale, 0, 100, 270, 135);
  methTankArc_ = lv_meter_add_arc(methTankMeter_, tankScale, 5,
                                  lv_palette_main(LV_PALETTE_GREEN), 0);
  makeLabel(parent, 390, 96, 78, "TANK", &lv_font_montserrat_12);

  methArmBtn_      = makeBtn(parent, "ON",         0, 142, 228, 52, onMethArmClicked,   this);
  methArmBtnLabel_ = btnLabel(methArmBtn_);

  methRatioBtn_      = makeBtn(parent, "RATIO 50%", 244, 142, 228, 52, onMethRatioClicked, this);
  methRatioBtnLabel_ = btnLabel(methRatioBtn_);
  styleChip(methStateLabel_);
  styleChip(methSensorLabel_);
  styleActionButton(methArmBtn_);
  styleActionButton(methRatioBtn_);
}

// ---------------------------------------------------------------------------
// buildTailPage
// ---------------------------------------------------------------------------

void ScreenDashboard::buildTailPage(lv_obj_t* parent) {
  tailStatusLabel_ = makeLabel(parent, 0, 0, 402,
      "Taillights: OFFLINE\nBright: 0  L:0  R:0", &lv_font_montserrat_18);

  tailOnlineLed_ = lv_led_create(parent);
  lv_obj_set_pos(tailOnlineLed_, 444, 6);
  lv_obj_set_size(tailOnlineLed_, 18, 18);
  lv_led_set_brightness(tailOnlineLed_, 180);
  lv_led_off(tailOnlineLed_);

  styleChip(tailStatusLabel_);

  // Mode button panel
  tailModePanel_ = lv_obj_create(parent);
  lv_obj_set_pos(tailModePanel_, 0, 58);
  lv_obj_set_size(tailModePanel_, 472, 92);
  lv_obj_set_style_pad_all(tailModePanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(tailModePanel_, 0, LV_PART_MAIN);
  setBgColor(tailModePanel_, lv_color_hex(0x0f1724), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tailModePanel_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_clear_flag(tailModePanel_, LV_OBJ_FLAG_SCROLLABLE);

  constexpr lv_coord_t tailBtnW = 112;
  constexpr lv_coord_t tailBtnH = 42;
  constexpr lv_coord_t tailGap  = 8;
  tailStockBtn_    = makeBtn(tailModePanel_, "STOCK",      0,                             0, tailBtnW, tailBtnH, onTailStockClicked,    this);
  tailSeqBtn_      = makeBtn(tailModePanel_, "SEQUENTIAL", tailBtnW + tailGap,            0, tailBtnW, tailBtnH, onTailSeqClicked,      this);
  tailShowMenuBtn_ = makeBtn(tailModePanel_, "SHOW",       2 * (tailBtnW + tailGap),      0, tailBtnW, tailBtnH, onTailShowMenuClicked, this);
  tailDemoBtn_     = makeBtn(tailModePanel_, "DEMO",       3 * (tailBtnW + tailGap),      0, tailBtnW, tailBtnH, onTailDemoClicked,     this);
  styleActionButton(tailStockBtn_);
  styleActionButton(tailSeqBtn_);
  styleActionButton(tailShowMenuBtn_);
  styleActionButton(tailDemoBtn_);

  // Show-option submenu panel
  tailShowPanel_ = lv_obj_create(parent);
  lv_obj_set_pos(tailShowPanel_, 0, 58);
  lv_obj_set_size(tailShowPanel_, 472, 166);
  lv_obj_set_style_pad_all(tailShowPanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(tailShowPanel_, 0, LV_PART_MAIN);
  setBgColor(tailShowPanel_, lv_color_hex(0x0f1724), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tailShowPanel_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_clear_flag(tailShowPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(tailShowPanel_, LV_OBJ_FLAG_HIDDEN);

  tailShowPageLabel_ = makeLabel(tailShowPanel_, 0, 0, 472, "Page 1/4");
  lv_obj_set_style_text_align(tailShowPageLabel_, LV_TEXT_ALIGN_CENTER, 0);

  tailShowPrevBtn_ = makeBtn(tailShowPanel_, "< PREV", 0,   24, 104, 34, onTailShowPrevClicked, this);
  tailShowBackBtn_ = makeBtn(tailShowPanel_, "BACK",   112, 24, 248, 34, onTailShowBackClicked, this);
  tailShowNextBtn_ = makeBtn(tailShowPanel_, "NEXT >", 368, 24, 104, 34, onTailShowNextClicked, this);
  styleActionButton(tailShowPrevBtn_);
  styleActionButton(tailShowBackBtn_);
  styleActionButton(tailShowNextBtn_);

  // 6 option buttons in a 3x2 grid
  const lv_coord_t optW = 150, optH = 38, optGap = 10;
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    const lv_coord_t col = static_cast<lv_coord_t>(i % 3);
    const lv_coord_t row = static_cast<lv_coord_t>(i / 3);
    const lv_coord_t bx  = col * (optW + optGap);
    const lv_coord_t by  = static_cast<lv_coord_t>(70) + row * (optH + optGap);
    tailShowOptBtns_[i] = makeBtn(tailShowPanel_, "N/A", bx, by, optW, optH, onTailShowOptClicked, this);
    styleActionButton(tailShowOptBtns_[i]);
  }
}

// ---------------------------------------------------------------------------
// buildLedsPage – LED mode selector + per-channel status
// ---------------------------------------------------------------------------

void ScreenDashboard::buildLedsPage(lv_obj_t* parent) {
  makeLabel(parent, 0, 0, kWidth, "Interior LEDs", &lv_font_montserrat_20);

  ledMasterSwitch_ = lv_switch_create(parent);
  lv_obj_set_pos(ledMasterSwitch_, 374, 2);
  lv_obj_set_size(ledMasterSwitch_, 88, 34);
  lv_obj_add_event_cb(ledMasterSwitch_, onLedMasterSwitchChanged, LV_EVENT_VALUE_CHANGED, this);

  ledMasterLabel_ = makeLabel(parent, 282, 10, 86, "MASTER OFF", &lv_font_montserrat_12);

  ledStatusLabel_ = makeLabel(parent, 0, 42, 472,
      "1:OFF  2:OFF  3:OFF", &lv_font_montserrat_18);

  static const char* const kModeNames[5] = { "OFF", "STATIC", "BREATHE", "RAINBOW", "RPM" };
  static const state::LedMode kModeValues[5] = {
    state::LedMode::OFF,
    state::LedMode::STATIC_COLOR,
    state::LedMode::BREATHING,
    state::LedMode::RAINBOW,
    state::LedMode::RPM_GAUGE,
  };
  constexpr lv_coord_t btnW = 89, btnH = 46;
  constexpr lv_coord_t startX = 0, gapX = 6;
  for (uint8_t i = 0; i < 5; i++) {
    s_ledModeCtxs[i] = {this, kModeValues[i]};
    const lv_coord_t bx = static_cast<lv_coord_t>(startX + i * (btnW + gapX));
    ledModeBtns_[i] = makeBtn(parent, kModeNames[i], bx, 82, btnW, btnH, onLedModeClicked, &s_ledModeCtxs[i]);
    lv_obj_set_style_text_font(btnLabel(ledModeBtns_[i]), &lv_font_montserrat_14, 0);
    styleActionButton(ledModeBtns_[i]);
  }
}

// ---------------------------------------------------------------------------
// buildGpsPage – GPS speed, fix, coordinates
// ---------------------------------------------------------------------------

void ScreenDashboard::buildGpsPage(lv_obj_t* parent) {
  gpsFixLed_ = lv_led_create(parent);
  lv_obj_set_pos(gpsFixLed_, 412, 4);
  lv_obj_set_size(gpsFixLed_, 18, 18);
  lv_led_set_brightness(gpsFixLed_, 180);
  lv_led_off(gpsFixLed_);

  gpsSatMeter_ = lv_meter_create(parent);
  lv_obj_set_pos(gpsSatMeter_, 372, 24);
  lv_obj_set_size(gpsSatMeter_, 92, 92);
  lv_obj_set_style_bg_opa(gpsSatMeter_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_meter_scale_t* satScale = lv_meter_add_scale(gpsSatMeter_);
  lv_meter_set_scale_ticks(gpsSatMeter_, satScale, 11, 1, 8, lv_color_hex(0x3a4a66));
  lv_meter_set_scale_major_ticks(gpsSatMeter_, satScale, 5, 2, 10, lv_color_hex(0x8aa0c8), 8);
  lv_meter_set_scale_range(gpsSatMeter_, satScale, 0, 20, 270, 135);
  gpsSatNeedle_ = lv_meter_add_needle_line(gpsSatMeter_, satScale, 3,
                                           lv_palette_main(LV_PALETTE_GREEN), -8);
  makeLabel(parent, 374, 118, 92, "SATS", &lv_font_montserrat_12);

  gpsSpdLabel_ = lv_label_create(parent);
  setLabelText(gpsSpdLabel_, "0 mph");
  lv_obj_set_width(gpsSpdLabel_, 340);
  lv_obj_set_style_text_font(gpsSpdLabel_, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_align(gpsSpdLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(gpsSpdLabel_, 0, 10);

  gpsInfoLabel_ = lv_label_create(parent);
  setLabelText(gpsInfoLabel_,
      "FIX: NO  SATS: 0\nLAT: 0.000000\nLON: 0.000000");
  lv_obj_set_width(gpsInfoLabel_, 340);
  lv_obj_set_style_text_font(gpsInfoLabel_, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_align(gpsInfoLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(gpsInfoLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_pos(gpsInfoLabel_, 0, 74);
  lv_obj_set_height(gpsInfoLabel_, 86);
  styleOpaqueRedrawSurface(gpsInfoLabel_);
}

// ---------------------------------------------------------------------------
// buildTempsPage – all temperature channels
// ---------------------------------------------------------------------------

void ScreenDashboard::buildTempsPage(lv_obj_t* parent) {
  tempsLabel_ = makeLabel(parent, 0, 0, kWidth - 8,
      LV_SYMBOL_WARNING "  SENSORS", &lv_font_montserrat_16);

  tempsTable_ = lv_table_create(parent);
  lv_obj_set_pos(tempsTable_, 0, 24);
  lv_obj_set_size(tempsTable_, 472, 184);
  lv_table_set_col_cnt(tempsTable_, 3);
  lv_table_set_row_cnt(tempsTable_, 6);
  lv_table_set_col_width(tempsTable_, 0, 150);
  lv_table_set_col_width(tempsTable_, 1, 160);
  lv_table_set_col_width(tempsTable_, 2, 140);

  lv_table_set_cell_value(tempsTable_, 0, 0, "SENSOR");
  lv_table_set_cell_value(tempsTable_, 0, 1, "VALUE");
  lv_table_set_cell_value(tempsTable_, 0, 2, "STATUS");
}

// ---------------------------------------------------------------------------
// buildDiagPage
// ---------------------------------------------------------------------------

void ScreenDashboard::buildDiagPage(lv_obj_t* parent) {
  diagLabel_ = makeLabel(parent, 0, 0, kWidth - 8,
  "Diagnostics initializing...", &lv_font_montserrat_14);
  lv_label_set_long_mode(diagLabel_, LV_LABEL_LONG_CLIP);
  lv_obj_set_width(diagLabel_, kWidth - 8);
  lv_obj_set_height(diagLabel_, 92);
  lv_label_set_recolor(diagLabel_, true);
  styleOpaqueRedrawSurface(diagLabel_);

  // Bench test mode toggle — bottom-right of DIAG page
  benchTestBtn_ = makeBtn(parent, "BENCH: OFF", 342, 58, 130, 28,
                          onBenchTestClicked, this);
  buildSdBrowser(parent);
}

void ScreenDashboard::buildSdBrowser(lv_obj_t* parent) {
  sdPathLabel_ = makeLabel(parent, 0, 98, 296, "SD: not mounted", &lv_font_montserrat_12);
  lv_label_set_long_mode(sdPathLabel_, LV_LABEL_LONG_DOT);

  sdUpBtn_ = makeBtn(parent, "UP", 300, 96, 38, 26, onSdUpClicked, this);
  sdPrevBtn_ = makeBtn(parent, "PREV", 340, 96, 42, 26, onSdPrevClicked, this);
  sdNextBtn_ = makeBtn(parent, "NEXT", 384, 96, 42, 26, onSdNextClicked, this);
  sdTestBtn_ = makeBtn(parent, "TEST", 428, 96, 44, 26, onSdTestClicked, this);

  for (uint8_t i = 0; i < kSdFileRowCount; ++i) {
    s_sdFileRowCtxs[i] = {this, i};
    lv_obj_t* row = lv_btn_create(parent);
    lv_obj_set_pos(row, 0, static_cast<lv_coord_t>(126 + i * 20));
    lv_obj_set_size(row, 472, 18);
    lv_obj_set_style_radius(row, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(row, lv_color_hex(0x101826), LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(row, lv_color_hex(0x26364d), LV_PART_MAIN);
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
  constexpr lv_coord_t leftW = 286;
  constexpr lv_coord_t barW = 280;
  constexpr lv_coord_t barH = 12;
  constexpr lv_coord_t barX = 4;

  // ---- Row 0: state header ----
  knockStateLabel_ = makeLabel(parent, 0, 0, leftW,
      "KNOCK  |  Response: WARN_ONLY", &lv_font_montserrat_12);
  lv_label_set_recolor(knockStateLabel_, true);

  // ---- Row 1: sensor status ----
  knockSensorLabel_ = makeLabel(parent, 0, 20, leftW,
      "En:YES  Sensor:OK  On:YES  Learn:NO  Fault:NO",
      &lv_font_montserrat_12);
  lv_label_set_long_mode(knockSensorLabel_, LV_LABEL_LONG_CLIP);

  // ---- Right panel: live chart trend ----
  knockGraphLabel_ = makeLabel(parent, 300, 0, 168, LV_SYMBOL_WARNING "  TREND", &lv_font_montserrat_12);
  knockGraphChart_ = lv_chart_create(parent);
  lv_obj_set_pos(knockGraphChart_, 300, 16);
  lv_obj_set_size(knockGraphChart_, 168, 96);
  lv_chart_set_type(knockGraphChart_, LV_CHART_TYPE_LINE);
  lv_chart_set_range(knockGraphChart_, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
  lv_chart_set_point_count(knockGraphChart_, 32);
  lv_chart_set_div_line_count(knockGraphChart_, 4, 6);
  setBgColor(knockGraphChart_, lv_color_hex(0x101826), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(knockGraphChart_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(knockGraphChart_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_color(knockGraphChart_, lv_color_hex(0x2b3d57), LV_PART_MAIN);
  lv_obj_set_style_border_width(knockGraphChart_, 1, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(knockGraphChart_, 0, LV_PART_MAIN);
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

  knockWarnLed_ = lv_led_create(parent);
  lv_obj_set_pos(knockWarnLed_, 300, 118);
  lv_obj_set_size(knockWarnLed_, 14, 14);
  lv_led_set_color(knockWarnLed_, lv_palette_main(LV_PALETTE_ORANGE));
  lv_led_off(knockWarnLed_);
  makeLabel(parent, 316, 116, 58, "WARN", &lv_font_montserrat_12);

  knockCritLed_ = lv_led_create(parent);
  lv_obj_set_pos(knockCritLed_, 300, 136);
  lv_obj_set_size(knockCritLed_, 14, 14);
  lv_led_set_color(knockCritLed_, lv_palette_main(LV_PALETTE_RED));
  lv_led_off(knockCritLed_);
  makeLabel(parent, 316, 134, 58, "CRIT", &lv_font_montserrat_12);

  knockLearningSpinner_ = lv_label_create(parent);
  setLabelText(knockLearningSpinner_, "LEARN");
  lv_obj_set_pos(knockLearningSpinner_, 396, 114);
  lv_obj_set_size(knockLearningSpinner_, 42, 42);
  lv_obj_set_style_text_font(knockLearningSpinner_, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_align(knockLearningSpinner_, LV_TEXT_ALIGN_CENTER, 0);
  setTextColor(knockLearningSpinner_, lv_palette_main(LV_PALETTE_ORANGE), 0);

  // ---- Row 2: energy label ----
  knockEnergyLabel_ = makeLabel(parent, 0, 40, leftW,
      "Energy: 0.0  (0%)", &lv_font_montserrat_12);

  // ---- Row 3: energy bar ----
  knockEnergyBar_ = lv_bar_create(parent);
  lv_obj_set_pos(knockEnergyBar_, barX, 56);
  lv_obj_set_size(knockEnergyBar_, barW, barH);
  lv_bar_set_range(knockEnergyBar_, 0, 100);
  lv_bar_set_value(knockEnergyBar_, 0, LV_ANIM_OFF);
  setBgColor(knockEnergyBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  setBgColor(knockEnergyBar_, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);

  // ---- Row 4: baseline label ----
  knockBaselineLabel_ = makeLabel(parent, 0, 74, leftW,
      "Baseline: 0.0  (0%)", &lv_font_montserrat_12);

  // ---- Row 5: baseline bar ----
  knockBaselineBar_ = lv_bar_create(parent);
  lv_obj_set_pos(knockBaselineBar_, barX, 90);
  lv_obj_set_size(knockBaselineBar_, barW, barH);
  lv_bar_set_range(knockBaselineBar_, 0, 100);
  lv_bar_set_value(knockBaselineBar_, 0, LV_ANIM_OFF);
  setBgColor(knockBaselineBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  setBgColor(knockBaselineBar_, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);

  // ---- Row 6: threshold label ----
  knockThresholdLabel_ = makeLabel(parent, 0, 108, leftW,
      "Threshold: 0.0  (100%)", &lv_font_montserrat_12);

  // ---- Row 7: threshold bar (always 100% — marks the maximum safe level) ----
  knockThresholdBar_ = lv_bar_create(parent);
  lv_obj_set_pos(knockThresholdBar_, barX, 124);
  lv_obj_set_size(knockThresholdBar_, barW, barH);
  lv_bar_set_range(knockThresholdBar_, 0, 100);
  lv_bar_set_value(knockThresholdBar_, 100, LV_ANIM_OFF);
  setBgColor(knockThresholdBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  setBgColor(knockThresholdBar_, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);

  // ---- Row 8: events / warning / critical ----
  knockEventLabel_ = makeLabel(parent, 0, 142, leftW,
      "Events: 0  Warn:NO  Crit:NO", &lv_font_montserrat_12);
  lv_label_set_recolor(knockEventLabel_, true);

  // ---- Row 9: last event details ----
  knockLastLabel_ = makeLabel(parent, 0, 158, leftW,
      "Last: --", &lv_font_montserrat_12);

  // ---- Row 10: control buttons (y=178, h=36) ----
  constexpr lv_coord_t btnW = 110, btnH = 32, btnGap = 4;
  knockEnableBtn_      = makeBtn(parent, "EN/DIS",  0 * (btnW + btnGap), 176, btnW, btnH, onKnockEnableClicked,        this);
  knockResetBlBtn_     = makeBtn(parent, "RESET",    1 * (btnW + btnGap), 176, btnW, btnH, onKnockResetBaselineClicked, this);
  knockClearEvtBtn_    = makeBtn(parent, "CLR",      2 * (btnW + btnGap), 176, btnW, btnH, onKnockClearEventsClicked,   this);
  knockSimulateBtn_    = makeBtn(parent, "SIM",      3 * (btnW + btnGap), 176, btnW, btnH, onKnockSimulateClicked,      this);
  knockEnableBtnLabel_ = btnLabel(knockEnableBtn_);

  lv_obj_set_style_text_font(btnLabel(knockEnableBtn_),   &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_font(btnLabel(knockResetBlBtn_),  &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_font(btnLabel(knockClearEvtBtn_), &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_font(btnLabel(knockSimulateBtn_), &lv_font_montserrat_12, 0);
  styleActionButton(knockEnableBtn_);
  styleActionButton(knockResetBlBtn_);
  styleActionButton(knockClearEvtBtn_);
  styleActionButton(knockSimulateBtn_);

  // ---- Row 11: logging status + disclaimer ----
  knockLogLabel_ = makeLabel(parent, 0, 218, kWidth - 8,
      "SD:--  Note: not ECU knock control", &lv_font_montserrat_12);
  lv_label_set_long_mode(knockLogLabel_, LV_LABEL_LONG_CLIP);
}

// ---------------------------------------------------------------------------
// Per-tick update helpers
// ---------------------------------------------------------------------------

void ScreenDashboard::updateHeader(const state::VehicleState& s, uint32_t nowMs) {
  // Battery voltage (left)
  char bat[12];
  snprintf(bat, sizeof(bat), "%.1fV", static_cast<double>(s.battery_voltage));
  setLabelTextStatic(hdrBatLabel_, hdrBatText_, sizeof(hdrBatText_), bat);

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
  } else {
    setLabelText(hdrFeedbackLabel_, "");
    actionFeedback_[0] = '\0';
  }
}

void ScreenDashboard::updateDashPage(const state::VehicleState& s) {
  // ---- RPM arc ----
  const int16_t rpmClamped = static_cast<int16_t>(
      (s.rpm > 8000U) ? 8000U : s.rpm);
  setArcValue(rpmArc_, dashRpmArcLast_, rpmClamped);

  lv_color_t arcColor;
  if (s.rpm >= 6500U)       arcColor = lv_palette_main(LV_PALETTE_RED);
  else if (s.rpm >= 4000U)  arcColor = lv_palette_main(LV_PALETTE_ORANGE);
  else                      arcColor = lv_palette_main(LV_PALETTE_BLUE);
  setArcColor(rpmArc_, arcColor, LV_PART_INDICATOR);

  char buf[64];
  snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(s.rpm));
  setLabelTextStatic(rpmValLabel_, rpmText_, sizeof(rpmText_), buf);

  // ---- Speed arc (mph) ----
  const float spdMph = s.speed * 0.621371f;
  const int16_t spdClamped = static_cast<int16_t>(
      (spdMph > 160.0f) ? 160 : (spdMph < 0.0f) ? 0 : static_cast<int16_t>(spdMph));
  setArcValue(spdArc_, dashSpdArcLast_, spdClamped);

  lv_color_t spdColor;
  if (spdMph >= 112.0f)      spdColor = lv_palette_main(LV_PALETTE_RED);
  else if (spdMph >= 62.0f)  spdColor = lv_palette_main(LV_PALETTE_ORANGE);
  else                       spdColor = lv_palette_main(LV_PALETTE_GREEN);
  setArcColor(spdArc_, spdColor, LV_PART_INDICATOR);

  snprintf(buf, sizeof(buf), "%.0f", static_cast<double>(spdMph));
  setLabelTextStatic(spdValLabel_, spdText_, sizeof(spdText_), buf);

  // ---- Boost and water-meth status ----
  const float boostPsi = s.boost_kpa * 0.145038f;
  const int16_t boostClamped = static_cast<int16_t>(
      (boostPsi > 40.0f) ? 40 : (boostPsi < 0.0f) ? 0 : static_cast<int16_t>(boostPsi));
  setBarValue(boostBar_, dashBoostBarLast_, boostClamped);
  lv_color_t boostColor;
  if (boostPsi >= 20.0f)       boostColor = lv_palette_main(LV_PALETTE_RED);
  else if (boostPsi >= 10.0f)  boostColor = lv_palette_main(LV_PALETTE_ORANGE);
  else                         boostColor = lv_palette_main(LV_PALETTE_CYAN);
  setBgColor(boostBar_, boostColor, LV_PART_INDICATOR);
  snprintf(buf, sizeof(buf), "BOOST\n%.1f PSI", static_cast<double>(boostPsi));
  setLabelTextStatic(boostValLabel_, boostText_, sizeof(boostText_), buf);

  snprintf(buf, sizeof(buf), "DUTY %u%%  TANK %u%%",
           static_cast<unsigned>(s.meth_pump_duty > 100U ? 100U : s.meth_pump_duty),
           static_cast<unsigned>(s.meth_tank_level));
  setLabelTextStatic(dashEnvLabel_, dashEnvText_, sizeof(dashEnvText_), buf);

  const bool methActive = (s.meth_state == state::MethState::SPRAYING) || s.manual_test_running;
  const char* methColor = kStatusColorOff;
  const char* methText = "OFFLINE";
  lv_color_t methBg = lv_color_hex(0x3a1d25);
  if (methActive) {
    methColor = "#FF9500";
    methText = "ACTIVE";
    methBg = lv_color_hex(0x5c2f00);
  } else if (!s.meth_online) {
    methColor = kStatusColorOff;
    methText = "OFFLINE";
    methBg = lv_color_hex(0x3a1d25);
  } else if (s.meth_desired_armed || s.meth_state == state::MethState::ARMED) {
    methColor = "#00AEEF";
    methText = "ARMED";
    methBg = lv_color_hex(0x113653);
  } else {
    methColor = "#8AA0C8";
    methText = "OFF";
    methBg = lv_color_hex(0x1a2538);
  }
  snprintf(buf, sizeof(buf), "METH\n%s%s#", methColor, methText);
  setBgColor(dashStatusLabel_, methBg, LV_PART_MAIN);
  setLabelTextStatic(dashStatusLabel_, dashStatusText_, sizeof(dashStatusText_), buf);

  snprintf(buf, sizeof(buf), "%.1fV  GPS %u/%u  IAT %.0fC",
           static_cast<double>(s.battery_voltage),
           static_cast<unsigned>(s.gps_satellites),
           static_cast<unsigned>(s.gps_satellites_in_view),
           static_cast<double>(s.intake_temp));
  setLabelTextStatic(dashRaceLabel_, dashRaceText_, sizeof(dashRaceText_), buf);

  // ---- G-force widgets ----
  if (s.imu_online) {
    const float g = s.imu_g_total;
    lv_color_t gColor;
    if (g >= 1.0f)       gColor = lv_palette_main(LV_PALETTE_RED);
    else if (g >= 0.7f)  gColor = lv_palette_main(LV_PALETTE_ORANGE);
    else if (g >= 0.3f)  gColor = lv_palette_main(LV_PALETTE_GREEN);
    else                 gColor = lv_color_hex(0x7090a0);

    snprintf(buf, sizeof(buf), "%.2f G", static_cast<double>(g));
    setLabelTextStatic(gLiveLabel_, gLiveText_, sizeof(gLiveText_), buf);
    setTextColor(gLiveLabel_, gColor, 0);

    snprintf(buf, sizeof(buf), "PK %.2f", static_cast<double>(s.imu_g_peak));
    setLabelTextStatic(gPeakLabel_, gPeakText_, sizeof(gPeakText_), buf);

    // Scale lateral G to bar: ±1.5 G → ±100
    const int32_t latScaled = static_cast<int32_t>(s.imu_g_lateral * 66.7f);
    const int32_t latClamped = latScaled > 100 ? 100 : (latScaled < -100 ? -100 : latScaled);
    setBarValue(gLatBar_, dashLatBarLast_, latClamped);
    setBgColor(gLatBar_, gColor, LV_PART_INDICATOR);
  } else {
    setLabelTextStatic(gLiveLabel_, gLiveText_, sizeof(gLiveText_), "-- G");
    setTextColor(gLiveLabel_, lv_color_hex(0x506070), 0);
    setLabelTextStatic(gPeakLabel_, gPeakText_, sizeof(gPeakText_), "PK --");
    setBarValue(gLatBar_, dashLatBarLast_, 0);
  }
}

void ScreenDashboard::updateMethPage(const state::VehicleState& s, uint32_t nowMs) {
  char buf[160];
  const bool extFresh = (s.last_analog_sensor_ms != 0U) &&
                        ((nowMs - s.last_analog_sensor_ms) <= kMethSensorStaleMs);
  const char* moduleLink = s.meth_online ? "ON" : "OFF";
  const char* extLink = extFresh ? "ON" : "OFF";
  const bool methActive = (s.meth_state == state::MethState::SPRAYING) || s.manual_test_running;

  if (methBadgeLabel_) {
    if (methActive) {
      snprintf(buf, sizeof(buf), LV_SYMBOL_TINT "  WATER-METH  |  #FF9500 ACTIVE#");
    } else if (s.meth_online) {
      snprintf(buf, sizeof(buf), LV_SYMBOL_TINT "  WATER-METH  |  #00C853 ONLINE#");
    } else {
      snprintf(buf, sizeof(buf), LV_SYMBOL_TINT "  WATER-METH  |  #FF3B30 OFFLINE#");
    }
    setLabelText(methBadgeLabel_, buf);
  }

  if (methOnlineLed_) {
    if (s.meth_online) lv_led_on(methOnlineLed_);
    else lv_led_off(methOnlineLed_);
  }

  if (methOfflineSpinner_) {
    setObjHidden(methOfflineSpinner_, s.meth_online);
  }

  if (methStateLabel_) {
    snprintf(buf, sizeof(buf),
             "LINK:%s  EXT:%s  ST:%s  D:%u%%  T:%u%%",
             moduleLink,
             extLink,
             methStateName(s.meth_state),
             static_cast<unsigned>(s.meth_pump_duty > 100U ? 100U : s.meth_pump_duty),
             static_cast<unsigned>(s.meth_tank_level));
    setLabelText(methStateLabel_, buf);
  }

  if (methSensorLabel_) {
    snprintf(buf, sizeof(buf), "MAP %.0f  IAT %.1f  BAY %.1f  MP %.0f",
             static_cast<double>(s.boost_kpa),
             static_cast<double>(s.intake_temp),
             static_cast<double>(s.engine_bay_temp),
             static_cast<double>(s.meth_pressure_psi));
    setLabelText(methSensorLabel_, buf);
  }

  const uint8_t duty = static_cast<uint8_t>(s.meth_pump_duty > 100U ? 100U : s.meth_pump_duty);
  setMeterValue(methDutyMeter_, methDutyNeedle_, methDutyMeterLast_, duty);

  if (methArmBtnLabel_) {
    setLabelText(methArmBtnLabel_, s.meth_desired_armed ? "OFF" : "ON");
  }

  if (methRatioBtnLabel_) {
    snprintf(buf, sizeof(buf), "RATIO  %u%%",
             static_cast<unsigned>(s.meth_selected_ratio_percent));
    setLabelText(methRatioBtnLabel_, buf);
  }

  if (methArmBtn_) {
    if (methActive) {
      const float wave = 0.5f + 0.5f * sinf(static_cast<float>(nowMs) * 0.010f);
      const uint8_t r = static_cast<uint8_t>(180.0f + 55.0f * wave);
      const uint8_t g = static_cast<uint8_t>(45.0f + 25.0f * wave);
      const uint8_t b = static_cast<uint8_t>(18.0f);
      setBgColor(methArmBtn_, lv_color_make(r, g, b), LV_PART_MAIN);
    } else if (s.meth_desired_armed) {
      setBgColor(methArmBtn_, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
    } else {
      setBgColor(methArmBtn_, lv_color_hex(0x2b3340), LV_PART_MAIN);
    }
  }
}

void ScreenDashboard::updateTailPage(const state::VehicleState& s) {
  char buf[96];

  snprintf(buf, sizeof(buf),
           LV_SYMBOL_LOOP " %s  %s\nBright:%u  L:%u  R:%u",
           s.taillight_online ? "ONLINE" : "OFFLINE",
           tailModeName(s.taillight_mode_commanded),
           static_cast<unsigned>(s.taillight_brightness),
           static_cast<unsigned>(s.taillight_left_state),
           static_cast<unsigned>(s.taillight_right_state));
  setLabelText(tailStatusLabel_, buf);

  if (tailOnlineLed_) {
    if (s.taillight_online) lv_led_on(tailOnlineLed_);
    else lv_led_off(tailOnlineLed_);
  }

  static constexpr const char* kTailShowNames[kTaillightShowOptionCount] = {
    "Rainbow",     // 0
    "Chase",       // 1
    "Theater",     // 2
    "Fire",        // 3
    "Meteor",      // 4
    "Police",      // 5
    "Night Rider", // 6
    "Color Cycle", // 7
    "Sparkle",     // 8
    "Plasma",      // 9
    "Matrix",      // 10
    "Juggle",      // 11
    "BPM",         // 12
    "Confetti",    // 13
    "Ocean",       // 14
    "Lightning",   // 15
    "Heartbeat",   // 16
    "Ripple",      // 17
    "Sunrise",     // 18
    "Text Scroll", // 19
    "Colorwaves",  // 20
    "TwinkleFox",  // 21
    "Bounce",      // 22
    "Fireworks",   // 23
    "Drip",        // 24
    "Cylon",       // 25
    "V8",          // 26
    "Drag Launch", // 27
    "Neon",        // 28
    "Streaks",     // 29
    "Radar",       // 30
    "Aurora",      // 31
    "Glitch",      // 32
  };
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    const uint16_t optVal = static_cast<uint16_t>(tailShowPage_) * kTaillightShowOptionsPerPage + i;
    if (optVal < kTaillightShowOptionCount) {
      setLabelText(btnLabel(tailShowOptBtns_[i]), kTailShowNames[optVal]);
    } else {
      setLabelText(btnLabel(tailShowOptBtns_[i]), "N/A");
    }
  }

  char page[16];
  snprintf(page, sizeof(page), "Page %u/%u",
           static_cast<unsigned>(tailShowPage_ + 1U),
           static_cast<unsigned>(kTaillightShowPageCount));
  setLabelText(tailShowPageLabel_, page);
}

void ScreenDashboard::updateLedsPage(const state::VehicleState& s) {
  static const char* const kModeStr[] = {
    "OFF", "STATIC", "BREATHE", "RAINBOW", "RPM", "FLASH", "METH", "FAULT", "SWEEP", "GAUGE"
  };
  constexpr uint8_t kModeStrCount = 10;

  auto modeName = [&](state::LedMode m) -> const char* {
    const uint8_t idx = static_cast<uint8_t>(m);
    return (idx < kModeStrCount) ? kModeStr[idx] : "?";
  };

  char buf[96];
  snprintf(buf, sizeof(buf), LV_SYMBOL_CHARGE " 1:%s  2:%s  3:%s",
           modeName(s.led_channel_1_mode),
           modeName(s.led_channel_2_mode),
           modeName(s.led_channel_3_mode));
  setLabelText(ledStatusLabel_, buf);

  const bool allEnabled = s.led_channel_1_enabled && s.led_channel_2_enabled && s.led_channel_3_enabled;
  if (ledMasterSwitch_) {
    if (allEnabled) lv_obj_add_state(ledMasterSwitch_, LV_STATE_CHECKED);
    else lv_obj_clear_state(ledMasterSwitch_, LV_STATE_CHECKED);
  }
  if (ledMasterLabel_) {
    setLabelText(ledMasterLabel_, allEnabled ? "MASTER ON" : "MASTER OFF");
  }

  // Highlight the active mode button for CH1 (as reference)
  for (uint8_t i = 0; i < 5; i++) {
    const bool active = (s.led_channel_1_mode == s_ledModeCtxs[i].mode);
    setBgColor(ledModeBtns_[i],
        active ? lv_palette_main(LV_PALETTE_BLUE) : lv_color_hex(0x1a2540),
        LV_PART_MAIN);
  }
}

void ScreenDashboard::updateGpsPage(const state::VehicleState& s) {
  char spd[24];
  const bool gpsLive = !s.gps_stale;
  if (s.gps_fix) {
    const float rawMph = (s.speed > 0.0f) ? (s.speed * 0.621371f) : 0.0f;
    if (!gpsSpeedFilterInitialized_) {
      gpsSpeedFilteredMph_ = rawMph;
      gpsSpeedFilterInitialized_ = true;
    } else {
      const float alpha = (rawMph < kGpsLowSpeedThresholdMph)
                              ? kGpsLowSpeedFilterAlpha
                              : kGpsHighSpeedFilterAlpha;
      gpsSpeedFilteredMph_ += (rawMph - gpsSpeedFilteredMph_) * alpha;
    }

    float displayMph = gpsSpeedFilteredMph_;
    if (displayMph < kGpsZeroClampMph) {
      displayMph = 0.0f;
      gpsSpeedFilteredMph_ = 0.0f;
    } else if (displayMph < kGpsLowSpeedThresholdMph) {
      displayMph = std::roundf(displayMph);
    }

    snprintf(spd, sizeof(spd), "%.0f mph", displayMph);
  } else {
    gpsSpeedFilterInitialized_ = false;
    gpsSpeedFilteredMph_ = 0.0f;
    snprintf(spd, sizeof(spd), "-- mph");
  }
  setLabelText(gpsSpdLabel_, spd);

  if (gpsFixLed_) {
    if (s.gps_fix) lv_led_on(gpsFixLed_);
    else lv_led_off(gpsFixLed_);
  }
  if (gpsSatMeter_ && gpsSatNeedle_) {
    uint8_t sats = s.gps_fix ? s.gps_satellites : s.gps_satellites_in_view;
    if (sats > 20U) sats = 20U;
    setMeterValue(gpsSatMeter_, gpsSatNeedle_, gpsSatMeterLast_, sats);
  }

  char info[160];
  snprintf(info, sizeof(info),
           LV_SYMBOL_GPS " %s  FIX:%s  USED:%u VIEW:%u\nQ:%u MODE:%u HDOP:%.1f\nLAT: %.6f\nLON: %.6f",
           gpsLive ? "LIVE" : "STALE",
           s.gps_fix ? "YES" : "NO",
           static_cast<unsigned>(s.gps_satellites),
           static_cast<unsigned>(s.gps_satellites_in_view),
           static_cast<unsigned>(s.gps_fix_quality),
           static_cast<unsigned>(s.gps_fix_mode),
           static_cast<double>(s.gps_hdop_x10 / 10.0f),
           s.gps_latitude,
           s.gps_longitude);
  setLabelText(gpsInfoLabel_, info);
}

void ScreenDashboard::updateTempsPage(const state::VehicleState& s) {
  char v[32];
  if (tempsLabel_) {
    snprintf(v, sizeof(v), LV_SYMBOL_WARNING " SENSOR BOARD  |  FaultMask:0x%04X",
             static_cast<unsigned>(s.analog_sensor_fault_flags));
    setLabelText(tempsLabel_, v);
  }
  if (!tempsTable_) return;

  auto setRow = [&](uint16_t row, const char* name, float value, bool ok, const char* unit) {
    char valueBuf[24];
    char statusBuf[8];
    snprintf(valueBuf, sizeof(valueBuf), "%.1f %s", static_cast<double>(value), unit);
    snprintf(statusBuf, sizeof(statusBuf), "%s", ok ? "OK" : "BAD");
    lv_table_set_cell_value(tempsTable_, row, 0, name);
    lv_table_set_cell_value(tempsTable_, row, 1, valueBuf);
    lv_table_set_cell_value(tempsTable_, row, 2, statusBuf);
  };

  setRow(1, "IAT", s.intake_temp, s.intake_temp_valid, "C");
  setRow(2, "ENGINE BAY", s.engine_bay_temp, s.engine_bay_temp_valid, "C");
  setRow(3, "CABIN", s.cabin_temp, s.cabin_temp_valid, "C");
  setRow(4, "AMBIENT", s.outside_temp, s.outside_temp_valid, "C");
  setRow(5, "OIL PRESS", s.oil_pressure_psi, s.oil_pressure_valid, "psi");
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

  // Color helpers
  auto col = [](bool ok) -> const char* {
    return ok ? "#00C853" : "#FF3B30";
  };

  // GPS: green=fix, yellow=connected/no-fix, red=stale
  const bool gpsActive = msAgo(s.last_gps_ms) < 5;
  const char* gpsCol = s.gps_fix ? "#00C853" : (gpsActive ? "#FFD600" : "#FF3B30");
  char gpsStat[12];
  if (s.gps_fix)      snprintf(gpsStat, sizeof(gpsStat), "FIX %u/%u", s.gps_satellites, s.gps_satellites_in_view);
  else if (gpsActive) snprintf(gpsStat, sizeof(gpsStat), "NO FIX");
  else                snprintf(gpsStat, sizeof(gpsStat), "OFFLINE");

  // CAN: green=rx active, yellow=online but silent rx, red=offline
  const bool canRxActive = s.can_rx_count > 0 && msAgo(s.can_last_rx_ms) < 10;
  const char* canCol = s.can_online ? (canRxActive ? "#00C853" : "#FFD600") : "#FF3B30";
  const char* canStat = s.can_online ? (canRxActive ? "RX OK" : "TX ONLY") : "OFFLINE";

  // Heap min watermark — suppress garbage initial value
  const uint32_t heapMin = (s.heap_min_free_bytes == 0xFFFFFFFFUL)
      ? s.heap_free_bytes : s.heap_min_free_bytes;

  constexpr size_t kBuf = 1024;
  char buf[kBuf];
  int n = 0;

  // ── Status grid  (2 columns, 3 rows) ─────────────────────────────────────
  // Format: LABEL  #COLOR STATUS #  |  LABEL  #COLOR STATUS #
  n += snprintf(buf + n, kBuf - n,
      " CAN   %s %-8s#  |  GPS    %s %-9s#\n",
      canCol, canStat,
      gpsCol, gpsStat);

  n += snprintf(buf + n, kBuf - n,
      " METH  %s %-8s#  |  TAILS  %s %-9s#\n",
      col(s.meth_online), s.meth_online ? "ONLINE" : "OFFLINE",
      col(s.taillight_online), s.taillight_online ? "ONLINE" : "OFFLINE");

  n += snprintf(buf + n, kBuf - n,
      " SD    %s %-8s#  |  TOUCH  %s %-9s#\n",
      col(s.sd_mounted), s.sd_mounted ? "MOUNTED" : "NO CARD",
      col(s.touch_online), s.touch_online ? "OK" : "OFFLINE");

  n += snprintf(buf + n, kBuf - n, "\n");

  // ── System ───────────────────────────────────────────────────────────────
  n += snprintf(buf + n, kBuf - n,
      "SYS   Heap %luK(min %luK)  Die %dC  Up %lus  Rst %s  Flt:%u\n",
      static_cast<unsigned long>(s.heap_free_bytes / 1024),
      static_cast<unsigned long>(heapMin / 1024),
      static_cast<int>(s.esp_die_temp_c),
      static_cast<unsigned long>(s.uptime_ms / 1000UL),
      resetStr,
      static_cast<unsigned>(s.fault_flags));

  // ── CAN bus ───────────────────────────────────────────────────────────────
  n += snprintf(buf + n, kBuf - n,
      "CAN   RX %lu  TX %lu  CRC %lu  LastRX 0x%03X %lus ago\n",
      static_cast<unsigned long>(s.can_rx_count),
      static_cast<unsigned long>(s.can_tx_count),
      static_cast<unsigned long>(s.can_bad_checksum_count),
      static_cast<unsigned>(s.can_last_rx_id),
      static_cast<unsigned long>(msAgo(s.can_last_rx_ms)));

  // ── Water-meth ────────────────────────────────────────────────────────────
  n += snprintf(buf + n, kBuf - n,
      "METH  Duty %u%%  Flow %s  Tank %s  Ratio %u%%  St %s\n",
      static_cast<unsigned>(s.meth_pump_duty),
      methFlowName(s.meth_flow_status),
      (s.meth_tank_level == 0) ? "EMPTY" : "OK",
      static_cast<unsigned>(s.meth_selected_ratio_percent),
      methStateName(s.meth_state));

  // ── Taillights ───────────────────────────────────────────────────────────
  n += snprintf(buf + n, kBuf - n,
      "TAILS L:%s  R:%s  Bri:%u  Mode:%s  Die:%dC\n",
      s.taillight_left_state  ? "ON" : "OFF",
      s.taillight_right_state ? "ON" : "OFF",
      static_cast<unsigned>(s.taillight_brightness),
      tailModeName(s.taillight_mode_commanded),
      static_cast<int>(s.taillight_die_temp_c));

  // ── GPS ──────────────────────────────────────────────────────────────────
  n += snprintf(buf + n, kBuf - n,
      "GPS   Used:%u View:%u Q:%u M:%u H:%.1f  Alt:%dm  Lat:%.4f  Lon:%.4f\n",
      static_cast<unsigned>(s.gps_satellites),
      static_cast<unsigned>(s.gps_satellites_in_view),
      static_cast<unsigned>(s.gps_fix_quality),
      static_cast<unsigned>(s.gps_fix_mode),
      static_cast<double>(s.gps_hdop_x10 / 10.0f),
      static_cast<int>(s.gps_altitude_m),
      s.gps_latitude,
      s.gps_longitude);

  // ── Knock ─────────────────────────────────────────────────────────────────
  n += snprintf(buf + n, kBuf - n,
      "KNOCK E:%.1f  B:%.1f  T:%.1f  Ev:%u  Sig:%s  Mode:%u\n",
      static_cast<double>(s.knock_energy),
      static_cast<double>(s.knock_baseline),
      static_cast<double>(s.knock_threshold),
      static_cast<unsigned>(s.knock_event_count),
      s.knock_signal_valid ? "OK" : "FAULT",
      static_cast<unsigned>(s.knock_response_mode));

  // ── Sensors ───────────────────────────────────────────────────────────────
  n += snprintf(buf + n, kBuf - n,
      "SENS  IAT:%.0f  Bay:%.0f  Oil:%.0fpsi  Boost:%.0fpsi  FPS:%.1f",
      static_cast<double>(s.intake_temp),
      static_cast<double>(s.engine_bay_temp),
      static_cast<double>(s.oil_pressure_psi),
      static_cast<double>(s.boost_ref_pressure_psi),
      static_cast<double>(s.ui_fps));

  // Append bench test mode indicator line
  n += snprintf(buf + n, kBuf - n, "\n%sBENCH TEST MODE: %s#",
      s.bench_test_mode ? "#FF8C00 " : "#607D8B ",
      s.bench_test_mode ? "ON" : "OFF");

  setLabelText(diagLabel_, buf);

  // Keep button label and colour in sync with runtime state
  if (benchTestBtn_) {
    setBgColor(benchTestBtn_,
        s.bench_test_mode ? lv_color_hex(0xFF8C00) : lv_color_hex(0x1565C0),
        LV_PART_MAIN);
    setLabelText(btnLabel(benchTestBtn_),
        s.bench_test_mode ? "BENCH: ON" : "BENCH: OFF");
  }

  refreshSdBrowser(now, false);
}

void ScreenDashboard::updateKnockPage(const state::VehicleState& s, uint32_t nowMs) {
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
    snprintf(buf, sizeof(buf), "#FF3B30 KNOCK SENSE#  |  %s  |  Response: %s",
             s.knock_online ? "ONLINE" : "STALE", respStr);
  } else if (s.knock_warning_active) {
    snprintf(buf, sizeof(buf), "#FF9500 KNOCK SENSE#  |  %s  |  Response: %s",
             s.knock_online ? "ONLINE" : "STALE", respStr);
  } else {
    snprintf(buf, sizeof(buf), "#00C853 KNOCK SENSE#  |  %s  |  Response: %s",
             s.knock_online ? "ONLINE" : "STALE", respStr);
  }
  setLabelText(knockStateLabel_, buf);

  // Sensor status row
  snprintf(buf, sizeof(buf), "En: %s  Sensor: %s  Learned: %s  Clip Hi: %u Lo: %u",
           s.knock_enabled ? "YES" : "NO",
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
  else                      eCo = lv_palette_main(LV_PALETTE_BLUE);
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
  if (knockWarnLed_) {
    if (s.knock_warning_active) lv_led_on(knockWarnLed_);
    else lv_led_off(knockWarnLed_);
  }
  if (knockCritLed_) {
    if (s.knock_critical_active) lv_led_on(knockCritLed_);
    else lv_led_off(knockCritLed_);
  }
  if (knockLearningSpinner_) {
    setObjHidden(knockLearningSpinner_, !(s.knock_enabled && !s.knock_baseline_learned));
  }

  // Events / warning / critical row
  if (s.knock_critical_active) {
    snprintf(buf, sizeof(buf), "#FF3B30 Events: %u#  |  #FF3B30 WARNING: YES#  |  #FF3B30 CRITICAL: YES#",
             static_cast<unsigned>(s.knock_event_count));
  } else if (s.knock_warning_active) {
    snprintf(buf, sizeof(buf), "#FF9500 Events: %u#  |  #FF9500 WARNING: YES#  |  Critical: NO",
             static_cast<unsigned>(s.knock_event_count));
  } else {
    snprintf(buf, sizeof(buf), "Events: %u  |  Warning: NO  |  Critical: NO",
             static_cast<unsigned>(s.knock_event_count));
  }
  setLabelText(knockEventLabel_, buf);

  // Last event row
  if (s.knock_last_event_rpm == 0 && s.knock_last_event_boost_kpa == 0) {
    setLabelText(knockLastLabel_, "Last event: none");
  } else {
    uint32_t agoS = 0;
    if (s.knock_last_event_time_ms > 0 && nowMs >= s.knock_last_event_time_ms) {
      agoS = (nowMs - s.knock_last_event_time_ms) / 1000UL;
    }
    snprintf(buf, sizeof(buf), "Last: %u RPM  %.0f kPa  %d\xb0""C  (%lus ago)",
             static_cast<unsigned>(s.knock_last_event_rpm),
             static_cast<double>(s.knock_last_event_boost_kpa),
             static_cast<int>(s.knock_last_event_iat_c),
             static_cast<unsigned long>(agoS));
    setLabelText(knockLastLabel_, buf);
  }

  // Enable button label
  setLabelText(knockEnableBtnLabel_, s.knock_enabled ? "DISABLE" : "ENABLE");
  // Dim simulate button if not in demo/dev mode
  const bool demoActive = s.knock_demo_mode_enabled;
#if defined(DEMO_MODE) && (DEMO_MODE == 1)
  (void)demoActive;
  setBgColor(knockSimulateBtn_, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);
#else
  setBgColor(knockSimulateBtn_,
      demoActive ? lv_palette_main(LV_PALETTE_ORANGE) : lv_color_hex(0x303030),
      LV_PART_MAIN);
#endif

  // Logging / note row
  snprintf(buf, sizeof(buf), "IAT %.1fC  Meth %.1fpsi  Fuel %.1fpsi  Oil %.1fpsi  | SD:%s Log:%s  |  \xe2\x80\xa0 Not ECU knock control",
           static_cast<double>(s.intake_temp),
           static_cast<double>(s.meth_pressure_psi),
           static_cast<double>(s.fuel_pressure_psi),
           static_cast<double>(s.oil_pressure_psi),
           s.sd_mounted ? "YES" : "NO",
            s.knock_logging_active ? "YES" : "NO");
  setLabelText(knockLogLabel_, buf);
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
  if (bottomEdgeGuard_) {
    lv_obj_move_foreground(bottomEdgeGuard_);
    lv_obj_invalidate(bottomEdgeGuard_);
  }
  if (kFullRepaintOnPageSwitch) {
    lv_obj_invalidate(lv_scr_act());
    return;
  }
  if (contentArea_) {
    lv_obj_invalidate(contentArea_);
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

uint8_t ScreenDashboard::nextMethRatio(uint8_t current) const {
  // Cycle in 5 % steps: 5 → 10 → 15 → … → 100 → 5
  const uint8_t rounded = static_cast<uint8_t>((current / 5U) * 5U);
  const uint8_t next    = static_cast<uint8_t>(rounded + 5U);
  return (next > 100U) ? 5U : next;
}

touch::TouchSample ScreenDashboard::normalizeRaw(const touch::TouchSample& raw) const {
  if (!raw.touched) return raw;
  touch::TouchSample t = raw;

  // Some FT62xx controllers report in a 480x320 landscape frame while the
  // dashboard renders in 320x480 portrait. Rotate into portrait space.
  if (t.x <= kHeight && t.y <= kWidth) {
    const uint16_t x = t.x;
    t.x = t.y;
    t.y = static_cast<uint16_t>(kHeight > x ? (kHeight - x) : 0U);
  }

  // Fallback: scale raw 12-bit (0..4095) ranges down to pixel coordinates.
  if (t.x > kWidth)  t.x = static_cast<uint16_t>((static_cast<uint32_t>(t.x) * kWidth)  / 4095U);
  if (t.y > kHeight) t.y = static_cast<uint16_t>((static_cast<uint32_t>(t.y) * kHeight) / 4095U);
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
  if (!force && sdBrowserLastRefreshMs_ != 0 &&
      static_cast<uint32_t>(nowMs - sdBrowserLastRefreshMs_) < 3000U) {
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
        setBgColor(sdFileRows_[i], lv_color_hex(0x101826), LV_PART_MAIN);
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
        setBgColor(sdFileRows_[i], lv_color_hex(0x101826), LV_PART_MAIN);
      }
      continue;
    }

    char line[72];
    if (sdEntries_[i].isDirectory) {
      snprintf(line, sizeof(line), "[DIR] %s", sdEntries_[i].name);
      if (sdFileRows_[i]) {
        setBgColor(sdFileRows_[i], lv_color_hex(0x173757), LV_PART_MAIN);
      }
    } else {
      char sizeBuf[12];
      formatFileSize(sdEntries_[i].sizeBytes, sizeBuf, sizeof(sizeBuf));
      snprintf(line, sizeof(line), "[FILE] %-36s %s", sdEntries_[i].name, sizeBuf);
      if (sdFileRows_[i]) {
        setBgColor(sdFileRows_[i], lv_color_hex(0x101826), LV_PART_MAIN);
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
  if (!self->canMgr_) return;
  const bool arm = !state::g_vehicle_state.read().meth_desired_armed;
  if (self->canMgr_->sendMethArm(arm)) {
    state::g_vehicle_state.mutate([arm](state::VehicleState& vs) { vs.meth_desired_armed = arm; });
    self->setActionFeedback(arm ? "METH ON" : "METH OFF", millis());
  } else {
    self->setActionFeedback("METH CMD REJECTED", millis());
  }
}

void ScreenDashboard::onMethRatioClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  const uint8_t ratio = self->nextMethRatio(state::g_vehicle_state.read().meth_selected_ratio_percent);
  state::g_vehicle_state.mutate([ratio](state::VehicleState& vs) { vs.meth_selected_ratio_percent = ratio; });
  if (self->settingsMgr_) {
    self->settingsMgr_->updateFromState(state::g_vehicle_state.read());
    self->settingsMgr_->save();
  }
  if (self->canMgr_) self->canMgr_->sendMethConfigBroadcast();
  self->setActionFeedback("METH RATIO UPDATED", millis());
}

void ScreenDashboard::onTailStockClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self->canMgr_) return;
  self->canMgr_->sendTaillightMode(can_protocol::taillight_mode::STOCK)
      ? self->setActionFeedback("TAIL STOCK", millis())
      : self->setActionFeedback("TAIL CMD REJECTED", millis());
}

void ScreenDashboard::onTailSeqClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self->canMgr_) return;
  self->canMgr_->sendTaillightMode(can_protocol::taillight_mode::SEQUENTIAL)
      ? self->setActionFeedback("TAIL SEQUENTIAL", millis())
      : self->setActionFeedback("TAIL CMD REJECTED", millis());
}

void ScreenDashboard::onTailDemoClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self->canMgr_) return;
  self->canMgr_->sendTaillightMode(can_protocol::taillight_mode::DEMO)
      ? self->setActionFeedback("TAIL DEMO", millis())
      : self->setActionFeedback("TAIL CMD REJECTED", millis());
}

void ScreenDashboard::onTailShowMenuClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  self->tailShowPage_ = 0;
  lv_obj_add_flag(self->tailModePanel_,   LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(self->tailShowPanel_, LV_OBJ_FLAG_HIDDEN);
  if (self->pages_[2]) lv_obj_invalidate(self->pages_[2]);
  self->setActionFeedback("SHOW MENU", millis());
}

void ScreenDashboard::onTailShowPrevClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  self->tailShowPage_ = (self->tailShowPage_ == 0)
      ? static_cast<uint8_t>(kTaillightShowPageCount - 1U)
      : static_cast<uint8_t>(self->tailShowPage_ - 1U);
  if (self->tailShowPanel_) lv_obj_invalidate(self->tailShowPanel_);
  self->setActionFeedback("SHOW PAGE", millis());
}

void ScreenDashboard::onTailShowNextClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  self->tailShowPage_ = static_cast<uint8_t>((self->tailShowPage_ + 1U) % kTaillightShowPageCount);
  if (self->tailShowPanel_) lv_obj_invalidate(self->tailShowPanel_);
  self->setActionFeedback("SHOW PAGE", millis());
}

void ScreenDashboard::onTailShowBackClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  lv_obj_clear_flag(self->tailModePanel_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(self->tailShowPanel_,   LV_OBJ_FLAG_HIDDEN);
  if (self->pages_[2]) lv_obj_invalidate(self->pages_[2]);
  self->setActionFeedback("SHOW MENU EXIT", millis());
}

void ScreenDashboard::onTailShowOptClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  lv_obj_t* btn = lv_event_get_target(e);

  uint8_t idx = kTaillightShowOptionsPerPage;
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    if (self->tailShowOptBtns_[i] == btn) { idx = i; break; }
  }
  if (idx >= kTaillightShowOptionsPerPage) return;

  const uint16_t optVal = static_cast<uint16_t>(self->tailShowPage_) * kTaillightShowOptionsPerPage + idx;
  if (optVal >= kTaillightShowOptionCount) {
    self->setActionFeedback("SHOW SLOT EMPTY", millis());
    return;
  }
  if (!self->canMgr_) return;
  const uint8_t option = static_cast<uint8_t>(optVal);
  static constexpr const char* kTailShowFbNames[] = {
    "Rainbow", "Chase", "Theater", "Fire", "Meteor", "Police",
    "Night Rider", "Color Cycle", "Sparkle", "Plasma", "Matrix",
    "Juggle", "BPM", "Confetti", "Ocean", "Lightning", "Heartbeat",
    "Ripple", "Sunrise", "Text Scroll", "Colorwaves", "TwinkleFox",
    "Bounce", "Fireworks", "Drip", "Cylon", "V8", "Drag Launch",
    "Neon", "Streaks", "Radar", "Aurora", "Glitch",
  };
  if (self->canMgr_->sendTaillightShowOption(option)) {
    const char* name = (option < kTaillightShowOptionCount)
        ? kTailShowFbNames[option] : "?";
    self->setActionFeedback(name, millis());
  } else {
    self->setActionFeedback("SHOW CMD REJECTED", millis());
  }
}

void ScreenDashboard::onRaceAccelClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (self->raceMgr_) {
    self->raceMgr_->startRun(state::RaceMode::ACCEL);
    self->setActionFeedback("RACE ACCEL START", millis());
  }
}

void ScreenDashboard::onRaceLapClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (self->raceMgr_) {
    self->raceMgr_->startRun(state::RaceMode::LAP);
    self->setActionFeedback("RACE LAP START", millis());
  }
}

void ScreenDashboard::onRaceStopClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (self->raceMgr_) {
    self->raceMgr_->stopRun();
    self->setActionFeedback("RACE STOPPED", millis());
  }
}

void ScreenDashboard::onRaceResetClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (self->raceMgr_) {
    self->raceMgr_->resetSession();
    self->setActionFeedback("RACE RESET", millis());
  }
}

void ScreenDashboard::onNavClicked(lv_event_t* e) {
  auto* ctx = static_cast<NavCtx*>(lv_event_get_user_data(e));
  if (!ctx || !ctx->self) return;
  ctx->self->showPage(ctx->page);
}

void ScreenDashboard::onLedModeClicked(lv_event_t* e) {
  auto* ctx = static_cast<LedModeCtx*>(lv_event_get_user_data(e));
  if (!ctx || !ctx->self) return;
  const state::LedMode mode = ctx->mode;
  state::g_vehicle_state.mutate([mode](state::VehicleState& vs) {
    vs.led_channel_1_mode = mode;
    vs.led_channel_2_mode = mode;
    vs.led_channel_3_mode = mode;
  });
  if (ctx->self->settingsMgr_) {
    ctx->self->settingsMgr_->updateFromState(state::g_vehicle_state.read());
    ctx->self->settingsMgr_->save();
  }
  ctx->self->setActionFeedback("LED MODE SET", millis());
}

void ScreenDashboard::onLedMasterSwitchChanged(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) return;
  lv_obj_t* sw = lv_event_get_target(e);
  const bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
  state::g_vehicle_state.mutate([enabled](state::VehicleState& vs) {
    vs.led_channel_1_enabled = enabled;
    vs.led_channel_2_enabled = enabled;
    vs.led_channel_3_enabled = enabled;
  });
  if (self->settingsMgr_) {
    self->settingsMgr_->updateFromState(state::g_vehicle_state.read());
    self->settingsMgr_->save();
  }
  self->setActionFeedback(enabled ? "LED MASTER ON" : "LED MASTER OFF", millis());
}

void ScreenDashboard::onKnockEnableClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  const bool en = !state::g_vehicle_state.read().knock_enabled;
  state::g_vehicle_state.mutate([en](state::VehicleState& vs) { vs.knock_enabled = en; });
  if (self->settingsMgr_) {
    self->settingsMgr_->updateFromState(state::g_vehicle_state.read());
    self->settingsMgr_->save();
  }
  self->setActionFeedback(en ? "KNOCK ENABLED" : "KNOCK DISABLED", millis());
}

void ScreenDashboard::onKnockResetBaselineClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  state::g_vehicle_state.mutate([](state::VehicleState& vs) {
    vs.knock_reset_baseline_request = true;
  });
  self->setActionFeedback("KNOCK BL RESET", millis());
}

void ScreenDashboard::onKnockClearEventsClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  state::g_vehicle_state.mutate([](state::VehicleState& vs) {
    vs.knock_clear_event_count_request = true;
  });
  self->setActionFeedback("KNOCK EVT CLEARED", millis());
}

void ScreenDashboard::onKnockSimulateClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
#if defined(DEMO_MODE) && (DEMO_MODE == 1)
  state::g_vehicle_state.mutate([](state::VehicleState& vs) {
    vs.knock_simulate_event_request = true;
  });
  self->setActionFeedback("KNOCK SIM TRIGGERED", millis());
#else
  if (state::g_vehicle_state.read().knock_demo_mode_enabled) {
    state::g_vehicle_state.mutate([](state::VehicleState& vs) {
      vs.knock_simulate_event_request = true;
    });
    self->setActionFeedback("KNOCK SIM TRIGGERED", millis());
  } else {
    self->setActionFeedback("DEMO MODE ONLY", millis());
  }
#endif
}

void ScreenDashboard::onBenchTestClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  const bool on = !state::g_vehicle_state.read().bench_test_mode;
  state::g_vehicle_state.mutate([on](state::VehicleState& vs) {
    vs.bench_test_mode = on;
  });
  self->setActionFeedback(on ? "BENCH TEST ON" : "BENCH TEST OFF", millis());
}

void ScreenDashboard::onSdFileRowClicked(lv_event_t* e) {
  auto* ctx = static_cast<SdFileRowCtx*>(lv_event_get_user_data(e));
  if (!ctx || !ctx->self || ctx->row >= ctx->self->sdEntryCount_) {
    return;
  }

  ScreenDashboard* self = ctx->self;
  const storage::SdFileEntry& entry = self->sdEntries_[ctx->row];
  const uint32_t now = millis();
  if (entry.isDirectory) {
    self->enterSdDirectory(entry.name, now);
    return;
  }

  char msg[48];
  snprintf(msg, sizeof(msg), "SD FILE: %.36s", entry.name);
  self->setActionFeedback(msg, now);
}

void ScreenDashboard::onSdUpClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) {
    return;
  }
  const uint32_t now = millis();
  self->setSdPathParent();
  self->refreshSdBrowser(now, true);
  self->setActionFeedback("SD UP", now);
}

void ScreenDashboard::onSdPrevClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) {
    return;
  }
  if (self->sdListOffset_ >= kSdFileRowCount) {
    self->sdListOffset_ = static_cast<uint16_t>(self->sdListOffset_ - kSdFileRowCount);
  } else {
    self->sdListOffset_ = 0;
  }
  const uint32_t now = millis();
  self->refreshSdBrowser(now, true);
  self->setActionFeedback("SD PREV", now);
}

void ScreenDashboard::onSdNextClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) {
    return;
  }
  if (self->sdListOffset_ + kSdFileRowCount < self->sdTotalEntries_) {
    self->sdListOffset_ = static_cast<uint16_t>(self->sdListOffset_ + kSdFileRowCount);
  }
  const uint32_t now = millis();
  self->refreshSdBrowser(now, true);
  self->setActionFeedback("SD NEXT", now);
}

void ScreenDashboard::onSdTestClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self) {
    return;
  }

  const uint32_t now = millis();
  if (!self->sdMgr_ || !self->sdMgr_->mounted()) {
    self->setActionFeedback("SD NOT MOUNTED", now);
    self->refreshSdBrowser(now, true);
    return;
  }

  char line[64];
  snprintf(line, sizeof(line), "sd_browser_test_ms=%lu", static_cast<unsigned long>(now));
  const bool ok = self->sdMgr_->ensureFolder("/logs") &&
                  self->sdMgr_->appendLine("/logs/sd_check.txt", line);
  if (ok) {
    snprintf(self->sdCurrentPath_, sizeof(self->sdCurrentPath_), "/logs");
    self->sdListOffset_ = 0;
    self->setActionFeedback("SD TEST OK", now);
  } else {
    self->setActionFeedback("SD TEST FAIL", now);
  }
  self->refreshSdBrowser(now, true);
}

}  // namespace ui
