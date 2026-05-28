#include "ui/screen_dashboard.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#if __has_include(<Arduino_GFX_Library.h>)
#include <Arduino_GFX_Library.h>
#define CCM_HAS_ARDUINO_GFX 1
#else
#define CCM_HAS_ARDUINO_GFX 0
#endif

#include "can/can_protocol.h"
#include "state/vehicle_state.h"

namespace ui {

// ---------------------------------------------------------------------------
// Static module-level storage
// ---------------------------------------------------------------------------

#if CCM_HAS_ARDUINO_GFX
static Arduino_DataBus* s_bus = nullptr;
static Arduino_GFX*     s_gfx = nullptr;
#endif

// LVGL draw buffers – two 480×40 slices (~38 KB each, double-buffered).
// Doubling the band height halves the number of SPI flush calls per full-screen
// redraw (8 instead of 16), which eliminates most visible tearing.
static lv_color_t s_buf1[480 * 40];
static lv_color_t s_buf2[480 * 40];

// Context structs for callbacks that need both self-pointer and a small value.
// Static storage – one ScreenDashboard instance only, set during buildXxxPage().
struct NavCtx     { ScreenDashboard* self; uint8_t page; };
struct LedModeCtx { ScreenDashboard* self; state::LedMode mode; };

static NavCtx     s_navCtxs[8];
static LedModeCtx s_ledModeCtxs[5];

constexpr const char* kStatusColorOn  = "#00C853";
constexpr const char* kStatusColorOff = "#FF3B30";
constexpr float kGpsLowSpeedThresholdMph = 12.0f;
constexpr float kGpsLowSpeedFilterAlpha  = 0.18f;
constexpr float kGpsHighSpeedFilterAlpha = 0.35f;
constexpr float kGpsZeroClampMph         = 1.0f;
constexpr uint32_t kMethSensorStaleMs    = 1000;

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

void styleCard(lv_obj_t* obj) {
  if (!obj) return;
  lv_obj_set_style_radius(obj, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(obj, lv_color_hex(0x0f1724), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(obj, lv_color_hex(0x27405d), LV_PART_MAIN);
  lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(obj, 8, LV_PART_MAIN);
  lv_obj_set_style_shadow_color(obj, lv_color_hex(0x04090f), LV_PART_MAIN);
}

void styleChip(lv_obj_t* obj) {
  if (!obj) return;
  lv_obj_set_style_radius(obj, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(obj, lv_color_hex(0x1a2538), LV_PART_MAIN);
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
  lv_obj_set_style_shadow_width(obj, 8, LV_PART_MAIN);
  lv_obj_set_style_shadow_color(obj, lv_color_hex(0x050c16), LV_PART_MAIN);
}

void animSetOpa(void* obj, int32_t v) {
  lv_obj_set_style_opa(static_cast<lv_obj_t*>(obj), static_cast<lv_opa_t>(v), LV_PART_MAIN);
}

void animSetY(void* obj, int32_t v) {
  lv_obj_set_y(static_cast<lv_obj_t*>(obj), static_cast<lv_coord_t>(v));
}

void animatePageEnter(lv_obj_t* page) {
  if (!page) return;
  lv_obj_set_style_opa(page, LV_OPA_0, LV_PART_MAIN);
  lv_obj_set_y(page, 6);

  lv_anim_t aOpa;
  lv_anim_init(&aOpa);
  lv_anim_set_var(&aOpa, page);
  lv_anim_set_values(&aOpa, LV_OPA_0, LV_OPA_COVER);
  lv_anim_set_time(&aOpa, 160);
  lv_anim_set_exec_cb(&aOpa, animSetOpa);
  lv_anim_start(&aOpa);

  lv_anim_t aY;
  lv_anim_init(&aY);
  lv_anim_set_var(&aY, page);
  lv_anim_set_values(&aY, 6, 0);
  lv_anim_set_time(&aY, 160);
  lv_anim_set_exec_cb(&aY, animSetY);
  lv_anim_start(&aY);
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
#if CCM_HAS_ARDUINO_GFX
  // ILI9488 uses 18-bit SPI (R8 G8 B8, top 6 bits each). Arduino_GFX
  // draw16bitRGBBitmap() converts RGB565 → 18-bit internally per pixel.
  // LV_COLOR_16_SWAP must be 0 so the 16-bit values arrive un-swapped.
  if (s_gfx) {
    const int32_t w = area->x2 - area->x1 + 1;
    const int32_t h = area->y2 - area->y1 + 1;
    s_gfx->draw16bitRGBBitmap(area->x1, area->y1, reinterpret_cast<uint16_t*>(colors), w, h);
  }
#else
  (void)area;
  (void)colors;
#endif
  lv_disp_flush_ready(drv);
}

void ScreenDashboard::lvglTouchReadCb(lv_indev_drv_t* drv, lv_indev_data_t* data) {
  auto* self = static_cast<ScreenDashboard*>(drv->user_data);
  if (self->lastTouch_.touched) {
    data->point.x = static_cast<lv_coord_t>(self->lastTouch_.x);
    data->point.y = static_cast<lv_coord_t>(self->lastTouch_.y);
    data->state   = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void ScreenDashboard::attach(canbus::CanManager* canMgr, race::RacePerformanceManager* raceMgr,
                              settings::SettingsManager* settingsMgr) {
  canMgr_      = canMgr;
  raceMgr_     = raceMgr;
  settingsMgr_ = settingsMgr;
}

bool ScreenDashboard::begin(uint8_t lcdCs, uint8_t lcdRst, uint8_t lcdDc,
                             uint8_t spiSck, uint8_t spiMosi, uint8_t spiMiso) {
#if CCM_HAS_ARDUINO_GFX
  // Use HSPI (SPI3) for the display so it never shares a host with the
  // Arduino SPI class / SD library (which both use FSPI / SPI2 by default).
  // This avoids all spi_bus_initialize() conflicts regardless of init order.
  s_bus = new Arduino_ESP32SPI(lcdDc, lcdCs, spiSck, spiMosi, spiMiso, HSPI);
  // Chip confirmed ILI9488 (mislabeled as ST7796S on this Hosyond module).
  // rotation=1: 90° CW landscape (480×320).
  s_gfx = new Arduino_ILI9488(s_bus, lcdRst, 1 /*rotation 90°CW*/, false /*ips*/);
  // 20 MHz: ILI9488 max SPI rate; full-screen refresh ~90 ms.
  if (!s_gfx || !s_gfx->begin(20000000UL)) {
    Serial0.println("[SCREEN] GFX begin FAILED");
    return false;
  }
  Serial0.println("[SCREEN] GFX begin OK");
  s_gfx->fillScreen(0x0000U);  // clear to black before LVGL builds first frame
#else
  (void)lcdCs; (void)lcdRst; (void)lcdDc;
  (void)spiSck; (void)spiMosi; (void)spiMiso;
#endif

  // ---- LVGL init ----
  lv_init();

  // Display driver
  static lv_disp_draw_buf_t drawBuf;
  lv_disp_draw_buf_init(&drawBuf, s_buf1, s_buf2, 480 * 40);

  static lv_disp_drv_t dispDrv;
  lv_disp_drv_init(&dispDrv);
  dispDrv.hor_res     = static_cast<lv_coord_t>(kWidth);
  dispDrv.ver_res     = static_cast<lv_coord_t>(kHeight);
  dispDrv.flush_cb    = lvglFlushCb;
  dispDrv.draw_buf    = &drawBuf;
  lv_disp_drv_register(&dispDrv);

  // Touch input device
  static lv_indev_drv_t indevDrv;
  lv_indev_drv_init(&indevDrv);
  indevDrv.type      = LV_INDEV_TYPE_POINTER;
  indevDrv.read_cb   = lvglTouchReadCb;
  indevDrv.user_data = this;
  lv_indev_drv_register(&indevDrv);

  // Dark theme
  lv_theme_t* theme = lv_theme_default_init(
      lv_disp_get_default(),
      lv_palette_main(LV_PALETTE_BLUE),
      lv_palette_main(LV_PALETTE_CYAN),
      true  /* dark mode */,
      &lv_font_montserrat_16);
  lv_disp_set_theme(lv_disp_get_default(), theme);

  buildUi();
  online_ = true;
  return true;
}

void ScreenDashboard::tick(const state::VehicleState& s, uint32_t nowMs) {
  if (!online_) return;
  updateHeader(s, nowMs);
  // Only update the currently-visible page.  Calling lv_label_set_text on
  // hidden objects still marks them dirty in LVGL's internal region tracker,
  // wasting CPU on bookkeeping for widgets that will never be flushed.
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
  lv_task_handler();
}

void ScreenDashboard::handleTouch(const touch::TouchSample& sample, uint32_t /*nowMs*/) {
  // Normalize raw coordinates and buffer for the LVGL indev driver.
  lastTouch_ = normalizeRaw(sample);

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
  lv_label_set_text(lbl, text);
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
  lv_label_set_text(lbl, text);
  lv_obj_center(lbl);
  if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, userData);
  return btn;
}

// Returns the label child of a button (first child).
static lv_obj_t* btnLabel(lv_obj_t* btn) {
  return lv_obj_get_child(btn, 0);
}

// ---------------------------------------------------------------------------
// buildUi – entry point: header + content panels + nav bar
// ---------------------------------------------------------------------------

void ScreenDashboard::buildUi() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
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
  lv_obj_set_style_bg_color(hdr, lv_color_hex(0x0d1520), LV_PART_MAIN);
  lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);
  styleCard(hdr);
  lv_obj_set_style_radius(hdr, 0, LV_PART_MAIN);

  // Left: battery voltage
  hdrBatLabel_ = lv_label_create(hdr);
  lv_label_set_text(hdrBatLabel_, "12.0V");
  lv_obj_set_style_text_font(hdrBatLabel_, &lv_font_montserrat_16, 0);
  lv_obj_align(hdrBatLabel_, LV_ALIGN_LEFT_MID, 8, 0);

  // Center: active page name (cyan accent)
  hdrTitleLabel_ = lv_label_create(hdr);
  lv_label_set_text(hdrTitleLabel_, "DASH");
  lv_obj_set_style_text_font(hdrTitleLabel_, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(hdrTitleLabel_, lv_palette_main(LV_PALETTE_CYAN), 0);
  lv_obj_align(hdrTitleLabel_, LV_ALIGN_CENTER, 0, 0);

  // Right: action feedback (green)
  hdrFeedbackLabel_ = lv_label_create(hdr);
  lv_label_set_text(hdrFeedbackLabel_, "");
  lv_obj_set_style_text_font(hdrFeedbackLabel_, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(hdrFeedbackLabel_, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_align(hdrFeedbackLabel_, LV_ALIGN_RIGHT_MID, -30, 0);

  // Far right: fault status dot (circle label)
  hdrFaultDot_ = lv_label_create(hdr);
  lv_label_set_text(hdrFaultDot_, LV_SYMBOL_STOP);
  lv_obj_set_style_text_color(hdrFaultDot_, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_set_style_text_font(hdrFaultDot_, &lv_font_montserrat_14, 0);
  lv_obj_align(hdrFaultDot_, LV_ALIGN_RIGHT_MID, -8, 0);
}

// ---------------------------------------------------------------------------
// buildContentArea – creates 7 page panels at (0, kHdrH), size 320 × kContentH
// ---------------------------------------------------------------------------

void ScreenDashboard::buildContentArea(lv_obj_t* scr) {
  // Container for all page panels (transparent pass-through)
  lv_obj_t* cont = lv_obj_create(scr);
  lv_obj_set_pos(cont, 0, kHdrH);
  lv_obj_set_size(cont, kWidth, kContentH);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(cont, 0, LV_PART_MAIN);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  for (uint8_t i = 0; i < kPageCount; i++) {
    lv_obj_t* pg = lv_obj_create(cont);
    lv_obj_set_pos(pg, 0, 0);
    lv_obj_set_size(pg, kWidth, kContentH);
    lv_obj_set_style_bg_color(pg, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(pg, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(pg, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pg, 4, LV_PART_MAIN);
    lv_obj_add_flag(pg, LV_OBJ_FLAG_HIDDEN);
    styleCard(pg);
    // In landscape, all pages except DASH (0) get vertical scroll.
    if (i == 0) lv_obj_clear_flag(pg, LV_OBJ_FLAG_SCROLLABLE);
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
  lv_obj_set_style_bg_color(bar, lv_color_hex(0x0d1520), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(bar, 0, LV_PART_MAIN);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
  styleCard(bar);
  lv_obj_set_style_radius(bar, 0, LV_PART_MAIN);

  // Divider line between content and nav bar
  lv_obj_t* sep = lv_obj_create(scr);
  lv_obj_set_pos(sep, 0, static_cast<lv_coord_t>(kHdrH + kContentH));
  lv_obj_set_size(sep, kWidth, 1);
  lv_obj_set_style_bg_color(sep, lv_color_hex(0x2a3a50), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(sep, 0, LV_PART_MAIN);

  // 7 buttons, each 45 px wide (last one gets remaining 5 px)
  constexpr lv_coord_t btnW = static_cast<lv_coord_t>(kWidth / kPageCount);  // 45
  for (uint8_t i = 0; i < kPageCount; i++) {
    s_navCtxs[i] = {this, i};

    const lv_coord_t bx = static_cast<lv_coord_t>(i * btnW);
    const lv_coord_t bw = (i == kPageCount - 1)
                          ? static_cast<lv_coord_t>(kWidth - bx)
                          : btnW;

    lv_obj_t* btn = lv_btn_create(bar);
    lv_obj_set_pos(btn, bx, 0);
    lv_obj_set_size(btn, bw, kNavH);
    lv_obj_set_style_radius(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x1a2540), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN);
    styleActionButton(btn);
    lv_obj_add_event_cb(btn, onNavClicked, LV_EVENT_CLICKED, &s_navCtxs[i]);
    navBtns_[i] = btn;

    // Symbol icon (top half of button)
    lv_obj_t* icon = lv_label_create(btn);
    lv_label_set_text(icon, kSymbols[i]);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_20, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -10);
    navBtnIcons_[i] = icon;

    // Text label (bottom half of button)
    lv_obj_t* txt = lv_label_create(btn);
    lv_label_set_text(txt, kNames[i]);
    lv_obj_set_style_text_font(txt, &lv_font_montserrat_14, 0);
    lv_obj_align(txt, LV_ALIGN_CENTER, 0, +14);
  }
}

// ---------------------------------------------------------------------------
// showPage – hide all pages, reveal the requested one, highlight nav button
// ---------------------------------------------------------------------------

void ScreenDashboard::showPage(uint8_t idx) {
  if (idx >= kPageCount) return;
  static const char* const kNames[8] = {
    "DASH", "METH", "TAIL", "LEDS", "GPS", "TEMPS", "DIAG", "KNOCK"
  };
  for (uint8_t i = 0; i < kPageCount; i++) {
    if (i == idx) {
      lv_obj_clear_flag(pages_[i], LV_OBJ_FLAG_HIDDEN);
      animatePageEnter(pages_[i]);
      if (navBtns_[i]) {
        lv_obj_set_style_bg_color(navBtns_[i], lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
      }
    } else {
      lv_obj_add_flag(pages_[i], LV_OBJ_FLAG_HIDDEN);
      if (navBtns_[i]) {
        lv_obj_set_style_bg_color(navBtns_[i], lv_color_hex(0x1a2540), LV_PART_MAIN);
      }
    }
  }
  activePage_ = idx;
  if (hdrTitleLabel_) lv_label_set_text(hdrTitleLabel_, kNames[idx]);
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
    lv_obj_set_style_arc_color(arc, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 12, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x1a2540), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_pad_all(arc, 0, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    return arc;
  };

  constexpr lv_coord_t arcSz = 160;
  constexpr lv_coord_t arcY  = 32;   // (224-160)/2

  // ---- LEFT: RPM arc ----
  rpmArc_ = makeArc(parent, 4, arcY, arcSz, 8000);

  rpmValLabel_ = lv_label_create(parent);
  lv_label_set_text(rpmValLabel_, "0");
  lv_obj_set_width(rpmValLabel_, arcSz);
  lv_obj_set_style_text_font(rpmValLabel_, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_align(rpmValLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(rpmValLabel_, 4, arcY + 52);

  lv_obj_t* rpmUnit = lv_label_create(parent);
  lv_label_set_text(rpmUnit, "RPM");
  lv_obj_set_width(rpmUnit, arcSz);
  lv_obj_set_style_text_align(rpmUnit, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(rpmUnit, lv_color_hex(0x7090a0), 0);
  lv_obj_set_style_text_font(rpmUnit, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(rpmUnit, 4, arcY + 84);

  // ---- RIGHT: Speed arc ----
  spdArc_ = makeArc(parent, 308, arcY, arcSz, 160);

  spdValLabel_ = lv_label_create(parent);
  lv_label_set_text(spdValLabel_, "0");
  lv_obj_set_width(spdValLabel_, arcSz);
  lv_obj_set_style_text_font(spdValLabel_, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_align(spdValLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(spdValLabel_, 308, arcY + 52);

  lv_obj_t* spdUnit = lv_label_create(parent);
  lv_label_set_text(spdUnit, "mph");
  lv_obj_set_width(spdUnit, arcSz);
  lv_obj_set_style_text_align(spdUnit, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(spdUnit, lv_color_hex(0x7090a0), 0);
  lv_obj_set_style_text_font(spdUnit, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(spdUnit, 308, arcY + 84);

  // ---- RIGHT: G-force widgets (below speed arc, y=130..196) ----
  gLiveLabel_ = lv_label_create(parent);
  lv_label_set_text(gLiveLabel_, "-- G");
  lv_obj_set_width(gLiveLabel_, arcSz);
  lv_obj_set_style_text_font(gLiveLabel_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_align(gLiveLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(gLiveLabel_, lv_color_hex(0x7090a0), 0);
  lv_obj_set_pos(gLiveLabel_, 308, arcY + 98);

  gPeakLabel_ = lv_label_create(parent);
  lv_label_set_text(gPeakLabel_, "PK --");
  lv_obj_set_width(gPeakLabel_, arcSz);
  lv_obj_set_style_text_font(gPeakLabel_, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_align(gPeakLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(gPeakLabel_, lv_color_hex(0x506070), 0);
  lv_obj_set_pos(gPeakLabel_, 308, arcY + 122);

  // Lateral bar: centred under label, range -100..+100 (maps ±1.5 G → ±100)
  gLatBar_ = lv_bar_create(parent);
  lv_obj_set_pos(gLatBar_, 320, arcY + 140);
  lv_obj_set_size(gLatBar_, 136, 7);
  lv_bar_set_mode(gLatBar_, LV_BAR_MODE_SYMMETRICAL);  // negative values grow left from centre
  lv_bar_set_range(gLatBar_, -100, 100);
  lv_bar_set_value(gLatBar_, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(gLatBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  lv_obj_set_style_bg_color(gLatBar_, lv_palette_main(LV_PALETTE_CYAN), LV_PART_INDICATOR);

  // ---- CENTRE strip (x=170, w=134) ----
  constexpr lv_coord_t cx = 170;
  constexpr lv_coord_t cw = 134;

  // Boost bar
  boostBar_ = lv_bar_create(parent);
  lv_obj_set_pos(boostBar_, cx, 8);
  lv_obj_set_size(boostBar_, cw, 16);
  lv_bar_set_range(boostBar_, -10, 250);
  lv_bar_set_value(boostBar_, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(boostBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  lv_obj_set_style_bg_color(boostBar_, lv_palette_main(LV_PALETTE_CYAN), LV_PART_INDICATOR);

  boostValLabel_ = makeLabel(parent, cx, 28, cw, "BOOST 0 kPa", &lv_font_montserrat_14);
  lv_label_set_long_mode(boostValLabel_, LV_LABEL_LONG_CLIP);

  dashEnvLabel_ = makeLabel(parent, cx, 50, cw, "BAT 0.0V  0 sats", &lv_font_montserrat_14);
  lv_label_set_long_mode(dashEnvLabel_, LV_LABEL_LONG_CLIP);

  dashStatusLabel_ = makeLabel(parent, cx, 72, cw, "TAIL / METH", &lv_font_montserrat_12);
  lv_label_set_long_mode(dashStatusLabel_, LV_LABEL_LONG_CLIP);
  lv_label_set_recolor(dashStatusLabel_, true);

  dashRaceLabel_ = makeLabel(parent, cx, 90, cw, "0-60:--  1/4:--", &lv_font_montserrat_12);
  lv_label_set_long_mode(dashRaceLabel_, LV_LABEL_LONG_CLIP);

  // Race control buttons – 2×2 grid inside centre strip
  constexpr lv_coord_t bw = 64, bh = 38, bgap = 4;
  constexpr lv_coord_t by = 114;
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

  methOfflineSpinner_ = lv_spinner_create(parent, 900, 60);
  lv_obj_set_pos(methOfflineSpinner_, 340, 2);
  lv_obj_set_size(methOfflineSpinner_, 22, 22);

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

  methArmBtn_      = makeBtn(parent, "ON",         4, 142, 148, 52, onMethArmClicked,   this);
  methArmBtnLabel_ = btnLabel(methArmBtn_);

  methRatioBtn_      = makeBtn(parent, "RATIO 50%", 160, 142, 148, 52, onMethRatioClicked, this);
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
  tailStatusLabel_ = makeLabel(parent, 0, 0, 308,
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
  lv_obj_set_size(tailModePanel_, 308, 188);
  lv_obj_set_style_pad_all(tailModePanel_, 2, LV_PART_MAIN);
  lv_obj_set_style_border_width(tailModePanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tailModePanel_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_clear_flag(tailModePanel_, LV_OBJ_FLAG_SCROLLABLE);

  tailStockBtn_    = makeBtn(tailModePanel_, "STOCK",       2,  0, 148, 42, onTailStockClicked,    this);
  tailSeqBtn_      = makeBtn(tailModePanel_, "SEQUENTIAL", 158,  0, 148, 42, onTailSeqClicked,      this);
  tailShowMenuBtn_ = makeBtn(tailModePanel_, "SHOW",        2, 46, 148, 42, onTailShowMenuClicked, this);
  tailDemoBtn_     = makeBtn(tailModePanel_, "DEMO",       158, 46, 148, 42, onTailDemoClicked,     this);
  styleActionButton(tailStockBtn_);
  styleActionButton(tailSeqBtn_);
  styleActionButton(tailShowMenuBtn_);
  styleActionButton(tailDemoBtn_);

  // Show-option submenu panel
  tailShowPanel_ = lv_obj_create(parent);
  lv_obj_set_pos(tailShowPanel_, 0, 68);
  lv_obj_set_size(tailShowPanel_, 312, 310);
  lv_obj_set_style_pad_all(tailShowPanel_, 2, LV_PART_MAIN);
  lv_obj_set_style_border_width(tailShowPanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tailShowPanel_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_clear_flag(tailShowPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(tailShowPanel_, LV_OBJ_FLAG_HIDDEN);

  tailShowPageLabel_ = makeLabel(tailShowPanel_, 0, 0, 312, "Page 1/4");

  tailShowPrevBtn_ = makeBtn(tailShowPanel_, "< PREV",  2,  22,  80, 38, onTailShowPrevClicked, this);
  tailShowBackBtn_ = makeBtn(tailShowPanel_, "BACK",   90,  22, 130, 38, onTailShowBackClicked, this);
  tailShowNextBtn_ = makeBtn(tailShowPanel_, "NEXT >", 228,  22,  80, 38, onTailShowNextClicked, this);

  // 6 option buttons in a 3x2 grid
  const lv_coord_t optW = 100, optH = 40, optGap = 4;
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    const lv_coord_t col = static_cast<lv_coord_t>(i % 3);
    const lv_coord_t row = static_cast<lv_coord_t>(i / 3);
    const lv_coord_t bx  = col * (optW + optGap);
    const lv_coord_t by  = static_cast<lv_coord_t>(66) + row * (optH + optGap);
    tailShowOptBtns_[i] = makeBtn(tailShowPanel_, "N/A", bx, by, optW, optH, onTailShowOptClicked, this);
  }
}

// ---------------------------------------------------------------------------
// buildLedsPage – LED mode selector + per-channel status
// ---------------------------------------------------------------------------

void ScreenDashboard::buildLedsPage(lv_obj_t* parent) {
  makeLabel(parent, 0, 0, kWidth, "Interior LEDs", &lv_font_montserrat_20);

  ledMasterSwitch_ = lv_switch_create(parent);
  lv_obj_set_pos(ledMasterSwitch_, 362, 2);
  lv_obj_set_size(ledMasterSwitch_, 98, 36);
  lv_obj_add_event_cb(ledMasterSwitch_, onLedMasterSwitchChanged, LV_EVENT_VALUE_CHANGED, this);

  ledMasterLabel_ = makeLabel(parent, 306, 10, 54, "MASTER", &lv_font_montserrat_12);

  ledStatusLabel_ = makeLabel(parent, 0, 36, kWidth,
      "CH1: OFF  CH2: OFF  CH3: OFF", &lv_font_montserrat_24);

  static const char* const kModeNames[5] = { "OFF", "STATIC", "BREATHE", "RAINBOW", "RPM" };
  constexpr lv_coord_t btnW = 58, btnH = 44;
  constexpr lv_coord_t startX = 4, gapX = 4;
  for (uint8_t i = 0; i < 5; i++) {
    s_ledModeCtxs[i] = {this, static_cast<state::LedMode>(i)};
    const lv_coord_t bx = static_cast<lv_coord_t>(startX + i * (btnW + gapX));
    ledModeBtns_[i] = makeBtn(parent, kModeNames[i], bx, 76, btnW, btnH, onLedModeClicked, &s_ledModeCtxs[i]);
    lv_obj_set_style_text_font(btnLabel(ledModeBtns_[i]), &lv_font_montserrat_14, 0);
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
  lv_label_set_text(gpsSpdLabel_, "0 mph");
  lv_obj_set_width(gpsSpdLabel_, 320);
  lv_obj_set_style_text_font(gpsSpdLabel_, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_align(gpsSpdLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(gpsSpdLabel_, 0, 10);

  gpsInfoLabel_ = lv_label_create(parent);
  lv_label_set_text(gpsInfoLabel_,
      "FIX: NO  SATS: 0\nLAT: 0.000000\nLON: 0.000000");
  lv_obj_set_width(gpsInfoLabel_, 328);
  lv_obj_set_style_text_font(gpsInfoLabel_, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_align(gpsInfoLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(gpsInfoLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_pos(gpsInfoLabel_, 0, 78);
}

// ---------------------------------------------------------------------------
// buildTempsPage – all temperature channels
// ---------------------------------------------------------------------------

void ScreenDashboard::buildTempsPage(lv_obj_t* parent) {
  tempsLabel_ = makeLabel(parent, 0, 0, kWidth,
      LV_SYMBOL_WARNING "  SENSORS", &lv_font_montserrat_16);

  tempsTable_ = lv_table_create(parent);
  lv_obj_set_pos(tempsTable_, 0, 24);
  lv_obj_set_size(tempsTable_, 470, 184);
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
  lv_label_set_long_mode(diagLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(diagLabel_, kWidth - 8);
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
  constexpr lv_coord_t barW = 280;
  constexpr lv_coord_t barH = 12;
  constexpr lv_coord_t barX = 4;

  // ---- Row 0: state header ----
  knockStateLabel_ = makeLabel(parent, 0, 0, 286,
      "KNOCK  |  Response: WARN_ONLY", &lv_font_montserrat_12);
  lv_label_set_recolor(knockStateLabel_, true);

  // ---- Row 1: sensor status ----
  knockSensorLabel_ = makeLabel(parent, 0, 20, 286,
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
  lv_obj_set_style_bg_color(knockGraphChart_, lv_color_hex(0x101826), LV_PART_MAIN);
  lv_obj_set_style_border_color(knockGraphChart_, lv_color_hex(0x2b3d57), LV_PART_MAIN);
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

  knockLearningSpinner_ = lv_spinner_create(parent, 900, 60);
  lv_obj_set_pos(knockLearningSpinner_, 396, 114);
  lv_obj_set_size(knockLearningSpinner_, 42, 42);

  // ---- Row 2: energy label ----
  knockEnergyLabel_ = makeLabel(parent, 0, 40, kWidth - 8,
      "Energy: 0.0  (0%)", &lv_font_montserrat_12);

  // ---- Row 3: energy bar ----
  knockEnergyBar_ = lv_bar_create(parent);
  lv_obj_set_pos(knockEnergyBar_, barX, 56);
  lv_obj_set_size(knockEnergyBar_, barW, barH);
  lv_bar_set_range(knockEnergyBar_, 0, 100);
  lv_bar_set_value(knockEnergyBar_, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(knockEnergyBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  lv_obj_set_style_bg_color(knockEnergyBar_, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);

  // ---- Row 4: baseline label ----
  knockBaselineLabel_ = makeLabel(parent, 0, 74, kWidth - 8,
      "Baseline: 0.0  (0%)", &lv_font_montserrat_12);

  // ---- Row 5: baseline bar ----
  knockBaselineBar_ = lv_bar_create(parent);
  lv_obj_set_pos(knockBaselineBar_, barX, 90);
  lv_obj_set_size(knockBaselineBar_, barW, barH);
  lv_bar_set_range(knockBaselineBar_, 0, 100);
  lv_bar_set_value(knockBaselineBar_, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(knockBaselineBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  lv_obj_set_style_bg_color(knockBaselineBar_, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);

  // ---- Row 6: threshold label ----
  knockThresholdLabel_ = makeLabel(parent, 0, 108, kWidth - 8,
      "Threshold: 0.0  (100%)", &lv_font_montserrat_12);

  // ---- Row 7: threshold bar (always 100% — marks the maximum safe level) ----
  knockThresholdBar_ = lv_bar_create(parent);
  lv_obj_set_pos(knockThresholdBar_, barX, 124);
  lv_obj_set_size(knockThresholdBar_, barW, barH);
  lv_bar_set_range(knockThresholdBar_, 0, 100);
  lv_bar_set_value(knockThresholdBar_, 100, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(knockThresholdBar_, lv_color_hex(0x1a2540), LV_PART_MAIN);
  lv_obj_set_style_bg_color(knockThresholdBar_, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);

  // ---- Row 8: events / warning / critical ----
  knockEventLabel_ = makeLabel(parent, 0, 142, kWidth - 8,
      "Events: 0  Warn:NO  Crit:NO", &lv_font_montserrat_12);
  lv_label_set_recolor(knockEventLabel_, true);

  // ---- Row 9: last event details ----
  knockLastLabel_ = makeLabel(parent, 0, 158, kWidth - 8,
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
  lv_label_set_text(hdrBatLabel_, bat);

  // Fault indicator (right dot)
  lv_obj_set_style_text_color(hdrFaultDot_,
      (s.fault_flags != 0)
          ? lv_palette_main(LV_PALETTE_RED)
          : lv_palette_main(LV_PALETTE_GREEN),
      0);

  // Action feedback
  if (actionFeedback_[0] != '\0' &&
      nowMs < actionFeedbackUntilMs_ &&
      (actionFeedbackUntilMs_ - nowMs) <= kActionFeedbackMs) {
    lv_label_set_text(hdrFeedbackLabel_, actionFeedback_);
  } else {
    lv_label_set_text(hdrFeedbackLabel_, "");
    actionFeedback_[0] = '\0';
  }
}

void ScreenDashboard::updateDashPage(const state::VehicleState& s) {
  // ---- RPM arc ----
  const int16_t rpmClamped = static_cast<int16_t>(
      (s.rpm > 8000U) ? 8000U : s.rpm);
  lv_arc_set_value(rpmArc_, rpmClamped);

  lv_color_t arcColor;
  if (s.rpm >= 6500U)       arcColor = lv_palette_main(LV_PALETTE_RED);
  else if (s.rpm >= 4000U)  arcColor = lv_palette_main(LV_PALETTE_ORANGE);
  else                      arcColor = lv_palette_main(LV_PALETTE_BLUE);
  lv_obj_set_style_arc_color(rpmArc_, arcColor, LV_PART_INDICATOR);

  char buf[64];
  snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(s.rpm));
  lv_label_set_text(rpmValLabel_, buf);

  // ---- Speed arc (mph) ----
  const float spdMph = s.speed * 0.621371f;
  const int16_t spdClamped = static_cast<int16_t>(
      (spdMph > 160.0f) ? 160 : (spdMph < 0.0f) ? 0 : static_cast<int16_t>(spdMph));
  lv_arc_set_value(spdArc_, spdClamped);

  lv_color_t spdColor;
  if (spdMph >= 112.0f)      spdColor = lv_palette_main(LV_PALETTE_RED);
  else if (spdMph >= 62.0f)  spdColor = lv_palette_main(LV_PALETTE_ORANGE);
  else                       spdColor = lv_palette_main(LV_PALETTE_GREEN);
  lv_obj_set_style_arc_color(spdArc_, spdColor, LV_PART_INDICATOR);

  snprintf(buf, sizeof(buf), "%.0f", static_cast<double>(spdMph));
  lv_label_set_text(spdValLabel_, buf);

  // ---- Centre strip ----
  const int16_t boostClamped = static_cast<int16_t>(
      (s.boost_kpa > 250.0f) ? 250 : (s.boost_kpa < -10.0f) ? -10 : static_cast<int16_t>(s.boost_kpa));
  lv_bar_set_value(boostBar_, boostClamped, LV_ANIM_OFF);
  snprintf(buf, sizeof(buf), "BOOST %.0f kPa", static_cast<double>(s.boost_kpa));
  lv_label_set_text(boostValLabel_, buf);

  snprintf(buf, sizeof(buf), LV_SYMBOL_BATTERY_FULL " %.1fV  " LV_SYMBOL_GPS " %u  IAT %.1fC  O/F/M %.0f/%.0f/%.0fpsi",
           static_cast<double>(s.battery_voltage),
           static_cast<unsigned>(s.gps_satellites),
           static_cast<double>(s.intake_temp),
           static_cast<double>(s.oil_pressure_psi),
           static_cast<double>(s.fuel_pressure_psi),
           static_cast<double>(s.meth_pressure_psi));
  lv_label_set_text(dashEnvLabel_, buf);

  const bool noKnockActive = !s.knock_warning_active && !s.knock_critical_active;
  // Knock status intentionally inverts the normal color semantics:
  // green = no knock warning/critical, red = warning/critical active.
  snprintf(buf, sizeof(buf), "%s TAIL#  %s METH#  %s KNOCK#",
            s.taillight_online ? kStatusColorOn : kStatusColorOff,
            s.meth_online ? kStatusColorOn : kStatusColorOff,
            noKnockActive ? kStatusColorOn : kStatusColorOff);
  lv_label_set_text(dashStatusLabel_, buf);

  snprintf(buf, sizeof(buf), "0-60:%.2fs  1/4:%.2fs",
           static_cast<double>(s.race_0_60_s),
           static_cast<double>(s.race_quarter_mile_et_s));
  lv_label_set_text(dashRaceLabel_, buf);

  // ---- G-force widgets ----
  if (s.imu_online) {
    const float g = s.imu_g_total;
    lv_color_t gColor;
    if (g >= 1.0f)       gColor = lv_palette_main(LV_PALETTE_RED);
    else if (g >= 0.7f)  gColor = lv_palette_main(LV_PALETTE_ORANGE);
    else if (g >= 0.3f)  gColor = lv_palette_main(LV_PALETTE_GREEN);
    else                 gColor = lv_color_hex(0x7090a0);

    snprintf(buf, sizeof(buf), "%.2f G", static_cast<double>(g));
    lv_label_set_text(gLiveLabel_, buf);
    lv_obj_set_style_text_color(gLiveLabel_, gColor, 0);

    snprintf(buf, sizeof(buf), "PK %.2f", static_cast<double>(s.imu_g_peak));
    lv_label_set_text(gPeakLabel_, buf);

    // Scale lateral G to bar: ±1.5 G → ±100
    const int32_t latScaled = static_cast<int32_t>(s.imu_g_lateral * 66.7f);
    const int32_t latClamped = latScaled > 100 ? 100 : (latScaled < -100 ? -100 : latScaled);
    lv_bar_set_value(gLatBar_, latClamped, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(gLatBar_, gColor, LV_PART_INDICATOR);
  } else {
    lv_label_set_text(gLiveLabel_, "-- G");
    lv_obj_set_style_text_color(gLiveLabel_, lv_color_hex(0x506070), 0);
    lv_label_set_text(gPeakLabel_, "PK --");
    lv_bar_set_value(gLatBar_, 0, LV_ANIM_OFF);
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
    lv_label_set_text(methBadgeLabel_, buf);
  }

  if (methOnlineLed_) {
    if (s.meth_online) lv_led_on(methOnlineLed_);
    else lv_led_off(methOnlineLed_);
  }

  if (methOfflineSpinner_) {
    if (s.meth_online) lv_obj_add_flag(methOfflineSpinner_, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(methOfflineSpinner_, LV_OBJ_FLAG_HIDDEN);
  }

  if (methStateLabel_) {
    snprintf(buf, sizeof(buf),
             "LINK:%s  EXT:%s  ST:%s  D:%u%%  T:%u%%",
             moduleLink,
             extLink,
             methStateName(s.meth_state),
             static_cast<unsigned>(s.meth_pump_duty > 100U ? 100U : s.meth_pump_duty),
             static_cast<unsigned>(s.meth_tank_level));
    lv_label_set_text(methStateLabel_, buf);
  }

  if (methSensorLabel_) {
    snprintf(buf, sizeof(buf), "MAP %.0f  IAT %.1f  BAY %.1f  MP %.0f",
             static_cast<double>(s.boost_kpa),
             static_cast<double>(s.intake_temp),
             static_cast<double>(s.engine_bay_temp),
             static_cast<double>(s.meth_pressure_psi));
    lv_label_set_text(methSensorLabel_, buf);
  }

    const uint8_t duty = static_cast<uint8_t>(s.meth_pump_duty > 100U ? 100U : s.meth_pump_duty);
    lv_meter_set_indicator_value(methDutyMeter_, methDutyNeedle_, duty);

  if (methArmBtnLabel_) {
    lv_label_set_text(methArmBtnLabel_, s.meth_desired_armed ? "OFF" : "ON");
  }

  if (methRatioBtnLabel_) {
    snprintf(buf, sizeof(buf), "RATIO  %u%%",
             static_cast<unsigned>(s.meth_selected_ratio_percent));
    lv_label_set_text(methRatioBtnLabel_, buf);
  }

  if (methArmBtn_) {
    if (methActive) {
      const float wave = 0.5f + 0.5f * sinf(static_cast<float>(nowMs) * 0.010f);
      const uint8_t r = static_cast<uint8_t>(180.0f + 55.0f * wave);
      const uint8_t g = static_cast<uint8_t>(45.0f + 25.0f * wave);
      const uint8_t b = static_cast<uint8_t>(18.0f);
      lv_obj_set_style_bg_color(methArmBtn_, lv_color_make(r, g, b), LV_PART_MAIN);
    } else if (s.meth_desired_armed) {
      lv_obj_set_style_bg_color(methArmBtn_, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
    } else {
      lv_obj_set_style_bg_color(methArmBtn_, lv_color_hex(0x2b3340), LV_PART_MAIN);
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
  lv_label_set_text(tailStatusLabel_, buf);

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
      lv_label_set_text(btnLabel(tailShowOptBtns_[i]), kTailShowNames[optVal]);
    } else {
      lv_label_set_text(btnLabel(tailShowOptBtns_[i]), "N/A");
    }
  }

  char page[16];
  snprintf(page, sizeof(page), "Page %u/%u",
           static_cast<unsigned>(tailShowPage_ + 1U),
           static_cast<unsigned>(kTaillightShowPageCount));
  lv_label_set_text(tailShowPageLabel_, page);
}

void ScreenDashboard::updateLedsPage(const state::VehicleState& s) {
  static const char* const kModeStr[] = {
    "OFF", "STATIC", "BREATHE", "RAINBOW", "RPM", "FLASH", "METH", "FAULT", "SWEEP"
  };
  constexpr uint8_t kModeStrCount = 9;

  auto modeName = [&](state::LedMode m) -> const char* {
    const uint8_t idx = static_cast<uint8_t>(m);
    return (idx < kModeStrCount) ? kModeStr[idx] : "?";
  };

  char buf[96];
  snprintf(buf, sizeof(buf), LV_SYMBOL_CHARGE " CH1:%s  CH2:%s  CH3:%s",
           modeName(s.led_channel_1_mode),
           modeName(s.led_channel_2_mode),
           modeName(s.led_channel_3_mode));
  lv_label_set_text(ledStatusLabel_, buf);

  const bool allEnabled = s.led_channel_1_enabled && s.led_channel_2_enabled && s.led_channel_3_enabled;
  if (ledMasterSwitch_) {
    if (allEnabled) lv_obj_add_state(ledMasterSwitch_, LV_STATE_CHECKED);
    else lv_obj_clear_state(ledMasterSwitch_, LV_STATE_CHECKED);
  }
  if (ledMasterLabel_) {
    lv_label_set_text(ledMasterLabel_, allEnabled ? "MASTER ON" : "MASTER OFF");
  }

  // Highlight the active mode button for CH1 (as reference)
  for (uint8_t i = 0; i < 5; i++) {
    const bool active = (static_cast<uint8_t>(s.led_channel_1_mode) == i);
    lv_obj_set_style_bg_color(ledModeBtns_[i],
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
  lv_label_set_text(gpsSpdLabel_, spd);

  if (gpsFixLed_) {
    if (s.gps_fix) lv_led_on(gpsFixLed_);
    else lv_led_off(gpsFixLed_);
  }
  if (gpsSatMeter_ && gpsSatNeedle_) {
    uint8_t sats = s.gps_satellites;
    if (sats > 20U) sats = 20U;
    lv_meter_set_indicator_value(gpsSatMeter_, gpsSatNeedle_, sats);
  }

  char info[128];
  snprintf(info, sizeof(info),
           LV_SYMBOL_GPS " %s  FIX:%s  SATS:%u\nLAT: %.6f\nLON: %.6f",
           gpsLive ? "LIVE" : "STALE",
           s.gps_fix ? "YES" : "NO",
           static_cast<unsigned>(s.gps_satellites),
           s.gps_latitude,
           s.gps_longitude);
  lv_label_set_text(gpsInfoLabel_, info);
}

void ScreenDashboard::updateTempsPage(const state::VehicleState& s) {
  char v[32];
  if (tempsLabel_) {
    snprintf(v, sizeof(v), LV_SYMBOL_WARNING " SENSOR BOARD  |  FaultMask:0x%04X",
             static_cast<unsigned>(s.analog_sensor_fault_flags));
    lv_label_set_text(tempsLabel_, v);
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
  // Compute seconds-ago values from uptime and last-seen timestamps
  const uint32_t now = s.uptime_ms;
  auto msAgo = [now](uint32_t ts) -> uint32_t {
    return (ts == 0 || now < ts) ? 9999UL : (now - ts) / 1000UL;
  };

  // Decode reset reason into a short string
  const char* resetStr;
  switch (s.reset_reason) {
    case 1:  resetStr = "POR";  break;
    case 3:  resetStr = "SW";   break;
    case 5:  resetStr = "WDT";  break;
    case 7:  resetStr = "SLP";  break;
    case 12: resetStr = "BOR";  break;
    case 14: resetStr = "EXT";  break;
    default: resetStr = "UNK";  break;
  }

  constexpr size_t kDiagBufSize = 640;
  char buf[kDiagBufSize];
  snprintf(buf, sizeof(buf),
    "SYS: Heap %luB  Die %dC  Uptime %lus  Reset %s  BO:%u WD:%u\n"
    "NET: WiFi %s  Clients %u  Touch %s  FPS %.1f\n"
    "CAN: RX %lu TX %lu BadCRC %lu  LastRX 0x%03X %lus  TX 0x%03X %lus\n"
    "SD: %s %llu/%llu MB  Errors %lu  Log %s  Write %s\n"
    "METH: St %u  Duty %u%%  Tank %s  On %s  Flow %u  Ratio %u%%\n"
    "KNOCK: En %s  Sig %s  Warn %s  Crit %s  Learn %s  E %.1f B %.1f T %.1f\n"
    "ANALOG: IAT %.1f(%s) Bay %.1f(%s) Cabin %.1f(%s) Amb %.1f(%s) Oil %.1f(%s) Fuel %.1f(%s) Meth %.1f(%s) Boost %.1f(%s)\n"
    "TACH %.1f/%.1f Hz Src %u  GPS %s type%u sats%u alt%d m",
    static_cast<unsigned long>(s.heap_free_bytes),
    static_cast<int>(s.esp_die_temp_c),
    static_cast<unsigned long>(s.uptime_ms / 1000UL),
    resetStr,
    static_cast<unsigned>(s.brownout_reset_count),
    static_cast<unsigned>(s.watchdog_reset_count),
    s.wifi_ap_mode ? "AP" : (s.wifi_connected ? "STA" : "OFF"),
    static_cast<unsigned>(s.web_connected_clients),
    s.touch_online ? "OK" : "OFFLINE",
    static_cast<double>(s.ui_fps),
    // CAN
    static_cast<unsigned long>(s.can_rx_count),
    static_cast<unsigned long>(s.can_tx_count),
    static_cast<unsigned long>(s.can_bad_checksum_count),
    static_cast<unsigned>(s.can_last_rx_id),
    static_cast<unsigned long>(msAgo(s.can_last_rx_ms)),
    static_cast<unsigned>(s.can_last_tx_id),
    static_cast<unsigned long>(msAgo(s.can_last_tx_ms)),
    static_cast<unsigned>(s.fault_flags),
    // STORAGE
    s.sd_mounted ? "YES" : "NO",
    static_cast<unsigned long long>(s.sd_used_bytes  / 1048576ULL),
    static_cast<unsigned long long>(s.sd_size_bytes  / 1048576ULL),
    static_cast<unsigned long>(s.sd_write_error_count),
    s.current_log_file[0] ? s.current_log_file : "none",
    s.last_sd_write_status[0] ? s.last_sd_write_status : "--",
    static_cast<unsigned>(s.meth_state),
    static_cast<unsigned>(s.meth_pump_duty),
    (s.meth_tank_level == 0) ? "EMPTY" : "OK",
    s.meth_online ? "YES" : "NO",
    static_cast<unsigned>(s.meth_flow_status),
    static_cast<unsigned>(s.meth_selected_ratio_percent),
    s.meth_desired_armed ? "YES" : "NO",
    s.manual_test_running ? "YES" : "NO",
    static_cast<unsigned>(s.meth_manual_test_reject_reason),
    static_cast<unsigned>(s.meth_manual_test_cooldown_ms_remaining),
    s.knock_enabled ? "YES" : "NO",
    s.knock_signal_valid ? "YES" : "NO",
    s.knock_warning_active ? "YES" : "NO",
    s.knock_critical_active ? "YES" : "NO",
    s.knock_baseline_learned ? "YES" : "NO",
    static_cast<double>(s.knock_energy),
    static_cast<double>(s.knock_baseline),
    static_cast<double>(s.knock_threshold),
    static_cast<unsigned>(s.knock_event_count),
    static_cast<unsigned>(s.knock_last_event_rpm),
    static_cast<double>(s.knock_last_event_boost_kpa),
    s.knock_sensor_fault ? "YES" : "NO",
    s.knock_clipping_detected ? "YES" : "NO",
    static_cast<unsigned>(s.knock_signal_clip_high_count),
    static_cast<unsigned>(s.knock_signal_clip_low_count),
    static_cast<unsigned>(s.knock_response_mode),
    static_cast<double>(s.intake_temp), s.intake_temp_valid ? "OK" : "FAULT",
    static_cast<double>(s.engine_bay_temp), s.engine_bay_temp_valid ? "OK" : "FAULT",
    static_cast<double>(s.cabin_temp), s.cabin_temp_valid ? "OK" : "FAULT",
    static_cast<double>(s.outside_temp), s.outside_temp_valid ? "OK" : "FAULT",
    static_cast<double>(s.oil_pressure_psi), s.oil_pressure_valid ? "OK" : "FAULT",
    static_cast<double>(s.fuel_pressure_psi), s.fuel_pressure_valid ? "OK" : "FAULT",
    static_cast<double>(s.meth_pressure_psi), s.meth_pressure_valid ? "OK" : "FAULT",
    static_cast<double>(s.boost_ref_pressure_psi), s.boost_ref_pressure_valid ? "OK" : "FAULT",
    static_cast<double>(s.tach_input_frequency_hz),
    static_cast<double>(s.tach_input_frequency_hz),
    static_cast<double>(s.tach_generated_frequency_hz),
    static_cast<unsigned>(s.tach_source),
    s.gps_fix ? "YES" : "NO",
    static_cast<unsigned>(s.gps_fix_type),
    static_cast<unsigned>(s.gps_satellites),
    static_cast<int>(s.gps_altitude_m));
  lv_label_set_text(diagLabel_, buf);
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
  lv_label_set_text(knockStateLabel_, buf);

  // Sensor status row
  snprintf(buf, sizeof(buf), "En: %s  Sensor: %s  Learned: %s  Clip Hi: %u Lo: %u",
           s.knock_enabled ? "YES" : "NO",
           sensorStr,
           s.knock_baseline_learned ? "YES" : "NO",
           static_cast<unsigned>(s.knock_signal_clip_high_count & 0xFFU),
           static_cast<unsigned>(s.knock_signal_clip_low_count  & 0xFFU));
  lv_label_set_text(knockSensorLabel_, buf);

  // Energy bar: 0..100 where 100 = threshold level
  const float thresh = (s.knock_threshold > 0.1f) ? s.knock_threshold : 1.0f;
  const int energyPct = static_cast<int>((s.knock_energy / thresh) * 100.0f);
  const int clampedE  = (energyPct > 100) ? 100 : (energyPct < 0 ? 0 : energyPct);
  snprintf(buf, sizeof(buf), "Knock Energy:  %.1f  (%d%% of threshold)",
           static_cast<double>(s.knock_energy), clampedE);
  lv_label_set_text(knockEnergyLabel_, buf);
  lv_bar_set_value(knockEnergyBar_, clampedE, LV_ANIM_OFF);

  // Energy bar colour: green → yellow → red based on level
  lv_color_t eCo;
  if (clampedE >= 90)       eCo = lv_palette_main(LV_PALETTE_RED);
  else if (clampedE >= 60)  eCo = lv_palette_main(LV_PALETTE_ORANGE);
  else                      eCo = lv_palette_main(LV_PALETTE_BLUE);
  lv_obj_set_style_bg_color(knockEnergyBar_, eCo, LV_PART_INDICATOR);

  // Baseline bar
  const int baselinePct = static_cast<int>((s.knock_baseline / thresh) * 100.0f);
  const int clampedB    = (baselinePct > 100) ? 100 : (baselinePct < 0 ? 0 : baselinePct);
  snprintf(buf, sizeof(buf), "Baseline:  %.1f  (%d%% of threshold)",
           static_cast<double>(s.knock_baseline), clampedB);
  lv_label_set_text(knockBaselineLabel_, buf);
  lv_bar_set_value(knockBaselineBar_, clampedB, LV_ANIM_OFF);

  // Threshold label (threshold bar is always 100% — it marks the limit)
  snprintf(buf, sizeof(buf), "Threshold:  %.1f  (×%.1f baseline + %.1f offset)",
           static_cast<double>(s.knock_threshold),
           static_cast<double>(s.knock_threshold_multiplier),
           static_cast<double>(s.knock_threshold_offset));
  lv_label_set_text(knockThresholdLabel_, buf);

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
    if (s.knock_enabled && !s.knock_baseline_learned) {
      lv_obj_clear_flag(knockLearningSpinner_, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(knockLearningSpinner_, LV_OBJ_FLAG_HIDDEN);
    }
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
  lv_label_set_text(knockEventLabel_, buf);

  // Last event row
  if (s.knock_last_event_rpm == 0 && s.knock_last_event_boost_kpa == 0) {
    lv_label_set_text(knockLastLabel_, "Last event: none");
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
    lv_label_set_text(knockLastLabel_, buf);
  }

  // Enable button label
  lv_label_set_text(knockEnableBtnLabel_, s.knock_enabled ? "DISABLE" : "ENABLE");
  // Dim simulate button if not in demo/dev mode
  const bool demoActive = s.knock_demo_mode_enabled;
#if defined(DEMO_MODE) && (DEMO_MODE == 1)
  (void)demoActive;
  lv_obj_set_style_bg_color(knockSimulateBtn_, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);
#else
  lv_obj_set_style_bg_color(knockSimulateBtn_,
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
  lv_label_set_text(knockLogLabel_, buf);
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
  self->setActionFeedback("SHOW MENU", millis());
}

void ScreenDashboard::onTailShowPrevClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  self->tailShowPage_ = (self->tailShowPage_ == 0)
      ? static_cast<uint8_t>(kTaillightShowPageCount - 1U)
      : static_cast<uint8_t>(self->tailShowPage_ - 1U);
  self->setActionFeedback("SHOW PAGE", millis());
}

void ScreenDashboard::onTailShowNextClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  self->tailShowPage_ = static_cast<uint8_t>((self->tailShowPage_ + 1U) % kTaillightShowPageCount);
  self->setActionFeedback("SHOW PAGE", millis());
}

void ScreenDashboard::onTailShowBackClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  lv_obj_clear_flag(self->tailModePanel_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(self->tailShowPanel_,   LV_OBJ_FLAG_HIDDEN);
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

}  // namespace ui
