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

// LVGL draw buffers – two 480×20 slices (~19 KB each, double-buffered).
static lv_color_t s_buf1[480 * 20];
static lv_color_t s_buf2[480 * 20];

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
  lv_disp_draw_buf_init(&drawBuf, s_buf1, s_buf2, 480 * 20);

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
  updateDashPage(s);
  updateMethPage(s, nowMs);
  updateTailPage(s);
  updateLedsPage(s);
  updateGpsPage(s);
  updateTempsPage(s);
  updateDiagPage(s);
  updateKnockPage(s, nowMs);
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
    LV_SYMBOL_AUDIO,     // 7 KNOCK
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
  spdArc_ = makeArc(parent, 308, arcY, arcSz, 260);

  spdValLabel_ = lv_label_create(parent);
  lv_label_set_text(spdValLabel_, "0");
  lv_obj_set_width(spdValLabel_, arcSz);
  lv_obj_set_style_text_font(spdValLabel_, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_align(spdValLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(spdValLabel_, 308, arcY + 52);

  lv_obj_t* spdUnit = lv_label_create(parent);
  lv_label_set_text(spdUnit, "km/h");
  lv_obj_set_width(spdUnit, arcSz);
  lv_obj_set_style_text_align(spdUnit, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(spdUnit, lv_color_hex(0x7090a0), 0);
  lv_obj_set_style_text_font(spdUnit, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(spdUnit, 308, arcY + 84);

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
}

// ---------------------------------------------------------------------------
// buildMethPage
// ---------------------------------------------------------------------------

void ScreenDashboard::buildMethPage(lv_obj_t* parent) {
  methStateLabel_  = makeLabel(parent, 0,  0, 310, "METH | State: 0 | Duty: 0% | Tank: 0%");
  methSensorLabel_ = makeLabel(parent, 0, 24, 310, "MAP: 0 kPa  IAT: 0.0 C  Bay: 0.0 C");
  methParamLabel_  = makeLabel(parent, 0, 48, 310, "Ratio: 0%  Flow: 0  Armed: NO");

  methArmBtn_      = makeBtn(parent, "ARM",        4,  82, 148, 48, onMethArmClicked,   this);
  methArmBtnLabel_ = btnLabel(methArmBtn_);

  methRatioBtn_      = makeBtn(parent, "RATIO 50%", 160, 82, 148, 48, onMethRatioClicked, this);
  methRatioBtnLabel_ = btnLabel(methRatioBtn_);
}

// ---------------------------------------------------------------------------
// buildTailPage
// ---------------------------------------------------------------------------

void ScreenDashboard::buildTailPage(lv_obj_t* parent) {
  tailStatusLabel_ = makeLabel(parent, 0, 0, 310,
      "Taillights: OFFLINE  Bright: 0\nL: 0  R: 0  Thermal derate: 0%");

  // Mode button panel
  tailModePanel_ = lv_obj_create(parent);
  lv_obj_set_pos(tailModePanel_, 0, 52);
  lv_obj_set_size(tailModePanel_, 312, 200);
  lv_obj_set_style_pad_all(tailModePanel_, 2, LV_PART_MAIN);
  lv_obj_set_style_border_width(tailModePanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tailModePanel_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_clear_flag(tailModePanel_, LV_OBJ_FLAG_SCROLLABLE);

  tailStockBtn_    = makeBtn(tailModePanel_, "STOCK",       2,  0, 148, 48, onTailStockClicked,    this);
  tailSeqBtn_      = makeBtn(tailModePanel_, "SEQUENTIAL", 158,  0, 148, 48, onTailSeqClicked,      this);
  tailShowMenuBtn_ = makeBtn(tailModePanel_, "SHOW MENU",   2, 54, 148, 48, onTailShowMenuClicked, this);
  tailDemoBtn_     = makeBtn(tailModePanel_, "DEMO",       158, 54, 148, 48, onTailDemoClicked,     this);

  // Show-option submenu panel
  tailShowPanel_ = lv_obj_create(parent);
  lv_obj_set_pos(tailShowPanel_, 0, 52);
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
    tailShowOptBtns_[i] = makeBtn(tailShowPanel_, "--", bx, by, optW, optH, onTailShowOptClicked, this);
  }
}

// ---------------------------------------------------------------------------
// buildLedsPage – LED mode selector + per-channel status
// ---------------------------------------------------------------------------

void ScreenDashboard::buildLedsPage(lv_obj_t* parent) {
  makeLabel(parent, 0, 0, 310, "Interior LEDs", &lv_font_montserrat_16);

  ledStatusLabel_ = makeLabel(parent, 0, 28, 310,
      "CH1: OFF  CH2: OFF  CH3: OFF", &lv_font_montserrat_12);

  makeLabel(parent, 0, 52, 310, "Select mode (all channels):", &lv_font_montserrat_12);

  static const char* const kModeNames[5] = { "OFF", "STATIC", "BREATHE", "RAINBOW", "RPM" };
  constexpr lv_coord_t btnW = 58, btnH = 44;
  constexpr lv_coord_t startX = 4, gapX = 4;
  for (uint8_t i = 0; i < 5; i++) {
    s_ledModeCtxs[i] = {this, static_cast<state::LedMode>(i)};
    const lv_coord_t bx = static_cast<lv_coord_t>(startX + i * (btnW + gapX));
    ledModeBtns_[i] = makeBtn(parent, kModeNames[i], bx, 72, btnW, btnH, onLedModeClicked, &s_ledModeCtxs[i]);
    lv_obj_set_style_text_font(btnLabel(ledModeBtns_[i]), &lv_font_montserrat_12, 0);
  }

  makeLabel(parent, 0, 130, 310,
      "Color presets:", &lv_font_montserrat_12);
  // Informational note
  makeLabel(parent, 0, 152, 310,
      "Colors are set via web interface\nor saved presets.", &lv_font_montserrat_12);
}

// ---------------------------------------------------------------------------
// buildGpsPage – GPS speed, fix, coordinates
// ---------------------------------------------------------------------------

void ScreenDashboard::buildGpsPage(lv_obj_t* parent) {
  // Speed — large centred number
  gpsSpdLabel_ = lv_label_create(parent);
  lv_label_set_text(gpsSpdLabel_, "-- mph");
  lv_obj_set_width(gpsSpdLabel_, kWidth);
  lv_obj_set_style_text_font(gpsSpdLabel_, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_align(gpsSpdLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(gpsSpdLabel_, 0, 8);

  // Fix / sats / coordinates
  gpsInfoLabel_ = lv_label_create(parent);
  lv_label_set_text(gpsInfoLabel_,
      "FIX: NO   SATS: 0\n"
      "LAT: --\n"
      "LON: --");
  lv_obj_set_width(gpsInfoLabel_, kWidth);
  lv_obj_set_style_text_font(gpsInfoLabel_, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_align(gpsInfoLabel_, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(gpsInfoLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_pos(gpsInfoLabel_, 0, 80);
}

// ---------------------------------------------------------------------------
// buildTempsPage – all temperature channels
// ---------------------------------------------------------------------------

void ScreenDashboard::buildTempsPage(lv_obj_t* parent) {
  makeLabel(parent, 0, 0, 310, "Temperatures", &lv_font_montserrat_16);

  tempsLabel_ = makeLabel(parent, 0, 30, 310,
      "Cabin:        --\n"
      "Outside:      --\n"
      "Engine Bay:   --\n"
      "Intake Air:   --\n"
      "Intercooler:  --\n"
      "ESP die:      --",
      &lv_font_montserrat_14);
  lv_label_set_long_mode(tempsLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(tempsLabel_, 310);
}

// ---------------------------------------------------------------------------
// buildDiagPage
// ---------------------------------------------------------------------------

void ScreenDashboard::buildDiagPage(lv_obj_t* parent) {
  diagLabel_ = makeLabel(parent, 0, 0, kWidth - 8,
      "Diagnostics loading...", &lv_font_montserrat_12);
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
  constexpr lv_coord_t barW = 460;
  constexpr lv_coord_t barH = 12;
  constexpr lv_coord_t barX = 4;

  // ---- Row 0: state header ----
  knockStateLabel_ = makeLabel(parent, 0, 0, kWidth - 8,
      "KNOCK SENSE  |  Response: WARN_ONLY", &lv_font_montserrat_14);
  lv_label_set_recolor(knockStateLabel_, true);

  // ---- Row 1: sensor status ----
  knockSensorLabel_ = makeLabel(parent, 0, 20, kWidth - 8,
      "En: YES  Sensor: OK  Online: YES  Learned: NO  Fault: NO",
      &lv_font_montserrat_12);
  lv_label_set_long_mode(knockSensorLabel_, LV_LABEL_LONG_CLIP);

  // ---- Row 2: energy label ----
  knockEnergyLabel_ = makeLabel(parent, 0, 40, kWidth - 8,
      "Knock Energy:  0.0 raw  (0%)", &lv_font_montserrat_12);

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
      "Baseline:  0.0 raw  (0%)", &lv_font_montserrat_12);

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
      "Threshold:  0.0 raw  (100%)", &lv_font_montserrat_12);

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
      "Events: 0  |  Warning: NO  Critical: NO", &lv_font_montserrat_12);
  lv_label_set_recolor(knockEventLabel_, true);

  // ---- Row 9: last event details ----
  knockLastLabel_ = makeLabel(parent, 0, 158, kWidth - 8,
      "Last event: --", &lv_font_montserrat_12);

  // ---- Row 10: control buttons (y=178, h=36) ----
  constexpr lv_coord_t btnW = 110, btnH = 36, btnGap = 4;
  knockEnableBtn_      = makeBtn(parent, "EN/DIS",  0 * (btnW + btnGap), 178, btnW, btnH, onKnockEnableClicked,        this);
  knockResetBlBtn_     = makeBtn(parent, "RESET BL", 1 * (btnW + btnGap), 178, btnW, btnH, onKnockResetBaselineClicked, this);
  knockClearEvtBtn_    = makeBtn(parent, "CLR EVTS", 2 * (btnW + btnGap), 178, btnW, btnH, onKnockClearEventsClicked,   this);
  knockSimulateBtn_    = makeBtn(parent, "SIMULATE", 3 * (btnW + btnGap), 178, btnW, btnH, onKnockSimulateClicked,      this);
  knockEnableBtnLabel_ = btnLabel(knockEnableBtn_);

  lv_obj_set_style_text_font(btnLabel(knockEnableBtn_),   &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_font(btnLabel(knockResetBlBtn_),  &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_font(btnLabel(knockClearEvtBtn_), &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_font(btnLabel(knockSimulateBtn_), &lv_font_montserrat_12, 0);

  // ---- Row 11: logging status + disclaimer ----
  knockLogLabel_ = makeLabel(parent, 0, 218, kWidth - 8,
      "SD Log: --  |  \xe2\x80\xa0 Not ECU knock control", &lv_font_montserrat_12);
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

  // ---- Speed arc ----
  const int16_t spdClamped = static_cast<int16_t>(
      (s.speed > 260.0f) ? 260 : (s.speed < 0.0f) ? 0 : static_cast<int16_t>(s.speed));
  lv_arc_set_value(spdArc_, spdClamped);

  lv_color_t spdColor;
  if (s.speed >= 180.0f)      spdColor = lv_palette_main(LV_PALETTE_RED);
  else if (s.speed >= 100.0f) spdColor = lv_palette_main(LV_PALETTE_ORANGE);
  else                        spdColor = lv_palette_main(LV_PALETTE_GREEN);
  lv_obj_set_style_arc_color(spdArc_, spdColor, LV_PART_INDICATOR);

  snprintf(buf, sizeof(buf), "%.0f", static_cast<double>(s.speed));
  lv_label_set_text(spdValLabel_, buf);

  // ---- Centre strip ----
  const int16_t boostClamped = static_cast<int16_t>(
      (s.boost_kpa > 250.0f) ? 250 : (s.boost_kpa < -10.0f) ? -10 : static_cast<int16_t>(s.boost_kpa));
  lv_bar_set_value(boostBar_, boostClamped, LV_ANIM_OFF);
  snprintf(buf, sizeof(buf), "BOOST %.0f kPa", static_cast<double>(s.boost_kpa));
  lv_label_set_text(boostValLabel_, buf);

  snprintf(buf, sizeof(buf), "BAT %.1fV  %u sats  IAT %.1fC  O/F/M %.0f/%.0f/%.0fpsi",
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
}

void ScreenDashboard::updateMethPage(const state::VehicleState& s, uint32_t /*nowMs*/) {
  char buf[96];

  snprintf(buf, sizeof(buf), "METH | State: %u | Duty: %u%% | Tank: %s",
           static_cast<unsigned>(s.meth_state),
           static_cast<unsigned>(s.meth_pump_duty),
           (s.meth_tank_level == 0) ? "EMPTY" : "OK");
  lv_label_set_text(methStateLabel_, buf);

  snprintf(buf, sizeof(buf), "MAP: %.0f kPa  IAT: %.1f C  Bay: %.1f C",
           static_cast<double>(s.boost_kpa),
           static_cast<double>(s.intake_temp),
           static_cast<double>(s.engine_bay_temp));
  lv_label_set_text(methSensorLabel_, buf);

  snprintf(buf, sizeof(buf), "Ratio: %u%%  Flow: %u  Armed: %s",
           static_cast<unsigned>(s.meth_selected_ratio_percent),
           static_cast<unsigned>(s.meth_flow_status),
           s.meth_desired_armed ? "YES" : "NO");
  lv_label_set_text(methParamLabel_, buf);

  lv_label_set_text(methArmBtnLabel_,
                    s.meth_desired_armed ? "DISARM" : "ARM");

  snprintf(buf, sizeof(buf), "RATIO  %u%%",
           static_cast<unsigned>(s.meth_selected_ratio_percent));
  lv_label_set_text(methRatioBtnLabel_, buf);
}

void ScreenDashboard::updateTailPage(const state::VehicleState& s) {
  char buf[96];

  snprintf(buf, sizeof(buf),
           "Taillights: %s  Bright: %u\nL: %u  R: %u  Thermal derate: %u%%",
           s.taillight_online ? "ONLINE" : "OFFLINE",
           static_cast<unsigned>(s.taillight_brightness),
           static_cast<unsigned>(s.taillight_left_state),
           static_cast<unsigned>(s.taillight_right_state),
           static_cast<unsigned>(s.taillight_thermal_derate));
  lv_label_set_text(tailStatusLabel_, buf);

  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    const uint16_t optVal = static_cast<uint16_t>(tailShowPage_) * kTaillightShowOptionsPerPage + i;
    char label[12];
    if (optVal < kTaillightShowOptionCount) {
      snprintf(label, sizeof(label), "SHOW %u", static_cast<unsigned>(optVal + 1U));
    } else {
      snprintf(label, sizeof(label), "--");
    }
    lv_label_set_text(btnLabel(tailShowOptBtns_[i]), label);
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
  snprintf(buf, sizeof(buf), "CH1:%s  CH2:%s  CH3:%s",
           modeName(s.led_channel_1_mode),
           modeName(s.led_channel_2_mode),
           modeName(s.led_channel_3_mode));
  lv_label_set_text(ledStatusLabel_, buf);

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

  char info[128];
  snprintf(info, sizeof(info),
           "FIX: %s   SATS: %u\n"
           "LAT: %.6f\n"
           "LON: %.6f",
           s.gps_fix ? "YES" : "NO",
           static_cast<unsigned>(s.gps_satellites),
           s.gps_latitude,
           s.gps_longitude);
  lv_label_set_text(gpsInfoLabel_, info);
}

void ScreenDashboard::updateTempsPage(const state::VehicleState& s) {
  // 12 lines @ ~32 chars/line plus labels, spacing, and numeric precision margin.
  constexpr size_t kTempsBufSize = 448;
  char buf[kTempsBufSize];
  snprintf(buf, sizeof(buf),
           "Cabin:        %.1f C\n"
           "Outside:      %.1f C\n"
           "Engine Bay:   %.1f C\n"
           "Intake Air:   %.1f C\n"
           "Intercooler:  %.1f C\n"
           "Oil Press:    %.1f psi\n"
           "Fuel Press:   %.1f psi\n"
           "Meth Press:   %.1f psi\n"
           "Boost Ref:    %.1f psi\n"
           "Spare 1/2:    %.1f / %.1f psi\n"
           "Validity T/P: %s%s%s%s / %s%s%s%s\n"
           "ESP die:      %d C",
           static_cast<double>(s.cabin_temp),
           static_cast<double>(s.outside_temp),
           static_cast<double>(s.engine_bay_temp),
           static_cast<double>(s.intake_temp),
           static_cast<double>(s.intercooler_temp),
           static_cast<double>(s.oil_pressure_psi),
           static_cast<double>(s.fuel_pressure_psi),
           static_cast<double>(s.meth_pressure_psi),
           static_cast<double>(s.boost_ref_pressure_psi),
           static_cast<double>(s.spare_pressure_1_psi),
           static_cast<double>(s.spare_pressure_2_psi),
           s.intake_temp_valid ? "IAT " : "",
           s.engine_bay_temp_valid ? "BAY " : "",
           s.cabin_temp_valid ? "CAB " : "",
           s.outside_temp_valid ? "AMB " : "",
           s.oil_pressure_valid ? "OIL " : "",
           s.fuel_pressure_valid ? "FUEL " : "",
           s.meth_pressure_valid ? "METH " : "",
           s.boost_ref_pressure_valid ? "BOOST " : "",
           static_cast<int>(s.esp_die_temp_c));
  lv_label_set_text(tempsLabel_, buf);
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

  // Multi-section diagnostics text block with analog sensor section and long numeric fields.
  constexpr size_t kDiagBufSize = 1152;
  char buf[kDiagBufSize];
  snprintf(buf, sizeof(buf),
    "-- SYSTEM --\n"
    "Heap: %lu B free   Die: %d C\n"
    "Uptime: %lu s   Reset: %s  BO:%u WD:%u\n"
    "WiFi: %s  Clients: %u  Touch: %s  FPS: %.1f\n"
    "\n"
    "-- CAN BUS --\n"
    "RX: %lu  TX: %lu  BadCRC: %lu\n"
    "Last RX: 0x%03X (%lus ago)   TX: 0x%03X (%lus ago)\n"
    "Fault flags: 0x%04X\n"
    "\n"
    "-- STORAGE --\n"
    "SD: %s  %llu/%llu MB  Errors: %lu\n"
    "Log: %s\n"
    "Write: %s\n"
    "\n"
    "-- METH MODULE --\n"
    "State: %u  Duty: %u%%  Tank: %s  Online: %s\n"
    "Flow: %u  Ratio: %u%%  Armed: %s\n"
    "TestRun: %s  Reject: %u  Cooldown: %u ms\n"
    "\n"
    "-- KNOCK --\n"
    "En:%s Valid:%s Warn:%s Crit:%s Learned:%s\n"
    "E:%.1f B:%.1f T:%.1f Cnt:%u Last:%u rpm %.0f kPa\n"
    "Fault:%s Clip:%s Hi:%u Lo:%u Resp:%u\n"
    "\n"
    "-- ANALOG SENSORS --\n"
    "IAT:%.1f(%s) Bay:%.1f(%s) Cabin:%.1f(%s) Amb:%.1f(%s)\n"
    "Oil:%.1f(%s) Fuel:%.1f(%s) Meth:%.1f(%s) Boost:%.1f(%s)\n"
    "Sp1:%.1f(%s) Sp2:%.1f(%s) FaultMask:0x%04X\n"
    "\n"
    "-- TACH --\n"
    "In: %.1f Hz   Out: %.1f Hz   Src: %u\n"
    "\n"
    "-- GPS --\n"
    "Fix: %s (type %u)  Sats: %u  Alt: %d m",
    // SYSTEM
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
    // METH
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
    // KNOCK
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
    // ANALOG
    static_cast<double>(s.intake_temp), s.intake_temp_valid ? "OK" : "FAULT",
    static_cast<double>(s.engine_bay_temp), s.engine_bay_temp_valid ? "OK" : "FAULT",
    static_cast<double>(s.cabin_temp), s.cabin_temp_valid ? "OK" : "FAULT",
    static_cast<double>(s.outside_temp), s.outside_temp_valid ? "OK" : "FAULT",
    static_cast<double>(s.oil_pressure_psi), s.oil_pressure_valid ? "OK" : "FAULT",
    static_cast<double>(s.fuel_pressure_psi), s.fuel_pressure_valid ? "OK" : "FAULT",
    static_cast<double>(s.meth_pressure_psi), s.meth_pressure_valid ? "OK" : "FAULT",
    static_cast<double>(s.boost_ref_pressure_psi), s.boost_ref_pressure_valid ? "OK" : "FAULT",
    static_cast<double>(s.spare_pressure_1_psi), s.spare_pressure_1_valid ? "OK" : "FAULT",
    static_cast<double>(s.spare_pressure_2_psi), s.spare_pressure_2_valid ? "OK" : "FAULT",
    static_cast<unsigned>(s.analog_sensor_fault_flags),
    // TACH
    static_cast<double>(s.tach_input_frequency_hz),
    static_cast<double>(s.tach_generated_frequency_hz),
    static_cast<unsigned>(s.tach_source),
    // GPS
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
    self->setActionFeedback(arm ? "METH ARMED" : "METH DISARMED", millis());
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
  if (self->canMgr_->sendTaillightShowOption(option)) {
    char fb[20];
    snprintf(fb, sizeof(fb), "SHOW %u", static_cast<unsigned>(option + 1U));
    self->setActionFeedback(fb, millis());
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
