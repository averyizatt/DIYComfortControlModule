#include <lvgl.h>

#include <cstdint>

#include "ui/DashboardTheme.hpp"
#include "ui/DashboardFonts.hpp"

#if LVGL_VERSION_MAJOR >= 9
#define CCM_PREVIEW_ACTIVE_SCREEN() lv_screen_active()
#define CCM_PREVIEW_BUTTON_CREATE(parent) lv_button_create(parent)
#else
#define CCM_PREVIEW_ACTIVE_SCREEN() lv_scr_act()
#define CCM_PREVIEW_BUTTON_CREATE(parent) lv_btn_create(parent)
#endif

namespace {

// Preview-only visual concept. These values intentionally do not affect the
// embedded dashboard until the design is approved on the desktop.
constexpr unsigned kBackground = 0x02070A;
constexpr unsigned kPanel = 0x071014;
constexpr unsigned kPanelRaised = 0x0A171C;
constexpr unsigned kDivider = 0x34434A;
constexpr unsigned kRed = 0xEF352F;
constexpr unsigned kCyan = 0x20D6E4;
constexpr unsigned kGreen = 0x42CB54;
constexpr unsigned kAmber = 0xF6B80B;
constexpr unsigned kBlue = 0x2798E5;
constexpr unsigned kPurple = 0x9850DD;
constexpr unsigned kMuted = 0x7F8B94;
constexpr unsigned kWhite = 0xF2F3F3;

const char* const kPageNames[] = {
    "DASH", "METH", "TAIL", "LEDS", "GPS", "TEMPS", "DIAG", "KNOCK"};
const char* const kNavSymbols[] = {
    LV_SYMBOL_BARS, LV_SYMBOL_TINT, LV_SYMBOL_LOOP, LV_SYMBOL_CHARGE,
    LV_SYMBOL_GPS, LV_SYMBOL_WARNING, LV_SYMBOL_LIST, LV_SYMBOL_WARNING};
constexpr unsigned kNavAccentColors[] = {
    kRed, kCyan, kRed, kAmber, kBlue, 0xFF4B20, kRed, kGreen};
constexpr unsigned kNavActiveBgColors[] = {
    0x10313A, 0x332A13, 0x351719, 0x281D3A,
    0x172743, 0x352015, 0x123125, 0x35172A};

lv_obj_t* s_pages[8] = {};
lv_obj_t* s_nav[8] = {};
lv_obj_t* s_title = nullptr;
lv_obj_t* s_clock = nullptr;
lv_obj_t* s_freshness = nullptr;
lv_obj_t* s_rpm = nullptr;
lv_obj_t* s_speed = nullptr;
lv_obj_t* s_boost = nullptr;
lv_obj_t* s_meth_summary = nullptr;
lv_obj_t* s_iat = nullptr;
lv_obj_t* s_battery = nullptr;
lv_obj_t* s_accel = nullptr;
lv_obj_t* s_gps_speed = nullptr;
lv_obj_t* s_gps_satellites = nullptr;
lv_obj_t* s_gps_hdop = nullptr;
lv_obj_t* s_page_data[8] = {};
lv_obj_t* s_temp_values[6] = {};
lv_obj_t* s_tail_main = nullptr;
lv_obj_t* s_tail_show = nullptr;
lv_obj_t* s_tail_show_page_label = nullptr;
lv_obj_t* s_tail_show_option_labels[6] = {};
lv_obj_t* s_led_main = nullptr;
lv_obj_t* s_led_show = nullptr;
lv_obj_t* s_diag_info = nullptr;
lv_obj_t* s_diag_tools = nullptr;
lv_obj_t* s_diag_storage = nullptr;
lv_obj_t* s_diag_trends = nullptr;
lv_obj_t* s_trend_charts[4] = {};
lv_chart_series_t* s_trend_a[4] = {};
lv_chart_series_t* s_trend_b[4] = {};
lv_obj_t* s_knock_energy = nullptr;
lv_obj_t* s_knock_baseline = nullptr;
lv_obj_t* s_knock_chart = nullptr;
lv_chart_series_t* s_knock_signal_series = nullptr;
lv_chart_series_t* s_knock_base_series = nullptr;
lv_obj_t* s_sd_log_title = nullptr;
lv_obj_t* s_sd_log_content = nullptr;
lv_obj_t* s_theme_tint = nullptr;
lv_obj_t* s_theme_button_label = nullptr;
lv_obj_t* s_alert_strip = nullptr;
lv_obj_t* s_alert_label = nullptr;
lv_obj_t* s_startup_overlay = nullptr;
lv_obj_t* s_startup_status = nullptr;
lv_obj_t* s_startup_progress = nullptr;
unsigned s_theme_mode = 0;
unsigned s_tail_show_page = 0;
uint32_t s_startup_begin = 0;
unsigned s_active_page = 0;

constexpr unsigned kTailShowOptionsPerPage = 6;
constexpr unsigned kTailShowOptionCount = 33;
constexpr unsigned kTailShowPageCount =
    (kTailShowOptionCount + kTailShowOptionsPerPage - 1U) / kTailShowOptionsPerPage;
const char* const kTailShowNames[kTailShowOptionCount] = {
    "Rainbow", "Chase", "Theater", "Fire", "Meteor", "Police",
    "Night Rider", "Color Cycle", "Sparkle", "Plasma", "Matrix", "Juggle",
    "BPM", "Confetti", "Ocean", "Lightning", "Heartbeat", "Ripple",
    "Sunrise", "Text Scroll", "Colorwaves", "TwinkleFox", "Bounce", "Fireworks",
    "Drip", "Cylon", "V8", "Drag Launch", "Neon", "Streaks", "Radar", "Aurora",
    "Glitch"};

void set_panel_style(lv_obj_t* obj) {
  lv_obj_set_style_bg_color(obj, lv_color_hex(kPanel), 0);
  lv_obj_set_style_bg_grad_color(obj, lv_color_hex(kPanelRaised), 0);
  lv_obj_set_style_bg_grad_dir(obj, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(obj, lv_color_hex(kDivider), 0);
  lv_obj_set_style_border_width(obj, 1, 0);
  lv_obj_set_style_radius(obj, 8, 0);
  lv_obj_set_style_pad_all(obj, 8, 0);
  lv_obj_set_style_pad_top(obj, 6, 0);
  lv_obj_set_style_pad_bottom(obj, 6, 0);
}

void accent_widget(lv_obj_t* obj, unsigned color, bool color_text = true) {
  if (!obj) return;
  lv_obj_set_style_border_color(obj, lv_color_hex(color), 0);
  if (color_text) lv_obj_set_style_text_color(obj, lv_color_hex(color), 0);
}

lv_obj_t* label(lv_obj_t* parent, const char* text, int x, int y,
                const lv_font_t* font = LV_FONT_DEFAULT,
                unsigned text_color = kWhite) {
  lv_obj_t* obj = lv_label_create(parent);
  lv_label_set_text(obj, text);
  lv_obj_set_style_text_font(obj, font, 0);
  lv_obj_set_style_text_color(obj, lv_color_hex(text_color), 0);
  lv_obj_set_pos(obj, x, y);
  return obj;
}

lv_obj_t* panel(lv_obj_t* parent, int x, int y, int width, int height) {
  lv_obj_t* obj = lv_obj_create(parent);
  lv_obj_set_pos(obj, x, y);
  lv_obj_set_size(obj, width, height);
  lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
  set_panel_style(obj);
  return obj;
}

lv_obj_t* build_metric(lv_obj_t* parent, int x, const char* value, const char* unit,
                       const char* caption) {
  lv_obj_t* tile = panel(parent, x, 8, 142, 105);
  lv_obj_t* value_label = label(tile, value, 8, 4, LV_FONT_DEFAULT, kCyan);
  label(tile, unit, 86, 18, LV_FONT_DEFAULT, kMuted);
  label(tile, caption, 8, 70, LV_FONT_DEFAULT, kMuted);
  return value_label;
}

lv_obj_t* metric_tile(lv_obj_t* parent, int x, int y, int width, int height,
                      const char* text, unsigned background = kPanel,
                      const lv_font_t* font = &lv_font_montserrat_20) {
  lv_obj_t* tile = label(parent, text, x, y, font);
  lv_obj_set_size(tile, width, height);
  lv_obj_set_style_text_align(tile, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(tile, LV_LABEL_LONG_WRAP);
  set_panel_style(tile);
  lv_obj_set_style_bg_color(tile, lv_color_hex(background), 0);
  unsigned lines = 1;
  for (const char* p = text; p && *p; ++p) {
    if (*p == '\n') ++lines;
  }
  const int line_height = lv_font_get_line_height(font);
  const int text_height = static_cast<int>(lines) * line_height +
                          static_cast<int>(lines - 1U) * 2;
  const int vertical_pad = height > text_height ? (height - text_height) / 2 : 2;
  lv_obj_set_style_text_line_space(tile, 2, 0);
  lv_obj_set_style_pad_top(tile, vertical_pad, 0);
  lv_obj_set_style_pad_bottom(tile, vertical_pad, 0);
  return tile;
}

lv_obj_t* action_button(lv_obj_t* parent, const char* text, int x, int y,
                        int width, int height, lv_event_cb_t callback = nullptr,
                        void* user_data = nullptr) {
  lv_obj_t* button = CCM_PREVIEW_BUTTON_CREATE(parent);
  lv_obj_set_pos(button, x, y);
  lv_obj_set_size(button, width, height);
  lv_obj_set_style_bg_color(button, lv_color_hex(kPanelRaised), 0);
  lv_obj_set_style_bg_color(button, lv_color_hex(0x1D3342), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(button, lv_color_hex(kDivider), 0);
  lv_obj_set_style_border_color(button, lv_color_hex(kCyan), LV_STATE_PRESSED);
  lv_obj_set_style_border_width(button, 1, 0);
  lv_obj_set_style_radius(button, 8, 0);
  lv_obj_set_style_shadow_width(button, 0, 0);
  lv_obj_t* text_obj = lv_label_create(button);
  lv_label_set_text(text_obj, text);
  lv_obj_set_style_text_color(text_obj, lv_color_hex(kWhite), 0);
  lv_obj_set_style_text_font(text_obj, &lv_font_montserrat_16, 0);
  lv_obj_center(text_obj);
  if (callback) lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, user_data);
  return button;
}

lv_obj_t* transparent_layer(lv_obj_t* parent) {
  lv_obj_t* layer = lv_obj_create(parent);
  lv_obj_set_pos(layer, 0, 0);
  lv_obj_set_size(layer, 472, 232);
  lv_obj_set_style_bg_opa(layer, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(layer, 0, 0);
  lv_obj_set_style_pad_all(layer, 0, 0);
  lv_obj_clear_flag(layer, LV_OBJ_FLAG_SCROLLABLE);
  return layer;
}

void show_layer(lv_obj_t* visible, lv_obj_t* hidden) {
  if (hidden) lv_obj_add_flag(hidden, LV_OBJ_FLAG_HIDDEN);
  if (visible) lv_obj_clear_flag(visible, LV_OBJ_FLAG_HIDDEN);
}

void update_tail_show_page() {
  for (unsigned i = 0; i < kTailShowOptionsPerPage; ++i) {
    const unsigned option = s_tail_show_page * kTailShowOptionsPerPage + i;
    if (s_tail_show_option_labels[i]) {
      lv_label_set_text(s_tail_show_option_labels[i],
                        option < kTailShowOptionCount ? kTailShowNames[option] : "N/A");
    }
  }
  if (s_tail_show_page_label) {
    lv_label_set_text_fmt(s_tail_show_page_label, "PAGE %u / %u",
                          s_tail_show_page + 1U, kTailShowPageCount);
  }
}

void tail_show_open(lv_event_t*) {
  s_tail_show_page = 0;
  update_tail_show_page();
  show_layer(s_tail_show, s_tail_main);
}
void tail_show_close(lv_event_t*) { show_layer(s_tail_main, s_tail_show); }
void tail_show_prev(lv_event_t*) {
  s_tail_show_page = s_tail_show_page == 0U
      ? kTailShowPageCount - 1U
      : s_tail_show_page - 1U;
  update_tail_show_page();
}
void tail_show_next(lv_event_t*) {
  s_tail_show_page = (s_tail_show_page + 1U) % kTailShowPageCount;
  update_tail_show_page();
}
void led_show_open(lv_event_t*) { show_layer(s_led_show, s_led_main); }
void led_show_close(lv_event_t*) { show_layer(s_led_main, s_led_show); }
void diag_info_open(lv_event_t*) {
  if (s_diag_storage) lv_obj_add_flag(s_diag_storage, LV_OBJ_FLAG_HIDDEN);
  if (s_diag_trends) lv_obj_add_flag(s_diag_trends, LV_OBJ_FLAG_HIDDEN);
  show_layer(s_diag_info, s_diag_tools);
}
void diag_tools_open(lv_event_t*) {
  if (s_diag_storage) lv_obj_add_flag(s_diag_storage, LV_OBJ_FLAG_HIDDEN);
  if (s_diag_trends) lv_obj_add_flag(s_diag_trends, LV_OBJ_FLAG_HIDDEN);
  show_layer(s_diag_tools, s_diag_info);
}
void diag_storage_open(lv_event_t*) {
  if (s_diag_info) lv_obj_add_flag(s_diag_info, LV_OBJ_FLAG_HIDDEN);
  if (s_diag_tools) lv_obj_add_flag(s_diag_tools, LV_OBJ_FLAG_HIDDEN);
  if (s_diag_trends) lv_obj_add_flag(s_diag_trends, LV_OBJ_FLAG_HIDDEN);
  if (s_diag_storage) lv_obj_clear_flag(s_diag_storage, LV_OBJ_FLAG_HIDDEN);
}
void diag_trends_open(lv_event_t*) {
  if (s_diag_info) lv_obj_add_flag(s_diag_info, LV_OBJ_FLAG_HIDDEN);
  if (s_diag_tools) lv_obj_add_flag(s_diag_tools, LV_OBJ_FLAG_HIDDEN);
  if (s_diag_storage) lv_obj_add_flag(s_diag_storage, LV_OBJ_FLAG_HIDDEN);
  if (s_diag_trends) lv_obj_clear_flag(s_diag_trends, LV_OBJ_FLAG_HIDDEN);
}

void theme_profile_cycle(lv_event_t*) {
  s_theme_mode = (s_theme_mode + 1U) % 3U;
  static const char* names[] = {"THEME AUTO", "THEME DAY", "THEME NIGHT"};
  if (s_theme_button_label) lv_label_set_text(s_theme_button_label, names[s_theme_mode]);
  const bool night = s_theme_mode == 2U ||
      (s_theme_mode == 0U && ((lv_tick_get() / 1000U / 20U) % 2U) != 0U);
  if (s_theme_tint) {
    lv_obj_set_style_bg_opa(s_theme_tint, night ? static_cast<lv_opa_t>(72) :
                            static_cast<lv_opa_t>(LV_OPA_TRANSP), 0);
  }
}

void sd_log_select(lv_event_t* event) {
  const unsigned index = static_cast<unsigned>(
      reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
  static const char* const titles[] = {
      "CAN / can_1242.csv", "GPS / gps_1242.csv",
      "METH / meth_1242.csv", "FAULTS / faults_1242.csv"};
  static const char* const contents[] = {
      "12:44:01  0x120  RPM  3250\n12:44:01  0x121  SPD  64\n12:44:02  0x130  BOOST  8.4\n12:44:02  0x140  STATUS  OK",
      "12:44:01  FIX  3D\n12:44:01  SAT  11/13\n12:44:02  LAT  39.739200\n12:44:02  LON -104.990300",
      "12:44:01  ARMED  YES\n12:44:01  DUTY   42%\n12:44:02  TANK   82%\n12:44:02  PRESS  118 psi",
      "12:40:18  CAN_TIMEOUT  CLEARED\n12:41:03  GPS_STALE    CLEARED\n12:44:02  ACTIVE_FAULTS  0"};
  if (index >= 4 || !s_sd_log_title || !s_sd_log_content) return;
  lv_label_set_text(s_sd_log_title, titles[index]);
  lv_label_set_text(s_sd_log_content, contents[index]);
}

lv_obj_t* sensor_card(lv_obj_t* parent, int x, int y, const char* name,
                      const char* value, unsigned accent) {
  lv_obj_t* card = panel(parent, x, y, 220, 52);
  lv_obj_set_style_border_color(card, lv_color_hex(accent), 0);
  lv_obj_set_style_border_side(card, LV_BORDER_SIDE_LEFT, 0);
  lv_obj_set_style_border_width(card, 3, 0);
  label(card, name, 8, 3, &lv_font_montserrat_12, kMuted);
  lv_obj_t* value_label = label(card, value, 8, 19, &lv_font_montserrat_20, accent);
  lv_obj_t* status = label(card, "OK", 176, 18, &lv_font_montserrat_12, kGreen);
  lv_obj_set_style_bg_color(status, lv_color_hex(0x123426), 0);
  lv_obj_set_style_bg_opa(status, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(status, 7, 0);
  lv_obj_set_style_pad_left(status, 6, 0);
  lv_obj_set_style_pad_right(status, 6, 0);
  lv_obj_set_style_pad_top(status, 2, 0);
  lv_obj_set_style_pad_bottom(status, 2, 0);
  return value_label;
}

void led_zone_card(lv_obj_t* parent, int x, int y, const char* name,
                   bool high_selected, unsigned accent) {
  lv_obj_t* card = panel(parent, x, y, 228, 48);
  lv_obj_set_style_pad_all(card, 0, 0);
  lv_obj_set_style_border_color(card, lv_color_hex(kDivider), 0);

  lv_obj_t* marker = lv_obj_create(card);
  lv_obj_set_pos(marker, 5, 8);
  lv_obj_set_size(marker, 5, 32);
  lv_obj_set_style_bg_color(marker, lv_color_hex(accent), 0);
  lv_obj_set_style_bg_opa(marker, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(marker, 0, 0);
  lv_obj_set_style_radius(marker, 3, 0);

  label(card, name, 18, 17, &lv_font_montserrat_12, kWhite);

  const char* modes[] = {"LOW", "HIGH"};
  for (unsigned i = 0; i < 2; ++i) {
    const bool selected = high_selected == (i == 1U);
    lv_obj_t* mode_button = action_button(card, modes[i], 106 + static_cast<int>(i) * 58,
                                          8, 54, 32);
    lv_obj_set_style_bg_color(mode_button,
        lv_color_hex(selected ? 0x1D3342 : 0x101A24), 0);
    lv_obj_set_style_border_color(mode_button,
        lv_color_hex(selected ? accent : kDivider), 0);
    lv_obj_t* mode_label = lv_obj_get_child(mode_button, 0);
    if (mode_label) {
      lv_obj_set_style_text_color(mode_label,
          lv_color_hex(selected ? accent : kMuted), 0);
      lv_obj_set_style_text_font(mode_label, &lv_font_montserrat_12, 0);
    }
  }
}

void build_dashboard(lv_obj_t* page) {
  s_speed = metric_tile(page, 8, 8, 226, 126, "SPEED\n52\nMPH", 0x040A0D,
                        &ccm_font_semibold_48);
  s_rpm = metric_tile(page, 8, 142, 109, 74, "RPM\n3864", kPanel, &ccm_font_semibold_20);
  s_iat = metric_tile(page, 125, 142, 109, 74, "FUEL\n49%", kPanel, &ccm_font_semibold_20);
  s_boost = metric_tile(page, 242, 8, 110, 62, "BOOST\n6.4 PSI", kPanel, &ccm_font_semibold_20);
  s_meth_summary = metric_tile(page, 360, 8, 112, 62, "METH ARM\n4%", kPanel, &ccm_font_semibold_20);
  metric_tile(page, 242, 78, 110, 62, "TANK\n82%", kPanel, &ccm_font_semibold_20);
  metric_tile(page, 360, 78, 112, 62, "KNOCK\nOK", kPanel, &ccm_font_semibold_20);
  s_battery = metric_tile(page, 242, 148, 110, 68, "OIL\n63 PSI", kPanel, &ccm_font_semibold_20);
  s_accel = metric_tile(page, 360, 148, 112, 68, "CAN BUS\nCAN OK", kPanel, &ccm_font_semibold_16);
  accent_widget(s_speed, kDivider, false);
  accent_widget(s_rpm, kRed);
  accent_widget(s_iat, kPurple);
  accent_widget(s_boost, kAmber);
  accent_widget(s_meth_summary, kGreen);
  accent_widget(lv_obj_get_child(page, 5), kBlue);
  accent_widget(lv_obj_get_child(page, 6), kGreen);
  accent_widget(s_battery, kAmber);
  accent_widget(s_accel, kGreen);
  return;

  s_speed = metric_tile(page, 8, 8, 224, 142, "64\nMPH", ui::dashboard_theme::heroPanel,
                        &ccm_font_semibold_48);
  s_boost = metric_tile(page, 240, 8, 112, 66, "BOOST\n8.4 PSI", kPanel, &ccm_font_semibold_20);
  s_meth_summary = metric_tile(page, 360, 8, 112, 66, "METH\nARM", kPanel, &ccm_font_semibold_20);
  metric_tile(page, 240, 82, 112, 66, "TANK\n82%", kPanel, &ccm_font_semibold_20);
  metric_tile(page, 360, 82, 112, 66, "KNOCK\nOK", kPanel, &ccm_font_semibold_20);
  s_rpm = metric_tile(page, 8, 158, 108, 58, "RPM 3250", kPanel, &ccm_font_semibold_16);
  s_iat = metric_tile(page, 124, 158, 108, 58, "FUEL 48", kPanel, &ccm_font_semibold_16);
  s_battery = metric_tile(page, 240, 158, 112, 58, "OIL 62", kPanel, &ccm_font_semibold_16);
  s_accel = metric_tile(page, 360, 158, 112, 58, "CAN OK", kPanel, &ccm_font_semibold_16);
  lv_obj_t* dash_text[] = {s_speed, s_boost, s_meth_summary,
      lv_obj_get_child(page, 3), lv_obj_get_child(page, 4),
      s_rpm, s_iat, s_battery, s_accel};
  for (lv_obj_t* obj : dash_text) {
    if (obj) lv_obj_set_style_text_letter_space(obj, 0, 0);
  }
  lv_obj_set_style_bg_color(s_speed, lv_color_hex(0x101E2A), 0);
  lv_obj_set_style_bg_grad_color(s_speed, lv_color_hex(0x142A38), 0);
  accent_widget(s_speed, kDivider, false);
  lv_obj_set_style_text_color(s_speed, lv_color_hex(kWhite), 0);
  accent_widget(s_boost, kAmber);
  accent_widget(s_meth_summary, kGreen);
  accent_widget(lv_obj_get_child(page, 3), kBlue);
  accent_widget(lv_obj_get_child(page, 4), kGreen);
  accent_widget(s_rpm, kRed);
  accent_widget(s_iat, kPurple);
  accent_widget(s_battery, kAmber);
  accent_widget(s_accel, kGreen);
}

void build_generic_page(lv_obj_t* page, unsigned index) {
  if (index == 1) {
    s_page_data[index] = metric_tile(page, 8, 8, 226, 142,
        "INJECTION STATUS\n42%", 0x040A0D, &ccm_font_semibold_48);
    metric_tile(page, 24, 116, 194, 26, "SYSTEM STANDBY", 0x071014,
                &lv_font_montserrat_12);
    metric_tile(page, 242, 8, 110, 62, "PUMP DUTY\n42%", kPanel, &ccm_font_semibold_20);
    metric_tile(page, 360, 8, 112, 62, "TANK LEVEL\n82%", kPanel, &ccm_font_semibold_20);
    metric_tile(page, 242, 78, 110, 62, "LINE PRESSURE\n141 PSI", kPanel, &ccm_font_semibold_20);
    metric_tile(page, 360, 78, 112, 62, "BOOST TRIGGER\n3.5 PSI", kPanel, &ccm_font_semibold_16);
    metric_tile(page, 242, 148, 110, 68, "FLOW RATE\nLOW", kPanel, &ccm_font_semibold_16);
    metric_tile(page, 360, 148, 112, 68, "IAT\n78 F", kPanel, &ccm_font_semibold_20);
    metric_tile(page, 8, 158, 108, 26, "SENSORS OK", kPanel, &lv_font_montserrat_12);
    metric_tile(page, 124, 158, 110, 26, "ARMED", kPanel, &lv_font_montserrat_12);
    action_button(page, "DISARM", 8, 190, 108, 26);
    action_button(page, "RATIO 50%", 124, 190, 110, 26);
    accent_widget(s_page_data[index], kGreen);
    return;

    metric_tile(page, 8, 8, 142, 30, "WATER-METH", kPanel, &lv_font_montserrat_12);
    metric_tile(page, 158, 8, 94, 30, "ONLINE", kPanel, &lv_font_montserrat_12);
    metric_tile(page, 260, 8, 96, 30, "EXT OK", kPanel, &lv_font_montserrat_12);
    metric_tile(page, 364, 8, 104, 30, "ARM ON", kPanel, &lv_font_montserrat_12);
    metric_tile(page, 8, 48, 110, 70, "DUTY\n42%");
    metric_tile(page, 126, 48, 110, 70, "TANK\n82%");
    metric_tile(page, 244, 48, 110, 70, "BOOST\n58 kPa");
    metric_tile(page, 362, 48, 106, 70, "METH\n118 psi");
    metric_tile(page, 8, 128, 226, 34, "IAT 90 F");
    metric_tile(page, 242, 128, 226, 34, "BAY 106 F");
    action_button(page, "DISARM", 8, 178, 226, 44);
    action_button(page, "RATIO 50%", 242, 178, 226, 44);
    s_page_data[index] = lv_obj_get_child(page, 4);
    accent_widget(lv_obj_get_child(page, 0), kCyan);
    accent_widget(lv_obj_get_child(page, 1), kGreen);
    accent_widget(lv_obj_get_child(page, 2), kGreen);
    accent_widget(lv_obj_get_child(page, 3), kAmber);
    accent_widget(lv_obj_get_child(page, 4), kCyan);
    accent_widget(lv_obj_get_child(page, 5), kBlue);
    accent_widget(lv_obj_get_child(page, 6), kAmber);
    accent_widget(lv_obj_get_child(page, 7), kPurple);
    accent_widget(lv_obj_get_child(page, 8), kCyan);
    accent_widget(lv_obj_get_child(page, 9), kAmber);
    accent_widget(lv_obj_get_child(page, 10), kRed, false);
    accent_widget(lv_obj_get_child(page, 11), kCyan, false);
  } else if (index == 2) {
    s_tail_main = transparent_layer(page);
    metric_tile(s_tail_main, 8, 8, 380, 30, "TAILS  ONLINE  SEQ  BRI 180", kPanel,
                &lv_font_montserrat_12);
    lv_obj_t* led = lv_led_create(s_tail_main);
    lv_obj_set_pos(led, 438, 11);
    lv_obj_set_size(led, 18, 18);
    lv_led_on(led);
    action_button(s_tail_main, "STOCK", 8, 48, 224, 72);
    action_button(s_tail_main, "SEQ", 240, 48, 224, 72);
    action_button(s_tail_main, "SHOW", 8, 128, 224, 72, tail_show_open);
    action_button(s_tail_main, "DEMO", 240, 128, 224, 72);
    accent_widget(lv_obj_get_child(s_tail_main, 0), kGreen);
    accent_widget(lv_obj_get_child(s_tail_main, 2), kMuted, false);
    accent_widget(lv_obj_get_child(s_tail_main, 3), kCyan, false);
    accent_widget(lv_obj_get_child(s_tail_main, 4), kPurple, false);
    accent_widget(lv_obj_get_child(s_tail_main, 5), kAmber, false);

    s_tail_show = transparent_layer(page);
    lv_obj_add_flag(s_tail_show, LV_OBJ_FLAG_HIDDEN);
    label(s_tail_show, "TAILLIGHT SHOW PRESETS", 8, 8, &lv_font_montserrat_18, kPurple);
    action_button(s_tail_show, "BACK", 372, 4, 92, 32, tail_show_close);
    const unsigned preset_colors[] = {kCyan, kRed, kBlue, kAmber, kPurple, kWhite};
    for (unsigned i = 0; i < 6; ++i) {
      const int x = 8 + static_cast<int>(i % 3U) * 154;
      const int y = 48 + static_cast<int>(i / 3U) * 72;
      lv_obj_t* preset = action_button(s_tail_show, kTailShowNames[i], x, y, 144, 62);
      accent_widget(preset, preset_colors[i], false);
      lv_obj_t* preset_label = lv_obj_get_child(preset, 0);
      s_tail_show_option_labels[i] = preset_label;
      if (preset_label) lv_obj_set_style_text_font(preset_label, &lv_font_montserrat_12, 0);
    }
    action_button(s_tail_show, "PREV", 8, 198, 100, 30, tail_show_prev);
    s_tail_show_page_label = label(s_tail_show, "PAGE 1 / 6", 190, 205,
                                   &lv_font_montserrat_12, kMuted);
    action_button(s_tail_show, "NEXT", 364, 198, 100, 30, tail_show_next);
  } else if (index == 3) {
    s_led_main = transparent_layer(page);
    lv_obj_t* header = panel(s_led_main, 8, 28, 464, 36);
    lv_obj_set_style_pad_all(header, 0, 0);
    label(header, "INTERIOR LIGHTING", 12, 8, &lv_font_montserrat_14, kWhite);
    label(header, "MASTER", 310, 10, &lv_font_montserrat_12, kMuted);
    lv_obj_t* master = lv_switch_create(header);
    lv_obj_set_pos(master, 384, 4);
    lv_obj_set_size(master, 70, 28);
    lv_obj_add_state(master, LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(master, lv_color_hex(0x162532), LV_PART_MAIN);
    lv_obj_set_style_bg_color(master, lv_color_hex(kGreen),
        static_cast<lv_style_selector_t>(LV_PART_INDICATOR) | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(master, lv_color_hex(kWhite), LV_PART_KNOB);
    accent_widget(header, kGreen, false);

    led_zone_card(s_led_main, 8, 72, "ALL ZONES", true, kRed);
    led_zone_card(s_led_main, 8, 126, "DRIVER", false, kCyan);
    led_zone_card(s_led_main, 8, 180, "PASSENGER", false, kBlue);
    led_zone_card(s_led_main, 244, 72, "ROOF / TOP", true, kPurple);
    led_zone_card(s_led_main, 244, 126, "FOOTWELL", false, kAmber);

    lv_obj_t* scene = panel(s_led_main, 244, 180, 228, 48);
    lv_obj_set_style_pad_all(scene, 0, 0);
    label(scene, "ACTIVE SCENE", 14, 5, &lv_font_montserrat_12, kMuted);
    label(scene, "RAINBOW", 14, 25, &lv_font_montserrat_12, kPurple);
    lv_obj_t* scene_button = action_button(scene, "EDIT", 154, 8, 64, 32, led_show_open);
    accent_widget(scene_button, kPurple, false);

    s_led_show = transparent_layer(page);
    lv_obj_add_flag(s_led_show, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t* show_header = panel(s_led_show, 8, 28, 464, 36);
    lv_obj_set_style_pad_all(show_header, 0, 0);
    label(show_header, "INTERIOR SHOW SCENES", 12, 9, &lv_font_montserrat_14, kPurple);
    action_button(show_header, "ON", 204, 3, 54, 30);
    action_button(show_header, "OFF", 264, 3, 54, 30);
    action_button(show_header, "CLEAR", 324, 3, 62, 30);
    action_button(show_header, "BACK", 392, 3, 64, 30, led_show_close);
    const char* scenes[] = {"RAINBOW", "BREATHE", "COLOR", "CHASE", "SPARKLE"};
    const unsigned scene_colors[] = {kPurple, kCyan, kBlue, kAmber, kGreen};
    for (unsigned i = 0; i < 5; ++i) {
      const int x = 8 + static_cast<int>(i % 3U) * 154;
      const int y = 72 + static_cast<int>(i / 3U) * 46;
      lv_obj_t* scene_mode = action_button(s_led_show, scenes[i], x, y, 144, 38);
      accent_widget(scene_mode, scene_colors[i], false);
      lv_obj_t* scene_label = lv_obj_get_child(scene_mode, 0);
      if (scene_label) lv_obj_set_style_text_font(scene_label, &lv_font_montserrat_12, 0);
    }
    label(s_led_show, "STATIC COLOR", 8, 177, &lv_font_montserrat_12, kMuted);
    const unsigned colors[] = {kWhite, kRed, kAmber, kGreen, kBlue, kPurple};
    for (unsigned i = 0; i < 6; ++i) {
      lv_obj_t* swatch = CCM_PREVIEW_BUTTON_CREATE(s_led_show);
      lv_obj_set_pos(swatch, 112 + static_cast<int>(i) * 56, 168);
      lv_obj_set_size(swatch, 44, 34);
      lv_obj_set_style_bg_color(swatch, lv_color_hex(colors[i]), 0);
      lv_obj_set_style_border_color(swatch, lv_color_hex(kWhite), 0);
      lv_obj_set_style_border_width(swatch, 1, 0);
      lv_obj_set_style_radius(swatch, 8, 0);
      lv_obj_set_style_shadow_width(swatch, 0, 0);
    }
  } else if (index == 4) {
    lv_obj_t* receiver = metric_tile(page, 8, 6, 464, 32,
        "GNSS RECEIVER     3D FIX     SIGNAL EXCELLENT", kPanel,
        &lv_font_montserrat_12);
    accent_widget(receiver, kGreen);

    lv_obj_t* speed_card = panel(page, 8, 46, 180, 168);
    accent_widget(speed_card, kCyan, false);
    label(speed_card, "GROUND SPEED", 18, 12, &lv_font_montserrat_12, kMuted);
    s_gps_speed = label(speed_card, "64", 28, 44, &lv_font_montserrat_48, kCyan);
    lv_obj_set_width(s_gps_speed, 108);
    lv_obj_set_style_text_align(s_gps_speed, LV_TEXT_ALIGN_CENTER, 0);
    label(speed_card, "MPH", 65, 108, &lv_font_montserrat_18, kWhite);

    lv_obj_t* quality = panel(page, 196, 46, 276, 76);
    label(quality, "SATELLITES", 10, 7, &lv_font_montserrat_12, kMuted);
    s_gps_satellites = label(quality, "11 USED / 13 VIEW", 10, 28,
                             &lv_font_montserrat_20, kBlue);
    label(quality, "HDOP", 202, 7, &lv_font_montserrat_12, kMuted);
    s_gps_hdop = label(quality, "0.8", 210, 29, &lv_font_montserrat_18, kGreen);
    accent_widget(quality, kBlue, false);

    lv_obj_t* coordinates = panel(page, 196, 130, 276, 84);
    label(coordinates, "CURRENT POSITION", 10, 6, &lv_font_montserrat_12, kMuted);
    label(coordinates, "LAT", 10, 30, &lv_font_montserrat_12, kCyan);
    label(coordinates, "39.739200", 48, 27, &lv_font_montserrat_18, kWhite);
    label(coordinates, "LON", 10, 55, &lv_font_montserrat_12, kPurple);
    label(coordinates, "-104.990300", 48, 52, &lv_font_montserrat_18, kWhite);
    accent_widget(coordinates, kPurple, false);
  } else if (index == 5) {
    metric_tile(page, 8, 4, 464, 26, "THERMAL SYSTEMS", kPanel, &lv_font_montserrat_12);
    s_temp_values[1] = metric_tile(page, 8, 34, 226, 182,
        "ENGINE BAY TEMP\n188 F\nMIN 156  MAX 205", 0x040A0D, &ccm_font_semibold_48);
    s_temp_values[0] = metric_tile(page, 242, 34, 110, 54, "INTAKE AIR\n84 F", kPanel, &ccm_font_semibold_20);
    s_temp_values[3] = metric_tile(page, 360, 34, 112, 54, "AMBIENT\n92 F", kPanel, &ccm_font_semibold_20);
    s_temp_values[2] = metric_tile(page, 242, 94, 110, 54, "CABIN\n75 F", kPanel, &ccm_font_semibold_20);
    s_temp_values[4] = metric_tile(page, 360, 94, 112, 54, "OIL PRESSURE\n63 PSI", kPanel, &ccm_font_semibold_16);
    s_temp_values[5] = metric_tile(page, 242, 154, 110, 62, "FUEL PRESSURE\n49 PSI", kPanel, &ccm_font_semibold_16);
    metric_tile(page, 360, 154, 112, 62, "SYSTEM STATUS\nNORMAL", kPanel, &ccm_font_semibold_16);
    s_page_data[index] = s_temp_values[0];
    return;

    lv_obj_t* status = metric_tile(page, 8, 6, 464, 32,
        "ENVIRONMENT SENSORS     6 / 6 ONLINE", kPanel, &lv_font_montserrat_12);
    accent_widget(status, kGreen);
    s_temp_values[0] = sensor_card(page, 8, 46, "INTAKE AIR", "89.6 F", kCyan);
    s_temp_values[1] = sensor_card(page, 244, 46, "ENGINE BAY", "105.8 F", kAmber);
    s_temp_values[2] = sensor_card(page, 8, 104, "CABIN", "71.6 F", kGreen);
    s_temp_values[3] = sensor_card(page, 244, 104, "AMBIENT", "64.4 F", kBlue);
    s_temp_values[4] = sensor_card(page, 8, 162, "OIL PRESSURE", "62.0 psi", kPurple);
    s_temp_values[5] = sensor_card(page, 244, 162, "FUEL PRESSURE", "48.0 psi", kRed);
  } else if (index == 6) {
    metric_tile(page, 8, 8, 226, 112, "HEALTH\n98%\nALL SYSTEMS OK", 0x040A0D, &ccm_font_semibold_20);
    metric_tile(page, 242, 8, 110, 54, "CAN BUS\nCAN OK", kPanel, &lv_font_montserrat_16);
    metric_tile(page, 360, 8, 112, 54, "GPS LOCK\n9 SAT", kPanel, &lv_font_montserrat_16);
    metric_tile(page, 242, 70, 110, 50, "SD LOGGING\nACTIVE", kPanel, &lv_font_montserrat_16);
    metric_tile(page, 360, 70, 112, 50, "ECU SYNC\nGOOD", kPanel, &lv_font_montserrat_16);
    metric_tile(page, 8, 128, 109, 54, "SENSOR HEALTH\n8 OF 8 OK", kPanel, &lv_font_montserrat_16);
    metric_tile(page, 125, 128, 109, 54, "METH CONTROLLER\nREADY", kPanel, &lv_font_montserrat_16);
    metric_tile(page, 242, 128, 110, 54, "BATTERY\n13.9 V", kPanel, &lv_font_montserrat_16);
    metric_tile(page, 360, 128, 112, 54, "FAULT HISTORY\nNONE", kPanel, &lv_font_montserrat_16);
    s_page_data[index] = metric_tile(page, 8, 190, 464, 26,
        "RECENT EVENTS     NO ACTIVE FAULTS", kPanel, &lv_font_montserrat_12);
    return;

    s_diag_info = transparent_layer(page);
    action_button(s_diag_info, "INFO", 8, 6, 108, 32, diag_info_open);
    action_button(s_diag_info, "TOOLS", 122, 6, 108, 32, diag_tools_open);
    action_button(s_diag_info, "STORAGE", 236, 6, 108, 32, diag_storage_open);
    action_button(s_diag_info, "TRENDS", 350, 6, 108, 32, diag_trends_open);
    const char* diag_text[] = {
      "CAN\nRX OK\n1248 FRAMES", "GPS\n3D FIX\n11 SAT",
      "METH\nONLINE\nDUTY 42%", "TAILS\nONLINE\nMODE SEQ",
      "STORAGE\nMOUNTED\nLOGGING", "SYSTEM\nRUNNING\n184K HEAP"
    };
    const unsigned diag_colors[] = {kCyan, kBlue, kGreen, kPurple, kAmber, kRed};
    for (unsigned i = 0; i < 6; ++i) {
      const int x = 4 + static_cast<int>(i % 3U) * 156;
      const int y = 44 + static_cast<int>(i / 3U) * 88;
      lv_obj_t* card = metric_tile(s_diag_info, x, y, 148, 80, diag_text[i],
                                   kPanel, &lv_font_montserrat_14);
      accent_widget(card, diag_colors[i]);
      if (i == 5U) s_page_data[index] = card;
    }
    accent_widget(lv_obj_get_child(s_diag_info, 0), kCyan, false);
    accent_widget(lv_obj_get_child(s_diag_info, 1), kPurple, false);

    s_diag_tools = transparent_layer(page);
    lv_obj_add_flag(s_diag_tools, LV_OBJ_FLAG_HIDDEN);
    action_button(s_diag_tools, "INFO", 8, 6, 108, 32, diag_info_open);
    action_button(s_diag_tools, "TOOLS", 122, 6, 108, 32, diag_tools_open);
    action_button(s_diag_tools, "STORAGE", 236, 6, 108, 32, diag_storage_open);
    action_button(s_diag_tools, "TRENDS", 350, 6, 108, 32, diag_trends_open);
    lv_obj_t* faults = action_button(s_diag_tools, "FAULTS", 8, 48, 108, 42);
    lv_obj_t* can_ping = action_button(s_diag_tools, "CAN PING", 124, 48, 108, 42);
    lv_obj_t* led_test = action_button(s_diag_tools, "LED TEST", 240, 48, 108, 42);
    lv_obj_t* restart = action_button(s_diag_tools, "RESTART", 356, 48, 108, 42);
    accent_widget(faults, kAmber, false);
    accent_widget(can_ping, kCyan, false);
    accent_widget(led_test, kPurple, false);
    accent_widget(restart, kRed, false);
    lv_obj_t* storage = panel(s_diag_tools, 8, 100, 456, 72);
    label(storage, "SD CARD", 10, 8, &lv_font_montserrat_12, kMuted);
    label(storage, "MOUNTED", 10, 28, &lv_font_montserrat_18, kGreen);
    label(storage, "LOGGING ACTIVE", 148, 10, &lv_font_montserrat_12, kCyan);
    label(storage, "/logs/preview.csv", 148, 34, &lv_font_montserrat_12, kWhite);
    action_button(s_diag_tools, "TOUCH CAL", 8, 182, 108, 38);
    lv_obj_t* theme = action_button(s_diag_tools, "THEME AUTO", 124, 182, 108, 38,
                                    theme_profile_cycle);
    s_theme_button_label = lv_obj_get_child(theme, 0);
    action_button(s_diag_tools, "RACE", 240, 182, 108, 38);
    action_button(s_diag_tools, "SD READ", 356, 182, 108, 38, diag_storage_open);

    s_diag_storage = transparent_layer(page);
    lv_obj_add_flag(s_diag_storage, LV_OBJ_FLAG_HIDDEN);
    action_button(s_diag_storage, "INFO", 8, 6, 108, 32, diag_info_open);
    action_button(s_diag_storage, "TOOLS", 122, 6, 108, 32, diag_tools_open);
    action_button(s_diag_storage, "STORAGE", 236, 6, 108, 32, diag_storage_open);
    action_button(s_diag_storage, "TRENDS", 350, 6, 108, 32, diag_trends_open);

    lv_obj_t* browser = panel(s_diag_storage, 8, 46, 170, 176);
    label(browser, "LOG FILES", 8, 5, &lv_font_montserrat_12, kMuted);
    const char* files[] = {"CAN  128 KB", "GPS   32 KB", "METH  46 KB", "FAULTS  4 KB"};
    for (unsigned i = 0; i < 4; ++i) {
      lv_obj_t* file = action_button(browser, files[i], 6, 28 + static_cast<int>(i) * 34,
                                     150, 28, sd_log_select,
                                     reinterpret_cast<void*>(static_cast<uintptr_t>(i)));
      lv_obj_t* file_label = lv_obj_get_child(file, 0);
      if (file_label) lv_obj_set_style_text_font(file_label, &lv_font_montserrat_12, 0);
    }

    lv_obj_t* viewer = panel(s_diag_storage, 186, 46, 286, 176);
    s_sd_log_title = label(viewer, "CAN / can_1242.csv", 8, 5,
                           &lv_font_montserrat_12, kCyan);
    lv_obj_t* divider = lv_obj_create(viewer);
    lv_obj_set_pos(divider, 8, 27);
    lv_obj_set_size(divider, 258, 1);
    lv_obj_set_style_bg_color(divider, lv_color_hex(kDivider), 0);
    lv_obj_set_style_bg_opa(divider, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(divider, 0, 0);
    s_sd_log_content = label(viewer,
        "12:44:01  0x120  RPM  3250\n12:44:01  0x121  SPD  64\n"
        "12:44:02  0x130  BOOST  8.4\n12:44:02  0x140  STATUS  OK",
        8, 36, &lv_font_montserrat_12, kWhite);
    lv_obj_set_width(s_sd_log_content, 258);
    lv_label_set_long_mode(s_sd_log_content, LV_LABEL_LONG_WRAP);

    s_diag_trends = transparent_layer(page);
    lv_obj_add_flag(s_diag_trends, LV_OBJ_FLAG_HIDDEN);
    action_button(s_diag_trends, "INFO", 8, 6, 108, 32, diag_info_open);
    action_button(s_diag_trends, "TOOLS", 122, 6, 108, 32, diag_tools_open);
    action_button(s_diag_trends, "STORAGE", 236, 6, 108, 32, diag_storage_open);
    action_button(s_diag_trends, "TRENDS", 350, 6, 108, 32, diag_trends_open);
    const char* trend_names[] = {
      "ENGINE  RPM / BOOST", "PRESSURE  OIL / FUEL",
      "THERMAL  IAT / BAY", "KNOCK  ENERGY / BASE"
    };
    const unsigned trend_a_colors[] = {kCyan, kPurple, kBlue, kAmber};
    const unsigned trend_b_colors[] = {kAmber, kGreen, kAmber, kPurple};
    for (unsigned i = 0; i < 4; ++i) {
      const int x = 4 + static_cast<int>(i % 2U) * 234;
      const int y = 44 + static_cast<int>(i / 2U) * 92;
      lv_obj_t* card = panel(s_diag_trends, x, y, 226, 86);
      label(card, trend_names[i], 5, 1, &lv_font_montserrat_12, kMuted);
      s_trend_charts[i] = lv_chart_create(card);
      lv_obj_set_pos(s_trend_charts[i], 5, 21);
      lv_obj_set_size(s_trend_charts[i], 202, 52);
      lv_chart_set_type(s_trend_charts[i], LV_CHART_TYPE_LINE);
      lv_chart_set_point_count(s_trend_charts[i], 48);
      lv_chart_set_range(s_trend_charts[i], LV_CHART_AXIS_PRIMARY_Y, 0, 100);
      lv_chart_set_div_line_count(s_trend_charts[i], 3, 4);
      lv_obj_set_style_bg_color(s_trend_charts[i], lv_color_hex(0x09131C), 0);
      lv_obj_set_style_border_width(s_trend_charts[i], 0, 0);
      lv_obj_set_style_line_width(s_trend_charts[i], 2, LV_PART_ITEMS);
      s_trend_a[i] = lv_chart_add_series(s_trend_charts[i],
          lv_color_hex(trend_a_colors[i]), LV_CHART_AXIS_PRIMARY_Y);
      s_trend_b[i] = lv_chart_add_series(s_trend_charts[i],
          lv_color_hex(trend_b_colors[i]), LV_CHART_AXIS_PRIMARY_Y);
    }
  } else if (index == 7) {
    lv_obj_t* knock_left = panel(page, 8, 8, 220, 208);
    label(knock_left, "KNOCK STATUS", 10, 4, &lv_font_montserrat_12, kMuted);
    s_page_data[index] = label(knock_left, "NO KNOCK", 12, 28, &ccm_font_semibold_48, kGreen);
    label(knock_left, "SENSOR OK  |  MONITOR ACTIVE", 12, 82, &lv_font_montserrat_12, kMuted);
    s_knock_chart = lv_chart_create(knock_left);
    lv_obj_set_pos(s_knock_chart, 8, 112);
    lv_obj_set_size(s_knock_chart, 196, 72);
    lv_chart_set_type(s_knock_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(s_knock_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_chart_set_point_count(s_knock_chart, 32);
    s_knock_signal_series = lv_chart_add_series(s_knock_chart, lv_color_hex(kGreen), LV_CHART_AXIS_PRIMARY_Y);
    s_knock_base_series = lv_chart_add_series(s_knock_chart, lv_color_hex(kCyan), LV_CHART_AXIS_PRIMARY_Y);
    s_knock_energy = metric_tile(page, 236, 8, 114, 48, "CURRENT LEVEL\n0.12 V", kPanel, &ccm_font_semibold_20);
    s_knock_baseline = metric_tile(page, 358, 8, 114, 48, "PEAK LEVEL\n0.28 V", kPanel, &ccm_font_semibold_20);
    metric_tile(page, 236, 64, 114, 48, "TIMING RETARD\n0.0 DEG", kPanel, &ccm_font_semibold_20);
    metric_tile(page, 358, 64, 114, 48, "NOISE FLOOR\n0.08 V", kPanel, &ccm_font_semibold_20);
    metric_tile(page, 236, 120, 114, 48, "LEARNED THRESHOLD\n0.35 V", kPanel, &lv_font_montserrat_16);
    metric_tile(page, 358, 120, 114, 48, "ENGINE LOAD\n72 kPa", kPanel, &lv_font_montserrat_16);
    action_button(page, "ENABLED", 236, 176, 114, 40);
    action_button(page, "RELEARN", 358, 176, 114, 40);
    return;

    lv_obj_t* status = metric_tile(page, 8, 28, 456, 30,
        "KNOCK MONITOR   |   ONLINE   |   WARN ONLY", kPanel, &lv_font_montserrat_12);
    accent_widget(status, kGreen);
    lv_obj_t* signal = panel(page, 8, 66, 292, 104);
    label(signal, "SENSOR OK   BASELINE LEARNED", 8, 2, &lv_font_montserrat_12, kMuted);
    label(signal, "ENERGY  18.0     18% OF LIMIT", 8, 20, &lv_font_montserrat_12, kWhite);
    s_knock_energy = lv_bar_create(signal);
    lv_obj_set_pos(s_knock_energy, 8, 35); lv_obj_set_size(s_knock_energy, 260, 7);
    lv_bar_set_value(s_knock_energy, 18, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_knock_energy, lv_color_hex(0x132331), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_knock_energy, lv_color_hex(kGreen), LV_PART_INDICATOR);
    label(signal, "BASELINE  11.0     11% OF LIMIT", 8, 45, &lv_font_montserrat_12, kWhite);
    s_knock_baseline = lv_bar_create(signal);
    lv_obj_set_pos(s_knock_baseline, 8, 60); lv_obj_set_size(s_knock_baseline, 260, 7);
    lv_bar_set_value(s_knock_baseline, 11, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_knock_baseline, lv_color_hex(0x132331), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_knock_baseline, lv_color_hex(kCyan), LV_PART_INDICATOR);
    label(signal, "LIMIT  35.0     MULT x2.5  OFFSET 8.0", 8, 70, &lv_font_montserrat_12, kWhite);
    lv_obj_t* limit = lv_bar_create(signal);
    lv_obj_set_pos(limit, 8, 87); lv_obj_set_size(limit, 260, 7);
    lv_bar_set_value(limit, 100, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(limit, lv_color_hex(0x132331), LV_PART_MAIN);
    lv_obj_set_style_bg_color(limit, lv_color_hex(kAmber), LV_PART_INDICATOR);

    s_page_data[index] = metric_tile(page, 308, 66, 156, 48, "CLEAN", kPanel,
                                     &lv_font_montserrat_20);
    accent_widget(s_page_data[index], kGreen);
    label(page, "LIVE SIGNAL", 312, 120, &lv_font_montserrat_12, kMuted);
    s_knock_chart = lv_chart_create(page);
    lv_obj_set_pos(s_knock_chart, 308, 136); lv_obj_set_size(s_knock_chart, 156, 34);
    lv_chart_set_type(s_knock_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(s_knock_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_chart_set_point_count(s_knock_chart, 32);
    lv_chart_set_div_line_count(s_knock_chart, 3, 4);
    lv_obj_set_style_bg_color(s_knock_chart, lv_color_hex(0x09131C), 0);
    lv_obj_set_style_border_width(s_knock_chart, 0, 0);
    s_knock_signal_series = lv_chart_add_series(s_knock_chart, lv_color_hex(kRed), LV_CHART_AXIS_PRIMARY_Y);
    s_knock_base_series = lv_chart_add_series(s_knock_chart, lv_color_hex(kCyan), LV_CHART_AXIS_PRIMARY_Y);

    lv_obj_t* tune = metric_tile(page, 8, 176, 292, 46, "GAIN 1.00\nMULT 2.5", kPanel,
                                 &lv_font_montserrat_12);
    lv_obj_set_style_text_align(tune, LV_TEXT_ALIGN_LEFT, 0);
    action_button(page, "G-", 122, 184, 38, 30);
    action_button(page, "G+", 164, 184, 38, 30);
    action_button(page, "M-", 208, 184, 38, 30);
    action_button(page, "M+", 250, 184, 38, 30);
    lv_obj_t* disable = action_button(page, "DISABLE", 308, 176, 74, 46);
    lv_obj_t* relearn = action_button(page, "RELEARN", 390, 176, 74, 46);
    accent_widget(disable, kRed, false);
    accent_widget(relearn, kAmber, false);
  }
}

void set_temperature_f(lv_obj_t* value_label, float celsius) {
  if (!value_label) return;
  const float fahrenheit = celsius * 1.8f + 32.0f;
  const int32_t signed_tenths = static_cast<int32_t>(
      fahrenheit * 10.0f + (fahrenheit >= 0.0f ? 0.5f : -0.5f));
  const int32_t magnitude = signed_tenths < 0 ? -signed_tenths : signed_tenths;
  lv_label_set_text_fmt(value_label, "%s%ld.%ld F",
                        signed_tenths < 0 ? "-" : "",
                        static_cast<long>(magnitude / 10),
                        static_cast<long>(magnitude % 10));
}

void demo_can_tick(lv_timer_t*) {
  // A repeating pull-and-coast cycle stands in for decoded CAN frames. Keeping
  // this model in the preview avoids introducing host-only behavior into the
  // production vehicle state or CAN manager.
  const uint32_t now_ms = lv_tick_get();
  const uint32_t seconds = now_ms / 1000U;
  const uint32_t phase_tenths = (now_ms / 100U) % 400U;
  const uint32_t phase = seconds % 40U;
  const uint32_t ramp_tenths = phase_tenths < 200U
      ? phase_tenths
      : 400U - phase_tenths;
  const uint32_t ramp = ramp_tenths / 10U;
  const uint32_t rpm = 900U + (ramp_tenths * 285U) / 10U;
  const uint32_t speed = ramp_tenths / 2U;
  const int32_t boost_tenths = -60 +
      static_cast<int32_t>((ramp_tenths * 12U) / 10U);
  const uint32_t meth_duty = rpm > 3600U ? (rpm - 3600U) / 55U : 0U;
  lv_label_set_text_fmt(s_rpm, "RPM\n%lu", static_cast<unsigned long>(rpm));
  lv_label_set_text_fmt(s_speed, "SPEED\n%lu\nMPH", static_cast<unsigned long>(speed));
  const int32_t boost_abs = boost_tenths < 0 ? -boost_tenths : boost_tenths;
  lv_label_set_text_fmt(s_boost, "BOOST\n%s%ld.%ld PSI", boost_tenths < 0 ? "-" : "",
                        static_cast<long>(boost_abs / 10),
                        static_cast<long>(boost_abs % 10));
  lv_label_set_text_fmt(s_meth_summary, "METH ARM\n%lu%%", static_cast<unsigned long>(meth_duty));
  lv_label_set_text_fmt(s_iat, "FUEL\n%lu%%", static_cast<unsigned long>(46U + ramp / 3U));
  lv_label_set_text_fmt(s_battery, "OIL\n%lu PSI", static_cast<unsigned long>(58U + ramp / 2U));
  lv_label_set_text(s_accel, "CAN BUS\nCAN OK");
  lv_label_set_text_fmt(s_clock, "12:%02lu", static_cast<unsigned long>(42U + (seconds / 60U) % 18U));
  if (s_freshness) {
    lv_label_set_text_fmt(s_freshness, "CAN 0.%lus  GPS 0.%lus",
        static_cast<unsigned long>(seconds % 4U + 1U),
        static_cast<unsigned long>(seconds % 7U + 2U));
  }
  if (s_alert_strip) {
    const bool show_warning = (phase >= 22U && phase <= 35U);
    const bool show_critical = phase >= 36U;
    const bool startup_visible = s_startup_overlay &&
        !lv_obj_has_flag(s_startup_overlay, LV_OBJ_FLAG_HIDDEN);
    if ((show_warning || show_critical) && !startup_visible) {
      static const char* const rotating_warnings[] = {
          "WARNING  CAN DATA FRESHNESS DEGRADED",
          "METH CONTROLLER OFFLINE",
          "STORAGE OFFLINE - LOGGING DISABLED"};
      const uint32_t warning_index = (seconds / 3U) %
          (sizeof(rotating_warnings) / sizeof(rotating_warnings[0]));
      lv_label_set_text(s_alert_label, show_critical
          ? "CRITICAL  KNOCK DETECTED - REDUCE LOAD"
          : rotating_warnings[warning_index]);
      lv_obj_set_style_bg_color(s_alert_strip,
          lv_color_hex(show_critical ? kRed : kAmber), 0);
      lv_obj_set_style_bg_opa(s_alert_strip, static_cast<lv_opa_t>(128), 0);
      lv_obj_clear_flag(s_alert_strip, LV_OBJ_FLAG_HIDDEN);
      lv_obj_move_foreground(s_alert_strip);
    } else {
      static const char* const page_status[] = {
          "VEHICLE STATUS", "INJECTION READY", "TAILLIGHT CONTROL",
          "CABIN LIGHTING", "GPS TELEMETRY", "THERMAL SYSTEMS",
          "SYSTEM HEALTH", "KNOCK MONITOR ACTIVE"};
      const bool green_band = s_active_page == 1U || s_active_page == 7U;
      lv_label_set_text(s_alert_label, page_status[s_active_page]);
      lv_obj_set_style_text_color(s_alert_label,
          lv_color_hex(green_band ? kGreen : kAmber), 0);
      lv_obj_set_style_bg_color(s_alert_strip,
          lv_color_hex(green_band ? 0x0A2412 : 0x241C04), 0);
      lv_obj_set_style_bg_opa(s_alert_strip, static_cast<lv_opa_t>(92), 0);
      lv_obj_clear_flag(s_alert_strip, LV_OBJ_FLAG_HIDDEN);
      lv_obj_move_foreground(s_alert_strip);
    }
  }

  lv_label_set_text_fmt(s_page_data[1],
      "INJECTION STATUS\n%lu%%", static_cast<unsigned long>(meth_duty));
  if (s_gps_speed) {
    lv_label_set_text_fmt(s_gps_speed, "%lu", static_cast<unsigned long>(speed));
    lv_label_set_text_fmt(s_gps_satellites, "%lu USED / %lu VIEW",
        static_cast<unsigned long>(10U + seconds % 3U),
        static_cast<unsigned long>(12U + seconds % 4U));
    lv_label_set_text_fmt(s_gps_hdop, "0.%lu",
        static_cast<unsigned long>(7U + seconds % 3U));
  }
  if (s_temp_values[0]) {
    const float ramp_value = static_cast<float>(ramp_tenths) / 10.0f;
    lv_label_set_text_fmt(s_temp_values[0], "INTAKE AIR\n%.0f F", 81.0 + ramp_value / 3.0);
    lv_label_set_text_fmt(s_temp_values[1], "ENGINE BAY TEMP\n%.0f F\nMIN 156  MAX 205", 97.0 + ramp_value / 2.0);
    lv_label_set_text_fmt(s_temp_values[2], "CABIN TEMP\n%.0f F", 72.0 + ramp_value / 8.0);
    lv_label_set_text_fmt(s_temp_values[3], "AMBIENT\n%.0f F", 64.0 + ramp_value / 8.0);
    lv_label_set_text_fmt(s_temp_values[4], "OIL PRESSURE\n%lu PSI",
                          static_cast<unsigned long>(55U + ramp / 2U));
    lv_label_set_text_fmt(s_temp_values[5], "FUEL PRESSURE\n%lu PSI",
                          static_cast<unsigned long>(44U + ramp / 3U));
  }
  if (s_trend_charts[0]) {
    const lv_coord_t values_a[4] = {
      static_cast<lv_coord_t>(rpm / 80U),
      static_cast<lv_coord_t>(55U + ramp / 2U),
      static_cast<lv_coord_t>(42U + ramp / 2U),
      static_cast<lv_coord_t>(12U + ramp * 2U)
    };
    const lv_coord_t values_b[4] = {
      static_cast<lv_coord_t>(12U + ramp * 3U),
      static_cast<lv_coord_t>(44U + ramp / 3U),
      static_cast<lv_coord_t>(55U + ramp),
      static_cast<lv_coord_t>(10U + ramp / 2U)
    };
    for (unsigned i = 0; i < 4; ++i) {
      lv_chart_set_next_value(s_trend_charts[i], s_trend_a[i], values_a[i]);
      lv_chart_set_next_value(s_trend_charts[i], s_trend_b[i], values_b[i]);
    }
  }
  const uint32_t knock_level = 12U + ramp * 3U;
  const uint32_t knock_value = knock_level > 100U ? 100U : knock_level;
  if (s_knock_energy) {
    lv_label_set_text_fmt(s_knock_energy, "CURRENT LEVEL\n0.%02lu V",
                          static_cast<unsigned long>(knock_value));
    lv_label_set_text_fmt(s_knock_baseline, "PEAK LEVEL\n0.%02lu V",
                          static_cast<unsigned long>(10U + ramp / 2U));
  }
  if (s_knock_chart) {
    lv_chart_set_next_value(s_knock_chart, s_knock_signal_series,
                            static_cast<lv_coord_t>(knock_value));
    lv_chart_set_next_value(s_knock_chart, s_knock_base_series,
                            static_cast<lv_coord_t>(10U + ramp / 2U));
  }
  if (s_page_data[7]) {
    lv_label_set_text(s_page_data[7], knock_value >= 90U ? "KNOCK" :
                      (knock_value >= 60U ? "KNOCK WARN" : "NO KNOCK"));
    accent_widget(s_page_data[7], knock_value >= 90U ? kRed :
                  (knock_value >= 60U ? kAmber : kGreen));
  }
}

void startup_tick(lv_timer_t* timer) {
  const uint32_t elapsed = lv_tick_get() - s_startup_begin;
  if (!s_startup_overlay) {
    lv_timer_del(timer);
    return;
  }
  const int progress = elapsed >= 1800U ? 100 : static_cast<int>(elapsed / 18U);
  lv_bar_set_value(s_startup_progress, progress, LV_ANIM_OFF);
  if (elapsed < 550U) lv_label_set_text(s_startup_status, "INITIALIZING DISPLAY");
  else if (elapsed < 1100U) lv_label_set_text(s_startup_status, "CHECKING CAN / GPS / STORAGE");
  else if (elapsed < 1650U) lv_label_set_text(s_startup_status, "LOADING DRIVER PROFILE");
  else lv_label_set_text(s_startup_status, "SYSTEM READY");
  if (elapsed >= 1900U) {
    lv_obj_add_flag(s_startup_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_timer_del(timer);
  }
}

void show_page(unsigned index) {
  if (index >= 8) return;
  for (unsigned i = 0; i < 8; ++i) {
    if (i == index) lv_obj_clear_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(s_nav[i],
        lv_color_hex(i == index ? kNavActiveBgColors[i] : 0x09131C), 0);
    lv_obj_set_style_border_width(s_nav[i], i == index ? 2 : 0, 0);
    lv_obj_set_style_border_side(s_nav[i], LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_color(s_nav[i], lv_color_hex(kNavAccentColors[i]), 0);
    lv_obj_t* nav_icon = lv_obj_get_child(s_nav[i], 0);
    if (nav_icon) {
      lv_obj_set_style_text_color(nav_icon, lv_color_hex(kNavAccentColors[i]), 0);
    }
    lv_obj_t* nav_text = lv_obj_get_child(s_nav[i], 1);
    if (nav_text) {
      lv_obj_set_style_text_color(nav_text, lv_color_hex(kWhite), 0);
    }
  }
  lv_label_set_text(s_title, kPageNames[index]);
  lv_obj_set_style_text_color(s_title, lv_color_hex(kNavAccentColors[index]), 0);
  s_active_page = index;
}

void nav_clicked(lv_event_t* event) {
  const auto index = static_cast<unsigned>(
      reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
  show_page(index);
}

void build_ui() {
  lv_obj_t* screen = CCM_PREVIEW_ACTIVE_SCREEN();
  lv_obj_set_style_bg_color(screen, lv_color_hex(kBackground), 0);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* header = lv_obj_create(screen);
  lv_obj_set_pos(header, 0, 0);
  lv_obj_set_size(header, 480, 26);
  lv_obj_set_style_bg_color(header, lv_color_hex(0x071018), 0);
  lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(header, 0, 0);
  lv_obj_set_style_radius(header, 0, 0);
  lv_obj_set_style_pad_all(header, 0, 0);
  s_clock = label(header, "12:42", 8, 5, &lv_font_montserrat_16, kWhite);
  s_title = label(header, "DASH", 216, 4, &lv_font_montserrat_18, kRed);
  s_freshness = label(header, "CAN 0.1s  GPS 0.2s", 322, 7, &lv_font_montserrat_12, kGreen);
  lv_obj_t* header_rule = lv_obj_create(screen);
  lv_obj_set_pos(header_rule, 0, 25);
  lv_obj_set_size(header_rule, 480, 1);
  lv_obj_set_style_bg_color(header_rule, lv_color_hex(kDivider), 0);
  lv_obj_set_style_bg_opa(header_rule, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(header_rule, 0, 0);
  lv_obj_set_style_radius(header_rule, 0, 0);

  lv_obj_t* content = lv_obj_create(screen);
  lv_obj_set_pos(content, 0, 44);
  lv_obj_set_size(content, 480, 224);
  lv_obj_set_style_bg_opa(content, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(content, 0, 0);
  lv_obj_set_style_pad_all(content, 0, 0);
  lv_obj_clear_flag(content, LV_OBJ_FLAG_SCROLLABLE);

  for (unsigned i = 0; i < 8; ++i) {
    s_pages[i] = lv_obj_create(content);
    lv_obj_set_size(s_pages[i], 480, 224);
    lv_obj_set_style_bg_opa(s_pages[i], LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_pages[i], 0, 0);
    lv_obj_set_style_pad_all(s_pages[i], 4, 0);
    lv_obj_clear_flag(s_pages[i], LV_OBJ_FLAG_SCROLLABLE);
    if (i == 0) build_dashboard(s_pages[i]);
    else build_generic_page(s_pages[i], i);
  }

  s_alert_strip = lv_obj_create(screen);
  lv_obj_set_pos(s_alert_strip, 0, 26);
  lv_obj_set_size(s_alert_strip, 480, 18);
  lv_obj_set_style_border_width(s_alert_strip, 0, 0);
  lv_obj_set_style_radius(s_alert_strip, 0, 0);
  lv_obj_set_style_pad_all(s_alert_strip, 0, 0);
  lv_obj_set_style_bg_opa(s_alert_strip, static_cast<lv_opa_t>(128), 0);
  s_alert_label = label(s_alert_strip, "VEHICLE SYSTEMS ONLINE", 0, 2, &lv_font_montserrat_12, kGreen);
  lv_obj_set_width(s_alert_label, 480);
  lv_obj_set_style_text_align(s_alert_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_bg_color(s_alert_strip, lv_color_hex(0x0A2412), 0);

  s_theme_tint = lv_obj_create(screen);
  lv_obj_set_pos(s_theme_tint, 0, 0);
  lv_obj_set_size(s_theme_tint, 480, 320);
  lv_obj_set_style_bg_color(s_theme_tint, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(s_theme_tint, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(s_theme_tint, 0, 0);
  lv_obj_set_style_radius(s_theme_tint, 0, 0);
  lv_obj_clear_flag(s_theme_tint, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(s_theme_tint, LV_OBJ_FLAG_SCROLLABLE);

  s_startup_overlay = lv_obj_create(screen);
  lv_obj_set_pos(s_startup_overlay, 0, 0);
  lv_obj_set_size(s_startup_overlay, 480, 320);
  lv_obj_set_style_bg_color(s_startup_overlay, lv_color_hex(0x03070B), 0);
  lv_obj_set_style_bg_opa(s_startup_overlay, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(s_startup_overlay, 0, 0);
  lv_obj_set_style_radius(s_startup_overlay, 0, 0);
  label(s_startup_overlay, "COMFORT  CONTROL", 122, 78, &lv_font_montserrat_20, kCyan);
  label(s_startup_overlay, "VEHICLE SYSTEMS", 174, 110, &lv_font_montserrat_12, kMuted);
  s_startup_status = label(s_startup_overlay, "INITIALIZING DISPLAY", 0, 174,
                           &lv_font_montserrat_14, kWhite);
  lv_obj_set_width(s_startup_status, 480);
  lv_obj_set_style_text_align(s_startup_status, LV_TEXT_ALIGN_CENTER, 0);
  s_startup_progress = lv_bar_create(s_startup_overlay);
  lv_obj_set_pos(s_startup_progress, 80, 210);
  lv_obj_set_size(s_startup_progress, 320, 6);
  lv_bar_set_range(s_startup_progress, 0, 100);
  lv_bar_set_value(s_startup_progress, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(s_startup_progress, lv_color_hex(0x132331), LV_PART_MAIN);
  lv_obj_set_style_bg_color(s_startup_progress, lv_color_hex(kCyan), LV_PART_INDICATOR);

  for (unsigned i = 0; i < 8; ++i) {
    s_nav[i] = CCM_PREVIEW_BUTTON_CREATE(screen);
    lv_obj_set_pos(s_nav[i], static_cast<int>(i * 60), 268);
    lv_obj_set_size(s_nav[i], 60, 52);
    lv_obj_set_style_radius(s_nav[i], 0, 0);
    lv_obj_set_style_border_width(s_nav[i], 0, 0);
    lv_obj_set_style_shadow_width(s_nav[i], 0, 0);
    lv_obj_set_style_bg_color(s_nav[i], lv_color_hex(0x09131C), 0);
    lv_obj_add_event_cb(s_nav[i], nav_clicked, LV_EVENT_CLICKED,
                        reinterpret_cast<void*>(static_cast<uintptr_t>(i)));
    lv_obj_t* icon = lv_label_create(s_nav[i]);
    lv_label_set_text(icon, kNavSymbols[i]);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(icon, lv_color_hex(kNavAccentColors[i]), 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -9);
    lv_obj_t* text = lv_label_create(s_nav[i]);
    lv_label_set_text(text, kPageNames[i]);
    lv_obj_set_style_text_font(text, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(text, lv_color_hex(kWhite), 0);
    lv_obj_align(text, LV_ALIGN_CENTER, 0, 13);
  }

  show_page(0);
  demo_can_tick(nullptr);
  // Ten updates per second keeps simulated CAN values and charts visibly
  // responsive while remaining well below the browser's 60 Hz render loop.
  lv_timer_create(demo_can_tick, 100, nullptr);
  s_startup_begin = lv_tick_get();
  lv_timer_create(startup_tick, 50, nullptr);
}

}  // namespace

#ifdef LVGL_LIVE_PREVIEW
extern "C" void lvgl_live_preview_init(void) {
  build_ui();
}
#endif
