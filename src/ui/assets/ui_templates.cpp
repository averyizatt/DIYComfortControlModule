#include "ui/assets/templates/ui_template_dash.h"
#include "ui/assets/templates/ui_template_diag.h"
#include "ui/assets/templates/ui_template_knock.h"
#include "ui/assets/templates/ui_template_meth.h"
#include "ui/assets/templates/ui_template_tail.h"
#include "ui/assets/templates/ui_template_temps.h"

extern const uint8_t dash_start[]
    asm("_binary_src_ui_assets_templates_ui_template_dash_bin_start");
extern const uint8_t meth_start[]
    asm("_binary_src_ui_assets_templates_ui_template_meth_bin_start");
extern const uint8_t tail_start[]
    asm("_binary_src_ui_assets_templates_ui_template_tail_bin_start");
extern const uint8_t temps_start[]
    asm("_binary_src_ui_assets_templates_ui_template_temps_bin_start");
extern const uint8_t diag_start[]
    asm("_binary_src_ui_assets_templates_ui_template_diag_bin_start");
extern const uint8_t knock_start[]
    asm("_binary_src_ui_assets_templates_ui_template_knock_bin_start");

#define CCM_TEMPLATE_DESCRIPTOR(name, data_symbol) \
  const lv_img_dsc_t name = {                    \
    .header = {                                   \
      .cf = LV_IMG_CF_TRUE_COLOR,                 \
      .always_zero = 0,                           \
      .reserved = 0,                              \
      .w = 480,                                   \
      .h = 320,                                   \
    },                                            \
    .data_size = 307200,                          \
    .data = data_symbol,                          \
  }

CCM_TEMPLATE_DESCRIPTOR(ui_template_dash, dash_start);
CCM_TEMPLATE_DESCRIPTOR(ui_template_meth, meth_start);
CCM_TEMPLATE_DESCRIPTOR(ui_template_tail, tail_start);
CCM_TEMPLATE_DESCRIPTOR(ui_template_temps, temps_start);
CCM_TEMPLATE_DESCRIPTOR(ui_template_diag, diag_start);
CCM_TEMPLATE_DESCRIPTOR(ui_template_knock, knock_start);
