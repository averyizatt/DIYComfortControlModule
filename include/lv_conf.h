/**
 * @file lv_conf.h
 * LVGL 8.3 configuration for the CCM ILI9488 320×480 display.
 * Place in include/ so the LV_CONF_INCLUDE_SIMPLE build flag can find it.
 */

#if 1  /* Set to "0" to disable this file */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/
#define LV_COLOR_DEPTH     16   /* RGB565 */
#define LV_COLOR_16_SWAP   0    /* ILI9488 uses 18-bit SPI (3 bytes/pixel);
                                   Arduino_GFX does its own R/G/B extraction
                                   from the 16-bit value, so byte-swapping
                                   here would corrupt colours. */
#define LV_COLOR_SCREEN_TRANSP 0

/*====================
   MEMORY SETTINGS
 *====================*/
#define LV_MEM_CUSTOM          0
#define LV_MEM_SIZE            (80U * 1024U)   /* 80 KB LVGL internal heap */
#define LV_MEM_ADR             0               /* 0 = place in normal BSS  */
#define LV_MEM_POOL_INCLUDE    "stdlib.h"
#define LV_MEM_POOL_ALLOC      malloc
#define LV_MEM_POOL_FREE       free
#define LV_MEMCPY_MEMSET_STD   1

/*====================
   HAL SETTINGS
 *====================*/
#define LV_DISP_DEF_REFR_PERIOD   16   /* ms  ~60 FPS                    */
#define LV_INDEV_DEF_READ_PERIOD  12   /* ms  touch polling interval     */

/* Use Arduino millis() as LVGL tick source */
#define LV_TICK_CUSTOM             1
#if LV_TICK_CUSTOM
#  define LV_TICK_CUSTOM_INCLUDE      "Arduino.h"
#  define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())
#endif

/*====================
   LOGGING
 *====================*/
#define LV_USE_LOG   0

/*====================
   ASSERT
 *====================*/
#define LV_USE_ASSERT_NULL          0
#define LV_USE_ASSERT_MALLOC        0
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0

/*====================
   RENDERING
 *====================*/
#define LV_DRAW_COMPLEX       1
#define LV_SHADOW_CACHE_SIZE  0
#define LV_CIRCLE_CACHE_SIZE  8   /* more arc/circle cache entries (arcs on dash/meth/gps) */
#define LV_IMG_CACHE_DEF_SIZE 0
#define LV_GRADIENT_MAX_STOPS 2
#define LV_GRAD_CACHE_DEF_SIZE 4096  /* bytes; must hold ≥1 entry: 480×(lv_color_t + lv_color32_t) ≈ 2.9 KB */
#define LV_DITHER_GRADIENT    1      /* ordered dithering for gradients – eliminates colour banding
                                        on 16-bit (RGB565) bars, arcs, and backgrounds */
#define LV_DISP_ROT_MAX_BUF   (10 * 1024)
#undef LV_GRAD_CACHE_DEF_SIZE
#define LV_GRAD_CACHE_DEF_SIZE 0
#undef LV_DITHER_GRADIENT
#define LV_DITHER_GRADIENT    0
#undef LV_DISP_ROT_MAX_BUF
#define LV_DISP_ROT_MAX_BUF   0

/*====================
   GPU (all off)
 *====================*/
#define LV_USE_GPU_STM32_DMA2D 0
#define LV_USE_GPU_SWM341_DMA  0
#define LV_USE_GPU_NXP_PXP     0
#define LV_USE_GPU_NXP_VG_LITE 0
#define LV_USE_GPU_SDL         0

/*====================
   FONTS
 *====================*/
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 0
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 1
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0

#define LV_FONT_UNSCII_8    0
#define LV_FONT_UNSCII_16   0

#define LV_FONT_CUSTOM_DECLARE
#define LV_FONT_DEFAULT &lv_font_montserrat_16

#define LV_FONT_FMT_TXT_LARGE 0
#define LV_USE_FONT_SUBPX     0
#define LV_FONT_SUBPX_BGR     0

/*====================
   WIDGETS
 *====================*/
#define LV_USE_ARC           1   /* RPM gauge */
#define LV_USE_BAR           1   /* boost bar, temp bars */
#define LV_USE_BTN           1
#define LV_USE_BTNMATRIX     0
#define LV_USE_CANVAS        0
#define LV_USE_CHECKBOX      0
#define LV_USE_DROPDOWN      0
#define LV_USE_IMG           0
#define LV_USE_LABEL         1
#  define LV_LABEL_TEXT_SELECTION 0
#  define LV_LABEL_LONG_TXT_HINT  0
#define LV_USE_LINE          0
#define LV_USE_ROLLER        0
#  define LV_ROLLER_INF_PAGES  7
#define LV_USE_SLIDER        0
#define LV_USE_SWITCH        1
#define LV_USE_TEXTAREA      0
#  define LV_TEXTAREA_DEF_PWD_SHOW_TIME 1500
#define LV_USE_TABLE         1

/*====================
   EXTRA COMPONENTS
 *====================*/
#define LV_USE_TABVIEW    0

#define LV_USE_WIN        0
#define LV_USE_SPAN       0
#define LV_USE_MSGBOX     0
#define LV_USE_TILEVIEW   0
#define LV_USE_LIST       0
#define LV_USE_MENU       0
#define LV_USE_METER      1
#define LV_USE_CALENDAR   0
#define LV_USE_KEYBOARD   0
#define LV_USE_CHART      1
#define LV_USE_COLORWHEEL 0
#define LV_USE_IMGBTN     0
#define LV_USE_ANIMIMG    0
#define LV_USE_LED        1
#define LV_USE_SPINNER    0
#define LV_USE_SPINBOX    0

/*====================
   THEMES
 *====================*/
#define LV_USE_THEME_DEFAULT 1
#  define LV_THEME_DEFAULT_DARK            1   /* Dark mode */
#  define LV_THEME_DEFAULT_GROW            0
#  define LV_THEME_DEFAULT_TRANSITION_TIME 0
#define LV_USE_THEME_BASIC   0
#define LV_USE_THEME_MONO    0

/*====================
   LAYOUTS
 *====================*/
#define LV_USE_FLEX 0
#define LV_USE_GRID 0

/*====================
   FILE SYSTEM (all off)
 *====================*/
#define LV_USE_FS_STDIO  0
#define LV_USE_FS_POSIX  0
#define LV_USE_FS_WIN32  0
#define LV_USE_FS_FATFS  0

/*====================
   IMAGE DECODERS (all off)
 *====================*/
#define LV_USE_PNG    0
#define LV_USE_BMP    0
#define LV_USE_SJPG   0
#define LV_USE_GIF    0
#define LV_USE_QRCODE 0

/*====================
   PERFORMANCE MONITOR
 *====================*/
#define LV_USE_PERF_MONITOR  0
#define LV_USE_MEM_MONITOR   0
#define LV_USE_REFR_DEBUG    0

/*====================
   I18N / MISC (all off)
 *====================*/
#define LV_USE_BIDI                  0
#define LV_USE_ARABIC_PERSIAN_CHARS  0
#define LV_USE_FRAGMENT              0
#define LV_USE_IMGFONT               0
#define LV_USE_SNAPSHOT              0

/*====================
   3rd PARTY (all off)
 *====================*/
#define LV_USE_FREETYPE 0
#define LV_USE_RLOTTIE  0
#define LV_USE_FFMPEG   0

/*====================
   DPI
 *====================*/
#define LV_DPI_DEF 130  /* ~130 DPI for a 3.5-inch 320×480 panel */

#endif /* LV_CONF_H */
#endif /* End of enable guard */
