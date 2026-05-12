#pragma once

#include <Arduino.h>

namespace pins {

// ESP32-S3 default pin map for the Cabin Master node.
// Every pin can be overridden at build time with:
//   -D CCM_PIN_<NAME>=<gpio_number>
// Example:
//   -D CCM_PIN_LED_DATA1=17

#ifndef CCM_PIN_LCD_CS
#define CCM_PIN_LCD_CS 10
#endif
#ifndef CCM_PIN_LCD_RST
#define CCM_PIN_LCD_RST 9
#endif
#ifndef CCM_PIN_LCD_DC
#define CCM_PIN_LCD_DC 8
#endif
#ifndef CCM_PIN_LCD_BACKLIGHT
#define CCM_PIN_LCD_BACKLIGHT 7
#endif

#ifndef CCM_PIN_SPI_MOSI
#define CCM_PIN_SPI_MOSI 11
#endif
#ifndef CCM_PIN_SPI_MISO
#define CCM_PIN_SPI_MISO 13
#endif
#ifndef CCM_PIN_SPI_SCK
#define CCM_PIN_SPI_SCK 12
#endif
#ifndef CCM_PIN_SD_CS
#define CCM_PIN_SD_CS 16
#endif

#ifndef CCM_PIN_TOUCH_SCL
#define CCM_PIN_TOUCH_SCL 47
#endif
#ifndef CCM_PIN_TOUCH_SDA
#define CCM_PIN_TOUCH_SDA 48
#endif
#ifndef CCM_PIN_TOUCH_RST
#define CCM_PIN_TOUCH_RST 14
#endif
#ifndef CCM_PIN_TOUCH_INT
#define CCM_PIN_TOUCH_INT 15
#endif

#ifndef CCM_PIN_CAN_TX
#define CCM_PIN_CAN_TX 5
#endif
#ifndef CCM_PIN_CAN_RX
#define CCM_PIN_CAN_RX 4
#endif

#ifndef CCM_PIN_LED_DATA1
#define CCM_PIN_LED_DATA1 38
#endif
#ifndef CCM_PIN_LED_DATA2
#define CCM_PIN_LED_DATA2 39
#endif
#ifndef CCM_PIN_LED_DATA3
#define CCM_PIN_LED_DATA3 40
#endif

static constexpr uint8_t kLcdCs = CCM_PIN_LCD_CS;
static constexpr uint8_t kLcdRst = CCM_PIN_LCD_RST;
static constexpr uint8_t kLcdDc = CCM_PIN_LCD_DC;
static constexpr uint8_t kLcdBacklight = CCM_PIN_LCD_BACKLIGHT;

static constexpr uint8_t kSpiMosi = CCM_PIN_SPI_MOSI;
static constexpr uint8_t kSpiMiso = CCM_PIN_SPI_MISO;
static constexpr uint8_t kSpiSck = CCM_PIN_SPI_SCK;
static constexpr uint8_t kSdCs = CCM_PIN_SD_CS;

static constexpr uint8_t kTouchScl = CCM_PIN_TOUCH_SCL;
static constexpr uint8_t kTouchSda = CCM_PIN_TOUCH_SDA;
static constexpr uint8_t kTouchRst = CCM_PIN_TOUCH_RST;
static constexpr uint8_t kTouchInt = CCM_PIN_TOUCH_INT;

static constexpr uint8_t kCanTx = CCM_PIN_CAN_TX;
static constexpr uint8_t kCanRx = CCM_PIN_CAN_RX;

static constexpr uint8_t kLedData1 = CCM_PIN_LED_DATA1;
static constexpr uint8_t kLedData2 = CCM_PIN_LED_DATA2;
static constexpr uint8_t kLedData3 = CCM_PIN_LED_DATA3;

}  // namespace pins
