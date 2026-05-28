#pragma once

#include <Arduino.h>

namespace pins {

// ESP32-S3 default pin map for the Cabin Master node.
// Every pin can be overridden at build time with:
//   -D CCM_PIN_<NAME>=<gpio_number>
//
// ── Display / SPI bus (ILI9488 320×480 – mislabeled ST7796S on the module) ──
//  LCD CS        10  (ADC1_CH9)
//  LCD RST        9  (ADC1_CH8)
//  LCD RS/DC     46  ⚠ STRAPPING PIN – ROM log verbosity. MCU pull-up keeps it
//                      HIGH during reset so boot is unaffected. Output-capable.
//  SCK            8  (ADC1_CH7)
//  SDI/MOSI       3  ⚠ STRAPPING PIN – JTAG source. HIGH/float = USB JTAG (ok).
//                      Display won't pull it LOW during boot, so safe in practice.
//  SDO/MISO      17  (ADC2_CH6)
//  CD/SD-CS       5  (ADC1_CH4)
//  LED           5V  hardwired – backlight pin disabled (255)
// ── Touch controller (CTP / FT6x36 I2C) ─────────────────────────────────────
//  CTP INT        6  (ADC1_CH5)
//  CTP SDA        7  (ADC1_CH6)
//  CTP RST       15  (ADC2_CH4)
//  CTP SCL       16  (ADC2_CH5)
// ── Other peripherals ────────────────────────────────────────────────────────
//  CAN TX        47  (GP)   legacy TWAI only; not used in SPI-CAN mode
//  CAN RX         4  (ADC1_CH3)  legacy TWAI only; reused for Gyro INT in SPI-CAN mode
//  CAN SPI CS    11  (ADC2_CH0)  was 17; 17 is SPI_MISO
//  CAN SPI INT   18  (ADC2_CH7)
//  CAN SPI RST   21  (GP)
//  Tach OUT      13  (ADC2_CH2)  was 6→39; 39 is LED_DATA2  [LEDC output]
//  Tach IN        2  (ADC1_CH1)
//  Gyro INT       4  (ADC1_CH3)  moved from 14; shares old legacy CAN RX pin
//  Battery ADC    1  (ADC1_CH0)  was 46; 46 is LCD_DC
//  Knock ADC     48  (ADC1_CH7)
//  GPS RX        41  (GP)
//  GPS TX        42  (GP)
//  LED strip 1   38  (GP)
//  LED strip 2   39  (GP)
//  LED strip 3   40  (GP)
//  AUX OUT1      33  ⚠ PSRAM on N8R8/N16R8 boards – unusable on those variants
//  AUX OUT2      34  ⚠ PSRAM on N8R8/N16R8 boards
//  Button UP     35  ⚠ PSRAM on N8R8/N16R8 boards
//  Button DOWN   36  ⚠ PSRAM on N8R8/N16R8 boards
//  Button SEL    37  ⚠ PSRAM on N8R8/N16R8 boards
// ── Reserved / do-not-use ────────────────────────────────────────────────────
//  GPIO 19, 20   USB-OTG D-/D+  (CDC serial / programming)
//  GPIO 26-32    Internal SPI flash
//  GPIO 33-37    PSRAM on N8R8/N16R8 variants (see above)
// ─────────────────────────────────────────────────────────────────────────────

#ifndef CCM_PIN_LCD_CS
#define CCM_PIN_LCD_CS 10
#endif
#ifndef CCM_PIN_LCD_RST
#define CCM_PIN_LCD_RST 9
#endif
#ifndef CCM_PIN_LCD_DC
#define CCM_PIN_LCD_DC 46
#endif
#ifndef CCM_PIN_LCD_BACKLIGHT
#define CCM_PIN_LCD_BACKLIGHT 255  // LED tied to 5V – no GPIO control
#endif

#ifndef CCM_PIN_SPI_MOSI
#define CCM_PIN_SPI_MOSI 3
#endif
#ifndef CCM_PIN_SPI_MISO
#define CCM_PIN_SPI_MISO 17
#endif
#ifndef CCM_PIN_SPI_SCK
#define CCM_PIN_SPI_SCK 8
#endif
#ifndef CCM_PIN_SD_CS
#define CCM_PIN_SD_CS 5
#endif

#ifndef CCM_PIN_TOUCH_SCL
#define CCM_PIN_TOUCH_SCL 16
#endif
#ifndef CCM_PIN_TOUCH_SDA
#define CCM_PIN_TOUCH_SDA 7
#endif
#ifndef CCM_PIN_TOUCH_RST
#define CCM_PIN_TOUCH_RST 15
#endif
#ifndef CCM_PIN_TOUCH_INT
#define CCM_PIN_TOUCH_INT 6
#endif

#ifndef CCM_PIN_CAN_TX
#define CCM_PIN_CAN_TX 47   // Legacy TWAI only; SPI-CAN uses CS/INT/RST on separate pins.
#endif
#ifndef CCM_PIN_CAN_RX
#define CCM_PIN_CAN_RX 4    // Legacy TWAI only; active SPI-CAN builds do not use this pin.
#endif
#ifndef CCM_PIN_CAN_SPI_CS
#define CCM_PIN_CAN_SPI_CS 11  // ← was 17; moved – GPIO 17 now used for SPI_MISO
#endif
#ifndef CCM_PIN_CAN_SPI_INT
#define CCM_PIN_CAN_SPI_INT 18
#endif
#ifndef CCM_PIN_CAN_SPI_RST
#define CCM_PIN_CAN_SPI_RST 21
#endif

#ifndef CCM_PIN_GPS_UART_PORT
#define CCM_PIN_GPS_UART_PORT 1
#endif
#ifndef CCM_PIN_GPS_RX
#define CCM_PIN_GPS_RX 41
#endif
#ifndef CCM_PIN_GPS_TX
#define CCM_PIN_GPS_TX 42
#endif
#ifndef CCM_GPS_BAUD
#define CCM_GPS_BAUD 9600
#endif

#ifndef CCM_PIN_TACH_OUT
#define CCM_PIN_TACH_OUT 13  // LEDC PWM → 2N2222A base (1kΩ) → collector pulls tach line to 12V
#endif
#ifndef CCM_PIN_TACH_IN
#define CCM_PIN_TACH_IN 2
#endif
#ifndef CCM_TACH_LEDC_CHANNEL
#define CCM_TACH_LEDC_CHANNEL 0
#endif
#ifndef CCM_TACH_DUTY
#define CCM_TACH_DUTY 128
#endif

#ifndef CCM_PIN_BUTTON_UP
#define CCM_PIN_BUTTON_UP 35
#endif
#ifndef CCM_PIN_BUTTON_DOWN
#define CCM_PIN_BUTTON_DOWN 36
#endif
#ifndef CCM_PIN_BUTTON_SELECT
#define CCM_PIN_BUTTON_SELECT 37
#endif

#ifndef CCM_PIN_GYRO_SCL
#define CCM_PIN_GYRO_SCL CCM_PIN_TOUCH_SCL
#endif
#ifndef CCM_PIN_GYRO_SDA
#define CCM_PIN_GYRO_SDA CCM_PIN_TOUCH_SDA
#endif
#ifndef CCM_PIN_GYRO_INT
#define CCM_PIN_GYRO_INT 4   // Reuse legacy CAN RX pin now that CAN runs over SPI only.
#endif
#ifndef CCM_PIN_GYRO_ADDR_SEL
#define CCM_PIN_GYRO_ADDR_SEL 255
#endif

#ifndef CCM_PIN_AUX_OUT1
#define CCM_PIN_AUX_OUT1 33
#endif
#ifndef CCM_PIN_AUX_OUT2
#define CCM_PIN_AUX_OUT2 34
#endif

#ifndef CCM_PIN_BATTERY_SENSE
#define CCM_PIN_BATTERY_SENSE 1   // ← was 46; moved – GPIO 46 now used for LCD_DC
#endif

#ifndef CCM_PIN_KNOCK_ADC
#define CCM_PIN_KNOCK_ADC 48
#endif

#ifndef CCM_GYRO_I2C_ADDR_PRIMARY
#define CCM_GYRO_I2C_ADDR_PRIMARY 0x68
#endif
#ifndef CCM_GYRO_I2C_ADDR_SECONDARY
#define CCM_GYRO_I2C_ADDR_SECONDARY 0x69
#endif

#ifndef CCM_BATTERY_DIVIDER_TOP_OHMS
#define CCM_BATTERY_DIVIDER_TOP_OHMS 100000.0f
#endif
#ifndef CCM_BATTERY_DIVIDER_BOTTOM_OHMS
#define CCM_BATTERY_DIVIDER_BOTTOM_OHMS 20000.0f
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
static constexpr uint8_t kCanSpiCs = CCM_PIN_CAN_SPI_CS;
static constexpr uint8_t kCanSpiInt = CCM_PIN_CAN_SPI_INT;
static constexpr uint8_t kCanSpiRst = CCM_PIN_CAN_SPI_RST;

static constexpr uint8_t kGpsUartPort = CCM_PIN_GPS_UART_PORT;
static constexpr uint8_t kGpsRx = CCM_PIN_GPS_RX;
static constexpr uint8_t kGpsTx = CCM_PIN_GPS_TX;
static constexpr uint32_t kGpsBaud = CCM_GPS_BAUD;

static constexpr uint8_t kTachOut = CCM_PIN_TACH_OUT;
static constexpr uint8_t kTachIn = CCM_PIN_TACH_IN;
static constexpr uint8_t kTachLedcChannel = CCM_TACH_LEDC_CHANNEL;
static constexpr uint8_t kTachDuty = CCM_TACH_DUTY;

static constexpr uint8_t kButtonUp = CCM_PIN_BUTTON_UP;
static constexpr uint8_t kButtonDown = CCM_PIN_BUTTON_DOWN;
static constexpr uint8_t kButtonSelect = CCM_PIN_BUTTON_SELECT;

static constexpr uint8_t kGyroScl = CCM_PIN_GYRO_SCL;
static constexpr uint8_t kGyroSda = CCM_PIN_GYRO_SDA;
static constexpr uint8_t kGyroInt = CCM_PIN_GYRO_INT;
static constexpr uint8_t kGyroAddrSel = CCM_PIN_GYRO_ADDR_SEL;

static constexpr uint8_t kAuxOut1 = CCM_PIN_AUX_OUT1;
static constexpr uint8_t kAuxOut2 = CCM_PIN_AUX_OUT2;
static constexpr uint8_t kBatterySense = CCM_PIN_BATTERY_SENSE;
static constexpr uint8_t kKnockAdc = CCM_PIN_KNOCK_ADC;

static constexpr uint8_t kGyroI2cAddrPrimary = CCM_GYRO_I2C_ADDR_PRIMARY;
static constexpr uint8_t kGyroI2cAddrSecondary = CCM_GYRO_I2C_ADDR_SECONDARY;

static constexpr float kBatteryDividerTopOhms = CCM_BATTERY_DIVIDER_TOP_OHMS;
static constexpr float kBatteryDividerBottomOhms = CCM_BATTERY_DIVIDER_BOTTOM_OHMS;

static constexpr uint8_t kLedData1 = CCM_PIN_LED_DATA1;
static constexpr uint8_t kLedData2 = CCM_PIN_LED_DATA2;
static constexpr uint8_t kLedData3 = CCM_PIN_LED_DATA3;

}  // namespace pins
