#pragma once

#include <Arduino.h>
#include "pin_map.h"

namespace ccm::config {

constexpr uint32_t kCanBitrate = CCM_CAN_BITRATE;
constexpr uint8_t kCanTxPin = pins::kCanTx;
constexpr uint8_t kCanRxPin = pins::kCanRx;
constexpr uint8_t kCanMcp2515CsPin = pins::kCanSpiCs;
constexpr uint8_t kCanMcp2515IntPin = pins::kCanSpiInt;
constexpr uint8_t kCanMcp2515RstPin = pins::kCanSpiRst;

constexpr uint8_t kGpsUartPort = pins::kGpsUartPort;
constexpr uint8_t kGpsRxPin = pins::kGpsRx;
constexpr uint8_t kGpsTxPin = pins::kGpsTx;
constexpr uint32_t kGpsBaud = pins::kGpsBaud;

constexpr uint8_t kButtonUpPin = pins::kButtonUp;
constexpr uint8_t kButtonDownPin = pins::kButtonDown;
constexpr uint8_t kButtonSelectPin = pins::kButtonSelect;

constexpr uint8_t kTachOutPin = pins::kTachOut;
constexpr uint8_t kTachInPin = pins::kTachIn;
constexpr uint8_t kTachOutChannel = pins::kTachLedcChannel;
constexpr uint8_t kTachDuty = pins::kTachDuty;

constexpr uint8_t kGyroSclPin = pins::kGyroScl;
constexpr uint8_t kGyroSdaPin = pins::kGyroSda;
constexpr uint8_t kGyroIntPin = pins::kGyroInt;
constexpr uint8_t kGyroAddrSelPin = pins::kGyroAddrSel;

constexpr uint8_t kAuxOut1Pin = pins::kAuxOut1;
constexpr uint8_t kAuxOut2Pin = pins::kAuxOut2;

constexpr float kUndervoltageThreshold = 11.6f;
constexpr uint32_t kDashboardRefreshMs = 33;
constexpr uint32_t kCanHeartbeatMs = 250;
constexpr uint32_t kNodeTimeoutMs = 1500;

}  // namespace ccm::config
