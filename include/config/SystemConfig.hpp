#pragma once

#include <Arduino.h>

namespace ccm::config {

constexpr uint32_t kCanBitrate = CCM_CAN_BITRATE;
constexpr uint8_t kCanTxPin = 5;
constexpr uint8_t kCanRxPin = 4;

constexpr uint8_t kGpsUartPort = 1;
constexpr uint8_t kGpsRxPin = 18;
constexpr uint8_t kGpsTxPin = 17;
constexpr uint32_t kGpsBaud = 9600;

constexpr uint8_t kButtonUpPin = 8;
constexpr uint8_t kButtonDownPin = 9;
constexpr uint8_t kButtonSelectPin = 10;

constexpr uint8_t kTachOutPin = 6;
constexpr uint8_t kTachOutChannel = 0;
constexpr uint8_t kTachDuty = 128;

constexpr float kUndervoltageThreshold = 11.6f;
constexpr uint32_t kDashboardRefreshMs = 33;
constexpr uint32_t kCanHeartbeatMs = 250;
constexpr uint32_t kNodeTimeoutMs = 1500;

}  // namespace ccm::config
