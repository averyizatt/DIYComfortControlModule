#pragma once

#include <deque>
#include <vector>

#include "hal/TestInterfaces.hpp"

struct MockCanBus : public ccm::hal::ICanBus {
  std::vector<can_protocol::CanFrame> sent;
  std::deque<can_protocol::CanFrame> inbox;

  bool send(const can_protocol::CanFrame& frame) override {
    sent.push_back(frame);
    return true;
  }

  bool receive(can_protocol::CanFrame& frame) override {
    if (inbox.empty()) return false;
    frame = inbox.front();
    inbox.pop_front();
    return true;
  }
};
