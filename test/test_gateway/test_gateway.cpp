#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include "can/CanFrameBuilders.hpp"
#include "gateway/logic.h"
#include "gateway/config.h"
#include <can_contract/gateway_protocol.h>

int main() {
  namespace wire = can_protocol::gateway;
  assert(gateway_config::tachPulsesPerRev10 == state::VehicleState{}.pulses_per_rev10);
  assert(gateway::rpm(10000, gateway_config::tachPulsesPerRev10) == 3000);
  // Compare against the dashboard's actual builder, not a second gateway decoder.
  for (bool live : {false, true}) {
    state::VehicleState state{};
    state.rpm = live ? 3000 : 0;
    state.generated_tach_hz10 = 0;
    state.tach_source = static_cast<uint8_t>(can_protocol::TachSource::GPIO_INPUT);
    state.tach_status_flags = live ? 1 : 2;
    state.pulses_per_rev10 = 40;
    can_protocol::CanFrame expected{};
    canbus::packTachState(state, expected);
    const auto actual = wire::packTach(state.rpm, 40, live);
    assert(actual.id == expected.id && actual.dlc == expected.dlc);
    for (unsigned i = 0; i < 8; ++i) assert(actual.data[i] == expected.data[i]);
  }
  gateway::CommandLease lease;
  assert(!lease.active && !lease.expire(5000, 5000));
  lease.refresh(100);
  assert(!lease.expire(5099, 5000) && lease.active);
  assert(lease.expire(5100, 5000) && !lease.active);
  assert(!lease.expire(6000, 5000));
  lease.refresh(0xFFFFFFF0);
  assert(!lease.expire(10, 100));
  assert(lease.expire(100, 100));
  // 8000 pulses/mile at 60 mph = 133.333 Hz / 7500 us.
  assert(gateway::speedKph10(7500, 8000) == 965);
  assert(gateway::speedKph10(0, 8000) == 0);
  assert(gateway::rpm(5000, 40) == 3000);
  assert(gateway::rpm(5000, 20) == 6000);
  assert(gateway::rpm(0, 40) == 0);
  assert(gateway::rpm(1, 1) == 65535);
  volatile gateway::Pulse pulse;
  pulse.edge(0xFFFFF000, 500, 500000);
  assert(pulse.periodAt(0xFFFFF000, 500000) == 0);
  pulse.edge(0xFFFFF010, 500, 500000); // ignored glitch
  pulse.edge(904, 500, 500000); // rollover, 5000 us after first edge
  assert(pulse.periodAt(904, 500000) == 5000);
  assert(pulse.periodAt(501000, 500000) == 0);
  pulse.edge(600000, 500, 500000);
  assert(pulse.periodAt(600000, 500000) == 0); // two fresh edges required
  pulse.edge(605000, 500, 500000);
  assert(pulse.periodAt(605000, 500000) == 5000);
  float ohms = 0;
  assert(gateway::fuelResistance(455, 3300, 100, ohms));
  assert(std::abs(ohms - 16) < 0.1f);
  assert(gateway::fuelPercent(ohms, 16, 158) == 0);
  assert(gateway::fuelResistance(2021, 3300, 100, ohms));
  assert(std::abs(ohms - 158) < 0.1f);
  assert(gateway::fuelPercent(87, 16, 158) == 50);
  assert(gateway::fuelPercent(10, 73, 10) == 100); // reverse-range sender
  assert(!gateway::fuelResistance(0, 3300, 100, ohms));
  assert(!gateway::fuelResistance(3300, 3300, 100, ohms));
  gateway::DebouncedButton button;
  assert(!button.update(true, 10));
  assert(!button.update(false, 20));
  assert(!button.update(true, 22));
  assert(!button.update(true, 46));
  assert(button.update(true, 47) && button.stable);
  assert(!button.update(false, 0xFFFFFFF0));
  assert(button.update(false, 10) && !button.stable);
  wire::Sensors input{965, 3000, 2048, 50, 7}, decoded;
  auto f = wire::packSensors(input);
  assert(f.data[0] == 3 && f.data[1] == 197 && f.data[2] == 11 && f.data[3] == 184);
  assert(wire::unpackSensors(f, decoded) && decoded.fuelRaw == 2048 && decoded.rpm == 3000);
  f.dlc = 7; assert(!wire::unpackSensors(f, decoded));
  wire::Buttons b{21, 31, 255}, out;
  f = wire::packButtons(b);
  assert(wire::unpackButtons(f, out) && out.pressed == 21 && out.sequence == 255);
  f.data[1] = 0; assert(!wire::unpackButtons(f, out));
  wire::Light l{2, 12, 34, 56, 78}, result;
  f = wire::packLight(l);
  assert(wire::unpackLight(f, result) && result.channel == 2 && result.green == 34);
  f.data[0] = 3; assert(!wire::unpackLight(f, result));
  f.data[0] = 0; f.dlc = 8; assert(!wire::unpackLight(f, result));
  f.dlc = 6; f.data[5] = 0; assert(!wire::unpackLight(f, result));
  puts("PASS: gateway calibration, divider conversion, debounce/rollover and CAN wire validation");
}
