# Bench Validation Checklist

## Display/UI
- [ ] Boot to dashboard in < 2s
- [ ] RPM gauge updates smoothly at >= 20 FPS visual cadence
- [ ] Speed/GPS/CAN indicators update without UI stalls
- [ ] Day/night brightness switching behaves as expected
- [ ] Warning popup visibility in both lighting conditions

## CAN
- [ ] 500 kbps bus operation verified with at least two slave nodes
- [ ] Master heartbeat period and timeout handling validated
- [ ] Water/meth command path verified (mix + enable)
- [ ] Taillight mode command path verified (stock/sequential/show/demo)
- [ ] Node offline detection and fault status propagation verified

## Tach
- [ ] Startup sweep executes once on boot
- [ ] RPM tracking latency and stability verified
- [ ] Frequency scaling validated for RPM/15 mode
- [ ] Frequency scaling validated for RPM/30 mode
- [ ] Duty cycle and signal integrity validated at cluster input

## GPS/Sensors
- [ ] GPS fix acquisition and loss-of-signal behavior verified
- [ ] Speed updates non-blocking while UI remains responsive
- [ ] Cabin/engine/outside/intake values update continuously
- [ ] Battery undervoltage threshold transitions into degraded mode

## Reliability
- [ ] CAN dropouts do not lock UI or task scheduling
- [ ] Sensor/GPS failures set fault flags and preserve operation
- [ ] No watchdog resets during 30-minute soak test
