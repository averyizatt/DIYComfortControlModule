# Water-Meth Controller Module

This module runs the standalone water-meth controller firmware and publishes state over the shared CAN contract.

## Analog sensor support in-module

The analog thermistor and pressure support is implemented in this module (not in CCM runtime logic).

### Thermistor channels (NTC, passive 2-wire)

Implemented channels:

- IAT
- Engine bay temperature
- Cabin temperature
- Ambient temperature

Wiring per channel:

```text
3.3V -> 10k pullup -> ADC node -> NTC sensor -> GND
```

The module performs:

- oversampled ADC reads
- voltage/resistance conversion
- Steinhart-Hart temperature conversion
- smoothing filter
- open/short/out-of-range fault detection

### Pressure channels (3-wire transducers)

Implemented channels:

- Oil pressure
- Fuel pressure
- Meth pressure
- Boost reference pressure
- Spare pressure 1
- Spare pressure 2

Sensor output scaling:

```text
Sensor OUT -> 220R protection -> 10k -> ADC node -> 20k -> GND
```

The module performs:

- ADC node voltage read + reconstruction of sensor output voltage
- configurable 0.5V–4.5V transfer conversion to PSI
- optional calibration scale/offset
- smoothing and fault detection

## CAN telemetry

- `0x300` (`ID_ENGINE_METH_STATE`) now publishes real IAT and engine-bay temperature values.
- `0x303` (`ID_ENGINE_SENSOR_EXT`) publishes pressure channels plus ambient/cabin temperatures and analog fault flags.

## Build

Build this module independently with PlatformIO from this directory:

```bash
cd modules/water-meth
pio run -e arduino_nano_esp32
```
