# Water/meth controller

The supported controller is the ESP32-S3 in
[DIYWaterMethInjection](https://github.com/averyizatt/DIYWaterMethInjection).
Check it out beside this repository; it includes CCM's shared CAN contract.

The copied Nano firmware has been removed. Build `esp32s3_water_meth` in the
standalone repository. Its hardware guide documents GPIO15 MOSFET pulse drive,
GPIO5/GPIO6 intake inputs, and firmware updates from the dash over CAN.
GPIO17 is unused; there is no master relay.
