# Vendored, patched third-party libraries

Not on the default `lib/` path on purpose. PlatformIO auto-injects `lib/` into **every**
environment; anything dropped there would silently replace the upstream library for all of
Meshtastic's boards. These are pulled in per-environment via `symlink://` in `lib_deps`
instead, so only the env that asks for the patch gets it.

## OneWire 2.3.8 — ESP32-S2/S3 GPIO range

Upstream's `util/OneWire_direct_gpio.h` targets the original ESP32, where GPIO tops out at 39
and pins 34-39 are input-only. Its guards (`pin < 46` for read/write, `pin <= 33` for
output-enable) mean that on an **ESP32-S3** any pin above 33 never becomes an output and any
pin above 45 reads as a constant 0.

`OneWire::reset()` therefore exhausts its 125 retries waiting for the bus to read high and
returns 0 — *"no presence pulse"* — without ever driving the line. A perfectly wired DS18B20
on GPIO 47 is indistinguishable from a dead one.

The patch replaces those two constants with `SOC_GPIO_PIN_COUNT` and a target-aware
`ONEWIRE_CAN_OUTPUT()`. Behaviour on the original ESP32 is byte-for-byte unchanged; S2/S3
pins now work. Nothing else in the library is touched.

Used by `env:heltec-vision-master-e290-fridge`. See `devices/vision-master-e290/WIRING-fridge.md`.
