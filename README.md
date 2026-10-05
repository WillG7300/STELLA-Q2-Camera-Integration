# STELLA-Q2-Camera-Integration
CircuitPython integration of an OV2640 Arducam camera with the NASA STELLA-Q2 spectrometer.

This project adds an OV2640 Arducam camera to the NASA STELLA-Q2 spectrometer in order to better understand recorded data.

## Current behavior
- One short button press records one spectral measurement.
- One JPEG is captured for each spectral measurement.
- JPEG filename is recorded in the CSV.
- UTC timestamp links photo and spectral data.
- SD-card failure triggers an OLED lockout.
- Field mode disables accidental RTC-setting menus.
- All measurements from one day are saved to the same file.

## Camera wiring
- MISO: D16
- CS: D17
- SCLK: D18
- MOSI: D19
- SDA: D20
- SCL: D21
- VCC: 3.3 V
- GND: GND

## Hardware
- SparkFun Thing Plus RP2040
- STELLA-Q2
- Arducam OV2640 Mini 2MP Plus

## Status
Prototype / field testing in DR 
Add Lidar is next step
