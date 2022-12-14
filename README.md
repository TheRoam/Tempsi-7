# Temps-i 7

Arduino .ino script for desktop clock display using Sparkfun ESP32-Thing, BMP-280 sensor and 4 digit 7 segment display.

Further documentation:

https://theroamingworkshop.cloud/b/?p=1942&lang=en

Features:
  - SNTP time sync via WiFi using Sparkfun ESP32-Thing microcontroller board.
  - Software clock (offline update using internal counter).
  - Temperature display using BMP-280 sensor.
  - Program switch using pushbutton
  
Changelog:
  v 1.0.1:
  - Initial release of working features (sntp, temperature display, program switch)
  - Serial communication commented for production
  - Second indicator led (dot blinker) commented as it desyncs hour digits (looking for a smarter fix)
