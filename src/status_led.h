#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

// WS2812B status LED (D2 on the PCB), data on RX / GPIO3 through a
// 74AHCT1G125 level shifter (U3). Driven by NeoPixelBus with the ESP8266 DMA
// (I2S) method, which only works on GPIO3 and does not block interrupts, so the
// heater PWM and WiFi are not disturbed. Serial must be TX-only (main.cpp).
//
//   Off                    heater off (no PID, no manual power)
//   Blue -> red            heater on: colour follows the PWM duty, blue = 0 %, red = 100 %
//   Flashing orange        emergency shutdown because the heater ran longer than MAX_HEATER_TIME
//   Flashing yellow        any other emergency shutdown (over-temperature, sensor fault, watchdog)

void setupStatusLed();   // Call after Serial.begin()
void statusLedLoop();    // Call from loop(); updates at most every 50 ms, sends only on change

#endif // STATUS_LED_H
