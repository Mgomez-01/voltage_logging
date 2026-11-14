#include "status_led.h"
#include <NeoPixelBus.h>
#include "heater_controller.h"
#include "safety_system.h"

// One pixel. The DMA method's pin is fixed to GPIO3 (RX); the constructor's
// pixel count is the only argument it uses.
static NeoPixelBus<NeoGrbFeature, NeoEsp8266Dma800KbpsMethod> statusLed(1);

// The WS2812B is very bright at full scale (and draws ~20 mA per colour from
// the USB 5 V). LED_BRIGHTNESS is the level of a full colour (0-255).
static const uint8_t LED_BRIGHTNESS = 64;
static const unsigned long LED_UPDATE_MS = 50;
static const unsigned long LED_FLASH_HALF_PERIOD_MS = 500;   // 1 Hz flashing

static bool ledReady = false;
static uint32_t lastColour = 0xFFFFFFFF;   // Forces the first Show()
static unsigned long lastUpdate = 0;

static uint8_t scale(uint8_t level) {
  return (uint16_t)level * LED_BRIGHTNESS / 255;
}

static void showColour(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t colour = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  if (colour == lastColour) return;   // Only send when it changes
  lastColour = colour;
  statusLed.SetPixelColor(0, RgbColor(r, g, b));
  statusLed.Show();
}

void setupStatusLed() {
  statusLed.Begin();     // Takes over GPIO3 (RX) for the I2S data output
  ledReady = true;
  showColour(0, 0, 0);
  Serial.println(F("Status LED: WS2812B on GPIO3 (RX), NeoPixelBus DMA"));
}

void statusLedLoop() {
  if (!ledReady) return;
  unsigned long now = millis();
  if (now - lastUpdate < LED_UPDATE_MS) return;
  lastUpdate = now;

  if (emergencyShutdown) {
    bool on = (now / LED_FLASH_HALF_PERIOD_MS) % 2 == 0;
    if (!on) { showColour(0, 0, 0); return; }
    if (shutdownReason == SHUTDOWN_TIMEOUT) showColour(scale(255), scale(64), 0);   // Orange
    else showColour(scale(255), scale(255), 0);                                     // Yellow
    return;
  }

  bool heaterOn = pidEnabled || heaterEnabled || heaterDutyCycle > 0;
  if (!heaterOn) { showColour(0, 0, 0); return; }

  // Blue (0 %) -> red (100 %), following the PWM duty
  float duty = heaterDutyCycle;
  if (duty < 0) duty = 0;
  if (duty > 100) duty = 100;
  uint8_t red = (uint8_t)lroundf(duty * 2.55f);
  showColour(scale(red), 0, scale(255 - red));
}
