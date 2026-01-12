#include <VL53L1X.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"

Adafruit_NeoPixel strip(2, STRIP_PIN, NEO_GRB + NEO_KHZ800);

void setupLedStrip() {
  Serial.println("Starting LED strip init...");

  //init led
  strip.begin();

  strip.setBrightness(100);
  strip.fill(strip.Color(0, 255, 0), 0, 2);

  Serial.println("LED strip init success.");

  strip.show();
}

void updateLEDStrip(uint8_t listel, uint8_t  mode) {
  // Free mode
  uint8_t colorR = map(listel, 1, NUM_LISTELS, 0, 255);
  uint8_t colorG = map(listel, 1, NUM_LISTELS, 255, 0);

  strip.fill(strip.Color(colorR, colorG, 0), 0, 2);
  // Single target mode

  // Range mode
}

void stripShow(){
  strip.show();
}