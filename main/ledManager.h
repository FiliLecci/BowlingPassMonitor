#include <Adafruit_NeoPixel.h>

#include "./config.h"

Adafruit_NeoPixel strip(NUM_LED, STRIP_PIN, NEO_GRB + NEO_KHZ800);

void setupLedStrip() {
  Serial.println("Starting LED strip init...");

  //init led
  strip.begin();

  strip.setBrightness(100);
  strip.fill(strip.Color(0, 255, 0), 0, NUM_LED);

  Serial.println("LED strip init success.");

  strip.show();
}

static uint8_t color = 0;
static int8_t dir = 1; 
static uint8_t lastMode = 0;

static void updateMode(uint8_t  mode) {
  lastMode = mode;
  Serial.printf("Mode setted to %d\n", lastMode);
}

void updateLEDStrip(uint8_t ballListel) {
  switch (lastMode) {
    // Free mode
    case 1:
      strip.fill(strip.Color(color, 0, 0), 0, NUM_LED);
      break;
    // Single target mode
    case 2:
      strip.fill(strip.Color(0, color, 0), 0, NUM_LED);
      break;
    // Range mode
    case 3:
      strip.fill(strip.Color(0, 0, color), 0, NUM_LED);
      break;
    default:
      strip.fill(strip.Color(color, color, color), 0, NUM_LED);
  }

  if(color >= 255)
    dir = -1;
  else if(color <= 0)
    dir = 1;

  color += 1*(dir);
}

void stripShow(){
  strip.show();
}