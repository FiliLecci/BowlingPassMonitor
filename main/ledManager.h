#include <Adafruit_NeoPixel.h>

#include "./config.h"

Adafruit_NeoPixel strip(NUM_LED, STRIP_PIN, NEO_GRB + NEO_KHZ800);

static volatile uint8_t* singleTargetBinder;
static volatile uint8_t* rangeBinder;

static unsigned long lastUpdate = 0;          // used to reset LEDs to default mode
static unsigned long updateTimeMillis = 10000; // time before reset

void setupLedStrip() {
  Serial.println("Starting LED strip init...");

  //init led
  strip.begin();

  strip.setBrightness(100);
  strip.fill(strip.Color(0, 255, 0), 0, NUM_LED);

  Serial.println("LED strip init success.");

  strip.show();
}

void setLEDSingleTargetBinder(volatile uint8_t* binder){
  singleTargetBinder = binder;
}

void setLEDRangeBinder(volatile uint8_t* binder){
  rangeBinder = binder;
}

static uint8_t lastMode = 0;

void updateMode(uint8_t  mode) {
  if(mode != 0)
    lastMode = mode;
}

static void showFreeMode(uint8_t ballListel) {
  strip.setPixelColor(ballListel-1, 0, 0, 255);
}

static void showTargetMode(uint8_t ballListel) {
  strip.setPixelColor(*singleTargetBinder-1, 0, 0, 255);

  if(ballListel == *singleTargetBinder){
    strip.setPixelColor(ballListel-1, 0, 255, 0);
  }
  else {
    strip.setPixelColor(ballListel-1, 255, 0, 0);
  }
}

static void showRangeMode(uint8_t ballListel) {
  strip.setPixelColor(rangeBinder[0]-1, 0, 0, 255);
  strip.setPixelColor(rangeBinder[1]-1, 0, 0, 255);

  if(ballListel >= rangeBinder[0] && ballListel <= rangeBinder[1]){
    strip.setPixelColor(ballListel-1, 0, 255, 0);
  }
  else {
    strip.setPixelColor(ballListel-1, 255, 0, 0);
  }
}

void updateLEDStrip(uint8_t ballListel) {
  uint8_t tempListel = ballListel;

  // If position was not updated for updateTimeMillis milliseconds reset ball position
  if (ballListel == -1){
    if(millis() - lastUpdate >= updateTimeMillis){
      tempListel = 0;
    }
    else{
      return;
    }
  }
  else{
    lastUpdate = millis();
  }

  strip.fill(strip.Color(0, 0, 0), 0, NUM_LED);
  
  switch (lastMode) {
    // Free mode
    case 1:
      showFreeMode(tempListel);
      break;
    // Single target mode
    case 2:
      showTargetMode(tempListel);
      break;
    // Range mode
    case 3:
      showRangeMode(tempListel);
      break;
    default:
    // When no mode is selected
      strip.fill(strip.Color(255, 255, 255), 0, NUM_LED);
  }
}

void stripShow(){
  strip.show();
}