#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static bool isTargetSelection = false;    // Allow to modify selected value (if possible)

static uint8_t singleTarget = 0;          // Single target selected listel
static uint8_t range[2] = {0,0};          // Range left and right selected listels

void setupScreen(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;) delay(100);
  }

  display.display();
  delay(2000);

  // Clear the buffer
  display.clearDisplay();
}

void menuBtnPress(){}

void selectBtnPress(){}

void increaseAction(){}

void decreaseAction(){}