#include "Adafruit_VL53L1X.h"
#include "Adafruit_NeoPixel.h"

#include "MenuManager.ino"

#define IRQ_PIN_L 2
#define XSHUT_PIN_L 4

#define IRQ_PIN_R 16
#define XSHUT_PIN_R 17

Adafruit_VL53L1X sensor_left = Adafruit_VL53L1X(XSHUT_PIN_L, IRQ_PIN_L);
Adafruit_VL53L1X sensor_right = Adafruit_VL53L1X(XSHUT_PIN_R, IRQ_PIN_R);

Adafruit_NeoPixel strip(2, 23, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  if (! sensor_left.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of left VL sensor: "));
    Serial.println(sensor_left.vl_status);
    while (1)       delay(10);
  }
  if (! sensor_right.begin(0x30, &Wire)) {
    Serial.print(F("Error on init of right VL sensor: "));
    Serial.println(sensor_right.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensors OK!"));

  Serial.print(F("Sensor left ID: 0x"));
  Serial.println(sensor_left.sensorID(), HEX);

  Serial.print(F("Sensor right ID: 0x"));
  Serial.println(sensor_right.sensorID(), HEX);

  if (! sensor_left.startRanging()) {
    Serial.print(F("Left couldn't start ranging: "));
    Serial.println(sensor_left.vl_status);
    while (1)       delay(10);
  }

  if (! sensor_right.startRanging()) {
    Serial.print(F("Right couldn't start ranging: "));
    Serial.println(sensor_right.vl_status);
    while (1)       delay(10);
  }

  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  sensor_left.setTimingBudget(100);
  Serial.print(F("Timing budgets (ms): L "));
  Serial.print(sensor_left.getTimingBudget());
  Serial.print(" R ");
  Serial.println(sensor_right.getTimingBudget());

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */

  //init led
  strip.setBrightness(50);
  strip.begin();
  strip.show();
}

int leftLastValidDistance = 0;
int rightLastValidDistance = 0;
int lastValidDistance = 0;

void loop() {
  int16_t distance;

  if (sensor_left.dataReady()) {
    // new measurement for the taking!
    distance = sensor_left.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get left distance: "));
      Serial.println(sensor_left.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // data is read out, time for another reading!
    sensor_left.clearInterrupt();

    leftLastValidDistance = distance;
  }

  if (sensor_right.dataReady()) {
    // new measurement for the taking!
    distance = sensor_right.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get right distance: "));
      Serial.println(sensor_right.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // data is read out, time for another reading!
    sensor_left.clearInterrupt();

    rightLastValidDistance = distance;
  }

  lastValidDistance = (leftLastValidDistance + rightLastValidDistance)/2;
  
  if(lastValidDistance > 1000){
    strip.setPixelColor(0, 0, 0, 255);
    strip.setPixelColor(1, 0, 0, 255);
  }
  else if(lastValidDistance < 1000 && lastValidDistance >500){
    strip.setPixelColor(0, 0, 255, 0);
    strip.setPixelColor(1, 0, 255, 0);  
  }
  else if(lastValidDistance < 500 && lastValidDistance > 200){
    strip.setPixelColor(0, 0, 255, 0);
    strip.setPixelColor(1, 255, 0, 0);  
  }
  else if(lastValidDistance < 200){
    strip.setPixelColor(0, 255, 0, 0);
    strip.setPixelColor(1, 255, 0, 0);  
  }
  strip.show();
}
