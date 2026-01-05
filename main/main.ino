#include "Adafruit_VL53L1X.h"
#include "Adafruit_NeoPixel.h"

#include "MenuManager.ino"

#define BALL_DIAMETER 217 //mm (average of allowed maximum and minimum ball diameter)

// TODO this parameters should be red dynamically in some way (probably configurable from the menu)
#define LANE_WIDTH 1070   //mm
#define GUTTER_WIDTH 250  //mm

#define IRQ_PIN_L 2
#define XSHUT_PIN_L 4

#define IRQ_PIN_R 16
#define XSHUT_PIN_R 17

Adafruit_VL53L1X sensor_left = Adafruit_VL53L1X(XSHUT_PIN_L, IRQ_PIN_L);
Adafruit_VL53L1X sensor_right = Adafruit_VL53L1X(XSHUT_PIN_R, IRQ_PIN_R);

Adafruit_NeoPixel strip(2, 23, NEO_GRB + NEO_KHZ800);

// TODO keep trying to connect sensor if it fails
void setupSensors(){
  // setup left sensor
  Wire.begin();
  if (! sensor_left.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of left VL sensor: "));
    Serial.println(sensor_left.vl_status);
    while (1)       
      delay(100);
  }

  // setup right sensor
  if (! sensor_right.begin(0x30, &Wire)) {
    Serial.print(F("Error on init of right VL sensor: "));
    Serial.println(sensor_right.vl_status);
    while (1)
      delay(100);
  }

  // SENSORS SETUP COMPLETE

  Serial.println(F("VL53L1X sensors OK!"));

  Serial.print(F("Sensor left ID: 0x"));
  Serial.println(sensor_left.sensorID(), HEX);

  Serial.print(F("Sensor right ID: 0x"));
  Serial.println(sensor_right.sensorID(), HEX);

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
}

void setupLedStrip(){
  //init led
  strip.setBrightness(50);
  strip.begin();
  strip.show();
}

void startLeftSensor(){
    if (! sensor_left.startRanging()) {
      Serial.print(F("Left couldn't start ranging: "));
      Serial.println(sensor_left.vl_status);
      while (1)
        delay(10);
  }

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  sensor_left.setTimingBudget(100);

  Serial.println(F("Left ranging started"));
}

void startRightSensor(){
    if (! sensor_right.startRanging()) {
      Serial.print(F("Right couldn't start ranging: "));
      Serial.println(sensor_right.vl_status);
      while (1)
        delay(10);
  }

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  sensor_right.setTimingBudget(100);

  Serial.println(F("Right ranging started"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  setupSensors();

  startLeftSensor();
  
  Serial.print(F("Timing budgets (ms): L "));
  Serial.print(sensor_left.getTimingBudget());
  Serial.print(" R ");
  Serial.println(sensor_right.getTimingBudget());

  setupLedStrip();
}

int leftLastValidDistance = 0;
int rightLastValidDistance = 0;
int lastValidDistance = 0;

// Fetch the sensor measurement and update the leftLastvalidDistance variable.
int getLeftSensorReading(){
  float distance = 0;

  if (sensor_left.dataReady()) {
    // new measurement for the taking!
    distance = sensor_left.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get left distance: "));
      Serial.println(sensor_left.vl_status);
      return -1;
    }
    // data is read out, time for another reading!
    sensor_left.clearInterrupt();

    leftLastValidDistance = distance;

    return 1;
  }
  return 0;
}

// Fetch the sensor measurement and update the rightLastvalidDistance variable.
int getRightSensorReading(){
  float distance = 0;

  if (sensor_right.dataReady()) {
    // new measurement for the taking!
    distance = sensor_right.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get right distance: "));
      Serial.println(sensor_right.vl_status);
      return -1;
    }
    // data is read out, time for another reading!
    sensor_left.clearInterrupt();

    rightLastValidDistance = distance;

    return 1;
  }
  return 0;
}

// Given a distance from a lane side (doesn't matter which) in mm, returns the closest listel to that point.
// This function doesn't account for gutter width
int distanceToListel(float distance){
  int numListels = 39;
  int laneWidth = 1070;  //mm
  
  return round(distance/(laneWidth/numListels));
}

// Calculate the ball center position from the LEFT of the bowling lane.
int calculateBallCenter(){
  if(getLeftSensorReading() != 1)
    return -1;

  if(getRightSensorReading() != 1)
    return -1;

  // estimated center position from the left side of the lane from sensors raw data
  int laneLeftPos = leftLastValidDistance - GUTTER_WIDTH + (BALL_DIAMETER/2);
  int laneRightPos = LANE_WIDTH - (rightLastValidDistance - GUTTER_WIDTH + (BALL_DIAMETER/2));

  return (laneLeftPos + laneRightPos) / 2;  // average of the two calculated centers
}

void loop() {
  int16_t distance;

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
