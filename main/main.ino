#include "Adafruit_VL53L1X.h"
#include <Adafruit_NeoPixel.h>

#include "MenuManager.ino"

#define BALL_DIAMETER 217 //mm (average of allowed maximum and minimum ball diameter)

// TODO this parameters should be red dynamically in some way (probably configurable from the menu)
#define LANE_WIDTH 1070   //mm
#define GUTTER_WIDTH 250  //mm

// SENSORS PARAMS
#define IRQ_PIN_L 2
#define XSHUT_PIN_L 4

#define IRQ_PIN_R 16
#define XSHUT_PIN_R 17

// BUTTONS AND ENCODER PARAMS
#define MENU_BTN_PIN 16 // TODO
#define SELECT_BTN_PIN -1 // TODO

#define ENCODER_A -1  // TODO
#define ENCODER_B -1  //TODO

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
  Wire1.begin();
  if (! sensor_right.begin(0x30, &Wire1)) {
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

void setupInputs(){
  // TODO check if this buttons can be set to INPUT_PULLDOWN to simplify the circuit.
  pinMode(MENU_BTN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MENU_BTN_PIN), menuBtnPress, RISING);

  pinMode(SELECT_BTN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SELECT_BTN_PIN), selectBtnPress, RISING);

  // TODO Check if the GPIOs are actually pulled up
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), read_encoder, CHANGE);
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

// code from https://github.com/mo-thunderz/RotaryEncoder/blob/main/Arduino/ArduinoRotaryEncoder/ArduinoRotaryEncoder.ino
void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    // counter = counter + changevalue;              // Perform action
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    // counter = counter + changevalue;              // Perform action
    encval = 0;
  }
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

int16_t leftLastValidDistance = 0;
int16_t rightLastValidDistance = 0;
int16_t lastValidDistance = 0;

// Fetch the sensor measurement and update the leftLastvalidDistance variable.
bool getLeftSensorReading(){
  int16_t distance = 0;

  if (!sensor_left.dataReady())
    return false;

  // new measurement for the taking!
  distance = sensor_left.distance();
  if (distance == -1) {
    // something went wrong!
    Serial.print(F("Couldn't get left distance: "));
    Serial.println(sensor_left.vl_status);
    return false;
  }
  // data is read out, time for another reading!
  sensor_left.clearInterrupt();

  leftLastValidDistance = distance;

  return true;
}

// Fetch the sensor measurement and update the rightLastvalidDistance variable.
bool getRightSensorReading(){
  int16_t distance = 0;

  if (sensor_right.dataReady())
    return false;

  // new measurement for the taking!
  distance = sensor_right.distance();
  if (distance == -1) {
    // something went wrong!
    Serial.print(F("Couldn't get right distance: "));
    Serial.println(sensor_right.vl_status);
    return false;
  }
  // data is read out, time for another reading!
  sensor_left.clearInterrupt();

  rightLastValidDistance = distance;

  return true;
}

// Given a distance from a lane side (doesn't matter which) in mm, returns the closest listel to that point.
// This function doesn't account for gutter width
int16_t distanceToListel(float distance){
  int16_t numListels = 39;
  int16_t laneWidth = 1070;  //mm
  
  return round(distance/(laneWidth/numListels));
}

// Calculate the ball center position from the LEFT of the bowling lane.
float calculateBallCenter(){
  if(!getLeftSensorReading())
    return -1;

  if(!getRightSensorReading())
    return -1;

  // estimated center position from the left side of the lane from sensors raw data
  int16_t laneLeftPos = leftLastValidDistance - GUTTER_WIDTH + (BALL_DIAMETER/2);
  int16_t laneRightPos = LANE_WIDTH - (rightLastValidDistance - GUTTER_WIDTH + (BALL_DIAMETER/2));

  return (laneLeftPos + laneRightPos) / 2;  // average of the two calculated centers
}

void updateLEDStrip(){
  // Free mode

  // Single target mode

  // Range mode
}

void loop() {
  int16_t ballCenter;

  // if the center has not been calculated don't do anything 
  if((ballCenter = calculateBallCenter()) == -1)
    return;

  if (ballCenter < 0 || ballCenter > LANE_WIDTH) // invalid center position
    return;

  lastValidDistance = ballCenter;
  
  // LED strip logic
  updateLEDStrip();

  strip.show();
}
