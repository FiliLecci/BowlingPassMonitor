#include <SPI.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_NeoPixel.h>

#include "menuManager.h"
#include "config.h"

VL53L1X sensor_left;
VL53L1X sensor_right;

// Mutex for menu manager to be used
portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;

Adafruit_NeoPixel strip(2, STRIP_PIN, NEO_GRB + NEO_KHZ800);

const uint16_t numListels = 39;
const uint16_t laneWidth = 1070;  //mm

// Variables for encoder reading function
unsigned long _lastIncReadTime = micros();
unsigned long _lastDecReadTime = micros();
unsigned long _lastSelTime = micros();
unsigned long _lastMenuTime = micros();
int _pauseLength = 25000;
int _fastIncrement = 10;

volatile int test = 0;

void scanI2C1() {
  byte error, address;
  Serial.println("Scansione bus I2C...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Dispositivo trovato a 0x");
      Serial.println(address, HEX);
    }
  }
}

void scanI2C2() {
  byte error, address;
  Serial.println("Scansione bus I2C...");
  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    if (Wire1.endTransmission() == 0) {
      Serial.print("Dispositivo trovato a 0x");
      Serial.println(address, HEX);
    }
  }
}

void setupSensors() {
  pinMode(XSHUT_PIN_L, OUTPUT);
  pinMode(XSHUT_PIN_R, OUTPUT);

  // Reset MCU
  digitalWrite(XSHUT_PIN_L, LOW);
  digitalWrite(XSHUT_PIN_R, LOW);
  delay(10);

  digitalWrite(XSHUT_PIN_L, HIGH);
  digitalWrite(XSHUT_PIN_R, HIGH);
  delay(10);

  // Sleep both sensors
  digitalWrite(XSHUT_PIN_L, LOW);
  digitalWrite(XSHUT_PIN_R, LOW);
  delay(10);

  // Wake up left sensor
  digitalWrite(XSHUT_PIN_L, HIGH);
  delay(50);

  Wire.begin(SDA_1, SCL_1);
  // setup left sensor
  Serial.println("Left sensor init...");

  sensor_left.setBus(&Wire);
  if (!sensor_left.init()) {
    Serial.print(F("Error on init of left VL sensor."));
    while (1)
      delay(100);
  }
  sensor_left.setAddress(0x30);

  Serial.print(F("Sensor left ID: 0x"));
  Serial.println(sensor_left.getAddress(), HEX);

  // setup right sensor
  // Wake up right sensor
  digitalWrite(XSHUT_PIN_R, HIGH);
  delay(50);

  Serial.println("Right sensor init...");

  sensor_right.setBus(&Wire);
  if (!sensor_right.init()) {
    Serial.print(F("Error on init of right VL sensor."));
    while (1)
      delay(100);
  }
  sensor_right.setAddress(0x31);

  // SENSORS SETUP COMPLETE
  Serial.print(F("Sensor right ID: 0x"));
  Serial.println(sensor_right.getAddress(), HEX);

  Wire.setClock(400000);

  Serial.println(F("VL53L1X sensors OK!"));
}

void setupLedStrip() {
  Serial.println("Starting LED strip init...");

  //init led
  strip.begin();

  strip.setBrightness(100);
  strip.fill(strip.Color(0, 255, 0), 0, 2);

  Serial.println("LED strip init success.");

  strip.show();
}

// code from https://github.com/mo-thunderz/RotaryEncoder/blob/main/Arduino/ArduinoRotaryEncoder/ArduinoRotaryEncoder.ino
void IRAM_ATTR read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent

  static uint8_t old_AB = 3;                                                                  // Lookup table index
  static int8_t encval = 0;                                                                   // Encoder value
  static const int8_t enc_states[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };  // Lookup table

  old_AB <<= 2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02;  // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01;  // Add current state of pin B

  encval += enc_states[(old_AB & 0x0f)];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if (encval > 3) {  // Four steps forward
    int changevalue = 1;
    if ((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue;
    }
    _lastIncReadTime = micros();
    encoderAction(-1);  // Perform action
    encval = 0;
  } else if (encval < -3) {  // Four steps backward
    int changevalue = -1;
    if ((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue;
    }
    _lastDecReadTime = micros();
    encoderAction(1);  // Perform action
    encval = 0;
  }
}

void IRAM_ATTR btnSelectPressed(){
  if((micros() - _lastSelTime) > _pauseLength) {
    _lastSelTime = micros();
    selectBtnPress();
  }
}

void IRAM_ATTR btnMenuPressed(){
  if((micros() - _lastMenuTime) > _pauseLength) {
    _lastMenuTime = micros();
    prevMenuAction();
  }
}

void setupInputs() {
  pinMode(MENU_BTN_PIN, INPUT_PULLUP);
  attachInterrupt(MENU_BTN_PIN, btnMenuPressed, FALLING);

  pinMode(SELECT_BTN_PIN, INPUT_PULLUP);
  attachInterrupt(SELECT_BTN_PIN, btnSelectPressed, FALLING);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(ENC_A, read_encoder, CHANGE);
  attachInterrupt(ENC_B, read_encoder, CHANGE);
}

void startLeftSensor() {
  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  if (!sensor_left.setMeasurementTimingBudget(50000)) {  // this is in us
    Serial.println("Invalid time budget for left sensor.");
    while (1)
      delay(100);
  }

  sensor_left.startContinuous(100);

  Serial.println(F("Left ranging started"));
}

void startRightSensor() {
  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  if (!sensor_right.setMeasurementTimingBudget(50000)) {  // this is in us
    Serial.println("Invalid time budget for right sensor.");
    while (1)
      delay(100);
  }

  sensor_right.startContinuous(100);

  Serial.println(F("Right ranging started"));
}

int16_t leftLastValidDistance = 0;
int16_t rightLastValidDistance = 0;
int16_t lastValidDistance = 0;

// Fetch the sensor measurement and update the leftLastvalidDistance variable.
bool getLeftSensorReading() {
  int16_t distance = 0;

  if (!sensor_left.dataReady())
    return false;

  // new measurement for the taking!
  distance = sensor_left.read(false);  // This also updates a ranging_data struct with info; might be useful later
  if (distance <= -1) {
    // something went wrong!
    Serial.print(F("Couldn't get left distance: "));
    Serial.println(sensor_left.last_status);
    return false;
  }

  if (!sensor_left.timeoutOccurred())
    leftLastValidDistance = distance;
  else
    Serial.println("LEFT Timeout.");

  return true;
}

// Fetch the sensor measurement and update the rightLastvalidDistance variable.
bool getRightSensorReading() {
  int16_t distance = 0;

  if (!sensor_right.dataReady())
    return false;

  // new measurement for the taking!
  distance = sensor_right.read(false);  // This also updates a ranging_data struct with info; might be useful later
  if (distance <= -1) {
    // something went wrong!
    Serial.print(F("Couldn't get right distance: "));
    Serial.println(sensor_right.last_status);
    return false;
  }

  if (!sensor_right.timeoutOccurred())
    rightLastValidDistance = distance;
  else
    Serial.println("RIGHT Timeout.");

  return true;
}

// Given a distance from a lane side (doesn't matter which) in mm, returns the closest listel to that point.
// This function doesn't account for gutter width
uint8_t distanceToListel(float distance) {
  return round(distance / (laneWidth / numListels));
}

// Calculate the ball center position from the LEFT of the bowling lane.
bool calculateBallCenter() {
  if (!getLeftSensorReading())
    return false;

  if (!getRightSensorReading())
    return false;

  // Check if measurements are valid
  if (leftLastValidDistance < GUTTER_WIDTH - (BALL_DIAMETER / 2) || leftLastValidDistance > LANE_WIDTH + GUTTER_WIDTH) {
    return false;
  }

  if (rightLastValidDistance < GUTTER_WIDTH - (BALL_DIAMETER / 2) || rightLastValidDistance > LANE_WIDTH + GUTTER_WIDTH) {
    return false;
  }

  // estimated center position from the left side of the lane from sensors raw data
  int16_t laneLeftPos = leftLastValidDistance - GUTTER_WIDTH + (BALL_DIAMETER / 2);
  int16_t laneRightPos = LANE_WIDTH - (rightLastValidDistance - GUTTER_WIDTH + (BALL_DIAMETER / 2));

  lastValidDistance = (laneLeftPos + laneRightPos) / 2;  // average of the two calculated centers

  return true;
}

void updateLEDStrip() {
  // Free mode
  uint8_t colorR = map(leftLastValidDistance, 0, LANE_WIDTH, 0, 255);
  uint8_t colorG = map(leftLastValidDistance, 0, LANE_WIDTH, 255, 0);

  strip.fill(strip.Color(colorR, colorG, 0), 0, 2);
  // Single target mode

  // Range mode
}

void setup() {
  pinMode(2, OUTPUT);

  Serial.begin(115200);
  while (!Serial) delay(10);

  setupSensors();

  startLeftSensor();
  startRightSensor();

  setupInputs();

  setupLedStrip();

  setupScreen();

  initMenu();

  scanI2C1();
  scanI2C2();
}

void loop() {
  calculateBallCenter();

  // Serial.printf("%d | %d | %d\n", leftLastValidDistance, rightLastValidDistance, lastValidDistance);

  //drawBars(leftLastValidDistance-GUTTER_WIDTH, rightLastValidDistance-GUTTER_WIDTH, lastValidDistance);
  // displayAndClear();

  // LED strip logic
  // updateLEDStrip();
  // strip.show();

  if (isMenuChanged()) {
    Serial.println("Updating menu...");

    updateMenuSafe();

    Serial.println("Done.");

    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
  }

  delay(10);
}