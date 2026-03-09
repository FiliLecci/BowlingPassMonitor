#include <SPI.h>
#include <Wire.h>
#include <VL53L1X.h>

#include "menuManager.h"
#include "./ledManager.h"
#include "config.h"

VL53L1X sensor_left;
VL53L1X sensor_right;

// Mutex for menu manager to be used
portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;

// Variables for encoder reading function
unsigned long _lastIncReadTime = micros();
unsigned long _lastDecReadTime = micros();
unsigned long _lastSelTime = micros();
unsigned long _lastMenuTime = micros();
int _pauseLength = 25000;
int _fastIncrement = 10;
bool _selBtnPressed = false;
bool _menuBtnPressed = false;

// sensors distances
int16_t leftLastValidDistance = 0;
int16_t rightLastValidDistance = 0;
int16_t lastCalculatedCenter = 0;
uint16_t *leftDistanceBuffer; // buffer with left distances used to calculate the average position
uint16_t *rightDistanceBuffer; // buffer with right distances used to calculate the average position
uint8_t bufferSize = 0;
// used to buffer data
unsigned long _lastMeasurementTime = 0; // last valid measurement time for both sensors
unsigned long updateTimeMillis = 500;   // time with no new data after which the position is calculated
unsigned long positionResetTime = 10000; // time with no new data after which the position is reset to avoid showing stale data
bool bufferCalculated = false; // flag to know if the position has been calculated with the buffer data

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
    setLeftSensorStatus(SETUP_ERROR);
    drawSetupStatus();

    Serial.print(F("Error on init of left VL sensor."));
    while (1)
      delay(100);
  }
  sensor_left.setAddress(0x30);

  Serial.print(F("Sensor left ID: 0x"));
  Serial.println(sensor_left.getAddress(), HEX);

  setLeftSensorStatus(SETUP_OK);
  drawSetupStatus();

  // setup right sensor
  // Wake up right sensor
  digitalWrite(XSHUT_PIN_R, HIGH);
  delay(50);

  Serial.println("Right sensor init...");

  sensor_right.setBus(&Wire);
  if (!sensor_right.init()) {
    setRightSensorStatus(SETUP_ERROR);
    drawSetupStatus();

    Serial.print(F("Error on init of right VL sensor."));
    while (1)
      delay(100);
  }
  sensor_right.setAddress(0x31);

  // SENSORS SETUP COMPLETE
  Serial.print(F("Sensor right ID: 0x"));
  Serial.println(sensor_right.getAddress(), HEX);

  setRightSensorStatus(SETUP_OK);
  drawSetupStatus();

  Wire.setClock(400000);

  Serial.println(F("VL53L1X sensors OK!"));
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
    if(_selBtnPressed){
      _selBtnPressed = false;
      return;
    }
    _selBtnPressed = true;
    selectBtnPress();
  }
}

void IRAM_ATTR btnMenuPressed(){
  if((micros() - _lastMenuTime) > _pauseLength) {
    _lastMenuTime = micros();
    if(_menuBtnPressed){
      _menuBtnPressed = false;
      return;
    }
    _menuBtnPressed = true;
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
  if(distance < 0)
    return -1; // invalid distance, return -1 to indicate no position
  return round(distance / (LANE_WIDTH / NUM_LISTELS));
}

// Read sensor data and, if both are valid, add the readings to the respective buffers.
bool readSensorData() {
  bool leftUpdated = getLeftSensorReading();

  if(!leftUpdated)
    return false;

  // check left measurement is within the lane limits (considering the gutter and the ball diameter)
  if (leftLastValidDistance < GUTTER_WIDTH - (BALL_DIAMETER / 2) || leftLastValidDistance > LANE_WIDTH + GUTTER_WIDTH) {
    return false;
  }

  bool rightUpdated = getRightSensorReading();

  if (!rightUpdated)
    return false;
  
    // check right measurement is valid
  if (rightLastValidDistance < GUTTER_WIDTH - (BALL_DIAMETER / 2) || rightLastValidDistance > LANE_WIDTH + GUTTER_WIDTH) {
    return false;
  }

  // both sensors data has been read and validated, we can update the buffers with the new data
  leftDistanceBuffer = (uint16_t*) realloc(leftDistanceBuffer, sizeof(uint16_t) * (bufferSize + 1));  
  rightDistanceBuffer = (uint16_t*) realloc(rightDistanceBuffer, sizeof(uint16_t) * (bufferSize + 1));

  leftDistanceBuffer[bufferSize] = leftLastValidDistance;
  rightDistanceBuffer[bufferSize] = rightLastValidDistance;

  bufferSize++;

  return true;
}

void resetBuffers() {
  bufferSize = 0;
  free(leftDistanceBuffer);
  free(rightDistanceBuffer);
  leftDistanceBuffer = (uint16_t*) malloc(sizeof(uint16_t) * bufferSize);
  rightDistanceBuffer = (uint16_t*) malloc(sizeof(uint16_t) * bufferSize);
}

// Calculate the ball center position from the LEFT of the bowling lane.
int16_t calculateBallCenter() {
  // average the distances in the buffers to get a more stable position
  uint32_t leftSum = 0;
  uint32_t rightSum = 0;

  for (uint8_t i = 0; i < bufferSize; i++) {
    leftSum += leftDistanceBuffer[i];
  }
  for (uint8_t i = 0; i < bufferSize; i++) {
    rightSum += rightDistanceBuffer[i];
  }

  uint16_t leftAverage = leftSum / bufferSize;
  uint16_t rightAverage = rightSum / bufferSize;

  // estimated center position from the left side of the lane from sensors raw data
  int16_t laneLeftPos = leftAverage - GUTTER_WIDTH + (BALL_DIAMETER / 2);
  int16_t laneRightPos = LANE_WIDTH - (rightAverage - GUTTER_WIDTH + (BALL_DIAMETER / 2));

  lastCalculatedCenter = (laneLeftPos + laneRightPos) / 2;  // average of the two calculated centers
  return lastCalculatedCenter;
}

void setup() {
  pinMode(INT_LED_PIN, OUTPUT);

  Serial.begin(115200);
  // while (!Serial) delay(10);

  setupScreen();
  drawSetupStatus();

  setupSensors();

  // initialize distance buffers to 1 element (will be dynamically resized when needed)
  bufferSize = 0;
  leftDistanceBuffer = (uint16_t*) malloc(sizeof(uint16_t) * bufferSize);
  rightDistanceBuffer = (uint16_t*) malloc(sizeof(uint16_t) * bufferSize);

  startLeftSensor();
  startRightSensor();
  
  setupInputs();

  setupLedStrip();
  setLEDSingleTargetBinder(getSingleTargetPtr());
  setLEDRangeBinder(getRangePtr());
  setLedStripStatus(SETUP_OK);
  drawSetupStatus();

  delay(1000);

  displayAndClear();
  initMenu();

  scanI2C1();
  scanI2C2();
}

void loop() {
  // This section keeps a buffer of continous positions and only shows the average
  bool positionUpdate = readSensorData();
  if (positionUpdate){
    _lastMeasurementTime = millis();
  }
  else{
    if(bufferSize > 0){
      if(millis() - _lastMeasurementTime > positionResetTime){
        lastCalculatedCenter = -1; // reset position to avoid showing stale data
        resetBuffers();
        bufferCalculated = false; // reset flag to allow calculating position with new data
        Serial.println("Position buffers reset.");
      }
      else if(millis() - _lastMeasurementTime > updateTimeMillis && !bufferCalculated){
        // calculate ball center with all the distances in the buffer to get a more stable position
        digitalWrite(INT_LED_PIN, HIGH);
        calculateBallCenter();
        bufferCalculated = true; // avoid recalculating position until new data is read
        delay(100);
        digitalWrite(INT_LED_PIN, LOW);
      }
    }
  }
  
  if (isMenuChanged()) {
    Serial.println("Updating menu...");
    
    updateMenuSafe();
    displayAndClear();
    
    Serial.println("Done.");
    
    // mode might have been changed
    updateMode(getMode());
    
    digitalWrite(INT_LED_PIN, HIGH);
    delay(100);
    digitalWrite(INT_LED_PIN, LOW);
  }

  // [DEBUG] print buffer and calculated position
  /*
  Serial.print("Left buffer:");
  for (uint8_t i = 0; i < bufferSize; i++)
    Serial.printf("%d ", leftDistanceBuffer[i]);

  Serial.print("\nRight buffer:");
  for (uint8_t i = 0; i < bufferSize; i++)
    Serial.printf("%d ", rightDistanceBuffer[i]);

  Serial.print("\nLeft last valid distance: ");
  Serial.println(distanceToListel(leftLastValidDistance));

  Serial.print("Right last valid distance: ");
  Serial.println(distanceToListel(rightLastValidDistance));

  Serial.print("Calculated position: ");
  Serial.println(lastCalculatedCenter);
  */

  // debugIndicator(distanceToListel(leftLastValidDistance), strip.Color(255, 255, 0));
  // debugIndicator(distanceToListel(rightLastValidDistance), strip.Color(255, 0, 255));
  
  // LED strip logic
  updateLEDStrip(distanceToListel(lastCalculatedCenter)); // if position was updated send position
  //Serial.printf("Update: %d\tDistance: %d\n", positionUpdate, lastCalculatedCenter);
  stripShow();

  delay(10);
}