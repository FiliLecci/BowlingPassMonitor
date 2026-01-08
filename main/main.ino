#include <SPI.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_NeoPixel.h>

#define BALL_DIAMETER 217 //mm (average of allowed maximum and minimum ball diameter)

// TODO this parameters should be red dynamically in some way (probably configurable from the menu)
#define LANE_WIDTH 1070   //mm
#define GUTTER_WIDTH 250  //mm

// THIS WORKS ON A ESP32 BECAUSE IT HAS 2 I2C INTERFACES
// First I2C interface
#define SDA_1 21
#define SCL_1 22
// Second I2C interface
#define SDA_2 16
#define SCL_2 17

// SENSORS PARAMS
// #define IRQ_PIN_L ... not used
#define XSHUT_PIN_L 19

// #define IRQ_PIN_R ... not used
#define XSHUT_PIN_R 18

// BUTTONS AND ENCODER PARAMS
#define MENU_BTN_PIN -1 // TODO
#define SELECT_BTN_PIN -1 // TODO

#define ENC_A -1  // TODO
#define ENC_B -1  //TODO

Adafruit_VL53L1X sensor_left = Adafruit_VL53L1X(XSHUT_PIN_L);
Adafruit_VL53L1X sensor_right = Adafruit_VL53L1X(XSHUT_PIN_R);

Adafruit_NeoPixel strip(2, 23, NEO_GRB + NEO_KHZ800);

const uint16_t numListels = 39;
const uint16_t laneWidth = 1070;  //mm

// Variables for encoder reading function
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 10;

void scanI2C() {
  byte error, address;
  Serial.println("Scansione bus I2C...");
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Dispositivo trovato a 0x");
      Serial.println(address, HEX);
    }
  }
}

// TODO keep trying to connect sensor if it fails
void setupSensors(){
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
  delay(10);

  Wire.begin(SDA_1, SCL_1);
  // setup left sensor
  Serial.println("Left sensor init...");

  if (! sensor_left.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of left VL sensor: "));
    Serial.println(sensor_left.vl_status);
    while (1)       
      delay(100);
  }
  sensor_left.VL53L1X_SetI2CAddress(0x30);

  Serial.print(F("Sensor left ID: 0x"));
  Serial.println(sensor_left.sensorID(), HEX);

  scanI2C();

  // setup right sensor
  // Wake up right sensor
  digitalWrite(XSHUT_PIN_R, HIGH);
  delay(10);

  Serial.println("Right sensor init...");

  if (! sensor_right.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of right VL sensor: "));
    Serial.println(sensor_right.vl_status);
    while (1)
      delay(100);
  }
  sensor_right.VL53L1X_SetI2CAddress(0x31);

  // SENSORS SETUP COMPLETE
  Serial.print(F("Sensor right ID: 0x"));
  Serial.println(sensor_right.sensorID(), HEX);

  scanI2C();

  Wire.setClock(400000);

  Serial.println(F("VL53L1X sensors OK!"));
}

void setupLedStrip(){
  Serial.println("Starting LED strip init...");
  //init led
  strip.begin();

  strip.setBrightness(100);
  strip.fill(strip.Color(0, 255, 0), 0, 39);
  
  Serial.println("LED strip init success.");

  strip.show();
}

void setupInputs(){
  // TODO check if this buttons can be set to INPUT_PULLDOWN to simplify the circuit.
  pinMode(MENU_BTN_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(MENU_BTN_PIN), prevMenuAction, RISING);

  pinMode(SELECT_BTN_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(SELECT_BTN_PIN), selectBtnPress, RISING);

  // TODO Check if the GPIOs are actually pulled up
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
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
    encoderAction(1);              // Perform action
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    encoderAction(-1);              // Perform action
    encval = 0;
  }
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

  if (sensor_right.dataReady()){
    Serial.println("Right data not ready.");
    return false;
  }

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

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  setupSensors();

  startLeftSensor();
  startRightSensor();

  setupLedStrip();

  setupScreen();

  initMenu();
  scanI2C();
}

void loop() {
  if(isMenuChanged()){
    updateDisplay();
    displayAndClear();
  }

  Serial.printf("%d - %d | %d - %d\n", getLeftSensorReading(), leftLastValidDistance, getRightSensorReading(), rightLastValidDistance);

  delay(1000);
  /*
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
  */
}