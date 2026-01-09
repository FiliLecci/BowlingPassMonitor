// BALL PROPERTIES
#define BALL_DIAMETER 217 //mm (average of allowed maximum and minimum ball diameter)

// LANE PROPERTIES
#define LANE_WIDTH 1070   //mm
#define GUTTER_WIDTH 250  //mm

// SENSORS PARAMS
#define XSHUT_PIN_L 19
#define XSHUT_PIN_R 18

// LED STRIP DIN PIN
#define STRIP_PIN 23

// BUTTONS AND ENCODER PARAMS
#define MENU_BTN_PIN 15 
#define SELECT_BTN_PIN 4

#define ENC_A 25
#define ENC_B 26

// THIS WORKS ON A ESP32 BECAUSE IT HAS 2 I2C INTERFACES
// First I2C interface
#define SDA_1 21
#define SCL_1 22
// Second I2C interface
#define SDA_2 16
#define SCL_2 17