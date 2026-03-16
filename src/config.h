// BALL PROPERTIES
#define BALL_DIAMETER 217 // mm (average of allowed maximum and minimum ball diameter)

// LANE PROPERTIES
#define NUM_LISTELS 39      // number of lane listels
#define LANE_WIDTH 1070     // mm
#define GUTTER_WIDTH 250    // mm

// SENSORS PARAMS
#define XSHUT_PIN_L 42
#define XSHUT_PIN_R 2

// LED STRIP DIN PIN
#define STRIP_PIN 11        // the pin to which the strip is connected
#define NUM_LED 39          // total number of LEDs
#define LED_SPACING 25.4    // mm (default is 1in = 25.4mm)

// INTEGRATED LED PIN
#define INT_LED_PIN 5       // the pin to which the integrated LED is connected

// BUTTONS AND ENCODER PARAMS
#define MENU_BTN_PIN 1 
#define SELECT_BTN_PIN 12

#define ENC_A 14
#define ENC_B 13

// THIS WORKS ON A ESP32 BECAUSE IT HAS 2 I2C INTERFACES
// First I2C interface
#define SDA_1 39
#define SCL_1 40
// Second I2C interface
#define SDA_2 48
#define SCL_2 47