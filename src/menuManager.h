#include "./config.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#pragma once

enum DataType : uint8_t {INT, UINT8, UINT16, FLOAT, BOOL};

enum SetupState : uint8_t {
  SETUP_PENDING,
  SETUP_OK,
  SETUP_ERROR
};

// Refer to README on how this structure can be visualized
struct ItemValue {
    volatile void* valuePtr;
    DataType type;
    float min;
    float max;
    float step;
};

struct MenuItem {
    const char* label;          // Menu voice name
    MenuItem* parent;           // Pointer to parent (above level menu)
    MenuItem* firstChild;       // First item of next menu
    MenuItem* nextSibling;      // Next menu item
    MenuItem* prevSibling;      // Previous menu item
    
    ItemValue* valueConfig;      // Item value configuration

    void (*action)();           // Action performed by this item
};

// Mutex for atomic operations
extern portMUX_TYPE myMux;

// A flag indicating if the menu has changed and thus requires an update
extern bool menuChanged;

// initialize display and I2C wire
void setupScreen();

// setup menu structure
void initMenu();

// perform current menu item action
void selectBtnPress();

// handle encoder rotation
void encoderAction(int8_t direction);

// go back to parent menu
void prevMenuAction();

// update the menu display safely using critical sections
void updateMenuSafe();

// display content of screen buffer, clear buffer and reset cursor
void displayAndClear();

// draw a visual representation of the distances as bars and ball center
void drawBars(uint16_t leftD, uint16_t rightD, uint16_t centerP);

// check if menu has changed
bool isMenuChanged();

volatile uint8_t* getSingleTargetPtr();

volatile uint8_t* getRangePtr();

uint8_t getMode();

void setLeftSensorStatus(uint8_t state);
void setRightSensorStatus(uint8_t state);
void setLedStripStatus(uint8_t state);
void drawSetupStatus();