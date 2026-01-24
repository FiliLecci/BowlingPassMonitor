#include "./config.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#pragma once

enum DataType {INT, UINT8, UINT16, FLOAT, BOOL};

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


void setupScreen();

void initMenu();

void selectBtnPress();

void encoderAction(int8_t direction);

void prevMenuAction();

void updateMenuSafe();

void displayAndClear();

void drawBars(uint16_t leftD, uint16_t rightD, uint16_t centerP);

bool isMenuChanged();

uint8_t getSingleTarget();

void getRange(uint8_t destination[2]);

uint8_t getMode();