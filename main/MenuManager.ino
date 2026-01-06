#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

enum DataType {NONE, INT, UINT8, UINT16, FLOAT, BOOL};

// Refer to README on how this structure can be visualized
struct MenuItem {
    const char* label;          // Menu voice name
    MenuItem* parent;           // Pointer to parent (above level menu)
    MenuItem* firstChild;       // First item of next menu
    MenuItem* nextSibling;      // Next menu item
    MenuItem* prevSibling;      // Previous menu item
    
    DataType valueType;         // Type of item value (NONE if there is no value)
    volatile void* value;       // Pointer to value
    void (*action)();           // Action performed by this item
};

// FUNCTIONS PROTOTYPES FOR FOLLOWING MENU ITEMS ACTIONS
void selectSubMenuAction();

void toggleValueEditingAction();



Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Not static because they are used in the LED management
volatile uint8_t singleTarget = 0;          // Single target selected listel
volatile uint8_t range[2] = {0,0};          // Range left and right selected listels

static volatile bool isValueEditingEnabled = false;   // Allow to modify selected value (if possible)
static volatile MenuItem* selectedItem = NULL;        // Current menu position
 
// MENU ITEMS

static MenuItem root              = {"ROOT", NULL, NULL, NULL, NULL, NONE, NULL, selectSubMenuAction};

static MenuItem freeMode          = {"Modalità libera", &root, NULL, NULL, NULL, NONE, NULL, NULL};
static MenuItem singleTargetMode  = {"Target", &root, NULL, NULL, NULL, NONE, NULL, selectSubMenuAction};
static MenuItem rangeMode         = {"Range", &root, NULL, NULL, NULL, NONE, NULL, selectSubMenuAction};

static MenuItem target            = {"Target", &singleTargetMode, NULL, NULL, NULL, UINT8, &singleTarget, toggleValueEditingAction};

static MenuItem leftTarget        = {"Target SX", &rangeMode, NULL, NULL, NULL, UINT8, &range[0], toggleValueEditingAction};
static MenuItem rightTarget       = {"Target DX", &rangeMode, NULL, NULL, NULL, UINT8, &range[1], toggleValueEditingAction};

void setupScreen(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;) delay(100);
  }

  display.display();
  delay(2000);

  // Clear the buffer
  display.clearDisplay();
}

void initMenu(){
  // Main menu
  root.firstChild = &freeMode;
  
  freeMode.nextSibling = &singleTargetMode;

  singleTargetMode.nextSibling = &rangeMode;
  singleTargetMode.prevSibling = &freeMode;
  singleTargetMode.firstChild = &target;

  rangeMode.firstChild = &leftTarget;
  rangeMode.prevSibling = &singleTargetMode;

  // No setup needed for single target menu since there are no sub-menus nor siblings

  // Range menu
  leftTarget.nextSibling = &rightTarget;
  rightTarget.prevSibling = &leftTarget;

  // Init starting position at root
  selectedItem = &root;
}

void selectBtnPress(){
  // Perform current menu item action
  selectedItem->action();
}

void increaseAction(){
  if(isValueEditingEnabled){ // modify item value
    return;
  }

  if(selectedItem->nextSibling == NULL){ // Check if next sibling is present
    return;
  }

  // move menu item pointer
  selectedItem = selectedItem->nextSibling;
}

void decreaseAction(){
  if(isValueEditingEnabled){ // modify item value
    return;
  }

  if(selectedItem->prevSibling == NULL){ // Check if previous sibling is present
    return;
  }

  // move menu item pointer
  selectedItem = selectedItem->prevSibling;
}

void selectSubMenuAction(){
  selectedItem = selectedItem->firstChild;
}

void prevMenuAction(){
  selectedItem = selectedItem->parent;
  isValueEditingEnabled = false;  // this action may be performed by something other than a menu item (back button) so it's better to also reset this
}

void toggleValueEditingAction(){
  isValueEditingEnabled = !isValueEditingEnabled;
}

void printItem(){}