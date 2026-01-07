#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

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

// FUNCTIONS PROTOTYPES FOR FOLLOWING MENU ITEMS ACTIONS
void selectSubMenuAction();

void toggleValueEditingAction();


String getItemString(MenuItem* item); // keep this here because Arduino is stupid and doesn't see the struct



Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Not static because they are used in the LED management
volatile uint8_t singleTarget = 0;          // Single target selected listel
volatile uint8_t range[2] = {0,0};          // Range left and right selected listels

static volatile bool isValueEditingEnabled = false;   // Allow to modify selected value (if possible)
static volatile MenuItem* selectedItem = NULL;        // Current menu position

// ITEMS VALUES

static ItemValue singleTargetValue  = {&singleTarget, UINT8, 1, 39, 1};
static ItemValue rangeLeftValue     = {&range[0], UINT8, 1, 39, 1};
static ItemValue rangeRightValue     = {&range[1], UINT8, 1, 39, 1};

// MENU ITEMS

static MenuItem root              = {"ROOT", NULL, NULL, NULL, NULL, NULL, selectSubMenuAction};

static MenuItem freeMode          = {"Modalità libera", &root, NULL, NULL, NULL, NULL, NULL};
static MenuItem singleTargetMode  = {"Target", &root, NULL, NULL, NULL, NULL, selectSubMenuAction};
static MenuItem rangeMode         = {"Range", &root, NULL, NULL, NULL, NULL, selectSubMenuAction};

static MenuItem target            = {"Target", &singleTargetMode, NULL, NULL, NULL, &singleTargetValue, toggleValueEditingAction};

static MenuItem leftTarget        = {"Target SX", &rangeMode, NULL, NULL, NULL, &rangeLeftValue, toggleValueEditingAction};
static MenuItem rightTarget       = {"Target DX", &rangeMode, NULL, NULL, NULL, &rangeRightValue, toggleValueEditingAction};

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
  if(selectedItem->action == NULL)
    return;

  // Perform current menu item action
  selectedItem->action();
}

void updateSelectedItemValue(int8_t direction){
  ItemValue* value = selectedItem->valueConfig;

  float stepAmount = value->step * direction;

  switch (value->type) {
    case UINT8:{
      uint8_t* p = (uint8_t*)value->valuePtr;
      int next = (int)*p + (int)stepAmount;

      if (next > value->max)
        *p = (uint8_t)value->min;
      else if (next < value->min)
        *p = (uint8_t)value->max;
      else 
        *p = (uint8_t)next;
      break;
    }
    case UINT16:{
      uint16_t* p = (uint16_t*)value->valuePtr;
      int next = (int)*p + (int)stepAmount;

      if (next > value->max)
        *p = (uint16_t)value->min;
      else if (next < value->min)
        *p = (uint16_t)value->max;
      else 
        *p = (uint16_t)next;
      break;
    }
    case INT:{
      int* p = (int*)value->valuePtr;
      int next = (int)*p + (int)stepAmount;

      if (next > value->max)
        *p = (int)value->min;
      else if (next < value->min)
        *p = (int)value->max;
      else 
        *p = (int)next;
      break;
    }
    case FLOAT:{
      float* p = (float*)value->valuePtr;
      *p += stepAmount;

      if (*p > value->max)
        *p = value->min;
      else if (*p < value->min)
        *p = value->max;

      break;
    }
    case BOOL:
      *(bool*)value->valuePtr = !*(bool*)value->valuePtr;
      break;

    default:
      break;
  }
}

void encoderAction(int8_t direction){
  if(isValueEditingEnabled){ // modify item value
    updateSelectedItemValue(direction);
    return;
  }

  if(direction == 1){  // clockwise
    if(selectedItem->nextSibling == NULL){ // Check if next sibling is present
      return;
    }
    
    // move menu item pointer
    selectedItem = selectedItem->nextSibling;
  }
  else if(direction == -1){
    if(selectedItem->prevSibling == NULL){ // Check if previous sibling is present
      return;
    }

    // move menu item pointer
    selectedItem = selectedItem->prevSibling;
  }
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

String getItemString(MenuItem* item) {
    if (item == NULL) return "";

    // Inizia con l'etichetta
    String output = String(item->label);

    // No value associated to the item: return label
    if (item->valueConfig == NULL) {
        return output;
    }

    output += ": ";

    ItemValue* value = item->valueConfig;

    switch (value->type) {
        case UINT8:
            output += String(*(uint8_t*)value->valuePtr);
            break;
        case UINT16:
            output += String(*(uint16_t*)value->valuePtr);
            break;
        case INT:
            output += String(*(int*)value->valuePtr);
            break;
        case FLOAT:
            // Formatta il float con 2 cifre decimali
            output += String(*(float*)value->valuePtr, 2);
            break;
        case BOOL:
            output += (*(bool*)value->valuePtr) ? "TRUE" : "FALSE";
            break;
        default:
            output += "TF is this?";
            break;
    }

    return output;
}