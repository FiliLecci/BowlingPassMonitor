#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"

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

// Mutex for atomic operations
extern portMUX_TYPE myMux;

// FUNCTIONS PROTOTYPES FOR FOLLOWING MENU ITEMS ACTIONS
static void selectSubMenuAction();

static void toggleValueEditingAction();


static String getItemString(MenuItem* item); // Keep this here because Arduino is stupid and doesn't see the struct
static bool isBefore(MenuItem* a, MenuItem* b);  // This also


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
const uint8_t maxLines = 4; // Maximum writable lines on the screen 
MenuItem* volatile topDisplayedItem = NULL; // First item to display (the "MenuItem* volatile" declaration means that the volatile part is the pointer and not the content itself)

bool menuChanged = false; // A flag indicating if the menù has changed and thus requires an update
static volatile int8_t valueUpdate = 0; // By how much the selected value should be updated at the next menù update 

static volatile uint8_t singleTarget = 0;          // Single target selected listel
static volatile uint8_t range[2] = {0,0};          // Range left and right selected listels

static volatile bool isValueEditingEnabled = false;   // Allow to modify selected value (if possible)
static MenuItem* volatile selectedItem = NULL;        // Current menu position

// ITEMS VALUES

static ItemValue singleTargetValue  = {&singleTarget, UINT8, 1, NUM_LISTELS, 1};
static ItemValue rangeLeftValue     = {&range[0], UINT8, 1, NUM_LISTELS, 1};
static ItemValue rangeRightValue     = {&range[1], UINT8, 1, NUM_LISTELS, 1};

// MENU ITEMS

static MenuItem root              = {"ROOT", NULL, NULL, NULL, NULL, NULL, selectSubMenuAction};

static MenuItem freeMode          = {"Modalità libera", &root, NULL, NULL, NULL, NULL, NULL};
static MenuItem singleTargetMode  = {"Target", &root, NULL, NULL, NULL, NULL, selectSubMenuAction};
static MenuItem rangeMode         = {"Range", &root, NULL, NULL, NULL, NULL, selectSubMenuAction};

static MenuItem target            = {"Target", &singleTargetMode, NULL, NULL, NULL, &singleTargetValue, toggleValueEditingAction};

static MenuItem leftTarget        = {"Target SX", &rangeMode, NULL, NULL, NULL, &rangeLeftValue, toggleValueEditingAction};
static MenuItem rightTarget       = {"Target DX", &rangeMode, NULL, NULL, NULL, &rangeRightValue, toggleValueEditingAction};

void setupScreen(){
  Serial.println("Starting display...");

  Wire1.begin(SDA_2, SCL_2);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;) delay(100);
  }

  Serial.println("Display setup success.");

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

  // Init starting position at first item below Root
  selectedItem = root.firstChild == NULL ? &root : root.firstChild;
  topDisplayedItem = selectedItem;

  menuChanged = true;
}

void IRAM_ATTR selectBtnPress(){
  if(selectedItem->action == NULL)
    return;

  // Perform current menu item action
  selectedItem->action();
}

static void updateSelectedItemValue(int8_t direction){
  ItemValue* value = selectedItem->valueConfig;

  float stepAmount = value->step * direction; // clockwise adds 1 step, counter-clockwise removes 1 step

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

  menuChanged = true;
}

static MenuItem* updateScrolling(MenuItem* selectedSnapshot) {
  MenuItem* topDisplayedSnapshot = (MenuItem*)topDisplayedItem;
  MenuItem* newTop = topDisplayedSnapshot;

  // Verify if selected item is before top displayed item
  if (isBefore(selectedSnapshot, topDisplayedSnapshot)) {
      newTop = selectedSnapshot;
  }

  // Verify if selected item is after last displayed item
  MenuItem* lastDisplayed = topDisplayedSnapshot;
  for (int i = 0; i < maxLines - 1; i++) {
      if (lastDisplayed->nextSibling != NULL){
        lastDisplayed = lastDisplayed->nextSibling;
      }
  }
  
  if (isBefore(lastDisplayed, selectedSnapshot)) {
      newTop = topDisplayedSnapshot->nextSibling;
  }

  return newTop;
}

// Return true if item a is before item b
static bool isBefore(MenuItem* a, MenuItem* b) {
  if(a == b)
    return false;

  MenuItem* curr = b;
  while (curr != NULL) {
      if (curr == a)
        return true;
      curr = curr->prevSibling;
  }
  return false;
}

void IRAM_ATTR encoderAction(int8_t direction){
  if(isValueEditingEnabled){ // modify item value
    valueUpdate += direction;
    menuChanged = true;
    return;
  }

  // create snapshots of volatile pointers to make sure they don't change from a row to another
  MenuItem* selectedSnapshot = (MenuItem*)selectedItem;

  if(direction == 1){  // clockwise
    if(selectedSnapshot->nextSibling == NULL){ // Check if next sibling is present
      return;
    }
    
    // move menu item pointer
    selectedItem = selectedSnapshot->nextSibling;
  }
  else if(direction == -1){ // counter-clockwise
    if(selectedSnapshot->prevSibling == NULL){ // Check if previous sibling is present
      return;
    }

    // move menu item pointer
    selectedItem = selectedSnapshot->prevSibling;
  }

  menuChanged = true;
}

static void selectSubMenuAction(){
  selectedItem = selectedItem->firstChild;
  topDisplayedItem = selectedItem;
  menuChanged = true;
}

void IRAM_ATTR prevMenuAction(){
  selectedItem = selectedItem->parent;
  topDisplayedItem = selectedItem;
  isValueEditingEnabled = false;  // this action may be performed by something other than a menu item (back button) so it's better to also reset this
  menuChanged = true;
}

static void toggleValueEditingAction(){
  isValueEditingEnabled = !isValueEditingEnabled;
  menuChanged = true;
}

static String getItemString(MenuItem* item) {
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

static void displayAndClear(){
  display.display();
  display.clearDisplay();
  display.setCursor(1,1);
}

// Print the menu on the screen buffer
static void updateMenuUnsafe(MenuItem* selectedItemSnapshot, uint8_t valueUpdateSnapshot){
  updateScrolling(selectedItemSnapshot);  // update topDisplayedItem if necessary

  if(valueUpdateSnapshot != 0)
    updateSelectedItemValue(valueUpdateSnapshot);

  MenuItem *temp = topDisplayedItem;
  
  for(int i=0;i<maxLines;i++){
    if(temp == NULL)
      break;  // make sure is a break or the menuChanged variable will remain true if there are less than maxLines items
    
    // Set item style if is the selected one
    if(temp == selectedItemSnapshot){
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);

      if(isValueEditingEnabled)
        display.print("> ");
      else
        display.print("  ");
    }

    display.println(getItemString(temp));

    // Reset text style
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

    temp = temp->nextSibling;
  }
}

// This fuction updates the menu by also taking care of critical parts.
void updateMenuSafe() {
    MenuItem* snapshotSelected = NULL;
    int8_t snapshotUpdateValue = 0;
    bool shouldProceed = false; 

    // --- BRIEF ATOMIC PROTECTION ---
    // Only use critical section to copy things and reset values
    portENTER_CRITICAL(&myMux); 
    if (menuChanged) {
        shouldProceed = true;
        menuChanged = false;
        snapshotSelected = (MenuItem*)selectedItem;
        snapshotUpdateValue = valueUpdate;
        valueUpdate = 0;
    }
    portEXIT_CRITICAL(&myMux);
    // ---
    
    if(shouldProceed){
      // Now it's safe to call logic things by passing copyed values
      // Now the watchdog shoud be happy
      topDisplayedItem = updateScrolling(snapshotSelected); // This can be done here because pinter assign in an ESP32 is atomic

      updateMenuUnsafe(snapshotSelected, snapshotUpdateValue);
    }
}

inline bool isMenuChanged(){
  return menuChanged;
}

inline uint8_t getSingleTarget() {
    return (uint8_t)singleTarget; 
}

inline void getSelectedRange(uint8_t destination[2]) {
    portENTER_CRITICAL(&myMux);
    destination[0] = range[0];
    destination[1] = range[1];
    portEXIT_CRITICAL(&myMux);
}

// Custom displayed things

// Gived the left and right distances and calculated center, draws two bars and a center point
void drawBars(uint16_t leftD, uint16_t rightD, uint16_t centerP){
  // Mappa la distanza (es. 0-1000mm) sulla larghezza dello schermo (0-128px)
  uint8_t barL = map(leftD, 0, LANE_WIDTH, 0, SCREEN_WIDTH);
  uint8_t barR = map(rightD, 0, LANE_WIDTH, 0, SCREEN_WIDTH);
  uint8_t center = map(centerP, 0, LANE_WIDTH, 0, SCREEN_WIDTH);

  display.fillRect(0, 10, barL, 6, SSD1306_WHITE);
  display.fillRect(SCREEN_WIDTH-barR, 10, barR, 6, SSD1306_WHITE);
  display.drawCircle(center, 13, 2, SSD1306_WHITE);
}