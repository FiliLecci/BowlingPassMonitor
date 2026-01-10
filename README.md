# BowlingPassMonitor
An arduino project that uses 2 VL53L1X sensors and an array of 39 addressable LEDs to detect and show a bowling ball passing point. The program features a menù with 3 modes (free, single target and range) selectable via a CL11 encoder and a stand-alone button to get back at the main menù.

## Menu

The project features a complete and flexible menu with the possibility to navigate through sub-menus, change values and perform any other action.

As for the current project state there isn't yet a "settings" page but any configuration can be changed via the `config.h` file.

### Menu structure
The menu is structured kinda like an n-ary tree:

![Menu structure](/imgs/menu_diagram.svg)

When the menu is first loaded the `selectedItem` variable points to `Root.firstChild`, that beign the first item of the first menu.

Moving the selection across the items moves the `selectedItem` pointer either to `nextSibling` or `prevSibling` to constantly keep track of the user current position.

Selecting an item that has a child moves `selectedItem` to `selectedItem.firstChild`.

In this example we would see a main menu containing the 3 items `item1`, `item2` and `item3`, after selecting the `item2` item, `selectedItem` pointer goes from `Item2` to `Item2.firstChild` aka `Item2.1`.

Going back to the previous menu level is done by moving `selectedItem` to `selectedItem.parent`.

### Item structure

Each item of the menu consists of the following elements:
- **Label**: The name of the item, this value is what's shown when the item is displayed;
- **firstChild**: If the item has a sub-menu this pointer refers to the first item of that menu;
- **nextSibling**: The next item that should be shown in the menu. This pointer is used to move along the menu and to show the items alongside *prevSibling* and thus should be kept coherent to it.
    > A "wrapped" menu can be achieved by making "nextSibling" of the last item point to the first item and vice-versa.\
    > Note that as of now there is no way for the menu to know if an item has already been printed, therefor if the number of items is less than maxLines, there will be repeated items.
- **prevSibling**: The previous item in the menu. Everything said for *nextSibling* is also true for this;
- **valueConfig**: If the item has a value associated to it, this pointer refers to an *ItemConfig* struct containing all necessary informations to work with the value;
- **action**: The action to perform when an item is selected and the select button is pressed.

> Unless an ad-hoc action is implemented, the *firstChild* and *valueConfig* elements cannot be managed both in the same element.\
> As of now an item with a *valueConfig* item associated is considered a "leaf" (item with no childs), and the only action it can perform is to enable the value editing mode.\
> The same goes for an item with a child; if it has a child there are no reasons for it to have a value and therefor the only action it can perform is to select it's child.

> An item with no action simply won't do anything.

## Conventions
- Listel numbering starts from the left;
- Bowling ball size is estimated at 217mm, this dimension is given by averaging the maximum and minimum allowed ball size;
