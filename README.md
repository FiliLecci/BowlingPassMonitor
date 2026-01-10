# BowlingPassMonitor
An arduino project that uses 2 VL53L1X sensors and an array of 39 addressable LEDs to detect and show a bowling ball passing point. The program features a menù with 3 modes (free, single target and range) selectable via a CL11 encoder and a stand-alone button to get back at the main menù.

## Menu
The menu is structured kinda like an n-ary tree:

![Menu structure](/imgs/menu_diagram.svg)

When the menu is first loaded the `selectedItem` variable points to the `Root` item.

Moving the selection across the items changes the `selectedItem` pointer that constantly keeps track of the user current position.

In this example we would see a main menu containing the 3 items `item1_1`, `item1_2` and `item1_3`, after selecting the `item1_2` item, `selectedItem` pointer goes from `Item1_2` to `Item1_2.firstChild` aka `Item2_1`.

Going back to the previous menu level is done by selecting the first item in the current level (`prevSibling=NULL`) and moving `selectedItem` to `.parent`.

# Conventions
- Listel numbering starts from the left;
- Bowling ball size is estimated at 217mm, this dimension is given by averaging the maximum and minimum allowed ball size;
