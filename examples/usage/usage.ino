// Example usage for modbus-reloaded library by peergum.

#include "modbus-reloaded.h"

// Initialize objects from the lib
Modbusreloaded modbusreloaded;

void setup() {
    // Call functions on initialized library objects that require hardware
    modbusreloaded.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    modbusreloaded.process();
}
