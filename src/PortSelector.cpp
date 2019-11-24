#include "PortSelector.h"
#include "stdarg.h"

/*
 * Constructor
 * 
 * @param pinCount number of pins used to address ports
 * @param ... each pin used LSB first
 */
PortSelector::PortSelector(uint8_t pinCount, bool activeHigh, ...) {
    _pinCount = pinCount;
    _activeHigh = activeHigh;
    va_list args;
    va_start(args, pinCount);
    for (int i = 0; i < pinCount; i++) {
        _pins[i] = va_arg(args, uint16_t);
    }
    va_end(args);
}

/*
 * Initialize address to zero
 */
void PortSelector::begin() {
    _address = 0;
    for (int i = 0; i < _pinCount; i++) {
        pinMode(_pins[i], OUTPUT);
        digitalWrite(_pins[i], !_activeHigh); // set to inactive
    }
}

/*
 * Switch port
 * 
 * @param address
 */
void PortSelector::selectPort(uint8_t address) {
    if (address >= (1 << _pinCount)) {
        return; // invalid address, discard
    })
    for (int i = 0; i < _pinCount; i++) {
        if (address & (1 << i)) {
            digitalWrite(_pins[i], _activeHigh);
        } else {
            digitalWrite(_pins[i], !_activeHigh);
        }
    }
}