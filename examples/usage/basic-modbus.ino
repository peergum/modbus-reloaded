// Example usage for modbus-reloaded library by peergum.

#include "src/modbus-reloaded.h"

// include if using Serial2/4/5
// #include "Serial2/Serial2.h"
// #include "Serial4/Serial4.h"
// #include "Serial5/Serial5.h"


#define TX_ENABLE_PIN D0
#define RX_ENABLE_PIN D1
#define TX_ACTIVE_HIGH true
#define RX_ACTIVE_HIGH true
#define SLEEP_ABLE true

#define REGISTER_TO_READ 1234

#define SERIAL Serial2
#define SPEED 9600
#define PARITY SERIAL_8N1

// Initialize objects from the lib
Modbus modbus(SERIAL, TX_ENABLE_PIN, TX_ACTIVE_HIGH, RX_ENABLE_PIN, RX_ACTIVE_HIGH, SLEEP_ABLE);

void setup() {
    modbus.begin(SPEED,PARITY);
    delay(1000);

    // send request to read 1 register
    modbus.readHoldingRegisters(REGISTER_TO_READ, 1);
}

void loop() {
    ModbusStatus status = modbus.checkResponse();
    if (status == DATA_READY) {
    } else if ( status == MODBUS_ERROR) {
        // didn't work: do something about it
        // ...
    } else
}
