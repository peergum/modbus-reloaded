#include "Sensor.h"

Sensor::Sensor(Modbus &port) {

}

Sensor::Sensor(USARTSerial &serial, uint16_t txRxPin, bool txWhenHigh = true, PortSelector &portSelector, uint8_t address = 0) {

}

void begin() {

}
/*
 * Conversion from 32 bits (MODBUS encoded) to various formats
 */
float Sensor::_convertHEXtoFLOAT(uint16_t* data){
    FloatData floatData;

    for(int i=0; i<2; i++) {
        floatData.u[1-i] = data[i];
    }

    return floatData.f;
}

void Sensor::_convertFLOATtoHEX(uint16_t *buffer, float value) {
    FloatData floatData;
    floatData.f = value;
    buffer[0] = floatData.u[1];
    buffer[1] = floatData.u[0];
}

int Sensor::_convertHEXtoINT(uint16_t* data){
    IntData intData;

    intData.u[0] = data[1];
    intData.u[1] = data[0];

    return intData.i;
}

char *Sensor::_convertHEXtoCHAR(char *text, uint16_t* data, uint8_t length){
    text[0] = 0;
    wchar_t w;
    for (uint8_t i=0; i<length; i++) {
        w = (wchar_t) *(data+i);
        if (!w) {
            text[i] = 0;
            break;
        } else if (w>=32 && w<128) {
            text[i] = (char)(w % 256);
        } else {
            text[i] = '_';
        }
    }
    text[length] = 0;
    return text;
}

// TODO
void Sensor::_convertCHARtoHEX(uint16_t* data, char *text, uint8_t length){
    for (int i=0; i<length; i++) {
        *(data+i) = text[i] ?
            (uint16_t) text[i] : 0;
    }
}

/*
 * This converts troll time to time_t
 */
time_t Sensor::_convertHEXtoTIME(uint16_t* data){
    TimeData timeData;
    timeData.u[0] = data[0];
    timeData.u[1] = data[1];
    return timeData.t;
}

void Sensor::_convertTIMEtoHEX(uint16_t* data, time_t now) {
    TimeData timeData;
    timeData.t = now;
    data[0] = timeData.u[0];
    data[1] = timeData.u[1];
}
