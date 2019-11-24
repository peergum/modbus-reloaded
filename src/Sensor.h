#pragma once

/*
 * Modbus.cpp
 * 
 * Copyright 2019 PeerGum
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// This will load the definition for common Particle variable types
#include "Particle.h"

#ifndef __MODBUS_SENSOR_H
#define __MODBUS_SENSOR_H

#include "modbus-reloaded.h"
#include "PortSelector.h"

/*
 * Unions for hex to various types conversion
 * (used by helper functions)
 */
union IntData
{
  uint16_t u[2];
  int i;
};

union FloatData
{
  uint16_t u[2];
  float f;
};

union ULongData
{
  uint16_t u[2];
  unsigned long ul;
};

union TimeData
{
  uint16_t u[2];
  time_t t;
};

union StringData
{
  uint16_t u[32];
  char c[64];
  wchar_t w[32];
};

enum SensorType
{
    SENSOR_UNKNOWN,
    SENSOR_TROLL,
    SENSOR_NUFLO
};

class Sensor : public Modbus
{
public:
    Sensor(Modbus &port);
    Sensor(USARTSerial &serial, uint16_t txRxPin, bool txWhenHigh = true, PortSelector &portSelector, uint8_t address = 0);
    void begin();
    void switchSpeed(uint32_t speed);
    void setSlaveID(uint8_t slaveID);
    SensorType type(void);

private:
    // define this to proper value in your derived class
    SensorType _sensorType = SENSOR_UNKNOWN;

    float _convertHEXtoFLOAT(uint16_t *data);
    void _convertFLOATtoHEX(uint16_t *buffer, float value);
    int _convertHEXtoINT(uint16_t* data);
    char *_convertHEXtoCHAR(char *text, uint16_t* data, uint8_t length);
    void _convertCHARtoHEX(uint16_t* data, char *text, uint8_t length);
    time_t _convertHEXtoTIME(uint16_t* data);
    void _convertTIMEtoHEX(uint16_t* data, time_t now);
}

#endif