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

#ifndef __MODBUS_RELOADED_H
#define __MODBUS_RELOADED_H

/*
 * Some constants that may need adjusting
 */

#define MB_MAX_DATA_LENGTH 80
#define MAX_WAIT_TO_TRANSMIT_MS 10000 // we should be able to write within 10ms
#define MAX_WAIT_FOR_RESPONSE_MS 20000 // we should get something within 20ms

/*
 * Helpers
 */

#define HIGH(value) (value >> 8)
#define LOW(value) (value & 0xff)
#define WORD(high, low) (uint16_t)(high * 256 + low)

enum
{
  MODBUS_UNKNOWN,
  MODBUS_BUSY,
  MODBUS_READY,
  MODBUS_SENDING,
  MODBUS_DATA_SENT
  MODBUS_RECEIVING,
  MODBUS_DATA_READY,
  MODBUS_ERROR
} ModbusStatus;

enum {
  MODBUS_SUCCESS,
  MODBUS_ILLEGAL_FUNCTION,
  MODBUS_ILLEGAL_DATA_ADDRESS,
  MODBUS_ILLEGAL_DATA_VALUE,
  MODBUS_SLAVE_DEVICE_FAILURE,
  MODBUS_INVALID_SLAVE_ID,
  MODBUS_INVALID_FUNCTION,
  MODBUS_RESPONSE_TIMEOUT,
  MODBUS_INVALID_CRC,
  MODBUS_WRONG_ADDRESS,
  MODBUS_WRONG_VALUE,
  MODBUS_WRONG_QUANTITY,
  MODBUS_UNHANDLED_FUNCTION
  MODBUS_WRONG_BYTE_COUNT,
} ModbusError;

// This is your main class that users will import into their application
class Modbus
{
public:
  Modbus(USARTSerial &serial, uint16_t txEnablePin, uint16_t rxEnablePin, bool txActiveHigh, bool rxActiveHigh, bool sleepAble);
  Modbus(USARTSerial &serial, uint16_t txRxPin, bool txWhenHigh = true);

  void begin(uint16_t speed, );
  void loop();

  uint16_t getResponseBuffer(uint8_t);
  void clearResponseBuffer();
  uint8_t setTransmitBuffer(uint8_t, uint16_t);
  void clearTransmitBuffer();

  void enableDebug(void);
  void disableDebug(void);

  void beginTransmission(uint16_t);
  uint8_t requestFrom(uint16_t, uint16_t);
  void sendBit(bool);
  void send(uint8_t);
  void send(uint16_t);
  void send(uint32_t);
  uint8_t available(void);
  uint16_t receive(void);

  uint8_t readCoils(uint16_t address, uint16_t quantity);
  uint8_t readDiscreteInputs(uint16_t address, uint16_t quantity);
  uint8_t readHoldingRegisters(uint16_t address, uint16_t quantity);
  uint8_t readInputRegisters(uint16_t address, uint8_t quantity);
  uint8_t writeSingleCoil(uint16_t address, uint8_t value);
  uint8_t writeSingleRegister(uint16_t address, uint16_t value);
  uint8_t writeMultipleCoils(uint16_t address, uint16_t quantity);
  uint8_t writeMultipleRegisters(uint16_t address, uint16_t quantity);
  uint8_t maskWriteRegister(uint16_t address, uint16_t andMask, uint16_t orMask);
  uint8_t readWriteMultipleRegisters(uint16_t readAddress, uint16_t readQuantity, uint16_t writeAddress, uint16_t writeQuantity);

private:
  // methods
  static uint16_t crc16(uint16_t crc, uint8_t value);
  // members
  uint16_t _txEnablePin;
  uint16_t _rxEnablePin;
  bool _txActiveHigh = true;
  bool _rxActiveHigh = true;
  bool _sleepAble = true;

  uint16_t _slaveID = 1;
  uint32_t _speed = 9600;
  uint16_t _timeout = 500;
  
  ModbusStatus _status = MODBUS_UNKNOWN;

  USARTSerial *_serial;
  static uint16_t _buffer[MB_MAX_DATA_LENGTH];
  uint8_t _length = 0;
  uint8_t *_txBuffer;
  uint8_t _txIndex = 0;
  uint16_t _txLength = 0;
  uint8_t *_rxBuffer;
  uint8_t _rxIndex = 0;
  uint16_t _rxLength = 0;
  uint8_t _errorCode = 0;
  unsigned long _timestamp = 0;

  uint8_t _function;
  uint16_t _address;
  uint16_t _quantity;
  uint16_t _address2;
  uint16_t _quantity2;

  unsigned long _maxWaitToTransmitMS = MAX_WAIT_TO_TRANSMIT_MS;
  unsigned long _maxWaitForResponseMS = MAX_WAIT_FOR_RESPONSE_MS;
  unsigned long _characterDurationUS = 1050; // default for 9600, 10 bits - adjusted in begin()
};

#endif