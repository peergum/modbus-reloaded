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

#include "modbus-reloaded.h"
#include "modbus-constants.h"

#include <string.h>

/**
 * Constructor with TX/RX enable pins
 */
Modbus::Modbus(USARTSerial &serial, uint16_t txEnablePin,
               uint16_t rxEnablePin,
               bool txActiveHigh,
               bool rxActiveHigh,
               bool sleepAble,
               PortSelector *portSelector,
               uint8_t address): _portSelector(portSelector), _address(address)
{
    _serial = &serial;
    _txEnablePin = txEnablePin;
    _rxEnablePin = rxEnablePin;
    _txActiveHigh = txActiveHigh;
    _rxActiveHigh = rxActiveHigh;
    _sleepAble = sleepAble;
}

/**
 * Constructor with TX/RX enable pin combined
 */
Modbus::Modbus(USARTSerial &serial,
    uint16_t txRxPin,
    bool txWhenHigh,
    PortSelector *portSelector,
    uint8_t address): _portSelector(portSelector), _address(address)
{
    _serial = &serial;
    _txEnablePin = txRxPin;
    _rxEnablePin = txRxPin; // same pin
    _txActiveHigh = txWhenHigh;
    _rxActiveHigh = !txWhenHigh;
    _sleepAble = false;
}

Modbus::Modbus(Modbus &modbus) {

}

/*
 * Initialize class
 */
void Modbus::begin(uint16_t speed, unsigned long config)
{
    // initialize hardware
    if (!_portSelector) {
        _portSelector = new PortSelector(0); // define a useless port selector to simplify
    }
    _portSelector->begin();
    _serial->begin(speed, config);
    _characterDurationUS = 10*1000000/speed; // in microseconds (considering 10bits/char)
    Serial.println("called begin");
    _state = MODBUS_READY;
}

/*
 * Set response timeout in ms
 * 
 * @parameter unsigned long timeoutMS
 */
void Modbus::setResponseTimeout(unsigned long timeoutMS) {
    _maxWaitForResponse = timeoutMS;
}

/*
 * Set transmit timeout in ms
 * 
 * @parameter unsigned long timeoutMS
 */
void Modbus::setTransmitTimeout(unsigned long timeoutMS) {
    _maxWaitToTransmitMS = timeoutMS;
}

/*
 * Prepare data for transmission
 * 
 * @parameter uint8_t functionCode
 * 
 * @return ModbusStatus success/error code
 */
ModBusStatus Modbus::send(uint8_t function)
{
    ModbusStatus status;
    // we should only send when state is READY
    if (_state != MODBUS_READY)
    {
        return MODBUS_BUSY;
    }
    _rxIndex = 0;
    _rxLength = 0;

    _txLength = 0;

    _txBuffer[_txLength++] = _slaveID;
    _txBuffer[_txLength++] = function;

    // read or write starting address    
    _txBuffer[_txLength++] = high(_address);
    _txBuffer[_txLength++] = low(_address);

    switch (function)
    {
    case _kMBReadCoils:
    case _kMBReadDiscreteInputs:
    case _kMBReadHoldingRegisters:
    case _kMBReadInputRegisters:
    case _kMBWriteSingleCoil: // value passed as quantity
    case _kMBWriteSingleRegister: // value passed as quantity
    case _kMBWriteMultipleCoils:
    case _kMBWriteMultipleRegisters:
    case _kMBReadWriteMultipleRegisters:
        // quantity or value
        _txBuffer[_txLength++] = high(_quantity);
        _txBuffer[_txLength++] = low(_quantity);
        break;

    case _kMBMaskWriteRegister:
        // masks
        _txBuffer[_txLength++] = _buffer[0];
        _txBuffer[_txLength++] = _buffer[1];
        break;
    }

    switch (function)
    {
    case _kMBWriteMultipleCoils:
        // byte count
        int counter = _txBuffer[_txLength++] = (_quantity + 7) / 8;
        // values from buffer
        for (int i = 0; i < counter; i++) {
            _txBuffer[_txLength++] = high(_buffer[i / 2]);
            _txBuffer[_txLength++] = low(_buffer[i / 2]);
        }
        break;
    case _kMBWriteMultipleRegisters:
        // byte count
        _txBuffer[_txLength++] = 2 * _quantity;
        // values from buffer
        for (int i = 0; i < _quantity; i++) {
            _txBuffer[_txLength++] = high(_buffer[i]);
            _txBuffer[_txLength++] = low(_buffer[i]);
        }
        break;
    case _kMBReadWriteMultipleRegisters:
        // write starting address
        _txBuffer[_txLength++] = high(_address2);
        _txBuffer[_txLength++] = low(_address2);
        // quantity to write
        _txBuffer[_txLength++] = high(_quantity2);
        _txBuffer[_txLength++] = low(_quantity2);
        // byte count
        _txBuffer[_txLength++] = 2 * _quantity2;
        // values from buffer
        for (int i = 0; i < _quantity2; i++) {
            _txBuffer[_txLength++] = high(_buffer[i]);
            _txBuffer[_txLength++] = low(_buffer[i]);
        }
        break;
    }

    return send();
}

/*
 * Transmit request
 * 
 * @return ModbusStatus success/error code
 */
ModbusStatus Modbus::send() {
    uint16_t crc = 0xffff;
    unsigned long timestamp = millis();

    unsigned long maxInterval = (50 * 1000000UL / _speed); //50 bit late at most

    _portSelector->selectPort(_address);

    // wait to write (ms)
    while (!_serial->availableForWrite() && millis()-timestamp < MAX_WAIT_TO_WRITE_MS)
        ;
    if (millis()-timestamp >= MAX_WAIT_TO_WRITE_MS) {
        return (_status = MODBUS_WRITE_TIMEOUT);
    }

    // use microsec inbetween chars
    timestamp = micros();

    for (int i = 0; i < _txLength; i++)
    {
        crc = crc16(crc, _txBuffer[i]);
        // allow a delay up to 5 chars
        while (!_serial->availableForWrite() && micros()-timestamp <  5 * _characterDurationUS)
            ;
        if (!_serial->availableForWrite()) {
            return (_status = MODBUS_WRITE_TIMEOUT);
        }
        _serial->write(_txBuffer[i]);
        timestamp = micros();
    }
    _serial->write(high(crc));
    _serial->write(low(crc));
    _timestamp = millis();
    return (_status = MODBUS_SUCCESS);
}

/*
 * Check Response
 * 
 * Waits until we got the whole response
 * 
 * @return ModbusStatus current state of transaction
 */
ModbusStatus Modbus::checkResponse()
{
    uint8_t length;
    uint8_t byteCount;

    _portSelector->selectPort(_address);
    
    while (_serial->available() > 0)
    {
        _rxBuffer[_rxLength++] = _serial->read();
    }

    if (_rxLength < 2 && millis()-_timestamp < _maxWaitForResponseMS) {
        // we need at least the slave ID and function code to start decoding
        return (_status = MODBUS_RECEIVING);
    }
    if (millis()-_timestamp >= _maxWaitForResponseMS) {
        return (_status = MODBUS_TIMEOUT); 
    }

    uint16_t crc = 0xffff;
    for (int i=0; i< _rxLength; i++) {
        crc = crc16(crc, _rxBuffer[i]);
    }

    if (_rxBuffer[0] != _slaveID) {
        // wrong slave response
        _errorCode = _kMBInvalidSlaveID;
        return (_status = MODBUS_ERROR);
    }

    if (_rxBuffer[1]>=0x80) {
        // error code
        _errorCode = _rxBuffer[2];
        return (_status = MODBUS_ERROR);
    }
    switch (_rxBuffer[1]) {
    case _kMBReadCoils:
        byteCount = _rxBuffer[2];
        if (byteCount % 8 != 0) {
            byteCount++;
        }
        length = count + 5;
        break;
    case _kMBReadDiscreteInputs:
    case _kMBReadWriteMultipleRegisters:
        byteCount = _rxBuffer[2];
        length = count + 5;
        break;
    case _kMBReadHoldingRegisters:
    case _kMBReadInputRegisters:
        byeCount = _rxBuffer[2] * 2;
        length = count + 5;
        break;
    case _kMBWriteSingleCoil:
    case _kMBWriteSingleRegister:
    case _kMBWriteMultipleCoils:
        length = 8;
        break;
    case _kMBMaskWriteRegister:
        length = 10; 
        break;
    default:
        return (_status = MODBUS_UNHANDLED_FUNCTION);
    }

    if (_rxLength < count) // slaveid + function + count + response bytes + crc16
    {
        return (_status = MODBUS_RECEIVING);
    }

    // decode bytes
    switch (_rxBuffer[1]) {
    case _kMBReadCoils:
    case _kMBReadDiscreteInputs:
        for (int i= 0; i<byteCount; i++) {
            _buffer[i] = _rxBuffer[i+3];
        }
        return (_status = MODBUS_DATA_READY);
        break;
    case _kMBReadHoldingRegisters:
    case _kMBReadInputRegisters:
        for (int i= 0; i<byteCount/2; i++) {
            _buffer[i] = word(_rxBuffer[2*i+3],_rxBuffer[2*i+4]);
        }
        return (_status = MODBUS_DATA_READY);
        break;
    case _kMBWriteSingleCoil:
    case _kMBWriteSingleRegister:
        if (_rxBuffer[2] != _txBuffer[2] || _rxBuffer[3] != _txBuffer[3]) {
            _errorCode = MODBUS_WRONG_ADDRESS;
            return (_status = MODBUS_ERROR);
        }
        if (_rxBuffer[4] != _txBuffer[4] || _rxBuffer[5] != _txBuffer[5]) {
            _errorCode = MODBUS_WRONG_VALUE;
            return (_status = MODBUS_ERROR);
        }
        return MODBUS_READY;
        break;
    case _kMBMaskWriteRegister:
        if (_rxBuffer[2] != _txBuffer[2] || _rxBuffer[3] != _txBuffer[3]) {
            _errorCode = MODBUS_WRONG_ADDRESS;
            return (_status = MODBUS_ERROR);
        }
        if (_rxBuffer[4] != _txBuffer[4] || _rxBuffer[5] != _txBuffer[5] || 
        _rxBuffer[6] != _txBuffer[4] || _rxBuffer[5] != _txBuffer[7]) {
            _errorCode = MODBUS_WRONG_MASK;
            return (_status = MODBUS_ERROR);
        }
        return (_status = MODBUS_READY);
        break;
    case _kMBWriteMultipleCoils:
    case _kMBWriteMultipleRegisters:
        if (_rxBuffer[2] != _txBuffer[2] || _rxBuffer[3] != _txBuffer[3]) {
            _errorCode = MODBUS_WRONG_ADDRESS;
            return (_status = MODBUS_ERROR);
        }
        if (_rxBuffer[4] != _txBuffer[4] || _rxBuffer[5] != _txBuffer[5]) {
            _errorCode = MODBUS_WRONG_QUANTITY;
            return (_status = MODBUS_ERROR);
        }
        return (_status = MODBUS_READY);
        break;
    case _kMBReadWriteMultipleRegisters:
        if (_rxBuffer[2] != _txBuffer[10]) {
            _errorCode = MODBUS_WRONG_BYTE_COUNT;
            return (_status = MODBUS_ERROR);
        }
        for (int i= 0; i<byteCount/2; i++) {
            _buffer[i] = word(_rxBuffer[2*i+3],_rxBuffer[2*i+4]);
        }
        return (_status = MODBUS_DATA_READY);
        break;
    }

    return (_status = MODBUS_DATA_READY);
}

/*
 * write into buffer for transmission
 */
void Modbus::clrBuffer(void) {
    for (int i=0; i<_length; i++) {
        _buffer[_length] = 0;
    }
}

/*
 * write into buffer for transmission
 */
void Modbus::writeBuffer(uint8_t index, uint16_t value) {
    _buffer[index] = value;
    _length = index+1;
}

/*
 * append data to buffer for transmission
 */
void Modbus::appendBuffer(uint16_t value) {
    writeBuffer(_length++, value);
}

/*
 * copy data to buffer for transmission
 */
void ModBus::copyBuffer(uint16_t *source, uint16_t length) {
    memcpy((void *)_buffer, (void *)source, length * 2);
    _length = length;
}

/*
 * write bit into buffer for transmission
 */
void ModBus::writeBitBuffer(uint16_t index, bool bit) {
    if (bit) {
        _buffer[index / 16] |= 1 << (index % 16);
    } else {
        _buffer[index / 16] &= ~(1 << (index % 16));
    }
    _length = index / 16;
}

/*
 * get length words from offset in _rxBuffer
 * 
 * @parameter uint16_t * buffer
 * @parameter uint16_t offset
 * @parameter uint16_t number of words to retrieve
 * 
 * @return number of words retrieved
 */
uint16_r Modbus::getBuffer(uint16_t *buffer, uint16_t offset, uint16_t length) {
    if (offset > _length || offset + length > _length) {
        return 0;
    }
    int i;
    for (i=0; i < length && i + offset < _length; i++) {
        buffer[i] = _rxBuffer[i + offset];
    }
    return i;
}

/*
 * write bit into buffer for transmission
 */

/**
 *  * Modbus function 0x01 Read Coils.
 * 
 * This function code is used to read from 1 to 2000 contiguous status of
 * coils in a remote device. The request specifies the starting address,
 * i.e. the address of the first coil specified, and the number of coils.
 * Coils are addressed starting at zero.
 * 
 * The coils in the response buffer are packed as one coil per bit of the
 * data field. Status is indicated as 1=ON and 0=OFF. The LSB of the first
 * data word contains the output addressed in the query. The other coils
 * follow toward the high order end of this word and from low order to high
 * order in subsequent words.
 * 
 * If the returned quantity is not a multiple of sixteen, the remaining
 * bits in the final data word will be padded with zeros (toward the high
 * order end of the word).
 * 
 * @param address address of first coil (0x0000..0xFFFF)
 * @param quantity quantity of coils to read (1..2000, enforced by remote device)
 * @return ModbusStatus status
 *
 */
ModbusStatus ModbusMaster::readCoils(uint16_t address, uint16_t quantity)
{
    if (quantity<1 || quantity>2000) {
        return (_status = MODBUS_BAD_QUANTITY);
    }
    _address = address;
    _quantity = quantity;
    return send(_kMBReadCoils);
}

/**
 * Modbus function 0x02 Read Discrete Inputs.
 * 
 * This function code is used to read from 1 to 2000 contiguous status of
 * discrete inputs in a remote device. The request specifies the starting
 * address, i.e. the address of the first input specified, and the number
 * of inputs. Discrete inputs are addressed starting at zero.
 * 
 * The discrete inputs in the response buffer are packed as one input per
 * bit of the data field. Status is indicated as 1=ON; 0=OFF. The LSB of
 * the first data word contains the input addressed in the query. The other
 * inputs follow toward the high order end of this word, and from low order
 * to high order in subsequent words.
 * 
 * If the returned quantity is not a multiple of sixteen, the remaining
 * bits in the final data word will be padded with zeros (toward the high
 * order end of the word).
 * 
 * @param address address of first discrete input (0x0000..0xFFFF)
 * @param quantity quantity of discrete inputs to read (1..2000, enforced by remote device)
 * @return ModbusStatus status
 */
ModbusStatus ModbusMaster::readDiscreteInputs(uint16_t address,
                                              uint16_t quantity)
{
    if (quantity<1 || quantity>2000) {
        return (_status = MODBUS_BAD_QUANTITY);
    }
    _address = address;
    _quantity = quantity;
    return send(_kMBReadDiscreteInputs);
}

/**
 * Modbus function 0x03 Read Holding Registers.
 * 
 * This function code is used to read the contents of a contiguous block of
 * holding registers in a remote device. The request specifies the starting
 * register address and the number of registers. Registers are addressed
 * starting at zero.
 * 
 * The register data in the response buffer is packed as one word per
 * register.
 * 
 * @param address address of the first holding register (0x0000..0xFFFF)
 * @param quantity quantity of holding registers to read (1..125, enforced by remote device)
 * @return ModbusStatus status
 */
ModbusStatus ModbusMaster::readHoldingRegisters(uint16_t address,
                                                uint16_t quantity)
{
    if (quantity<1 || quantity>125) {
        return (_status = MODBUS_BAD_QUANTITY);
    }
    _address = address;
    _quantity = quantity;
    return send(_kMBReadHoldingRegisters);
}

/**
 * Modbus function 0x04 Read Input Registers.
 * 
 * This function code is used to read from 1 to 125 contiguous input
 * registers in a remote device. The request specifies the starting
 * register address and the number of registers. Registers are addressed
 * starting at zero.
 * 
 * The register data in the response buffer is packed as one word per
 * register.
 * 
 * @param address address of the first input register (0x0000..0xFFFF)
 * @param quantity quantity of input registers to read (1..125, enforced by remote device)
 * @return ModbusStatus status
 */
ModbusStatus ModbusMaster::readInputRegisters(uint16_t address,
                                              uint8_t quantity)
{
    if (quantity<1 || quantity>125) {
        return (_status = MODBUS_BAD_QUANTITY);
    }
    _address = address;
    _quantity = quantity;
    return send(_kMBReadInputRegisters);
}

/**
 * Modbus function 0x05 Write Single Coil.
 * 
 * This function code is used to write a single output to either ON or OFF
 * in a remote device. The requested ON/OFF state is specified by a
 * constant in the state field. A non-zero value requests the output to be
 * ON and a value of 0 requests it to be OFF. The request specifies the
 * address of the coil to be forced. Coils are addressed starting at zero.
 * 
 * @param address address of the coil (0x0000..0xFFFF)
 * @param state true or false
 * @return ModbusStatus status
 */
ModbusStatus ModbusMaster::writeSingleCoil(uint16_t address, bool state)
{
    _address = address;
    _quantity = (state ? 0xFF00 : 0x0000);
    return send(_kMBWriteSingleCoil);
}

/**
 * Modbus function 0x06 Write Single Register.
 * 
 * This function code is used to write a single holding register in a
 * remote device. The request specifies the address of the register to be
 * written. Registers are addressed starting at zero.
 * 
 * @param address address of the holding register (0x0000..0xFFFF)
 * @param u16WriteValue value to be written to holding register (0x0000..0xFFFF)
 * @return ModbusStatus status
 */
ModbusStatus ModbusMaster::writeSingleRegister(uint16_t address, uint16_t value)
{
    _address = address;
    _quantity = value;
    return send(kMB_WriteSingleRegister);
}

/**
 * Modbus function 0x0F Write Multiple Coils.
 * 
 * This function code is used to force each coil in a sequence of coils to
 * either ON or OFF in a remote device. The request specifies the coil
 * references to be forced. Coils are addressed starting at zero.
 * 
 * The requested ON/OFF states are specified by contents of the transmit
 * buffer. A logical '1' in a bit position of the buffer requests the
 * corresponding output to be ON. A logical '0' requests it to be OFF.
 * 
 * @param address address of the first coil (0x0000..0xFFFF)
 * @param quantity quantity of coils to write (1..0x7b0, enforced by remote device)
 * @return ModbusStatus status
 */
ModbusStatus ModbusMaster::writeMultipleCoils(uint16_t address,
                                              uint16_t quantity)
{
        if (quantity<1 || quantity>0x7b0) {
        return (_status = MODBUS_BAD_QUANTITY);
    }
    _address = address;
    _quantity = quantity;
    return send(_kMBWriteMultipleCoils);
}

/**
 * Modbus function 0x10 Write Multiple Registers.
 * 
 * This function code is used to write a block of contiguous registers (1
 * to 123 registers) in a remote device.
 * 
 * The requested written values are specified in the transmit buffer. Data
 * is packed as one word per register.
 * 
 * @param address address of the holding register (0x0000..0xFFFF)
 * @param quantity quantity of holding registers to write (1..123, enforced by remote device)
 * @return ModbusStatus status
 */
ModbusStatus ModbusMaster::writeMultipleRegisters(uint16_t address,
                                                  uint16_t quantity)
{
    if (quantity<1 || quantity>123) {
        return (_status = MODBUS_BAD_QUANTITY);
    }
    _address = address;
    _quantity = quantity;
    return send(_kMBWriteMultipleRegisters);
}

/**
 * Modbus function 0x16 Mask Write Register.
 * 
 * This function code is used to modify the contents of a specified holding
 * register using a combination of an AND mask, an OR mask, and the
 * register's current contents. The function can be used to set or clear
 * individual bits in the register.
 * 
 * The request specifies the holding register to be written, the data to be
 * used as the AND mask, and the data to be used as the OR mask. Registers
 * are addressed starting at zero.
 * 
 * The function's algorithm is:
 * 
 * Result = (Current Contents && And_Mask) || (Or_Mask && (~And_Mask))
 * 
 * @param address address of the holding register (0x0000..0xFFFF)
 * @param u16AndMask AND mask (0x0000..0xFFFF)
 * @param u16OrMask OR mask (0x0000..0xFFFF)
 * @return ModbusStatus status
*/
ModbusStatus ModbusMaster::maskWriteRegister(uint16_t address,
                                             uint16_t andMask, uint16_t orMask)
{
    _address = address;
    _buffer[0] = andMask;
    _buffer[1] = orMask;
    return send(_kMBMaskWriteRegister);
}

/**
 * Modbus function 0x17 Read Write Multiple Registers.
 * 
 * This function code performs a combination of one read operation and one
 * write operation in a single MODBUS transaction. The write operation is
 * performed before the read. Holding registers are addressed starting at
 * zero.
 * 
 * The request specifies the starting address and number of holding
 * registers to be read as well as the starting address, and the number of
 * holding registers. The data to be written is specified in the transmit
 * buffer.
 * 
 * @param address address of the first holding register (0x0000..0xFFFF)
 * @param quantity quantity of holding registers to read (1..125, enforced by remote device)
 * @param address address of the first holding register (0x0000..0xFFFF)
 * @param quantity quantity of holding registers to write (1..121, enforced by remote device)
 * @return 0 on success; exception number on failure
 * @ingroup register
*/
ModbusStatus ModbusMaster::readWriteMultipleRegisters(uint16_t readAddress,
                                                      uint16_t readQuantity,
                                                      uint16_t writeAddress, uint16_t writeQuantity)
{
    if (readQuantity<1 || readQuantity>0x7d || writeQuantity<1 || writeQuantity>0x79) {
        return (_status = MODBUS_BAD_QUANTITY);
    }
    _address = readAddress;
    _quantity = readQuantity;
    _address2 = writeAddress;
    _quantity2 = writeQuantity;
    return send(_kMBReadWriteMultipleRegisters);
}

/*
 * calculate crc 16
 */
static uint16_t Modbus::crc16(uint16_t crc, uint8_t value)
{
    int i;

    crc ^= value;
    for (i = 0; i < 8; ++i)
    {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc = (crc >> 1);
    }

    return crc;
}

/*
 *
 */
