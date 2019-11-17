// Modbus exception codes
/**
 * Modbus protocol illegal function exception.
 * 
 * The function code received in the query is not an allowable action for
 * the server (or slave). This may be because the function code is only
 * applicable to newer devices, and was not implemented in the unit
 * selected. It could also indicate that the server (or slave) is in the
 * wrong state to process a request of this type, for example because it is
 * unconfigured and is being asked to return register values.
 * 
 * @ingroup constant
 */
static const uint8_t _kMBIllegalFunction            = 0x01;

/**
 * Modbus protocol illegal data address exception.
 * 
 * The data address received in the query is not an allowable address for
 * the server (or slave). More specifically, the combination of reference
 * number and transfer length is invalid. For a controller with 100
 * registers, the ADU addresses the first register as 0, and the last one
 * as 99. If a request is submitted with a starting register address of 96
 * and a quantity of registers of 4, then this request will successfully
 * operate (address-wise at least) on registers 96, 97, 98, 99. If a
 * request is submitted with a starting register address of 96 and a
 * quantity of registers of 5, then this request will fail with Exception
 * Code 0x02 "Illegal Data Address" since it attempts to operate on
 * registers 96, 97, 98, 99 and 100, and there is no register with address
 * 100.
 * 
 * @ingroup constant
 */
static const uint8_t _kMBIllegalDataAddress         = 0x02;

/**
 * Modbus protocol illegal data value exception.
 * 
 * A value contained in the query data field is not an allowable value for
 * server (or slave). This indicates a fault in the structure of the
 * remainder of a complex request, such as that the implied length is
 * incorrect. It specifically does NOT mean that a data item submitted for
 * storage in a register has a value outside the expectation of the
 * application program, since the MODBUS protocol is unaware of the
 * significance of any particular value of any particular register.
 * 
 * @ingroup constant
 */
static const uint8_t _kMBIllegalDataValue           = 0x03;

/**
 * Modbus protocol slave device failure exception.
 * 
 * An unrecoverable error occurred while the server (or slave) was
 * attempting to perform the requested action.
 * 
 * @ingroup constant
 */
static const uint8_t _kMBSlaveDeviceFailure         = 0x04;

// Class-defined success/exception codes
/**
 * ModbusMaster success.
 * 
 * Modbus transaction was successful; the following checks were valid:
 *     - slave ID
 *     - function code
 *     - response code
 *     - data
 *     - CRC
 * 
 * @ingroup constant
 */
static const uint8_t _kMBSuccess                    = 0x00;

/**
 * ModbusMaster invalid response slave ID exception.
 * 
 * The slave ID in the response does not match that of the request.
 * 
 * @ingroup constant
 */
static const uint8_t _kMBInvalidSlaveID             = 0xE0;

/**
 * ModbusMaster invalid response function exception.
 * 
 * The function code in the response does not match that of the request.
 * 
 * @ingroup constant
 */
static const uint8_t _kMBInvalidFunction            = 0xE1;

/**
 * ModbusMaster response timed out exception.
 * 
 * The entire response was not received within the timeout period,
 * ModbusMaster::_kMBResponseTimeout.
 * 
 * @ingroup constant
 */
static const uint8_t _kMBResponseTimedOut           = 0xE2;

/**
 * ModbusMaster invalid response CRC exception.
 * 
 * The CRC in the response does not match the one calculated.
 * 
 * @ingroup constant
 */
static const uint8_t _kMBInvalidCRC                 = 0xE3;

// Modbus function codes for bit access
static const uint8_t _kMBReadCoils                  = 0x01; ///< Modbus function 0x01 Read Coils
static const uint8_t _kMBReadDiscreteInputs         = 0x02; ///< Modbus function 0x02 Read Discrete Inputs
static const uint8_t _kMBWriteSingleCoil            = 0x05; ///< Modbus function 0x05 Write Single Coil
static const uint8_t _kMBWriteMultipleCoils         = 0x0F; ///< Modbus function 0x0F Write Multiple Coils

// Modbus function codes for 16 bit access
static const uint8_t _kMBReadHoldingRegisters       = 0x03; ///< Modbus function 0x03 Read Holding Registers
static const uint8_t _kMBReadInputRegisters         = 0x04; ///< Modbus function 0x04 Read Input Registers
static const uint8_t _kMBWriteSingleRegister        = 0x06; ///< Modbus function 0x06 Write Single Register
static const uint8_t _kMBWriteMultipleRegisters     = 0x10; ///< Modbus function 0x10 Write Multiple Registers
static const uint8_t _kMBMaskWriteRegister          = 0x16; ///< Modbus function 0x16 Mask Write Register
static const uint8_t _kMBReadWriteMultipleRegisters = 0x17; ///< Modbus function 0x17 Read Write Multiple Registers
