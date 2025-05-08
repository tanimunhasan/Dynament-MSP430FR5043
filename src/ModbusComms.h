/*
 * ModbusComms.h
 *
 *  Created on: 7 May 2025
 *      Author: B4T
 */

#ifndef SRC_MODBUSCOMMS_H_
#define SRC_MODBUSCOMMS_H_

#include <stdint.h>
#include <stdio.h>

/* Global type definitions ----------------------------------------------------*/
typedef void (*GetFloat_CallBack_t)(int valueStatus, float fval);

/* Global constants -----------------------------------------------------------*/
#define MODBUS_ADDRESS 0  // Use 0 for broadcast interrogation

#define FRAME_TIMEOUT 3        // Timeout steps for frame reception (in 10ms increments, for example)
#define MESSAGE_TIMEOUT 50     // Timeout steps for receiving a response

// Modbus function codes
#define MODBUS_FUNCTION_READ_INPUT_REGISTER     4
#define MODBUS_FUNCTION_READ_HOLDING_REGISTER   3
#define MODBUS_FUNCTION_WRITE_HOLDING_REGISTER  6
#define MODBUS_FUNCTION_WRITE_HOLDING_REGISTERS 16

// Response status definitions used in the read measurand callback
#define READ_RESPONSE_TIMED_OUT           0
#define READ_RESPONSE_INVALID_REGISTER    1
#define READ_RESPONSE_VALUE_INVALID       2
#define READ_RESPONSE_VALUE_VALID         3

/* Function Prototypes --------------------------------------------------------*/
void DecodeMessage(void);
void ReadMeasurand(uint16_t address, GetFloat_CallBack_t cb);
void MessageTimedOut(void);
void ModbusCommsHandler(void);
void printModbusMessage(void);  // For debug output

#endif
