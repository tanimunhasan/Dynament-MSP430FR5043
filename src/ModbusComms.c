/*
 * ModbusComms.c
 *
 *  Created on: 7 May 2025
 *      Author: B4T
 */


#include "ModbusComms.h"
#include <msp430.h>
#include <stdio.h>
#include "main.h"
#include <stdbool.h>
#include "DynamentComms.h"
#include <stdint.h>
#include "comms.h"
//------------------------------------------------------------------------------
// Note: The following low-level UART functions must be defined elsewhere:
// p2pRxByte: reads one byte from the serial interface; returns a status (p2pRxOk if successful)
// p2pTxData: transmits a buffer of data over the UART interface.

//------------------------------------------------------------------------------

/* Constant declarations --------------------------------*/

#define PACKET_RX_TIMEOUT   3

// CRC lookup tables for high and low order bytes for MODBUS
const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};

const uint8_t auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

//------------------------------------------------------------------------------
// Global variables for Modbus communication
uint8_t modbusRxBuffer[300];
uint16_t modbusRxBufferCount = 0;

GetFloat_CallBack_t readMeasurandCallBack = 0;
bool readingMeasurand = false; // Indicates a measurand value is being requested

//------------------------------------------------------------------------------
// Local function prototypes
void DecodeMeasurand(uint16_t *registers, uint16_t numReg);
void ReadRegistersResponse(uint8_t *data, uint16_t len);

//------------------------------------------------------------------------------
// Function: CRC16
// Computes a 16-bit CRC using the Modbus lookup tables.
uint16_t CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF; // High CRC byte
    uint8_t uchCRCLo = 0xFF; // Low CRC byte
    uint8_t uIndex;
    uint16_t len = usDataLen;
    int x = 0;

    while ((len--) > 0)
    {
        uIndex = (uint8_t)(uchCRCLo ^ puchMsg[x++]);
        uchCRCLo = (uint8_t)(uchCRCHi ^ auchCRCHi[uIndex]);
        uchCRCHi = auchCRCLo[uIndex];
    }
    return (((uint16_t)uchCRCHi << 8) | uchCRCLo);
}

//------------------------------------------------------------------------------
// Function: SendPacket
// Forms a MODBUS packet from the given function code and data and sends it via UART.
void SendPacket(uint8_t function, uint8_t *data, int len)
{
    //printf("TX: ");

    uint8_t txBuffer[256];
    uint16_t txLen = 0;

    txBuffer[txLen++] = MODBUS_ADDRESS;  // MODBUS address (broadcast)
    txBuffer[txLen++] = function;
    int x;
    for (x = 0; x < len; x++)
    {
        txBuffer[txLen++] = data[x];
    }

    uint16_t crc = CRC16(txBuffer, txLen);
    txBuffer[txLen++] = (uint8_t)(crc & 0xff);           // Low CRC byte
    txBuffer[txLen++] = (uint8_t)((crc >> 8) & 0xff);      // High CRC byte

    p2pTxData(txBuffer, txLen);
}

//------------------------------------------------------------------------------
// Function: ReadRegisters
// Sends a request to read registers from a Modbus slave.
void ReadRegisters(uint8_t func, uint16_t address, uint16_t numReg)
{
    uint8_t data[4];

    // Create the data portion: register address and number of registers
    data[0] = (uint8_t)((address >> 8) & 0xff);
    data[1] = (uint8_t)(address & 0xff);
    data[2] = (uint8_t)((numReg >> 8) & 0xff);
    data[3] = (uint8_t)(numReg & 0xff);

    SendPacket(func, data, 4);

    // Set a message timeout (assumed to be handled by a timer elsewhere)
    messageTimeout = MESSAGE_TIMEOUT;
}

//------------------------------------------------------------------------------
// Function: ReadMeasurand
// Initiates a Modbus read for a floating point measurand.
void ReadMeasurand(uint16_t address, GetFloat_CallBack_t cb)
{
    readingMeasurand = true;
    readMeasurandCallBack = cb;
    ReadRegisters(MODBUS_FUNCTION_READ_INPUT_REGISTER, address, 5);
}

//------------------------------------------------------------------------------
// Function: DecodeMessage
// Called when a complete frame has been received. Decodes and validates the packet.
void DecodeMessage(void)
{
    uint8_t data[257];
    unsigned int len = 0;
    uint8_t ch;

    // Read available bytes until none remain or maximum length reached
    while (p2pRxByte(&ch) == p2pRxOk && len < 256)
    {
        data[len++] = ch;
    }

    // Minimum packet length check (address, function, at least one data byte, and CRC = 4 bytes)
    if (len < 4) return;

    // Extract received CRC (last two bytes) and calculate expected CRC
    uint16_t rcvCsum = (uint16_t)((data[len - 1] << 8) + data[len - 2]);
    uint16_t calcCsum = CRC16(data, len - 2);
    if (rcvCsum != calcCsum)
    {
        return;  // CRC mismatch, discard packet
    }

    // Check for error response (if function code has high bit set)
    if ((data[1] & 0x80) > 0)
    {
        if (readingMeasurand && readMeasurandCallBack != 0)
        {
            readMeasurandCallBack(READ_RESPONSE_INVALID_REGISTER, 0);
            readingMeasurand = false;
            readMeasurandCallBack = 0;
        }
    }
    else
    {
        // Dispatch based on function code
        switch (data[1])
        {
            case MODBUS_FUNCTION_READ_INPUT_REGISTER:
            case MODBUS_FUNCTION_READ_HOLDING_REGISTER:
                ReadRegistersResponse(data, len);
                break;
            case MODBUS_FUNCTION_WRITE_HOLDING_REGISTER:
                // Implement write confirmation if needed
                break;
            case MODBUS_FUNCTION_WRITE_HOLDING_REGISTERS:
                // Implement multi-register write confirmation if needed
                break;
        }
        // Cancel timeout since a valid response has been received
        messageTimeout = 0;
    }
}

//------------------------------------------------------------------------------
// Function: ReadRegistersResponse
// Processes a response packet containing register data.
void ReadRegistersResponse(uint8_t *data, uint16_t len)
{

    // data[2] contains byte count; each register is 2 bytes
    uint16_t regs = (uint16_t)data[2] / 2;
    if (len < ((regs * 2) + 5)) return;  // Verify packet length

    // convert the bytes in the data packet into 16 register data
    int x;
    uint16_t registers[128];
    for ( x = 0; x < regs; x++)
    {
        registers[x] = (uint16_t)((data[(x * 2) + 3] << 8) + data[(x * 2) + 4]);
    }

    if (readingMeasurand)
    {
        DecodeMeasurand(registers, regs);
    }
}

//------------------------------------------------------------------------------
// Function: DecodeMeasurand
// Converts the register values into a float and calls the callback.
void DecodeMeasurand(uint16_t *registers, uint16_t numReg)
{
    if (readingMeasurand && readMeasurandCallBack != 0)
    {
        if (registers[0] > 0)  // Non-zero status indicates a valid reading
        {
            uint32_t intVal = ((uint32_t)registers[1] << 16) | (uint32_t)registers[2];
            float *fPtr = (float *)&intVal;
            float f = *fPtr;

            readMeasurandCallBack(READ_RESPONSE_VALUE_VALID, f);
        }
        else  // Invalid reading (e.g., sensor error or warming up)
        {
            readMeasurandCallBack(READ_RESPONSE_VALUE_INVALID, 0);
        }
    }
    readingMeasurand = false;
    readMeasurandCallBack = 0;
}

//------------------------------------------------------------------------------
// Function: MessageTimedOut
// Called when no response is received within the timeout period.
void MessageTimedOut(void)
{
    if (readingMeasurand && readMeasurandCallBack != 0)
    {
        readMeasurandCallBack(READ_RESPONSE_TIMED_OUT, 0);
    }
    readingMeasurand = false;
    readMeasurandCallBack = 0;
}

//------------------------------------------------------------------------------
// Function: ModbusCommsHandler
// To be called periodically (e.g., from the main loop or a timer interrupt)
// to check for received packets or timeouts.
void ModbusCommsHandler(void)
{
    // frameComplete is assumed to be set when a full packet is received.

    if (frameComplete)
    {
        frameComplete = 0;
        DecodeMessage();
    }

    // Check for message timeout

    if (messageTimeout)
    {
        messageTimeout = 0;
        MessageTimedOut();
    }
}

//------------------------------------------------------------------------------
// Optional function: printModbusMessage
// For debugging purposes (if desired)
void printModbusMessage(void)
{
    // Implementation for printing a received Modbus message can be added here.
    // For example, iterate through modbusRxBuffer and print each byte.
}
