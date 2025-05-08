/*
 * DynamentComms.h
 *
 *  Created on: 7 May 2025
 *      Author: B4T
 */

#ifndef SRC_DYNAMENTCOMMS_H_
#define SRC_DYNAMENTCOMMS_H_


#include <stdint.h>
#include <stdbool.h>
#include "src/ModbusComms.h"

/* Timeout and packet size definitions */
#define DYNAMENT_FRAME_TIMEOUT    30
#define DYNAMENT_MAX_PACKET_SIZE  300
#define DYNAMENT_RESPONSE_TIMEOUT 70

/* Packet constants */
#define READ_VAR        0x13
#define DLE             0x10
#define WRITE_REQUEST   0x15
#define ACK             0x16
#define NAK             0x19
#define DAT             0x1a

#undef  EOF
#define EOF             0x1f

#define WRITE_PASSWORD_1 0xe5
#define WRITE_PASSWORD_2 0xa2

/* Variable IDs */
#define LIVE_DATA_SIMPLE    0x06
#define LIVE_DATA_2         0x2c

/* CRC calculation constants */
#define CRC_POLYNOMIAL  0x8005
#define CRC_VALUE_MASK  0x8000

/* Global type definitions ----------------------------------------------------*/
typedef void (*GetDualFloat_CallBack_t)(int valueStatus, float fval1, float fval2);
extern bool receivingPacket;

/* Public function prototypes */
void InitialiseDynamentComms(void);
void DynamentCommsHandler(void);
void RequestLiveDataSimple(GetFloat_CallBack_t cb);
void RequestLiveData2(GetDualFloat_CallBack_t cb);
extern void CharReceived(uint8_t chr);

#endif // DYNAMENTCOMMS_H
