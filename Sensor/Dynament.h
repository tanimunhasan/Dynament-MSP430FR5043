#ifndef DYNAMENTCOMMS_H
#define DYNAMENTCOMMS_H

#include <stdint.h>
#include <stdbool.h>


/* Timeout and packet size definitions */
#define DYNAMENT_FRAME_TIMEOUT    30
#define DYNAMENT_MAX_PACKET_SIZE  64  /* Need to change or replace with ?  */
#define DYNAMENT_RESPONSE_TIMEOUT 70
#define FINISH_SENSOR 4
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
extern void InitialiseDynamentComms(void);
extern void DynamentCommsHandler(void);

extern void CharReceived(uint8_t chr);
extern void initialise_comms(void);
extern void DynamentSensorApplication(void);

// Response status definitions used in the read measurand callback
#define READ_RESPONSE_TIMED_OUT           0
#define READ_RESPONSE_INVALID_REGISTER    1
#define READ_RESPONSE_VALUE_INVALID       2
#define READ_RESPONSE_VALUE_VALID         3

typedef void (*GetFloat_CallBack_t)(int valueStatus, float fval);

#define GAS_READING_MEASURAND   30057
extern volatile bool readyToPoll;
extern bool packetComplete;
void DynamentTick(void);
extern void RequestGasReading(void);
extern void DualReadingReceived(int status, float reading1, float reading2);
extern void ReadingReceived(int status, float value);

#define DYNAMENT_RX_BUFFER_SIZE  10 /* Remove or replace to ?   */

// Comms type
#define MODBUS_PROTOCOL     0
#define DYNAMENT_PROTOCOL   1
#define COMMS_PROTOCOL      DYNAMENT_PROTOCOL

// Checksum type
#define CSUM_STANDARD   0
#define CSUM_CRC        1
#define CSUM_TYPE       CSUM_STANDARD


#endif // DYNAMENTCOMMS_H
