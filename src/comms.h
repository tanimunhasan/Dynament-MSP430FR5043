/*
 * comms.h
 *
 *  Created on: 10 Mar 2025
 *      Author: Sayed
 */

#ifndef SRC_COMMS_H_
#define SRC_COMMS_H_

#include <stdint.h>
#include "main.h"

#define P2P_BUFFER_SIZE  256
extern volatile int frameTimeout;
extern volatile int messageTimeout;
extern volatile bool frameComplete;
extern volatile bool messageTimeOut;
extern volatile uint16_t rxCount;

// Circular buffer for UART reception
extern uint8_t RX_Buffer[P2P_BUFFER_SIZE]  __attribute__((aligned(16)));
extern volatile uint16_t rxHead;
extern volatile uint16_t rxTail;

enum
{
    p2pRxNone,
    p2pRxOk,
    p2pRxError
};

//UART initialization function
void p2pTxData(uint8_t* pucData, int len);
void p2pTxByte(uint8_t ucData);
uint8_t p2pRxByte(uint8_t* pucData);

//uint8_t UART_Read(void);


//UART interrupt service routine


#endif /* SRC_COMMS_H_ */
