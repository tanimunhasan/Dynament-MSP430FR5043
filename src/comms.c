/*
 * comms.c
 *
 *  Created on: 7 Mar 2025
 *      Author: Sayed
 */
#include <msp430.h>
#include "stdint.h"
#include "stdbool.h"
#include "comms.h"
#include "DynamentComms.h"
#include "ModbusComms.h"
#include "main.h"
#include <stdio.h>
#include "studiolib.h"
#include <stdarg.h>



/* Global variables ------------------------------*/
volatile int frameTimeout = 0;  // used to determine when a frame/packet has been received (no data received within a period of time after data has been received)
volatile int messageTimeout = 0; // used to determine when a message has been sent but no response received
volatile bool frameComplete = false;
volatile bool messageTimeOut = false;
volatile bool rxLock = false;
volatile uint16_t g_uiCommsTimeout  = 0;
uint8_t RX_Buffer[P2P_BUFFER_SIZE] __attribute__((aligned(16)));
volatile uint16_t rxHead = 0;  // Define it ONCE here
volatile uint16_t rxTail = 0;  // Define it ONCE here
volatile uint16_t rxCount = 0;




#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
{
    // Handle 10ms based timeouts (same as pico style)
    if (frameTimeout > 0)
    {
        frameTimeout--;
        if (frameTimeout == 0)
        {
            frameComplete = true;
        }
    }

    if (messageTimeout > 0)
    {
        messageTimeout--;
        if (messageTimeout == 0)
        {
            messageTimeOut = true;
        }
    }

}



#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
    switch(__even_in_range(UCA2IV, USCI_UART_UCTXIFG))
    {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
        rxLock = true;

        RX_Buffer[rxHead++] = UCA2RXBUF;
        if (rxHead >= P2P_BUFFER_SIZE)
            rxHead = 0;

        rxCount++;
        rxLock = false;
        break;
    case USCI_UART_UCTXIFG: break;
    }
}



// Function : transmit a given number of bytes of data
// Inputs : pucData - the array of data to send
//          len - the number of bytes to send
void p2pTxData(uint8_t* pucData, int len) {
    unsigned int x;
    for (x = 0; x < len; x++) {
        p2pTxByte(pucData[x]);
    }
}

// Function : Transmit a single byte of data
// Inputs : ucData - the data byte to send
void p2pTxByte(uint8_t ucData) {
    while (!(UCA2IFG & UCTXIFG)); // Wait until the TX buffer is ready
    UCA2TXBUF = ucData;           // Send the byte
}


/* TEST  */

uint8_t p2pRxByte(uint8_t* data)
{
    while (rxLock);  // wait until ISR is not updating

    if (rxCount == 0)
        return p2pRxNone;

    *data = RX_Buffer[rxTail++];
    if (rxTail >= P2P_BUFFER_SIZE)
        rxTail = 0;

    rxCount--;
    return p2pRxOk;
}


/* TEST  */

uint8_t readByte(void)
{
    if (rxHead == rxTail)
    {
        // Buffer is empty
        DEBUG_STRING("UART_Read: Buffer Empty!\n");
        return 0xFF; // Return a special value indicating "no data"
    }
    uint8_t data = RX_Buffer[rxTail];
    rxTail = (rxTail + 1) % P2P_BUFFER_SIZE;
    DEBUG_STRING("DATA: ");
    printInt(data);
    DEBUG_STRING("\n");
    return data;
}
