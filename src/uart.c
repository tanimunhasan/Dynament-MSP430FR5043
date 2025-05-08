#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "comms.h"
#include "DynamentComms.h"
#include "ModbusComms.h"
#include "uart.h"
#include "studiolib.h"
#include "main.h"



void uart_init(void)
{
    // Configure UART pins

    //-----eUSCI_A3 for USB Terminal (P2.0 TX, P2.1 RX) ---------------
    P2SEL1 |= BIT0 | BIT1;      // TX: P2.0, RX: P2.1
    P2SEL0 &= ~(BIT0 | BIT1);

    // Put eUSCI_A3 into reset mode before configuring
    UCA3CTLW0 = UCSWRST;
    UCA3CTLW0 |= UCSSEL__SMCLK;  // Select SMCLK as clock source

    // Baud Rate = 38400 bps @ 1 MHz
    UCA3BRW = 1;
    UCA3MCTLW = UCOS16 | (10 << 4) | (0x00 << 8);  // UCBRFx = 10, UCBRSx = 0x00
    // ---------- eUSCI_A2 for N2O Sensor (P5.0 TX, P5.1 RX) ----------
    P5SEL1 |= BIT0 | BIT1;             // Set P5.0 -> TXD, P5.1 -> RXD (Dynament Sensor)
    P5SEL0 &= ~(BIT0 | BIT1);          // Secondary function

    UCA2CTLW0 = UCSWRST;               // Put eUSCI_A1 in reset mode
    UCA2CTLW0 |= UCSSEL__SMCLK;        // Use SMCLK



    UCA2BRW = 1; // 🔹 Fix: Set Baud Rate for UCA2
    UCA2MCTLW = UCOS16 | (10 << 4) | (0x00 << 8);  // Modulation settings

    // Release from reset
    UCA3CTLW0 &= ~UCSWRST;
    UCA2CTLW0 &= ~UCSWRST; // Correctly release from reset

    // Enable RX interrupt for UCA2 N2O
    UCA2IE |= UCRXIE;

    // Debugging output
    DEBUG_STRING("UCA1CTLW0 After Release: ");
    UART_sendHex(UCA1CTLW0);
    DEBUG_STRING("\n");

    DEBUG_STRING("CSCTL1: ");
    UART_sendHex(CSCTL1);
    DEBUG_STRING("\n");

    // Manually trigger the interrupt for testing (optional)
   // UCA1IFG |= UCRXIFG;  // Force RX ISR to execute
}

void delay_ms(unsigned int ms)
{
    while (ms--)
    {
        __delay_cycles(1000);          // 1 ms delay at 1 MHz
    }
}


void initGLED(void)
{
    P6DIR |= BIT0;  // Set P1.0 as output
    P6OUT &= ~BIT0; // Turn off LED initially
}


void toggleGLED(void)
{
    P6OUT ^= BIT0;  // Toggle the state of P1.0


}
