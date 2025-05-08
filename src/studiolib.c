/*
 * studiolib.c
 *
 *  Created on: 7 May 2025
 *      Author: B4T
 */

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


void UART_sendChar(char c)
{
    // Wait for the transmit buffer to be ready
    while (!(UCA3IFG & UCTXIFG));
    // Transmit the character
    UCA3TXBUF = c;
}

void DEBUG_STRING(const char *str)
{
    while (*str)
    {
        UART_sendChar(*str++);
    }
}

void UART_sendFloat(float value)
{
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%.2f", value);
    DEBUG_STRING(buffer);
}

void UART_sendHex(uint8_t byte)
{
    // Declare a buffer for the hex string (2 characters + null terminator)
    char hexBuffer[3];

    // Convert the high nibble (upper 4 bits) to a hex character
    hexBuffer[0] = (byte >> 4) > 9 ? (byte >> 4) + 'A' - 10 : (byte >> 4) + '0';

    // Convert the low nibble (lower 4 bits) to a hex character
    hexBuffer[1] = (byte & 0x0F) > 9 ? (byte & 0x0F) + 'A' - 10 : (byte & 0x0F) + '0';

    // Null terminate the string
    hexBuffer[2] = '\0';

    // Send the two characters over UART (one at a time)
    DEBUG_STRING(hexBuffer);
}

void UART_sendPointer(void *ptr)
{
    // Cast the pointer to an unsigned long to ensure it's correctly formatted for transmission
    unsigned long ptrValue = (unsigned long)ptr;
    int i;
    // Send the pointer value byte by byte
    for (i = sizeof(ptrValue) - 1; i >= 0; i--) {
        UART_sendHex((uint8_t)(ptrValue >> (i * 8)));  // Shift the pointer to get each byte
    }

    // Optionally, send a newline or other separator to make it easier to read
    DEBUG_STRING("\n");
}

void printInt(int num) {
    char buffer[20];  // Assuming the integer won't be larger than 20 digits (which is a huge number)
    int index = 0;

    if (num == 0) {
        UART_sendChar('0');
        return;
    }

    if (num < 0) {
        UART_sendChar('-');
        num = -num;  // Make the number positive for further processing
    }

    // Convert the integer to a string (in reverse order)
    while (num > 0) {
        buffer[index++] = (num % 10) + '0'; // Get the last digit as a character
        num = num / 10; // Remove the last digit
    }
    int i;
    // Print the number in the correct order
    for (i = index - 1; i >= 0; i--) {
        UART_sendChar(buffer[i]);
    }
}




