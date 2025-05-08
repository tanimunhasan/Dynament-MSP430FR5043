/*
 * studiolib.h
 *
 *  Created on: 7 May 2025
 *      Author: B4T
 */

#ifndef SRC_STUDIOLIB_H_
#define SRC_STUDIOLIB_H_


#include <stdint.h>
#include <stdio.h>

void UART_sendHex(uint8_t byte);
void UART_sendChar(char c);

void DEBUG_STRING(const char *str);

void UART_sendFloat(float value);
// Function to read a bybte from the circular buffer
void UART_sendPointer(void *ptr);
void printInt(int num);


#endif /* SRC_STUDIOLIB_H_ */
