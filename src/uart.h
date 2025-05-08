/*
 * uart.h
 *
 *  Created on: 27 Mar 2025
 *      Author: Sayed
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_
#include <stdint.h>
#include <stdio.h>

void delay_ms(unsigned int ms);
void uart_init(void);
void delay_ms(unsigned int ms);
void initGLED(void);
void initREDLED(void);
void toggleGLED(void);
void toggleRLED(void);

#endif /* SRC_UART_H_ */
