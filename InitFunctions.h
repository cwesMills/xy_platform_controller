/*
 * InitFunctions.h
 *
 *  Created on: Nov 18, 2019
 *      Author: bayan
 */
#include "main.h"
#include "driverlib.h"
#include "hal_LCD.h"
#include <stdio.h>
#include <msp430.h>
#ifndef INITFUNCTIONS_H_
#define INITFUNCTIONS_H_

void Init_GPIO(void);
void Init_Clock(void);
void Init_UART(void);
void Init_PWM(void);
void Init_ADC(void);
void MCUInit();

#endif /* INITFUNCTIONS_H_ */
