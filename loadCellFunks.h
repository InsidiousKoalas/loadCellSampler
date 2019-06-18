/*
 * loadCellFunks.h
 *
 *  Created on: May 20, 2019
 *      Author: bhunt
 */
#include <msp430.h>

#ifndef LOADCELLFUNKS_H_
#define LOADCELLFUNKS_H_


// Various defines
#define 	SDI			0x20
#define 	CLK 		0x10
#define 	HOLD		__delay_cycles(8)
#define		HI_GAIN		25
#define		MED_GAIN	27
#define		LO_GAIN		26


// Functions
void loadCellInit();
long int readData(int);


#endif /* LOADCELLFUNKS_H_ */
