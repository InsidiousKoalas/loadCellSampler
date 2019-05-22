/*
 * loadCellFunks.c
 *
 *  Created on: May 20, 2019
 *      Author: bhunt
 */
#include "loadCellFunks.h"

// Initialize pins
void loadCellInit(){
	P1DIR |= CLK;		// set CLK as output
	P1OUT &= ~CLK;		// set CLK low
	P1DIR &= ~SDI;		// set SDI as input
}



// read data from HX711 chip
long int readData(int gain){
	long int data = 0;
	unsigned char i;

	// check if ready, trap if not
	while(P1IN&SDI);
	HOLD;

	// begin reading data
	for(i=0; i<24; i++){			// for 24 bits...
		P1OUT |= CLK;				// toggle clock
		data <<= 1;					// bitshift data var
		HOLD;						// wait

		P1OUT &= ~CLK;				// toggle clock
		if(P1IN & SDI)(data++);		// set last bit
		HOLD;						// wait
	}

	// extra clock ticks set gain of chip
	for(i=0; i<gain-24; i++){		// for (extra clock cycles)...
		P1OUT |= CLK;				// toggle clock
		HOLD;						// wait
		P1OUT &= ~CLK;				// toggle clock
		HOLD;						// wait
	}

	if(data&(0x08000000)){			// if 24th bit is 1 (num is neg)...
		data = ~data;				// invert numbers
		data++;						// add one (two's comp)
	}

	return data;
}



