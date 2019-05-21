/*
 * loadCellFunks.c
 *
 *  Created on: May 20, 2019
 *      Author: bhunt
 */
#include "loadCellFunks.h"

void loadCellInit(){
	P1DIR |= CLK;
	P1OUT &= ~CLK;
	P1DIR &= ~SDI;
}


// read data from chip
unsigned long int readData(int gain){
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

	return data;
}



