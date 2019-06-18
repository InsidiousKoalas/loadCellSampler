/*
 * thFunks.c
 *
 *  Created on: May 29, 2019
 *      Author: bhunt
 *
 * Samples a DHT11 temperature and humidity sensor. The mainloop (CHANGE?)
 * dictates the states of the device as follows:
 *
 * 		State 0:	device is ready to be woken
 * 		State 1:	device is ready to sample
 * 		State 2:	wait while device is waking
 * 		State 3:	wait while device sleeps
 *
 *
 */

#include <msp430.h>
#include "thFunks.h"

#define HOLD __delay_cycles(0x0FF);
#define  CHK_TO	if(TA1R>(tic+30)){ P1DIR |= DATA; return 1;}


volatile char thBuffer[5] = { 0 }, restFlag = 0;		// change to char when

// initialize function
void thInit(){
	P1DIR |= DATA;
}

void thStart(){
	// set pin high
	P1OUT |= DATA;
	HOLD;

	// pull pin low
	P1OUT &= ~DATA;

	// wait for 18 ms (unobtrusive, frees CPU)
	TA1CCR0 = TA1R+5000;		// 18 ms * 250 kHz = 4500 cycles
}

int thRead(){
	unsigned char thNdx, bit, checkSum;
	unsigned int tic;

	TA1CCTL2 &= ~CCIE;
//	P1OUT |= DATA;
//	__delay_cycles(5);

	P1DIR &= ~DATA;		// switch dataline to input
	__delay_cycles(5);
	// ignore first DHT pulse
	tic = TA1R;
	while(P1IN & DATA){	// trap line, wait to go low
		CHK_TO
	}
	__delay_cycles(5);

	tic = TA1R;
	while(!(P1IN & DATA)){	// trap line, wait to go high
		CHK_TO
	}
	__delay_cycles(5);

	tic = TA1R;
	while(P1IN & DATA){	// trap line again
		CHK_TO
	}
	__delay_cycles(5);

	for(thNdx=0; thNdx<5; thNdx++){		// for ( each pos in array ) ...
		thBuffer[thNdx] = 0;
		P1OUT ^= BIT0;
		for(bit=0; bit<8; bit++){		// for ( each bit in byte ) ...
			thBuffer[thNdx] <<= 1;

			tic = TA1R;
			while(!(P1IN & DATA)){		// hold while line low, wait to go high
				CHK_TO
			}

			__delay_cycles(28);			// threshold time; if line is 0 at this time, data is 0; else, 1

//			tic = TA1R;
//			while(TA1R<(tic+6));		// hold for length of clock pulse

			if(P1IN&DATA)(thBuffer[thNdx] |= 1);
			while(P1IN & DATA){
				CHK_TO
			}

		}

	}

	checkSum = thBuffer[0]+thBuffer[1]+thBuffer[2]+thBuffer[3];

	if(checkSum != thBuffer[4]){
		P1DIR |= DATA;
		return 1;
	}


	P1DIR |= DATA;		// return to output
	TA1CCR0 = TA1R+0xFFFF;	// maximum integer value
	restFlag = 1;
	return 0;

}

