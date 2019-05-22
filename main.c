
/*
 * (c) Brandon Hunt, Charles Rawlins 2019
 *
 * loadCellSampler
 *
 * The purpose of this code is to sample the load cell (over SPI)
 * and send the data to a computer via UART.
 *
 * P1.4 --> CLK
 * P1.3 --> SDI
 *
 * P1.2	-->	TX (RX on computer side)
 * P1.1 --> RX (TX on computer side)
 *
 */

//includes
#include <msp430.h>
#include "loadCellFunks.h"
#include "serial_handler.h"

// defines
#define NUM_LENG	8		// length of max 24-bit number reading (2^(24) = 16777216, 8 chars long)

// functions
void num2str(long int);
long int absVal(long int);

// global variables
long int data;
unsigned char sampDataFlag = 0;
char buffer;


/*
 * mainloop
 */
int main(void){
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // Timer interrupt init
  CCTL0 = CCIE;                         	// CCR0 interrupt enabled
  CCR0 = 512-1;								// set for PWM
  TACTL = TASSEL_2 | MC_1 | ID_3;       	// SMCLK, upmode, divide by 8 (1MHz / 8 = 125 kHz)

  // PWM init
  P1DIR |= BIT6;                            // P1.2 output
  P1SEL |= BIT6;                            // P1.2 for TA0.1 output
  P1SEL2 = 0;								// Select default function for P1.2
  CCTL1 = OUTMOD_7;                         // CCR1 reset/set
  CCR1 = 189;                               // CCR1 PWM duty cycle


  // other intializations
  uart_init(8);							// initialize UART
  loadCellInit();						// initialize pins for load cell
  int gain = HI_GAIN;					// see loadCellFunks.h for different gain levels

  int timerCycles = 12500/CCR0;			// 0.1 sec * 125000 Hz / 511 cycles = 24.5 = 24 cycs

  // Flash ReLED to verify initialization
  P1DIR |= BIT0;				// enable ReLED
  P1OUT |= BIT0;
  __delay_cycles(0xFFFF);
  __delay_cycles(0xFFFF);
  __delay_cycles(0xFFFF);
  P1OUT &= ~BIT0;
  P1DIR &= ~BIT6;				// disable PWM on startup

  __bis_SR_register(CPUOFF);


  while(1){

	  // if ( 100 ms have passed since previous sample ) ...
	  if(sampDataFlag == timerCycles){				// 24*CCR0 = 12288 cycles; 12288/125kHz = 98.1 ms
		  data = readData(gain);			//
		  num2str(data);
		  uart_write_string(0,NUM_LENG+2);		// add one for sign, one for comma
		  P1OUT ^= BIT0;			// Toggle P1.0, visual indicator
		  sampDataFlag = 0;
	  }


	  // if ( command has been received ) ...
	  if(rx_flag == 1){

		  buffer = uart_get_char(0);		// copy received char to buffer variable

		  if(buffer == 'S'){				// Stop command, halt
			  CCR1 = 189;
		  }
		  else if(buffer == 'R'){			// Reverse command
			  CCR1 = 125;
		  }
		  else if(buffer == 'F'){			// Forward command
			  CCR1 = 250;
		  }
		  else if(buffer == 'Q'){			// Quit command
			  P1DIR &= ~BIT6;				// turn off PWM
			  __bis_SR_register(CPUOFF);	// turn off CPU
		  }
		  rx_flag = 0;		// clr rx_flag
	  }
  }
}



/*
 * 				   ----- Timer A0 interrupt -----
 *
 * This interrupt establishes a sampling frequency as determined
 * by TA0CCR0 (CCR0) above. MSP430 will only sample when sampDataFlag
 * reaches the timerCycles variable. This establishes an fs of 10 Hz.
 *
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
  sampDataFlag++;
}



/*
 * 					    ----- Functions -----
 *
 * Local functions related specifically to sending load cell data
 * to the computer. All of the load-cell-data read and interfacing
 * functions are located in the source and headers files "loadCellFunks.x"
 *
 *
 */


/*
 *  === num2str ===
 *
 *  This num2str command converts a signed 24 bit number to a string of
 *  characters which populate the tx_data_str[] buffer. The string is NUM_LENG
 *  characters long plus a sign character and a comma for parsing.
 *
 *  If the number is fewer than 7 digits, zeros are placed before the number
 *  and sign bit as MatLab automatically parses the leading zeros. This also
 *  maintains a consistent tx string length. Ex:
 *
 *  	data = 2450;
 *  	num2str(data) = "00002450,";
 *
 *  	data = -52000;
 *  	num2str(data) = "00-52000,";
 *
 *
 */
void num2str(long int data){
	unsigned char i, negFlag, ndx;
	int shft = sizeof(data)*8;		// length of (long int) in bits

	if(data&(0x00800000)){			// if 24th bit is 1 (num is neg)...
		data = absVal(data);		// get the equivalent positive number
		negFlag = 1;				// set flag indicating data is negative
	}

	for(i=NUM_LENG; i>0; i--){				// begin at end of array, iterate to zero
		tx_data_str[i] = (data % 10)+'0';	// clips LSB and stores to corresponding pos. in str.
		data /= 10;
		if(tx_data_str[i] != '0')(ndx=i);	// if ( number is not zero ) { save ndx of this number }
	}


	if(negFlag == 1){
		tx_data_str[ndx-1] = '-';	// print sign character if data is negative
	}
}


/*
 *  === absVal ===
 *
 *  absolute val for a 24 bit signed number in a 32 bit register.
 *
 *  This function takes the 2's complement of the entire 32 bit number,
 *  then sets the most significant 8 bits to zero. This returns the magnitude
 *  of the negative number
 */

long int absVal(long int num){

	if(data&(0x00800000)){			// only takes 2's comp if num is < 0
		num = ~num;					// undo two's complement
		num++;						// undo two's complement
		num &= ~0xFF000000;			// clears most sig. bits
	}

	return num;
}
