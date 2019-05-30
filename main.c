
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
 * P1.6 -->	500 Hz PWM
 * P1.7	-->	Temp/Humidity sensor
 *
 *
 * Frame sent to MatLab has the structure:
 *
 * 		12345678,12,123,\r\n
 *		   ^     ^   ^
 *		___|	 |   |____
 *	   |		 |   	  |
 *	  Load     Temp    Humidity
 *
 * Where the first eight characters represent a signed value from the
 * load cell, the next two are temperature in C, and the last three
 * are relative humidity*10, i.e. a value of 273 = 27.3% humidity
 *
 */

//includes
#include <msp430.h>
#include "loadCellFunks.h"
#include "serial_handler.h"
#include "thFunks.h"

// defines
#define FRAME_LENGTH  15		// length of max 24-bit number reading (2^(24) = 16777216, 8 chars long)
#define BUFF_LENG	  4		// length of received UART buffer excluding the \n terminator
#define	FULL_STP	  375		// for 50Hz PWM
#define FULL_FOR	  480		// ""
#define FULL_REV 	  250		// ""
//#define	FULL_STP	250		// for 500Hz PWM
//#define FULL_FOR	490		// ""
//#define FULL_REV	10		// ""



// functions
void num2str24(long int);
void num2str(volatile char*);
long int absVal(long int);
void pulseOut(char*);
void pulseOutParabolic(char* cmd);

// global variables
long int data;
unsigned char sampDataFlag = 0, thState = 0, thRefreshFlag = 0;
char buffer[BUFF_LENG];
int loopCounter = 0;


/*
 * mainloop
 */
int main(void){
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // TimerA0 interrupt init
  TA0CCTL0 = CCIE;                         	// CCR0 interrupt enabled
//  CCR0 = 5000;								// 50 Hz PWM
  TA0CCR0 = 500;								// 500 Hz PWM
//  CCR0 = 498;								// 500 hz pwm sync
  TA0CTL = TASSEL_2 | MC_1 | ID_2;       	// SMCLK, upmode, divide by 8 (1MHz / 4 = 250 kHz)

  // PWM init
//  P1DIR |= BIT6;                            // P1.6 output
  P1SEL |= BIT6;                            // P1.6 for TA0.1 output
  P1SEL2 = 0;								// Select default function for P1.6
  TA0CCTL1 = OUTMOD_7;                         // CCR1 reset/set
  TA0CCR1 = FULL_STP;                          // CCR1 PWM duty cycle, init to STOP

  // Temp/Humidity sensor initialization
  TA1CCTL0 = CCIE;                         	// CCR0 interrupt enabled
  TA1CTL = TASSEL_2 | MC_2 | ID_2;       	// SMCLK, contmode, divide by 8 (1MHz / 4 = 250 kHz)
  int error;
  thInit();

  // other intializations
  uart_init(8);							// initialize UART
  loadCellInit();						// initialize pins for load cell
  int gain = HI_GAIN;					// see loadCellFunks.h for different gain levels
  P2DIR |= BIT0;		// enable GrLED
  P2OUT &=~BIT0;

  int timerCycles = 25000/CCR0;			// 0.1 sec * 125000 Hz / 511 cycles = 24.5 = 24 cycs

  // Flash ReLED to verify initialization
  P1DIR |= BIT0;				// enable ReLED
  P1OUT |= BIT0;
  __delay_cycles(0xFFFF);
  __delay_cycles(0xFFFF);
  __delay_cycles(0xFFFF);
  P1OUT &= ~BIT0;
  P1DIR &= ~BIT6;				// disable PWM on startup


  __bis_SR_register(CPUOFF);			// Turn off CPU, wait for start command
//  pulseOut("F000");



  while(1){

	  // if temp/humid sensor is ready to begin
	  if(thState == 0){
		  thStart();
		  thState = 2;		// put into "wait" state
		  TA1CCTL0 |= CCIE;
	  }
	  else if(thState == 1){
		  error = thRead();
		  thRefreshFlag = 1;

		  // if ( timeout or checksum error ) ...
		  if(error == 1){
			  thRefreshFlag = 0;		// do not update; resample
		  }
		  thState = 3;
//			  TA1CCTL0 |= CCIE;
	  }

	  // if ( 100 ms have passed since previous sample ) ...
	  if(sampDataFlag >= timerCycles){				// timerCycles*
		  data = readData(gain);
		  num2str24(data);

		  if(thRefreshFlag == 1){
			  num2str(thBuffer);
			  thRefreshFlag = 0;
		  }
		  else{
			  unsigned char i;
			  for(i=10; i<16; i++){		// sign, 8bits, comma, 2 bits, comma, 3 bits
				  if(i==12){
					  continue;		// do not overwrite comma
				  }
				  tx_data_str[i] = 'X';
				  if(error == 1){
					  tx_data_str[10] = 'E';
					  error=0;
				  }

			  }
		  }

		  uart_write_string(0,FRAME_LENGTH+2);			// add one for sign, one for comma
		  P1OUT ^= BIT0;							// Toggle P1.0, visual indicator
		  sampDataFlag = 0;
	  }


	  // if ( command has been received ) ...
	  if(eos_flag != 0){
		  unsigned char i;

		  for(i=0; i<BUFF_LENG; i++){			// for ( each value in rx_data_str ) ...
			  buffer[i] = uart_get_char(i);			// copy received leading char to buffer variable
		  }

		  if(buffer[0] == 'Q'){					// Quit command
			  P1DIR &= ~BIT6;						// turn off PWM
			  __bis_SR_register(CPUOFF);			// turn off CPU
		  }
		  else if(buffer[0] == 'S' || buffer[0] == 'G'){			// Stop command
			  CCR1 = FULL_STP;						// Stop motors
		  }
		  else{
			  pulseOut(buffer);
//			  pulseOutParabolic(buffer);
		  }

		  eos_flag = 0;		// clr eos_flag
	  }

  }

}



/*
 * 				   ----- Timer0 A0 interrupt -----
 *
 * This interrupt establishes a sampling frequency as determined
 * by TA0CCR0 (CCR0) above. MSP430 will only sample when sampDataFlag
 * reaches the timerCycles variable. This establishes an fs of 10 Hz.
 *
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
{
  sampDataFlag++;
  P1OUT ^= BIT0;
}


/*
 * 				   ----- Timer1 A0 interrupt -----
 *
 * This interrupt sets the state of the temp/humidity sensor. The states
 * are also changed in the mainloop after the thStart() function is set as
 * well as after a sample is taken. The states are detailed in the source file.
 *
 */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A1(void)
{
	// might not work, look into what triggers this interrupt
	switch(thState){
	case 0:				// ready to wake thSensor
		break;
	case 1:				// ready to sample thSensor
		P2OUT ^= BIT0;
		break;
	case 2:				// was waiting for thSensor to wake
		thState = 1;
		break;
	case 3:				// waiting for thSensor to rest
		loopCounter++;
		if(loopCounter == 5){		// 5 loops ~=~ 1.25 seconds, max fs = 1hz
			thState = 0;	// ready to wake sensor up
			loopCounter = 0;
		}
		break;
	default:
		thState = 3;
	}
	TA1CCTL0 &= ~CCIFG;
}


/*
 * -------------------------- Functions --------------------------------
 *
 * Local functions related specifically to sending load cell data
 * to the computer. All of the load-cell-data read and interfacing
 * functions are located in the source and headers files "loadCellFunks.x"
 *
 * ---------------------------------------------------------------------
 *
 */


/*
 *  === num2str24 ===
 *
 *  This num2str24 command converts a signed 24 bit number to a string of
 *  characters which populate the tx_data_str[] buffer. The string is NUM_LENG
 *  characters long plus a sign character and a comma for parsing.
 *
 *  If the number is fewer than 7 digits, zeros are placed before the number
 *  and sign bit as MatLab automatically parses the leading zeros. This also
 *  maintains a consistent tx string length. Ex:
 *
 *  	data = 2450;
 *  	num2str24(data) = "00002450,";
 *
 *  	data = -52000;
 *  	num2str24(data) = "00-52000,";
 *
 */
void num2str24(long int data){
	unsigned char i, negFlag, ndx;
	int shft = sizeof(data)*8;		// length of (long int) in bits

	if(data&(0x00800000)){			// if 24th bit is 1 (num is neg)...
		data = absVal(data);		// get the equivalent positive number
		negFlag = 1;				// set flag indicating data is negative
	}

	for(i=8; i>0; i--){				// begin at end of array, iterate to zero
		tx_data_str[i] = (data % 10)+'0';	// clips LSB and stores to corresponding pos. in str.
		data /= 10;
		if(tx_data_str[i] != '0')(ndx=i);	// if ( number is not zero ) { save ndx of this number }
	}


	if(negFlag == 1){
		tx_data_str[ndx-1] = '-';	// print sign character if data is negative
	}
}


/*
 *  === num2str ===
 *
 *  Converts an array of numbers (configured for use with thBuffer) into their corresponding
 *  string and populates the next 6 characters of tx_send_str[] accordingly.
 *
 */
void num2str(volatile char* thData){

	tx_data_str[9] = ',';

	tx_data_str[11] = thData[0]%10+'0';
	thData[0] /= 10;
	tx_data_str[10] = thData[0]%10+'0';

	tx_data_str[12] = ',';

	tx_data_str[14] = thData[2]%10+'0';
	thData[2] /= 10;
	tx_data_str[13] = thData[2]%10+'0';

	// if decimal value is greater than 10
	if(thData[3]>9)(thData[3]/=10);

	tx_data_str[15] = thData[3]+'0';
	tx_data_str[16] = ',';

}

/*
 *  === absVal ===
 *
 *  absolute val for a 24 bit signed number in a 32 bit register.
 *
 *  This function takes the 2's complement of the entire 32 bit number,
 *  then sets the most significant 8 bits to zero. This returns the magnitude
 *  of the negative number
 *
 */

long int absVal(long int num){

	if(data&(0x00800000)){			// only takes 2's comp if num is < 0
		num = ~num;					// undo two's complement
		num++;						// undo two's complement
		num &= ~0xFF000000;			// clears most sig. bits
	}

	return num;
}


/*
 *  === pulseOut ===
 *
 * 	adjusts the PWM duty cycle according to the buffer variable. From MatLab, a
 * 	string of 4 characters is sent whose firstmost character is a command character
 * 	and the next three characters are a digit percentage
 *
 * 	E.g.: F053 commands the PWM to the forward direction with a duty cycle of 53% of
 * 	its full forward duty cycle. For 50 Hz PWM, 53% = 1.765 ms of uptime.
 *
 * 				rx	  |   command
 * 			 ----------------------
 * 	 			F100  |	 100% forward
 * 				R100  |	 100% reverse
 * 				F000  |	 0%, same as stop
 * 				R032  |	 32% reverse
 */


void pulseOut(char* cmd){
	unsigned char i;
	float pctComm = 0;

	// convert chars to num
	for(i=1; i<BUFF_LENG; i++){
		pctComm *= 10;					// "shift" decimal places left
		pctComm += cmd[i] - '0';			// subtract ASCII offset
	}

	pctComm /= 100;
	float temp;


	if((cmd[0] == 'F') || (cmd[0] == 'G')){
		temp = (FULL_FOR-FULL_STP)*pctComm+FULL_STP;
//		rx_ndx = 0;
		CCR1 = (int) temp;
	}
	else if(cmd[0] == 'R'){
		temp = (FULL_REV-FULL_STP)*pctComm+FULL_STP;
//		rx_ndx = 0;
		CCR1 = (int) temp;
	}


}


void pulseOutParabolic(char* cmd){
	unsigned char i;
	float pctComm = 0;

	// convert chars to num
	for(i=1; i<BUFF_LENG; i++){
		pctComm *= 10;					// "shift" decimal places left
		pctComm += cmd[i] - '0';			// subtract ASCII offset
	}

	pctComm *= pctComm;
	pctComm /= 10000;



	float temp;


	if((cmd[0] == 'F') || (cmd[0] == 'G')){
		temp = (FULL_FOR-FULL_STP)*pctComm+FULL_STP;
//		rx_ndx = 0;
		CCR1 = (int) temp;
	}
	else if(cmd[0] == 'R'){
		temp = (FULL_REV-FULL_STP)*pctComm+FULL_STP;
//		rx_ndx = 0;
		CCR1 = (int) temp;
	}


}
