
/*
 * (c) Brandon Hunt, Charles Rawlins 2019
 *
 * loadCellSampler
 *
 * The purpose of this code is to sample the load cell (over SPI)
 * and send the data to a computer via UART.
 *
 * P1.5 --> SDI
 * P1.4 --> CLK
 *
 * P1.3 --> ADC (battery voltage)
 *
 * P1.2	-->	TX (RX on computer side)
 * P1.1 --> RX (TX on computer side)
 *
 * P1.6 -->	500 Hz PWM for ESC
 * P1.7	-->	Temp/Humidity sensor
 *
 * P2.2 --> 50 Hz PWM for spray
 *
 *
 * Frame sent to MatLab has the structure:
 *
 * 		12345678,123,123,1234,\r\n
 *		   ^     ^    ^	    ^
 *		___|	 |    |___	|______
 *	   |		 |        |		   |
 *	  Load     Temp    Humidity	 Voltage
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
#define FRAME_LENGTH  22		// length of max 24-bit number reading (2^(24) = 16777216, 8 chars long)
#define BUFF_LENG	  4		// length of received UART buffer excluding the \n terminator
#define	FULL_STP	  375		// for 50Hz PWM
#define FULL_FOR	  480		// ""
#define FULL_REV 	  250		// ""
#define SPRAY_ON      250
#define SPRAY_OFF     500


// functions
void num2str24(long int);
void th2str(volatile char*);
void volt2str(float);
long int absVal(long int);
void pulseOut(char*);
void pulseOutParabolic(char* cmd);

// global variables
long int data;
unsigned char sampDataFlag = 0, thState = 0, thRefreshFlag = 0;
char buffer[BUFF_LENG], DHT_REST[2] = {5,10}, TH_REST_ST = 0;
int loopCounter = 0;
float adcMem, voltage;


/*
 * mainloop
 */
int main(void){
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // TimerA0 interrupt init
  TA0CCTL0 = CCIE;                         	// CCR0 interrupt enabled
//  CCR0 = 5000;							// 50 Hz PWM
  TA0CCR0 = 500;							// 500 Hz PWM
//  CCR0 = 498;								// 500 hz pwm sync
  TA0CTL = TASSEL_2 | MC_1 | ID_2;       	// SMCLK, upmode, divide by 8 (1MHz / 4 = 250 kHz)

  // PWM init
  P1SEL |= BIT6;                            // P1.6 for TA0.1 output
  P1SEL2 = 0;								// Select default function for P1.6
  TA0CCTL1 = OUTMOD_7;                      // CCR1 reset/set
  TA0CCR1 = FULL_STP;                       // CCR1 PWM duty cycle, init to STOP

  // Temp/Humidity sensor initialization
//  TA1CCTL0 = CCIE;                         	// CCR0 interrupt enabled
  int error;
  int count;
  thInit();

  // 50 Hz PWM init
  TA1CTL = TASSEL_2 | MC_1 | ID_2;          // SMCLK, upmode, divide by 4 (1MHz / 4 = 250 kHz)
  P2SEL |= BIT2;
  P2SEL2 = 0;
  P2DIR |= BIT2;
  TA1CCTL1 = OUTMOD_7;
  TA1CCR0 = 5000;
  TA1CCR1 = SPRAY_OFF;

  // Port interrupt (DHT sample frequency selector)
  P1REN |= BIT3;                   // Enable internal pull-up/down resistors
  P1OUT |= BIT3;                   //Select pull-up mode for P1.3
  P1IE |= BIT3;                    // P1.3 interrupt enabled
  P1IES |= BIT3;                   // P1.3 Hi/lo edge
  P1IFG &= ~BIT3;                  // P1.3 IFG cleared

  // ADC initialization
  ADC10CTL0 = ADC10SHT_2 + ADC10ON + ADC10IE;   // ADC10ON, interrupt enabled
  ADC10CTL1 = INCH_3;                           // input A3
  ADC10AE0 |= 0x08;                             // P1.3 ADC option select


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


  th2str(thBuffer);		// init tx string
  __bis_SR_register(CPUOFF);			// Turn off CPU, wait for start command
//  pulseOut("F000");



  while(1){

	  // if temp/humid sensor is ready to begin
	  if(thState == 0){
		  count = thStart();
		  thState = 2;		// put into "wait" state
//		  TA1CCTL0 |= CCIE;
	  }
	  else if(thState == 1){
		  error = thRead();
		  thRefreshFlag = 1;

		  // if ( timeout or checksum error ) ...
		  if(error == 1){
			  thRefreshFlag = 0;		// do not update; resample
		  }
		  thState = 3;
		  count = TA1R + 65535;         // rest time for DHT
//			  TA1CCTL0 |= CCIE;
	  }

	  // if ( 100 ms have passed since previous sample ) ...
	  if(sampDataFlag >= timerCycles){				// timerCycles*
		  data = readData(gain);
//		  data = 0x0000;
		  num2str24(data);

		  // update voltage (auto-populates string, see ISR)
		  ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start


		  // if th data is new, refresh tx_str; else, replace with Xs and leave ADC voltage
		  if(thRefreshFlag == 1){
			  th2str(thBuffer);
			  thRefreshFlag = 0;
		  }
		  else{
			  unsigned char i;
			  for(i=10; i<18; i++){		// sign, 8bits, comma, 2 bits, comma, 3 bits
				  if(i==13){
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
			  TA0CCR1 = FULL_STP;						// Stop motors
		  }
		  else if(buffer[0] == 'M' || buffer[0] == 'N'){   // spray command
		      if(buffer[0] == 'M')(TA1CCR1 = SPRAY_ON);
		      else(TA1CCR1 = SPRAY_OFF);
		  }
		  else{
			  pulseOut(buffer);
//			  pulseOutParabolic(buffer);
		  }

		  eos_flag = 0;		// clr eos_flag
	  }

  }

  // check states of DHT
  if((thState==2)&&(TA1R >= count))(thState = 1);       // if ready, sample DHT22
  else if((thState==3)&&(TA1R>=count)){
      loopCounter++;
      if(loopCounter==3){
          thState = 0;
      }
  }


}       // <------- end while loop



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
//	// might not work, look into what triggers this interrupt
//	switch(thState){
//	case 0:				// ready to wake thSensor
//		break;
//	case 1:				// ready to sample thSensor
//		P2OUT ^= BIT0;
//		break;
//	case 2:				// was waiting for thSensor to wake
//		thState = 1;
//		break;
//	case 3:				// waiting for thSensor to rest
//		loopCounter++;
//		if(loopCounter >= DHT_REST[TH_REST_ST]){		// 10 loops ~=~ 2.5 seconds, max fs = 0.5hz
//			thState = 0;	// ready to wake sensor up
//			loopCounter = 0;
//		}
//		break;
//	default:
//		thState = 3;
//	}
//	TA1CCTL0 &= ~CCIFG;
}



// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
   TH_REST_ST ^= 0x01;				   // Toggle sampling frequency (oversample / undersample)
   P1IFG &= ~BIT3;                     // P1.3 IFG cleared
}

// ADC10 interrupt service routine -- measure battery voltage
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
	adcMem = ADC10MEM;
	voltage = 14.4*(adcMem/894);		// voltage divider with max voltage of 14.4 V means max adc val is 894
	volt2str(voltage);
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
 *  === th2str ===
 *
 *  Converts an array of numbers (configured for use with thBuffer) into their corresponding
 *  string and populates the next 6 characters of tx_send_str[] accordingly.
 *
 */
void th2str(volatile char* thData){
	int temp[2] = { 0 };

	temp[0] = (int) thData[0]<<8;
	temp[0] |= thData[1];
	temp[1] = (int) thData[2]<<8;
	temp[1] |= thData[3];


	// RH data
	tx_data_str[9] = ',';

	tx_data_str[12] = temp[0]%10+'0';
	temp[0] /= 10;
	tx_data_str[11] = temp[0]%10+'0';
	temp[0] /= 10;
	tx_data_str[10] = temp[0]%10+'0';

	tx_data_str[13] = ',';

	// Temp Data - check negtative
	if(temp[1]&0x8000){
		tx_data_str[14] = '-';
		temp[1] ^= 0x8000;
	}
	else{
		tx_data_str[14] = '0';
	}

	tx_data_str[17] = temp[1]%10+'0';
	temp[1] /= 10;
	tx_data_str[16] = temp[1]%10+'0';
	temp[1] /= 10;
	tx_data_str[15] = temp[1]+'0';
	tx_data_str[18] = ',';

}

void volt2str(float voltage){
	int vt = voltage*100;

	tx_data_str[23] = ',';		// end of string
	tx_data_str[22] = vt%10+'0';
	vt /= 10;
	tx_data_str[21] = vt%10+'0';
	vt /= 10;
	tx_data_str[20] = vt%10+'0';
	vt /= 10;
	tx_data_str[19] = vt%10+'0';
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

	// determine CCR1 for given PWM value
	if((cmd[0] == 'F') || (cmd[0] == 'G')){
		temp = (FULL_FOR-FULL_STP)*pctComm+FULL_STP;
		CCR1 = (int) temp;
	}
	else if(cmd[0] == 'R'){
		temp = (FULL_REV-FULL_STP)*pctComm+FULL_STP;
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
