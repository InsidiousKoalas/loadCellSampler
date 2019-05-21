
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
#define END	7		// length of tx string

// functions
//void UART_init();
void num2str(unsigned long int);
//void UART_wd(unsigned long int);
//void UART_ws(char*);
//void UART_read();

// global variables
unsigned long int data;
unsigned char sampDataFlag = 0;
char buffer;


/*
 * mainloop
 */
int main(void){
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // timer interrupt initialization
  CCTL0 = CCIE;                         // CCR0 interrupt enabled
  CCR0 = 512-1;							// set for PWM, for samp is 24*512 cycles
  TACTL = TASSEL_2 | MC_1 | ID_3;       // SMCLK, upmode, divide by 8 (1MHz / 8 = 125 kHz)

  // PWM init
  P1DIR |= BIT6;                            // P1.2 output
  P1SEL |= BIT6;                            // P1.2 for TA0.1 output
  P1SEL2 = 0;								// Select default function for P1.2
  CCTL1 = OUTMOD_7;                         // CCR1 reset/set
  CCR1 = 189;                               // CCR1 PWM duty cycle


  // other intializations
  uart_init(8);							// initialize UART
  loadCellInit();						// initialize pins for load cell

  // Flash ReLED to verify initialization
  P1DIR |= BIT0;
  P1OUT |= BIT0;
  __delay_cycles(0xFFFF);
  __delay_cycles(0xFFFF);
  __delay_cycles(0xFFFF);
  P1OUT &= ~BIT0;

  __bis_SR_register(CPUOFF);

  while(1){

	  if(sampDataFlag == 24){
		  data = readData(HI_GAIN);
		  num2str(data);
		  uart_write_string(0,8);
		  P1OUT ^= BIT0;			// Toggle P1.0, visual indicator
		  sampDataFlag = 0;
	  }


	  if(rx_flag == 1){

//		   test UART rx is working
//		  tx_data_str[0] = uart_get_char(0);
//		  uart_write_string(0,1);

		  buffer = uart_get_char(0);

		  if(buffer == 'S'){
			  strcpy(tx_data_str,"\r\nreceived: S\r\n");
			  uart_write_string(0,15);
			  CCR1 = 189;
		  }
		  else if(buffer == 'R'){
			  strcpy(tx_data_str,"\r\nreceived: R\r\n");
			  uart_write_string(0,15);
			  CCR1 = 125;
		  }
		  else if(buffer == 'F'){
			  strcpy(tx_data_str,"\r\nreceived: F\r\n");
			  uart_write_string(0,15);
			  CCR1 = 250;
		  }
		  else if(buffer == 'Q'){
			  P1DIR &= ~BIT6;
			  __bis_SR_register(CPUOFF);
		  }
		  rx_flag = 0;
	  }
  }
}



/*
 * 				   ----- Timer A0 interrupt -----
 *
 * This interrupt establishes a sampling frequency as determined
 * by TA0CCR0 (CCR0) above. MSP430 will only sample when sampDataFlag
 * is set high (max slave fs = 10Hz)
 *
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
  sampDataFlag += 1;
}



/*
 * 					    ----- Functions -----
 *
 * Local functions related specifically to sending load cell data
 * to the computer. All of the load-cell-data read and interfacing
 * functions are located in the source files.
 *
 */


// UART initialization
void UART_init(){
	  if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
	  {
	    while(1);                               // do not load, trap CPU!!
	  }
	  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
	  DCOCTL = CALDCO_1MHZ;
	  P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	  P1SEL2 = BIT1 + BIT2;
	  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	  UCA0BR0 = 8;                              // 1MHz 115200
	  UCA0BR1 = 0;                              // 1MHz 115200
	  UCA0MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 5
	  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}


//convert data to a string and copy the result into the variable buffer
void num2str(unsigned long int data){
	unsigned char i;

	for(i=0; i<24; i++){
		tx_data_str[i] = '\0';
	}

	for(i=END-1; i>0; i--){				// begin at end of array
		tx_data_str[i] = (data % 10)+'0';	//
		data /= 10;
	}

}


// UART write data
//void UART_wd(unsigned long int data){
//	char *toSend = num2str(data);
//	unsigned char i;
//
//	for(i=0; i<END; i++){
//		while (!(IFG2&UCA0TXIFG));            // USCI_A0 TX buffer ready?
//		UCA0TXBUF = toSend[i];
//	}
//
//	while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//	UCA0TXBUF = ',';
//
//	while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//	UCA0TXBUF = '\r';
//
//	while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//	UCA0TXBUF = '\n';
////	while (!(IFG2&UCA0TXIFG));
////	UCA0TXBUF = buffer;
//}
//
//// UART write string
//void UART_ws(char* txt){
//	int size = sizeof(txt)/sizeof(txt[0]);
//	unsigned char i;
//
//	for(i=0; i<size; i++){
//		while (!(IFG2&UCA0TXIFG));
//		UCA0TXBUF = txt[i];
//	}
//}



void UART_read(){
	buffer = UCA0RXBUF;
}





