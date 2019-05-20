
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

// defines
#define END	8

// functions
void UART_init();
void num2str(long int);

// global variables
long int data;
char buffer[END] = { "0" };
unsigned char i, dataFlag=0;


/*
 * mainloop
 */
int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  UART_init();
  loadCellInit();

  // Flash ReLED to verify initialization
  P1DIR &= ~BIT0;
  P1OUT |= BIT0;
  __delay_cycles(0xFFFF);
  __delay_cycles(0xFFFF);
  __delay_cycles(0xFFFF);
  P1OUT &= ~BIT0;

  // enable interrupts
  __bis_SR_register(GIE);


  while(1){
	  data = readData(HI_GAIN);
	  num2str(data);
	  __bis_SR_register(GIE);
  }
}


/*
 * UART interrupt
 *
 * send load cell data to computer for plotting/digestion
 *
 */
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
	while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
	UCA0TXBUF = buffer[i];
	i++;
	if(i>END){
		dataFlag = 0;
		i = 0;
		__bis_SR_register(GIE);
	}
}


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
	  IE2 |= UCA0TXIE;                          // Enable USCI_A0 RX interrupt
}


//convert data to a string and copy the result into the variable buffer
void num2str(long int data){
	char string[END], i;

	for(i=0; i<END; i++){
		string[i] = (data % 10)+'0';	//
		data /= 10;
	}

	string[END-1] = ',';

	strcpy(buffer, string);
}







