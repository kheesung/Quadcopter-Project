/*
 * MY_function.h
 *
 *  Created on: 2014. 5. 16.
 *      Author: KHJ
 */

#ifndef MY_FUNCTION_H_
#define MY_FUNCTION_H_

//*****************************************************
											   //define
#define PACKET_SIZE 90
//*****************************************************

//*****************************************************
//º¯¼ö
unsigned char data[6] = {0};
//*****************************************************

//*****************************************************
									   //start_port_set
void PORT1_SET(){
	//P1OUT = 0xff;
	//P1IES = 0x20; //interrupt edge select
	//P1IE = 0X20; //interrupt enable
	P1DIR |= 0x0C;
	P1SEL |= 0x0C;
}
void PORT2_SET(){
	//P2OUT = 0xff;
	//P2IES = 0x10; //interrupt edge select
	//P2IE = 0X10; //interrupt enable
	P2DIR |= 0x30;
	P2SEL |= 0x30;
	P2SEL |= BIT7;                            // P2.7 option select
}
void PORT3_SET(){
	//P3SEL |= 0x18;
	P3DIR |= 0x60;//CLCD
	//P3OUT = 0xff;
	P3SEL |= BIT3+BIT4;                       // P3.3,4 option select
}
void PORT4_SET(){
	//P4SEL = 0x18;
	//P4DIR = 0xff;
	//P4OUT = 0xff;
}
void PORT5_SET(){
	//P5SEL = 0x18;
	//P5DIR = 0xff;
	//P5OUT = 0xff;
}
void PORT6_SET(){
	//P6SEL = 0x18;
	P6DIR = 0x1f;//CLCD
	//P6OUT = 0xff;
}
void PORT7_SET(){
	//P7SEL = 0x18;
	P7DIR = 0x01;//CLCD
	//P7OUT = 0xff;
}
void ALL_PORT_SET(){
	PORT1_SET();
	PORT2_SET();
	PORT3_SET();
}
//*****************************************************

//*****************************************************
												//Delay
void Delay_ms(unsigned int Delay_cnt){
	while(Delay_cnt--){
		//__delay_cycles(1048);//12M -> 12000 1M ->1048
		__delay_cycles(12000);
	}
}
void Delay_us(unsigned int Delay_cnt){
	while(Delay_cnt--){
		//__delay_cycles(1);//12M -> 12 1M -> 1
		__delay_cycles(12);
	}
}
//*****************************************************

//*****************************************************
		//UART(Universal Serial Communication Interface)
void USCI_UART_SET(){
	UCA0CTL1 |= UCSWRST;// **Put state machine in reset**
	UCA0CTL0 = 0x00;
	UCA0CTL1 = 0x80; //SMCLK
	UCA0BR0 = 104; // 1MHz 115200 (see User's Guide) 12M -> 104 1M -> 9?
	UCA0BR1 = 0; // 1MHz
	UCA0MCTL = UCBRS_1; // Modln UCBRSx=0, UCBRFx=0,
			 								// over sampling -> different UCA0BR0 value
	UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
	UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupt
}

void USCI_UART_SET1(){
	UCA0CTL1 |= UCSWRST;// **Put state machine in reset**
	UCA0CTL0 = 0x00;
	UCA0CTL1 = 0x80; //SMCLK
	UCA0BR0 = 109; // 1MHz 9600 (see User's Guide)
	UCA0BR1 = 0; // 1MHz
	UCA0MCTL = UCBRS_2; // Modln UCBRSx=0, UCBRFx=0,
			 								// over sampling -> different UCA0BR0 value
	UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
	UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupt
}

//*****************************************************

//*****************************************************
												  //GPS

//*****************************************************



#endif /* MY_FUNCTION_H_ */





















