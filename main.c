
#include <msp430.h>
#include "MY_function.h"
#include "SensorData1.h"
#include <math.h>


#define MF 20550 //  1MHz(2427=432hz,6990=150hz)  12MHz(12m/2 : 40000=150Hz) -> X  12MHz(12m/4 : 20550=150Hz)


float M1=0,M2=0,M3=0,M4=0;
int i=0,k=0;
int End_Activate=0;
int RX_data=0;
char first_data[42] = {0};
char route_data[15][42] = {0};
int Activate_sel=0;
char RX_gate = 0,RX_cnt=0;
char ak;
char save_gate = 0;
int timer_cnt = 0;
int start_send = 0;
int yaw_data = 0;
int pitch_data = 0;
int roll_data = 0;
float altitude_data = 0;
int start_data = 0;
int start_gate = 0;
int running_save_data = 0;
int sizeof_data = 0;

float standard_PWM_M1 = 0;
float standard_PWM_M2 = 0;
float standard_PWM_M3 = 0;
float standard_PWM_M4 = 0;

float Pitch_d_Temp = 0;				// This stores the old ADC value
float Pitch_i_Temp = 0;				// This stores the accumulated Integral value
float Pitch_PWM_Temp = 166;			// Given an initial value, after that just stores the old value for calculation
float Roll_d_Temp = 0;				// This stores the old ADC value
float Roll_i_Temp = 0;				// This stores the accumulated Integral value
float Roll_PWM_Temp = 166;			// Given an initial value, after that just stores the old value for calculation
float Pitch_sum_data = 0;
float Roll_sum_data = 0;
float PItch_PWM_temp_early = 0;


unsigned long fdata = 0;


void TA02_set();
void TA1_set();
void start_mode();
void Activate();
void Port_set();
void RPM_control(float,float,float,float);
void End_control();
void test();
void SPI_set();
void save_data(char first_data[]);
void substitution(char first_data[]);
void UCSinit();
void move_data();
void send();
void init_set();
void get_yaw_pitch_roll();
void Pitch_PID();
void Roll_PID();

void main()
{
	init_set();

	start_data = 1;

	while(1){
		if(start_gate==0){
			Activate();
			start_gate = 1;
		}
		if(start_data==1) break;
	}


	while(1){
		M1=M3=M2=M4=0.015;
		RPM_control(0.015,0.015,0.015,0.015);
		//if(pitch_data < 180 && pitch_data > -180) Pitch_PID();
		//if(roll_data < 180 && roll_data > -180) Roll_PID();
/*
		if(Pitch_sum_data<0){
			if(Roll_sum_data<0){
				RPM_control(
						M1+Pitch_PWM_Temp/4+Roll_PWM_Temp/8,
						M1+Pitch_PWM_Temp/4+Roll_PWM_Temp/4,
						M3+Pitch_PWM_Temp/8+Roll_PWM_Temp/4,
						M4+Pitch_PWM_Temp/8+Roll_PWM_Temp/8);
			}
			else{
				RPM_control(
						M1+Pitch_PWM_Temp/4+Roll_PWM_Temp/4,
						M1+Pitch_PWM_Temp/4+Roll_PWM_Temp/8,
						M3+Pitch_PWM_Temp/8+Roll_PWM_Temp/8,
						M4+Pitch_PWM_Temp/8+Roll_PWM_Temp/4);
			}
		}
		else{
			if(Roll_sum_data<0){
				RPM_control(
						M1+Pitch_PWM_Temp/8+Roll_PWM_Temp/8,
						M1+Pitch_PWM_Temp/8+Roll_PWM_Temp/4,
						M3+Pitch_PWM_Temp/4+Roll_PWM_Temp/4,
						M4+Pitch_PWM_Temp/4+Roll_PWM_Temp/8);
			}
			else{
				RPM_control(
						M1+Pitch_PWM_Temp/8+Roll_PWM_Temp/4,
						M1+Pitch_PWM_Temp/8+Roll_PWM_Temp/8,
						M3+Pitch_PWM_Temp/4+Roll_PWM_Temp/8,
						M4+Pitch_PWM_Temp/4+Roll_PWM_Temp/4);
			}
		}
*/

/*
		if(Pitch_sum_data<0){
			RPM_control(M1+PWM_Duty/2,M1+PWM_Duty/2,M3+PWM_Duty/4,M4+PWM_Duty/4);
		}
		else{
			RPM_control(M1+PWM_Duty/4,M2+PWM_Duty/4,M3+PWM_Duty/2,M4+PWM_Duty/2);
		}

		if(Roll_sum_data<0){
			RPM_control(M1+PWM_Duty/2,M1+PWM_Duty/4,M3+PWM_Duty/4,M4+PWM_Duty/2);
		}
		else{
			RPM_control(M1+PWM_Duty/4,M2+PWM_Duty/2,M3+PWM_Duty/2,M4+PWM_Duty/4);
		}
*/
	}

/*
	while(1) test();

	while(1)
	{
		if(start_send==1){
			start_send = 0;
			//send();
		}
	}
	*/
}

void init_set()
{
	WDTCTL = WDTPW + WDTHOLD;

	ALL_PORT_SET();
	TA1_set();
	UCSinit();
	TA02_set();
	//SPI_set();
	USCI_UART_SET();
	__enable_interrupt();

	SET_SILENT_MODE(); //set Silent mode
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
	/*
	if(timer_cnt==1){
		__enable_interrupt();
		timer_cnt = 0;
	}
	else if(timer_cnt==0){
		__disable_interrupt();
		start_send = 1;
		timer_cnt = 1;
	}
	*/
	GET_DATA_MODE();
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,4))
  {
    case 0:break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG
      while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
      save_data(first_data);
      //substitution(first_data);
      break;
    case 4:break;                             // Vector 4 - TXIFG
    default: break;
  }
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  switch(__even_in_range(UCB0IV,4))
  {
    case 0:break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG
      while (!(UCB0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
      //save_data(first_data);
      //substitution(first_data);
      break;
    case 4:break;                             // Vector 4 - TXIFG
    default: break;
  }
}

void send()
{
	int i=0;
	//USCI_UART_SET1();
	for(i=0;i<42;i++){
		UCA0TXBUF = first_data[i];
		Delay_ms(3);
	}
	UCA0TXBUF = 10; // \n
	Delay_ms(3);
	UCA0TXBUF = 13; // \r
	Delay_ms(3);
	USCI_UART_SET();
}

void get_yaw_pitch_roll()
{


	yaw_data &= 0x00;
	yaw_data = first_data[4]<<8;
	yaw_data |= first_data[5];
	yaw_data += 180;

	pitch_data &= 0x00;
	pitch_data = first_data[6]<<8;
	pitch_data |= first_data[7];
	//pitch_data += 180;

	roll_data &= 0x00;
	roll_data = first_data[8]<<8;
	roll_data |= first_data[9];
	//roll_data += 180;

	//altitude_data = 0x0000;
	//altitude_data = (first_data[18]<<24) + (first_data[19]<<16) + (first_data[20]<<8) + first_data[21];
	//altitude_data = first_data[18];
	//altitude_data = (int)altitude_data<<8 + first_data[19];
	//altitude_data = (int)altitude_data<<8 + first_data[20];
	//altitude_data = (int)altitude_data<<8 + first_data[21];
	//altitude_data = 0x0003713f;
	//fdata = 0x01;
	fdata = first_data[14];
	fdata = (fdata<<8) + first_data[15];
	fdata = (fdata<<8) + first_data[16];
	fdata = (fdata<<8) + first_data[17];
	altitude_data = *(float*)&fdata;

}

void save_data(char first_data[])
{
	if(UCA0RXBUF=='s'){
		RX_data = 1;
	}
	else if(UCA0RXBUF=='n' && RX_data==1){
		RX_data = 2;
	}
	else if(UCA0RXBUF=='p' && RX_data==2){
		RX_data = 3;
		RX_cnt = 0;
		//move_data();
	}

	if(RX_data==3 && UCA0RXBUF!='p'){
		first_data[RX_cnt]=UCA0RXBUF;
		if(RX_cnt>=41){
			save_gate=1;
			//__disable_interrupt();
			get_yaw_pitch_roll();
			running_save_data = 0;
			RX_data = 0;
			//RX_cnt = 0;
		}
		RX_cnt++;
	}


	/*

	ak=UCA0RXBUF;
	if(UCA0RXBUF=='$'||RX_gate){
		//UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
		first_data[RX_cnt] = UCA0RXBUF; //input packet data
		RX_gate = 1;
		RX_cnt++;
		if(UCA0RXBUF=='*')
		{
			RX_cnt = 0;
			RX_gate = 0;
		}
	}
	UCA0TXBUF = 'P';
	*/
}
void Pitch_PID()
{
	//Local variables for PID
	//__disable_interrupt();
	const float Kp = 0.0007;		// The value for Proportional gain
	const float Ki = 0.00007;		// The value for Integral gain
	const float Kd = 0.0001;	// The value for Differential gain

	int Set_Point = 0;	// The ADC reference point we are aiming to regulate to
	float iMax = 180;			// Used to prevent integral wind-up
	float iMin = -180;		// Used to prevent integral wind-up
	float Err_Value;			// Holds the calculated Error value
	float P_Term;			// Holds the calculated Proportional value
	float I_Term;			// Holds the calculated Integral value
	float D_Term;			// Holds the calculated Differential value

	float PWM_Duty;			// Holds the new PWM value
	//int new_ADC_value;		// Holds the new ADC value



	// More efficient to read this once and store as used 3 times
	//new_ADC_value = read_ADC();

	Err_Value = (Set_Point - pitch_data);

	// This calculates Proportional value, Kp is multiplied with Err_Value and the result is assigned to P_Term
	P_Term = Kp * Err_Value;

	// Prepare Integral value, add the current error value to the integral value and assign the total to i_Temp
	Pitch_i_Temp += Err_Value;

	// Prevents integral wind-up, limits i_Temp from getting too positive or negative
	if (Pitch_i_Temp > iMax)
	{Pitch_i_Temp = iMax;}
	else if (Pitch_i_Temp < iMin)
	{Pitch_i_Temp = iMin;}

	// Calculates the Integral value, Ki is multiplied with i_Temp and the result is assigned to I_Term
	I_Term = Ki * Pitch_i_Temp;


	// Calculates Differential value, Kd is multiplied with (d_Temp minus new_ADC_value) and the result is assigned to D_Term
	// The new_ADC_value will become the old ADC value on the next function call, this is assigned to d_Temp so it can be used
	D_Term = Kd * (Pitch_d_Temp - Err_Value);
	Pitch_d_Temp = Err_Value;

	/****** Now we have the P_Term, I_Term and D_Term *****/
	Pitch_sum_data = (P_Term + I_Term + D_Term);
	//PWM_Duty = Pitch_PWM_Temp - (P_Term + I_Term + D_Term);
	PWM_Duty = fabs(Pitch_sum_data);

	// PWM overflow prevention

	if (PWM_Duty < 0.012)
	{PWM_Duty = 0.012;}
	else if (PWM_Duty > 0.05)
	{PWM_Duty = 0.05;}

	if(Pitch_sum_data<0){
		RPM_control(M1+PWM_Duty/2,M1+PWM_Duty/2,M3+PWM_Duty/4,M4+PWM_Duty/4);
	}
	else{
		RPM_control(M1+PWM_Duty/4,M2+PWM_Duty/4,M3+PWM_Duty/2,M4+PWM_Duty/2);
	}

	// Adjusts the PWM duty cycle
	//adjust_PWM(PWM_Duty);
	// Assigns the current PWM duty cycle value to PWM_Temp
	PItch_PWM_temp_early = Pitch_PWM_Temp;
	Pitch_PWM_Temp = PWM_Duty;
	//__enable_interrupt();
}

void Roll_PID()
{
	//Local variables for PID
	//__disable_interrupt();
	const float Kp = 0.0007;		// The value for Proportional gain
	const float Ki = 0.00007;		// The value for Integral gain
	const float Kd = 0.0001;	// The value for Differential gain

	int Set_Point = 0;	// The ADC reference point we are aiming to regulate to
	float iMax = 180;			// Used to prevent integral wind-up
	float iMin = -180;		// Used to prevent integral wind-up
	float Err_Value;			// Holds the calculated Error value
	float P_Term;			// Holds the calculated Proportional value
	float I_Term;			// Holds the calculated Integral value
	float D_Term;			// Holds the calculated Differential value
	//int new_ADC_value;		// Holds the new ADC value
	float PWM_Duty;			// Holds the new PWM value


	// More efficient to read this once and store as used 3 times
	//new_ADC_value = read_ADC();

	Err_Value = (Set_Point - roll_data);

	// This calculates Proportional value, Kp is multiplied with Err_Value and the result is assigned to P_Term
	P_Term = Kp * Err_Value;

	// Prepare Integral value, add the current error value to the integral value and assign the total to i_Temp
	Roll_i_Temp += Err_Value;

	// Prevents integral wind-up, limits i_Temp from getting too positive or negative
	if (Roll_i_Temp > iMax)
	{Roll_i_Temp = iMax;}
	else if (Roll_i_Temp < iMin)
	{Roll_i_Temp = iMin;}

	// Calculates the Integral value, Ki is multiplied with i_Temp and the result is assigned to I_Term
	I_Term = Ki * Roll_i_Temp;


	// Calculates Differential value, Kd is multiplied with (d_Temp minus new_ADC_value) and the result is assigned to D_Term
	// The new_ADC_value will become the old ADC value on the next function call, this is assigned to d_Temp so it can be used
	D_Term = Kd * (Roll_d_Temp - Err_Value);
	Roll_d_Temp = Err_Value;

	/****** Now we have the P_Term, I_Term and D_Term *****/
	Roll_sum_data = (P_Term + I_Term + D_Term);
	//PWM_Duty = Roll_PWM_Temp - (P_Term + I_Term + D_Term);
	PWM_Duty = fabs(Roll_sum_data);

	// PWM overflow prevention
	if (PWM_Duty < 0.012)
	{PWM_Duty = 0.012;}
	else if (PWM_Duty > 0.05)
	{PWM_Duty = 0.05;}


	if(Roll_sum_data<0){
		RPM_control(M1+PWM_Duty/2,M1+PWM_Duty/4,M3+PWM_Duty/4,M4+PWM_Duty/2);
	}
	else{
		RPM_control(M1+PWM_Duty/4,M2+PWM_Duty/2,M3+PWM_Duty/2,M4+PWM_Duty/4);
	}



	/*
	if (PWM_Duty < 0.012)
	{PWM_Duty = 0.012;}
	else if (PWM_Duty > 0.25)
	{PWM_Duty = 0.25;}

	//
	if(Pitch_sum_data<0){
		RPM_control(PWM_Duty,PWM_Duty,M3,M4);
	}
	else{
		RPM_control(M1,M2,PWM_Duty,PWM_Duty);
	}
	*/
	// Adjusts the PWM duty cycle
	//adjust_PWM(PWM_Duty);
	// Assigns the current PWM duty cycle value to PWM_Temp
	Roll_PWM_Temp = PWM_Duty;
	//__enable_interrupt();
}
void move_data()
{
	int i = 0,k = 0;

	for(i=13;i>0;i--){
		for(k=0;k<42;k++){
			route_data[i+1][k]=route_data[i][k];
		}
	}
	for(i=0;i<42;i++){
		route_data[0][i] = first_data[i];
	}
}
void substitution(char first_data[])
{
	if(first_data[5]=='1') Activate_sel=1;
}

void SPI_set()
{
	UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
	UCA0CTL0 |= UCSYNC+UCCKPL+UCMSB;          // 3-pin, 8-bit SPI slave,
											// Clock polarity high, MSB
	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void Activate()
{
	//RPM_control(0.008,0.008,0.008,0.008);
	RPM_control(0.1,0.1,0.1,0.1);
	Delay_ms(200);

	M1=M3=M2=M4=0.15;
	RPM_control(M1,M2,M3,M4);
	for(i=2;i>0;i--)Delay_ms(1000);

	M1=M3=M2=M4=0.008;
	RPM_control(M1,M2,M3,M4);
	for(i=10;i>0;i--)Delay_ms(1000);

	End_Activate=1;
}

void test(){

	M1=M3=0.008;
	M2=M4=0.008;
	RPM_control(M1,M2,M3,M4);
	for(i=3;i>0;i--)Delay_ms(1000);

	//M1=M3=M2=M4=0.02;
	//RPM_control(M1,M2,M3,M4);
	//for(i=4;i>0;i--)Delay_ms(1000);

	for(i=5;i>0;i--)Delay_ms(1000);
	//End_control();

}

void End_control(){
	M1=M3=0.0;
	M2=M4=0.0;
	RPM_control(M1,M2,M3,M4);
}

void start_mode(){

}

void TA02_set(){
	TA0CTL = TASSEL_2 + MC_1 + TACLR + ID_2;         // SMCLK, up_mode, clear TAR
	TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_2;
	TA0CCR0 = MF;
	TA2CCR0 = MF;

	TA0CCTL1 = OUTMOD_7;
	TA0CCTL2 = OUTMOD_7;
	TA2CCTL1 = OUTMOD_7;
	TA2CCTL2 = OUTMOD_7;
	RPM_control(0,0,0,0);
}

void TA1_set(){
	TA1CCTL0 = CCIE;
	TA1CCR0 = 1024;
	TA1CTL = TASSEL_1 + MC_1 + TACLR;         // ACLK, upmode, clear TAR
}

void RPM_control(float M1,float M2,float M3,float M4){
	TA2CCR2 = (int)(MF)*(M4);
	TA2CCR1 = (int)(MF)*(M3);
	TA0CCR2 = (int)(MF)*(M2);
	TA0CCR1 = (int)(MF)*(M1);
}

void UCSinit()
{
	UCSCTL3 |= SELREF_2;                    // Set DCO FLL reference = REFO
	UCSCTL4 |= SELA_2;                      // Set ACLK = REFO

	__bis_SR_register(SCG0);                // Disable the FLL control loop
	UCSCTL0 = 0x0000;                       // Set lowest possible DCOx, MODx
	UCSCTL1 = DCORSEL_5;                    // Select DCO range 24MHz operation
	UCSCTL2 = FLLD_1 + 374;                 // Set DCO Multiplier for 12MHz
											// (N + 1) * FLLRef = Fdco
											// (374 + 1) * 32768 = 12MHz
											// Set FLL Div = fDCOCLK/2
	__bic_SR_register(SCG0);                // Enable the FLL control loop
}


/*
#include <msp430.h>
#include "MY_function.h"

#define MF 6990 //2427=432,6990=150

void TA02_set();
void TA1_set();
void start_mode();
void Activate();
void Port_set();
void RPM_control(float,float,float,float);

float M1=0,M2=0,M3=0,M4=0;
int i=0;

void main()
{
WDTCTL = WDTPW + WDTHOLD;

ALL_PORT_SET();

Activate();

while(1)
{
start_mode();
}
}
void Activate()
{
TA02_set();
//RPM_control(1,0.5,0.25,0.0);

for(i=1;i>0;i--)Delay_ms(1000);

RPM_control(0.1,0.1,0.1,0.1);

for(i=3;i>0;i--)Delay_ms(1000);
Delay_ms(450);
Delay_ms(450);

M1=M3=0.15;
M2=M4=0.15;
RPM_control(M1,M2,M3,M4);
for(i=3;i>0;i--)Delay_ms(1000);

M1=M3=0.008;
M2=M4=0.008;
RPM_control(M1,M2,M3,M4);
//for(i=5;i>0;i--)Delay_ms(1000);
Delay_ms(150);

M1=M3=0.186;
M2=M4=0.186;
RPM_control(M1,M2,M3,M4);
//for(i=5;i>0;i--)Delay_ms(1000);
Delay_ms(400);

M1=M3=0.16;
M2=M4=0.16;
RPM_control(M1,M2,M3,M4);
for(i=5;i>0;i--)Delay_ms(1000);

M1=M3=0.18;
M2=M4=0.18;
RPM_control(M1,M2,M3,M4);
for(i=2;i>0;i--)Delay_ms(1000);
/*
M1=M3=0.19;
M2=M4=0.19;
RPM_control(M1,M2,M3,M4);
for(i=2;i>0;i--)Delay_ms(1000);

M1=M3=0.20;
M2=M4=0.20;
RPM_control(M1,M2,M3,M4);
for(i=2;i>0;i--)Delay_ms(1000);

M1=M3=0.22;
M2=M4=0.22;
RPM_control(M1,M2,M3,M4);
for(i=5;i>0;i--)Delay_ms(1000);

M1=M3=0.19;
M2=M4=0.19;
RPM_control(M1,M2,M3,M4);
for(i=2;i>0;i--)Delay_ms(1000);

M1=M3=0.18;
M2=M4=0.18;
RPM_control(M1,M2,M3,M4);
for(i=2;i>0;i--)Delay_ms(1000);


M1=M3=0.0;
M2=M4=0.0;
RPM_control(M1,M2,M3,M4);
for(i=2;i>0;i--)Delay_ms(1000);

}
void start_mode()
{

}
void TA02_set()
{
TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up_mode, clear TAR
TA2CTL = TASSEL_2 + MC_1 + TACLR;
TA0CCR0 = MF;
TA2CCR0 = MF;

TA0CCTL1 = OUTMOD_7;
TA0CCTL2 = OUTMOD_7;
TA2CCTL1 = OUTMOD_7;
TA2CCTL2 = OUTMOD_7;
RPM_control(0,0,0,0);
}
void TA1_set()
{
TA1CTL = TASSEL_2 + MC_1 + TACLR;
}
void RPM_control(float M1,float M2,float M3,float M4)
{
TA2CCR2 = (int)(MF)*(M4);
TA2CCR1 = (int)(MF)*(M3);
TA0CCR2 = (int)(MF)*(M2);
TA0CCR1 = (int)(MF)*(M1);
}

*/




































/*
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
	switch(TA0IV)
	{
	case 0x02:
		break;
	case 0x04:
		break;
	case 0x06:
		break;
	case 0x08:
		break;
	case 0x0A:
		break;
	case 0x0C:
		break;
	case 0x0E:
		break;
	}
}

#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR(void)
{
	switch(TA2IV)
	{
	case 0x02:
		break;
	case 0x04:
		break;
	case 0x06:
		break;
	case 0x08:
		break;
	case 0x0A:
		break;
	case 0x0C:
		break;
	case 0x0E:
		break;
	}
}
*/
