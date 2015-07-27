/*
 * SensorData.c
 *
 *  Created on: 2015. 1. 7.
 *      Author: KHJ
 */
#include "SensorData1.h"
#include <msp430.h>

char SILENT_MODE[7] = {'s','n','p',0x81,0,0x01,0xd2}; //set silent mode
char BROADCAST_MODE[8] = {'s','n','p',0x82,1,0x00,0x01,0xd4}; // [set 20Hz]      The broadcast frequency is given by f = ((280/255)*transmite rate + 20) Hz <  0(20Hz) <= transmite rate <= 255(300Hz)  >
char GET_DATA[7] = {'s','n','p',0x01,0,0x01,0x52}; // get sensor data

void SET_SILENT_MODE()
{
	int i = 0;
	int k = 2;
	for(i=0;i<7;i++){
		UCA0TXBUF = SILENT_MODE[i];
		while(k--){
				__delay_cycles(12000);
				if(k<0) break;
			}
	}
}

void SET_BROADCAST_MODE()
{
	int i = 0;
	int k = 2;
	for(i=0;i<8;i++){
		UCA0TXBUF = BROADCAST_MODE[i];
		while(k--){
				__delay_cycles(12000);
				if(k<0) break;
			}
	}
}

void GET_DATA_MODE()
{
	int i = 0;
	int k = 2;
	for(i=0;i<7;i++){
		UCA0TXBUF = GET_DATA[i];
		while(k--){
				__delay_cycles(12000);
				if(k<0) break;
			}
	}
}

















