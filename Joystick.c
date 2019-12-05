/*
 * Joystick.c
 *
 *  Created on: Nov 5, 2019
 *      Author: Henry
 */

#include <stdint.h>
#include "Joystick.h"
#include "tm4c123gh6pm.h"
#include "UART.h"

//In clock cycles (12.5ns)
#define ADC_PERIOD 8000 // = 100us

void ADC_Init(){
	SYSCTL_RCGCGPIO_R 	|=	0x10;		// 1) activate clock for Port E
	while((SYSCTL_PRGPIO_R&0x10)==0){};	// allow time for clock to start
	GPIO_PORTE_DIR_R 	&=	~(0x0C);	// 2) make PE2,3 input
	GPIO_PORTE_AFSEL_R 	|=	0x0C;		// 3) enable alternate function on PE2,3
	GPIO_PORTE_DEN_R 	&=	~(0x0C);  	// 4) disable digital I/O on PE2,3
	GPIO_PORTE_AMSEL_R 	|=	0x0C; 		// 5) enable analog functionality on PE2,3

	SYSCTL_RCGCADC_R |= 0x01;     	// activate ADC0 
	SYSCTL_RCGCTIMER_R |= 0x01;   	// activate timer0
	while((SYSCTL_RCGCTIMER_R&0x01)==0){};	// allow time for clock to start
	TIMER0_CTL_R = 0x00000000;    	// disable timer0A during setup
	TIMER0_CTL_R |= 0x00000020;   	// enable timer0A trigger to ADC
	TIMER0_CFG_R = 0;             	// configure for 32-bit timer mode
	TIMER0_TAMR_R = 0x00000002;   	// configure for periodic mode, default down-count settings
	TIMER0_TAPR_R = 0;            	// prescale value for trigger
	TIMER0_TAILR_R = ADC_PERIOD-1;	// start value for trigger
	TIMER0_IMR_R = 0x00000000;   	// disable all interrupts
	TIMER0_CTL_R |= 0x00000001;   	// enable timer0A 32-b, periodic, no interrupts

	ADC0_PC_R = 0x01;         		// configure for 125K samples/sec
	ADC0_SSPRI_R = 0x3210;    		// sequencer 0 is highest, sequencer 3 is lowest
	ADC0_ACTSS_R &= ~0x04;    		// disable sample sequencer 2
	ADC0_EMUX_R = (ADC0_EMUX_R&0xFFFFF0FF)+0x500; // timer trigger event
	ADC0_SSMUX2_R = 0x10;			// AIN0 and AIN1
	ADC0_SAC_R = 0x1;				// Enable 2x hardware averaging
	ADC0_SSCTL2_R = 0x0060;			// no TS0 D0 IE0 END0 TS1 D1, yes IE1 END1
	ADC0_IM_R |= 0x0004;			// enable SS2 interrupts
	ADC0_ACTSS_R |= 0x04;			// enable sample sequencer 2

	NVIC_PRI4_R = (NVIC_PRI4_R&0xFFFFFF00)|0x00000040; //priority 2
	NVIC_EN0_R = 1<<16;				// enable interrupt 16 in NVIC
}

volatile uint16_t ADCvalueX;
volatile uint16_t ADCvalueY;
void ADC0Seq2_Handler(){
	ADC0_ISC_R |= 0x4;	//Ack
	ADCvalueX = ADC0_SSFIFO2_R;
	ADCvalueY = ADC0_SSFIFO2_R;
}
