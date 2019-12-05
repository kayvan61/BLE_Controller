/*
 * Buttons.c
 *
 *  Created on: Nov 5, 2019
 *      Author: Henry
 */

#include <stdint.h>
#include "Buttons.h"
#include "tm4c123gh6pm.h"

#define All_BUTTONS 0x3F

//Inits PD0-PD5 as inputs
void Buttons_Init(){
	SYSCTL_RCGCGPIO_R |= 0x8;					// 1) activate clock for Port D
	while((SYSCTL_PRGPIO_R&0x8)==0){};			// 2) allow time for clock to start
	GPIO_PORTD_PCTL_R &= ~(0x00FFFFFF);			// 3) regular GPIO
	GPIO_PORTD_AMSEL_R &= ~(All_BUTTONS);		// 4) disable analog function on PD0-PD5
	GPIO_PORTD_DIR_R &= ~(All_BUTTONS);			// 5) set direction to input
	GPIO_PORTD_AFSEL_R &= ~(All_BUTTONS);		// 6) regular port function
	GPIO_PORTD_DEN_R |= All_BUTTONS;			// 7) enable digital port
}
