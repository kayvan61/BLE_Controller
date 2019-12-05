/**
 * main.c
 * Runs on TM4C123
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdlib.h>
#include "SPI.h"
#include "SysTick.h"
#include "UART.h"
#include "Joystick.h"
#include "BLE.h"
#include "Buttons.h"

#define PF1             (*((volatile uint32_t *) 0x40025008))
#define PA6             (*((volatile uint32_t *) 0x40004100))
#define PC4             (*((volatile uint32_t *) 0x40006040))
#define WAIT_AFTER 40000     //500us

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(void);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void PortF_Init(void);
void PortC_Init(void);

uint8_t gb_responseBuf[200];
const uint8_t gb_responseMaxLength = 200;
//sdepMsgResponse_t gb_Response;

uint8_t hwrandom[]  = "+HWRANDOM";
uint8_t bat[]       = "+HWVBAT";

uint8_t dData = 0;

int main(void){
	PLL_Init(Bus80MHz);                   // 80 MHz
	DisableInterrupts();
	SysTick_Init();
//	PortF_Init();
	PortC_Init();
	ADC_Init();
	Buttons_Init();
	UART_Init();

	//Indicate starting init
//	PF1 = 0x02;
    PC4 = 0x10;
    SPI_Init();

//    static const uint8_t setLEDspi[] = "+HWMODELED=SPI";
////    static const uint8_t setLEDspi[] = "i";
//    while(1){
//        SPI_SendATCommand(setLEDspi, sizeof(setLEDspi)-1);
////        while(PA6 != 0x40);
//        SPI_GetResponse(gb_responseBuf, gb_responseMaxLength);
//    }


	//BLE_init();
    BLE_init_default();

//	PF1 = 0;    // turn off LED
	PC4 = 0;
	SysTick_Wait(WAIT_AFTER); //Wait 500us

	EnableInterrupts();

//    SPI_SendArrayCommand(atBatMessage, (sizeof(atBatMessage)/sizeof(atBatMessage[0])));
//    SPI_SendATCommand(hwrandom, sizeof(hwrandom)-1);

	while(1){
//		PF1 ^= 0x02; //Heartbeat
//	    PC4 ^= 0x10; //Heartbeat
	    dData = GPIO_PORTD_DATA_R;
//		WaitForInterrupt();
	}
}


// Make PF1 an output, enable digital I/O, ensure alt. functions off
void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= 0x20;			// 1) activate clock for Port F
	while((SYSCTL_PRGPIO_R&0x20)==0){};	// allow time for clock to start
	GPIO_PORTF_PCTL_R &= ~(0x000000F0);	// 3) regular GPIO
	GPIO_PORTF_AMSEL_R &= ~(0x2);		// 4) disable analog function on PF2, PF4
	GPIO_PORTF_DIR_R |= 0x2;			// 5) set direction to output
	GPIO_PORTF_AFSEL_R &= ~(0x2);		// 6) regular port function
	GPIO_PORTF_DEN_R |= 0x2;			// 7) enable digital port
}

// Make PC4 an output, enable digital I/O, ensure alt. functions off
void PortC_Init(void){
    SYSCTL_RCGCGPIO_R |= 0x4;          // 1) activate clock for Port C
    while((SYSCTL_PRGPIO_R&0x4)==0){}; // allow time for clock to start
    GPIO_PORTC_PCTL_R &= ~(0x000F0000); // 3) regular GPIO
    GPIO_PORTC_AMSEL_R &= ~(0x10);       // 4) disable analog function on PC4
    GPIO_PORTC_DIR_R |= 0x10;            // 5) set direction to output
    GPIO_PORTC_AFSEL_R &= ~(0x10);       // 6) regular port function
    GPIO_PORTC_DEN_R |= 0x10;            // 7) enable digital port
}
