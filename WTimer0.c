// WTimer0.c
// Runs on LM4F120/TM4C123
// Use Timer3 in 32-bit periodic mode to request interrupts at a periodic rate
// Daniel Valvano
// September 20, 2018

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
  Program 7.5, example 7.6

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"

void (*PeriodicTask0)(void);   // user function

// ***************** Timer0_Init ****************
// Activate Timer0 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void WTimer0_Init(void(*task)(void), uint32_t period){
//    long sr = StartCritical();
    SYSCTL_RCGCWTIMER_R |= 0x01;   // 0) activate TIMER0
    while((SYSCTL_RCGCWTIMER_R&0x01)==0){};   // allow time for clock to start
    PeriodicTask0 = task;         // user function
    WTIMER0_CTL_R = 0x00000000;    // 1) disable TIMER0 during setup
    WTIMER0_CFG_R = 0x00000000;    // 2) configure for 64-bit mode
    WTIMER0_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
    WTIMER0_TAILR_R = period-1;    // 4) reload value
    WTIMER0_TAPR_R = 0;            // 5) bus clock resolution
    WTIMER0_ICR_R = 0x00000001;    // 6) clear TIMER0 timeout flag
    WTIMER0_IMR_R = 0x00000001;    // 7) arm timeout interrupt
    NVIC_PRI23_R = (NVIC_PRI23_R & 0xFF00FFFF)|0x00800000;  // 8) priority 4
    NVIC_EN2_R = (0x1 << (94-64));  // 9) enable interrupt 94 in NVIC

    WTIMER0_CTL_R = 0x00000001;    // 10) enable TIMER4A
//    EndCritical(sr);
}

void WideTimer0A_Handler(void){
    WTIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
    (*PeriodicTask0)();               // execute user task
}
