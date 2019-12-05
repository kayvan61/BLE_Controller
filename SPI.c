/*
 * SPI.c
 *
 *  Created on: Oct 19, 2019
 *      Author: Henry
 */

#include "SPI.h"
#include <stdint.h>
#include "UART.h"
#include "SDEP.h"
#include "SysTick.h"
#include "tm4c123gh6pm.h"
//#include "FIFOsimple.h"

//Pin 2,4,5 = 0x34
//Pin 3,7 = 0x88

#define PA3             (*((volatile uint32_t *) 0x40004020))
#define PA6             (*((volatile uint32_t *) 0x40004100))
#define PA7             (*((volatile uint32_t *) 0x40004200))

void SPI_Init(void){
	//Configure PA6 as input and an interrupt, and  PA7,PA3 as an output
	SYSCTL_RCGCGPIO_R |= 0x1;	//Activate GPIO port A
	while((SYSCTL_PRGPIO_R & 0x1) == 0);	//Wait for port A to activate
	GPIO_PORTA_PCTL_R &= ~(0xFF00F000); //Regular GPIO on PA3,PA6,PA7
	GPIO_PORTA_DIR_R = (GPIO_PORTA_DIR_R & ~(0x40)) | 0x88; //PA6=input, PA7,PA3 = output
	GPIO_PORTA_AFSEL_R &= ~(0xC8);	//Regular port function
	GPIO_PORTA_AMSEL_R &= ~(0xC8);	//Disable analog
	GPIO_PORTA_DEN_R |= 0xC8;	//Digital enable PA6,PA7
	
	GPIO_PORTA_IS_R &= ~(0x40); //Enable edge interrupts on PA6
	GPIO_PORTA_IBE_R &= ~(0x40); //Enable only one edge
	GPIO_PORTA_IEV_R |= 0x40;	//Rising edge

	GPIO_PORTA_ICR_R = 0x40;      // (e) clear flag for button
	GPIO_PORTA_IM_R |= 0x40;      // (f) arm interrupt on button pin *** No IME bit as mentioned in Book ***
	NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFFFF00)|0x0000000A0; // (g) priority 5
	NVIC_EN0_R = 0x1;      // (h) enable interrupt 0 in NVIC


	PA3 = 0x8;
	PA7 = 0; //Set PA7 to Low to hold Bluefruit in reset
	SysTick_Wait(800000);
	//Configure SSI0
	SYSCTL_RCGCSSI_R |= 0x1;	//Activate SSI0
	SYSCTL_RCGCGPIO_R |= 0x1;	//Activate GPIO port A
	while((SYSCTL_PRGPIO_R & 0x1) == 0);	//Wait for port A to activate
	GPIO_PORTA_AFSEL_R |= 0x34;	//Enable alternate function on PA2,4,5
	GPIO_PORTA_DEN_R |= 0x34;	//Enable digital on PA2,4,5
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFF00F0FF) + 0x00220200;	//Set pin mux to 2, which is SSI0 for PA2,4,5
	GPIO_PORTA_AMSEL_R &= ~(0x34);

	SSI0_CR1_R = 0x0;	//Disable SSI0 while config and set master mode
	SSI0_CPSR_R = 0x20;	//Select 500kHz SSI clock (80MHz / (CPSR * (1 + SCR))
	SSI0_CR0_R &= ~(0x0000FFFF);	//Clear bits
	SSI0_CR0_R |= 0x407;	//SCR = 4, SPO = 0, SPH = 0, FRF = Freescale, DSS = for 8bit data
	SSI0_DR_R = 0; //Give a 0 to start at a data of 0
	SSI0_CR1_R |= 0x2;	//Enable SSI0
	PA7 = 0x80; //Set PA7 to high to unreset Bluefruit 

	void RxFifo_Init(void);
}

uint8_t sendAfterWaiting(uint8_t value){
	while((SSI0_SR_R&SSI_SR_TFE) == 0);    //Wait for FIFO to be empty
	SSI0_DR_R = value;              //Output data
	while((SSI0_SR_R&SSI_SR_RNE)==0){};// wait until response
	uint8_t receive = SSI0_DR_R;
	return receive; // acknowledge response
}

void SPI_Reset(void){
    PA7 = 0; //Set PA7 to Low to hold Bluefruit in reset
    SysTick_Wait(800000);
    PA7 = 0x80; //Set PA7 to high to unreset Bluefruit
}

#define sendTimeout 80000

void SPI_SendArrayCommand(uint8_t command[20], uint8_t commandLength){
    if(commandLength > (SDEP_MAX_PACKETSIZE+4)){
        return;
    }

    while((SSI0_SR_R&SSI_SR_TFE) == 0);    //Wait for FIFO to be empty

	PA3 = 0;	// !CS = 1

	//100us delay = 8000 cycles
	SysTick_Wait(8000);

	uint32_t timeoutCnt = 0;

    // Bluefruit may not be ready
    while ( ( sendAfterWaiting(command[0]) == 0xFE ) && (timeoutCnt < sendTimeout))
    {
        // Disable & Re-enable CS with a bit of delay for Bluefruit to ready itself
        PA3 = 0x8;  // !CS = 0
        timeoutCnt++;
        SysTick_Wait(4000);  //Wait 50us
        PA3 = 0;    // !CS = 1
        SysTick_Wait(8000); //Wait 100us
    }
    if(timeoutCnt >= sendTimeout){
        return;
    }

	for(uint8_t i = 1; i < commandLength; i++){
		while((SSI0_SR_R&SSI_SR_TNF) == 0);	//Wait for FIFO to not be full
//		RxFifo_Put(
        sendAfterWaiting(command[i]);   //Output data and save any inputs to software FIFO
//		SSI0_DR_R = command[i];		//Output data
	}
	
	while((SSI0_SR_R&SSI_SR_BSY) == SSI_SR_BSY);    //Wait for FIFO to be empty
	PA3 = 0x8;	// !CS = 0
}

void SPI_SendCommand(sdepMsgCommand_t commandStruct){
    uint8_t command[20];
	command[0] = commandStruct.header.msg_type;
	command[1] = commandStruct.header.cmd_id_high;
	command[2] = commandStruct.header.cmd_id_low;
	command[3] = (commandStruct.header.length & 0x7F) + (commandStruct.header.more_data << 7);
	memcpy(command+4, commandStruct.payload, commandStruct.header.length);
	SPI_SendArrayCommand(command, 4 + commandStruct.header.length);
}

void SPI_SendATCommand(uint8_t command[200], uint16_t commandLength){

    sdepMsgCommand_t commandStruct;
    commandStruct.header.msg_type = SDEP_MSGTYPE_COMMAND;
    commandStruct.header.cmd_id = SDEP_CMDTYPE_AT_WRAPPER;

    if(commandLength < (SDEP_MAX_PACKETSIZE-1)){
        commandStruct.header.length = commandLength+2;
        commandStruct.header.more_data = 0;
        commandStruct.payload[0] = 'A';
        commandStruct.payload[1] = 'T';
        memcpy((commandStruct.payload)+2, command, commandLength);
        SPI_SendCommand(commandStruct);
    }else{
        commandStruct.header.length = SDEP_MAX_PACKETSIZE;
        commandStruct.header.more_data = 1;
        commandStruct.payload[0] = 'A';
        commandStruct.payload[1] = 'T';
        memcpy((commandStruct.payload)+2, command, (SDEP_MAX_PACKETSIZE-2));
        SPI_SendCommand(commandStruct);

        uint16_t currentLength = commandLength - (SDEP_MAX_PACKETSIZE-2);
        while(currentLength > 0){
            SysTick_Wait(80000); //Wait 1ms
            if(currentLength < SDEP_MAX_PACKETSIZE){
                commandStruct.header.more_data = 0;
                commandStruct.header.length = currentLength;
                memcpy(commandStruct.payload, command+(commandLength - currentLength), currentLength);
                SPI_SendCommand(commandStruct);
                currentLength = 0;
            }else{
                commandStruct.header.more_data = 1;
                commandStruct.header.length = SDEP_MAX_PACKETSIZE;
                memcpy(commandStruct.payload, command+(commandLength - currentLength), SDEP_MAX_PACKETSIZE);
                SPI_SendCommand(commandStruct);
                currentLength -= SDEP_MAX_PACKETSIZE;
            }
        }
    }

}


#define emptyCmd 0xFF
#define MAX_TIMEOUT 16000

uint8_t SPI_GetPacket(sdepMsgResponse_t *responseStruct){
	uint8_t success = 0;
//	uint8_t responseBuf[RXFIFOSIZE];
//	int i = 0;
//	while(RxFifo_Size() > 0){
//		RxFifo_Get(&responseBuf[i]);
//		i++;
//	}
	if(PA6 == 0){
	    return 0; //Cant get packet
	}

	PA3 = 0;    // !CS = 1

	//100us delay = 8000 cycles
	SysTick_Wait(8000);

	uint8_t messageTypeByte;
	uint32_t timeoutCnt = 0;

	do{
	    messageTypeByte = sendAfterWaiting(emptyCmd);
	    if(messageTypeByte == 0xFF || messageTypeByte == 0xFE){
	        PA3 = 0x8;  // !CS = 0
            SysTick_Wait(4000);  //Wait 50us
            PA3 = 0x0;  // !CS = 1
	    }
	    timeoutCnt++;

	}while((messageTypeByte == 0xFF || messageTypeByte == 0xFE) && (timeoutCnt < MAX_TIMEOUT));

	if(timeoutCnt >= MAX_TIMEOUT){
            return 0;
        }
	timeoutCnt = 0;
	while(messageTypeByte != 0x10 && messageTypeByte != 0x20 && messageTypeByte != 0x40 && messageTypeByte != 0x80 && (timeoutCnt < MAX_TIMEOUT)){
	    if(PA6 == 0){
	            return 0; //Cant get packet
        }
	    timeoutCnt++;
	    SysTick_Wait(40000);  //Wait 500us
		messageTypeByte = sendAfterWaiting(emptyCmd);
	}
	if(timeoutCnt >= MAX_TIMEOUT){
        return 0;
    }
    timeoutCnt = 0;
	responseStruct->header.msg_type = messageTypeByte;
	//response[0] = messageTypeByte;

	uint8_t fourthHeaderByte = 0;
	switch(messageTypeByte){
		case SDEP_MSGTYPE_COMMAND:  //Command Message
		case SDEP_MSGTYPE_RESPONSE:  //Response Message
					//response[1] = sendAfterWaiting(emptyCmd);  //Command ID0
					responseStruct->header.cmd_id_low = sendAfterWaiting(emptyCmd);  //Command ID0
					//response[2] = sendAfterWaiting(emptyCmd);  //Command ID1
					responseStruct->header.cmd_id_high = sendAfterWaiting(emptyCmd);  //Command ID1
					//response[3]
					fourthHeaderByte = sendAfterWaiting(emptyCmd);  //Payload length + more data indicator
					responseStruct->header.length = (fourthHeaderByte & 0x7F);
					responseStruct->header.more_data = (fourthHeaderByte & 0x80)>>7;

					if(responseStruct->header.length > SDEP_MAX_PACKETSIZE){
						break;
					}

					for(uint8_t j = 0; j < (responseStruct->header.length) && (j < SDEP_MAX_PACKETSIZE); j++){
						//response[4+j] = sendAfterWaiting(emptyCmd);    //Payload
						responseStruct->payload[j] = sendAfterWaiting(emptyCmd);    //Payload
					}
					//*responseLength = 4 + (response[3] & 0x1F);
					success = 1;
					break;
		case SDEP_MSGTYPE_ALERT:  //Alert Message
					//response[1] = sendAfterWaiting(emptyCmd);
					responseStruct->header.cmd_id_low = sendAfterWaiting(emptyCmd); //Alert ID0
					//response[2] = sendAfterWaiting(emptyCmd);
					responseStruct->header.cmd_id_high = sendAfterWaiting(emptyCmd); //Alert ID1
				   // response[3] = sendAfterWaiting(emptyCmd);  //Payload length
					fourthHeaderByte = sendAfterWaiting(emptyCmd);  //Payload length + more data indicator
					responseStruct->header.length = (fourthHeaderByte & 0x7F);
					responseStruct->header.more_data = (fourthHeaderByte & 0x80)>>7;

					if(responseStruct->header.length > SDEP_MAX_PACKETSIZE){
						break;
					}

					for(uint8_t k = 0; k < responseStruct->header.length && (k < SDEP_MAX_PACKETSIZE); k++){
						//response[4+k] = sendAfterWaiting(emptyCmd);    //Payload
						responseStruct->payload[k] = sendAfterWaiting(emptyCmd);    //Payload
					}
					//*responseLength = 5 + response[3];
					success = 1;
					break;
		case SDEP_MSGTYPE_ERROR:  //Error Message
					//response[1] = sendAfterWaiting(emptyCmd);
					responseStruct->header.cmd_id_low = sendAfterWaiting(emptyCmd); //Error ID0
					//response[2] = sendAfterWaiting(emptyCmd);
					responseStruct->header.cmd_id_high = sendAfterWaiting(emptyCmd);    //Error ID1
					fourthHeaderByte = sendAfterWaiting(emptyCmd);  //Payload length + more data indicator
					responseStruct->header.length = (fourthHeaderByte & 0x7F);
					responseStruct->header.more_data = (fourthHeaderByte & 0x80)>>7;
					success = 1;
					break;
	}

	while((SSI0_SR_R&SSI_SR_BSY) == SSI_SR_BSY);    //Wait for FIFO to be empty
	PA3 = 0x8;  // !CS = 0
	return success;
}


uint8_t SPI_GetResponse(uint8_t *response, uint16_t maxResponseLength){
    if(PA6 == 0){
        return 0; //No message available
    }
    uint8_t success = 0;
    uint16_t responseIdx = 0;
    sdepMsgResponse_t msg_response;
    if(SPI_GetPacket(&msg_response)){   //hopefully get first packet
        success = 1;
        if(msg_response.header.msg_type == SDEP_MSGTYPE_ERROR){
            UART_OutChar(' ');
            UART_OutUHex(msg_response.header.cmd_id);
            if(responseIdx < maxResponseLength){
               response[responseIdx] = 0; //null terminate
            }
            return 3;
        }
        for(; (responseIdx < msg_response.header.length) && (responseIdx < maxResponseLength) && (responseIdx < (sizeof(msg_response.payload))); responseIdx++){
            response[responseIdx] = msg_response.payload[responseIdx];
        }

        if(responseIdx == maxResponseLength){
            return 2;
        }

        while(msg_response.header.more_data){
            if(SPI_GetPacket(&msg_response)){
                if ( msg_response.header.length > 0){
                    for(uint8_t packetIdx = 0; (packetIdx < msg_response.header.length) && (responseIdx < maxResponseLength) && (packetIdx < (sizeof(msg_response.payload))); packetIdx++,responseIdx++){
                        response[responseIdx] = msg_response.payload[packetIdx];
                    }

                    if(responseIdx == maxResponseLength){
                        return 2;
                    }
                }
            }
        }
    }
    if(responseIdx < maxResponseLength){
        response[responseIdx] = 0; //null terminate
    }
    return success;
}

//uint8_t gb_responseBuf[20];
//int8_t gb_responseLength;


void GPIOPortA_Handler(void){
	uint8_t pin = GPIO_PORTA_MIS_R;
	if((pin & 0x40) == 0x40){
		GPIO_PORTA_ICR_R = 0x40; //ack pin
		//SPI_ReceiveResponse(gb_responseBuf, &gb_responseLength);
		//Handle IRQ
	}
	
}


