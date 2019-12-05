/*
 * SPI.h
 *
 *  Created on: Oct 19, 2019
 *      Author: Henry
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include "SDEP.h"

void SPI_Init(void);

//void SPI_SendCommand(sdepMsgCommand_t commandStruct);
//void SPI_SendArrayCommand(uint8_t command[20], uint8_t commandLength);
void SPI_SendATCommand(uint8_t command[200], uint16_t commandLength);
uint8_t SPI_GetResponse(uint8_t *response, uint16_t maxResponseLength);
void SPI_Reset(void);
//uint8_t SPI_GetPacket(sdepMsgResponse_t *responseStruct);



#endif /* SPI_H_ */
