/*
 * BLE.h
 *
 *  Created on: Oct 19, 2019
 *      Author: Henry
 */

#ifndef BLE_H_
#define BLE_H_

#include "SPI.h"
#include "stdint.h"

typedef struct {
    uint16_t buttons;
    uint8_t left_x;
    uint8_t left_y;
} BLE_data;

int BLE_init();
void BLE_update();
int BLE_init_default();
void BLE_update_default();

#endif /* BLE_H_ */
