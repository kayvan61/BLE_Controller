/*
 * BLE.c
 *
 *  Created on: Oct 19, 2019
 *      Author: Henry
 */

#include <stdint.h>
#include <WTimer0.h>
#include "tm4c123gh6pm.h"
#include "BLE.h"
#include "SysTick.h"

#define DEAD_LOW 1600
#define DEAD_HIGH 2400

//#define RESET_TIMEOUT 640000000
#define FRESET_TIMEOUT 100 //100 * 10ms = 1s
#define RESET_TIMEOUT 50 //50 * 10ms = 500ms
#define MAX_TIMEOUT 8000000
#define UPDATE_PERIOD 10000000 //250ms
//#define WAIT_BETWEEN 160000000     //2s
#define WAIT_BETWEEN 30 //50 * 10ms = 500ms

#define PA6             (*((volatile uint32_t *) 0x40004100))
#define PC4             (*((volatile uint32_t *) 0x40006040))

static const uint8_t factoryRes[] = "+FACTORYRESET";

static const uint8_t connectableGAP[]   = "+GAPCONNECTABLE=1";

static const uint8_t setLEDspi[]        = "+HWMODELED=4";
static const uint8_t setBLEName[]       = "+GAPDEVNAME=UTBLE";
static const uint8_t stopAdvBLE[]       = "+GAPSTOPADV";
static const uint8_t advertiseBLE[]     = "+GAPSTARTADV";
static const uint8_t advertiseBLEData[] = "+GAPSETADVDATA=03-02-12-18";

static const uint8_t clearGATT[] = "+GATTCLEAR";
static const uint8_t addHIDInformation[]    = "+GATTADDCHAR=UUID=0x2A4A,PROPERTIES=0x02,MIN_LEN=1,MAX_LEN=32,VALUE=02-00-11-10";
static const uint8_t addHIDControlPoint[]   = "+GATTADDCHAR=UUID=0x2A4C,PROPERTIES=0x04,MIN_LEN=1,MAX_LEN=32,VALUE=01";
static const uint8_t addHIDProtMode[]       = "+GATTADDCHAR=UUID=0x2A4E,PROPERTIES=0x06,MIN_LEN=1,MAX_LEN=32,VALUE=01";
static const uint8_t addCCCD[]              = "+GATTADDCHAR=UUID=0x2902,PROPERTIES=0x03,MIN_LEN=1,MAX_LEN=32,VALUE=0";
static const uint8_t addReportRef[]         = "+GATTADDCHAR=UUID=0x2908,PROPERTIES=0x02,MIN_LEN=1,MAX_LEN=32,VALUE=00-01";
static const uint8_t addHID[]               = "+GATTADDSERVICE=UUID=0x1812";

//static const uint8_t sendReportMap[]  = "+GATTADDCHAR=UUID=0x2A4B,PROPERTIES=0x10,MIN_LEN=32,MAX_LEN=32,VALUE=05-01-09-05-a1-01-a1-00-05-09-19-01-29-10-15-00-25-01-95-10-75-01-81-02-05-01-09-30-09-31-15-81";
static const uint8_t sendReportMap[]    = "+GATTADDCHAR=UUID=0x2A4B,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=32,VALUE=05-01-09-05-a1-01-a1-00-05-09-19-01-29-10-15-00-25-01-95-10-75-01-81-02-c0-c0";
//static const uint8_t sendReportMap[]  = "+GATTADDCHAR=UUID=0x2A4B,PROPERTIES=0x08,MIN_LEN=4,VALUE=05-01-09-05,DATATYPE=2";
//static const uint8_t sendReportMap[]  = "+GATTADDCHAR=UUID=0x2A19,PROPERTIES=0x08,MIN_LEN=1,VALUE=99";

//static const uint8_t addReportChar[]  = "+GATTADDCHAR=UUID=0x2A4D,PROPERTIES=0x10,MAX_LEN=4,DATATYPE=2,VALUE=0";
static const uint8_t sendReport[]       = "+GATTCHAR=1,";
static const uint8_t sendReportLength   = sizeof(sendReport)-1;

static const uint8_t sendTestReport[] = "+GATTCHAR=2,01-01-00-00-7f-ff-00-00";
static const uint8_t addTestReport[] = "+GATTADDCHAR=UUID=0x2A4D,PROPERTIES=0x12,MIN_LEN=1,MAX_LEN=32,VALUE=01-01-00-00-7f-ff-00-00";
static const uint8_t enableHID[] = "+BLEHIDEN=on";

//only used for default HID controller
static const uint8_t enableBLEController[] = "+BLEHIDGAMEPADEN=1";
static const uint8_t sendBLEControllerPacket[] = "+BLEHIDGAMEPAD=";
static const uint8_t sendPacketLength = sizeof(sendBLEControllerPacket)-1;

void strCat(const char* a, const char* const b){

}

uint8_t isStrEq(const uint8_t* str1, uint16_t str1Len, const uint8_t* str2, uint16_t str2Len){
    if(str1Len != str2Len){
        return 0;
    }
    for(uint16_t i = 0; i < str1Len; i++){
        if(str1[i] != str2[i]){
            return 0;
        }
    }
    return 1;
}

int BLE_init() {

    WTimer0_Init(&BLE_update, UPDATE_PERIOD);

    uint8_t resp[200];
    uint16_t respLen = 200;

    uint32_t timeoutCounter = 0;
    volatile int b = 0;
    uint8_t timedOut = 1;
    while(timedOut){
        timedOut = 0;
        timeoutCounter = 0;
        SPI_Reset();
        SysTick80_Wait10ms(RESET_TIMEOUT); //Wait

        SPI_SendATCommand(factoryRes, sizeof(factoryRes)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 1;
        }
        timeoutCounter=0;
        resp[0] = 0;
        SysTick80_Wait10ms(FRESET_TIMEOUT);

        SPI_SendATCommand(setLEDspi, sizeof(setLEDspi)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 2;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        SPI_SendATCommand(stopAdvBLE, sizeof(stopAdvBLE)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 3;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //set up GAP
        SPI_SendATCommand(setBLEName, sizeof(setBLEName)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 4;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //advertise
        SPI_SendATCommand(advertiseBLE, sizeof(advertiseBLE)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 5;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        SPI_Reset();
        SysTick80_Wait10ms(RESET_TIMEOUT); //Wait


        SPI_SendATCommand(advertiseBLEData, sizeof(advertiseBLEData)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 6;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //set up GATT and HID
//        SPI_SendATCommand(enableHID, sizeof(enableHID)-1);
//        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
//            timeoutCounter++;
//        }
//        SPI_GetResponse(resp, respLen);
//        if(timeoutCounter >= MAX_TIMEOUT){
//            timedOut = 8;
//        }
//        timeoutCounter=0;
//        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

//        SPI_SendATCommand(clearGATT, sizeof(clearGATT)-1);
//        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
//            timeoutCounter++;
//        }
//        SPI_GetResponse(resp, respLen);
//        if(timeoutCounter >= MAX_TIMEOUT){
//            timedOut = 9;
//        }
//        timeoutCounter=0;
//        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait


        SPI_SendATCommand(addHID, sizeof(addHID)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 9;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //addHIDProtMode
        SPI_SendATCommand(addHIDProtMode, sizeof(addHIDProtMode)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 16;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //add report char
        //addReportChar
        SPI_SendATCommand(addTestReport, sizeof(addTestReport)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 11;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //addCCCD
//        SPI_SendATCommand(addCCCD, sizeof(addCCCD)-1);
//        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
//            timeoutCounter++;
//        }
//        SPI_GetResponse(resp, respLen);
//        if(timeoutCounter >= MAX_TIMEOUT){
//            timedOut = 11;
//        }
//        timeoutCounter=0;
//        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //addReportRef
        SPI_SendATCommand(addReportRef, sizeof(addReportRef)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 11;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //send report map
        SPI_SendATCommand(sendReportMap, sizeof(sendReportMap)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 10;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //addHIDInformation
        SPI_SendATCommand(addHIDInformation, sizeof(addHIDInformation)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 13;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //addHIDControlPoint
        SPI_SendATCommand(addHIDControlPoint, sizeof(addHIDControlPoint)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 15;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait


        SPI_SendATCommand(connectableGAP, sizeof(connectableGAP)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 14;
        }
        timeoutCounter=0;


        SPI_Reset();

        if(timedOut){
            SysTick80_Wait10ms(RESET_TIMEOUT); //Wait
        }
    }
//    SPI_SendATCommand(sendTestReport, sizeof(sendTestReport)-1);
    return 1;
}

extern volatile uint16_t ADCvalueX;
extern volatile uint16_t ADCvalueY;

static const uint8_t hexDigit[16] ={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void BLE_update(){

    PC4 ^= 0x10; //Heartbeat
    PC4 ^= 0x10; //Heartbeat
    uint8_t resp[8];
    const uint16_t respLen = 8;
    uint32_t timeoutCounter = 0;
//
//    uint8_t report[81];
//    const uint8_t reportLength = 81;
//    memcpy(report,sendReport,sendReportLength);
//
//    uint8_t maskedButtons = GPIO_PORTD_DATA_R & 0x3F;
//    uint16_t joystickX = ADCvalueX;
//    uint16_t joystickY = ADCvalueY;
//
//    report[sendReportLength+1]  = hexDigit[((maskedButtons >> 4)&0x3)];
//    report[sendReportLength+2]  = hexDigit[((maskedButtons)&0xF)];
//    report[sendReportLength+3]  = '-';
//    report[sendReportLength+4]  = '0';
//    report[sendReportLength+5]  = '0';
//    report[sendReportLength+6]  = '-';
//    report[sendReportLength+7]  = hexDigit[(((joystickX & 0xF000) >> 12)&0xF)];
//    report[sendReportLength+8]  = hexDigit[(((joystickX & 0x0F00) >> 8 )&0xF)];
//    report[sendReportLength+9]  = '-';
//    report[sendReportLength+10] = hexDigit[(((joystickX & 0xF000) >> 12)&0xF)];
//    report[sendReportLength+11] = hexDigit[(((joystickX & 0x0F00) >> 8 )&0xF)];
//    report[sendReportLength+12] = '-';
//    report[sendReportLength+13] = hexDigit[(((joystickY & 0xF000) >> 12)&0xF)];
//    report[sendReportLength+14] = hexDigit[(((joystickY & 0x0F00) >> 8 )&0xF)];
//    report[sendReportLength+15] = '-';
//    report[sendReportLength+16] = hexDigit[(((joystickY & 0x00F0) >> 4 )&0xF)];
//    report[sendReportLength+17] = hexDigit[(((joystickY & 0x000F)      )&0xF)];
//
    SPI_SendATCommand(sendTestReport, sizeof(sendTestReport)-1);
    while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
        timeoutCounter++;
    }
    SPI_GetResponse(resp, respLen); // clear out response
    PC4 ^= 0x10; //Heartbeat
}


int BLE_init_default() {
    WTimer0_Init(&BLE_update_default, UPDATE_PERIOD);

    uint8_t resp[200];
    uint16_t respLen = 200;

    uint32_t timeoutCounter = 0;
    volatile int b = 0;
    uint8_t timedOut = 1;
    while(timedOut){
        timedOut = 0;
        timeoutCounter = 0;
        SPI_Reset();
        SysTick80_Wait10ms(RESET_TIMEOUT); //Wait

//        SPI_SendATCommand(factoryRes, sizeof(factoryRes)-1);
//        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
//            timeoutCounter++;
//        }
//        SPI_GetResponse(resp, respLen);
//        if(timeoutCounter >= MAX_TIMEOUT){
//            timedOut = 1;
//        }
//        timeoutCounter=0;
//        resp[0] = 0;
//        SysTick80_Wait10ms(FRESET_TIMEOUT);

        SPI_SendATCommand(setLEDspi, sizeof(setLEDspi)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 2;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        SPI_SendATCommand(stopAdvBLE, sizeof(stopAdvBLE)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 3;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //set up GAP
        SPI_SendATCommand(setBLEName, sizeof(setBLEName)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 4;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //advertise
        SPI_SendATCommand(advertiseBLE, sizeof(advertiseBLE)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 5;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        SPI_Reset();
        SysTick80_Wait10ms(RESET_TIMEOUT); //Wait

        //enable HID
        SPI_SendATCommand(enableHID, sizeof(enableHID)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 6;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait


        //turn on BLE controller
        SPI_SendATCommand(enableBLEController, sizeof(enableBLEController)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 6;
        }
        timeoutCounter=0;
        SysTick80_Wait10ms(WAIT_BETWEEN); //Wait

        //start connecting with GAP
        SPI_SendATCommand(connectableGAP, sizeof(connectableGAP)-1);
        while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
            timeoutCounter++;
        }
        SPI_GetResponse(resp, respLen);
        if(timeoutCounter >= MAX_TIMEOUT){
            timedOut = 14;
        }
        timeoutCounter=0;


        SPI_Reset();

        if(timedOut){
            SysTick80_Wait10ms(RESET_TIMEOUT); //Wait
        }
    }
//    SPI_SendATCommand(sendTestReport, sizeof(sendTestReport)-1);
    return 1;

}

void BLE_update_default(){
    PC4 ^= 0x10; //Heartbeat
    PC4 ^= 0x10; //Heartbeat
    uint8_t resp[8];
    const uint16_t respLen = 8;
    uint32_t timeoutCounter = 0;

    uint8_t report[81];
    uint8_t reportIndex = 0;
    const uint8_t reportLength = 81;
    memcpy(report, sendBLEControllerPacket, sendPacketLength);

    uint8_t maskedButtons = GPIO_PORTD_DATA_R & 0x3F;
    uint16_t joystickX = ADCvalueX;
    uint16_t joystickY = ADCvalueY;

    uint8_t leftright[3] = "0";
    uint8_t updown[3] = "0";

    uint8_t leftrightLen = 1,
            updownLen = 1;

    if(joystickX > DEAD_HIGH){
        leftright[0] = '1';
    }
    else if(joystickX < DEAD_LOW){
        leftright[0] = '-';
        leftright[1] = '1';
        leftrightLen = 2;
    }

    if(joystickY > DEAD_HIGH){
        updown[0] = '1';
    }
    else if(joystickY < DEAD_LOW){
        updown[0] = '-';
        updown[1] = '1';
        updownLen = 2;
    }

    for(int i = 0; i < leftrightLen; i++){
        report[sendPacketLength + reportIndex + i] = leftright[i];
    }
    reportIndex += leftrightLen;
    report[sendPacketLength + (reportIndex++)] = ',';

    for(int i = 0; i < updownLen; i++){
        report[sendPacketLength + reportIndex + i] = updown[i];
    }
    reportIndex += updownLen;
    report[sendPacketLength + (reportIndex++)] = ',';
    report[sendPacketLength + (reportIndex++)] = '0';
    report[sendPacketLength + (reportIndex++)] = 'x';
    report[sendPacketLength + (reportIndex++)] = hexDigit[(maskedButtons >> 4) & 0x0F];
    report[sendPacketLength + (reportIndex++)] = hexDigit[(maskedButtons) & 0x0F];

    SPI_SendATCommand(report, sendPacketLength + reportIndex);
    while(PA6 != 0x40 && timeoutCounter < MAX_TIMEOUT){
        timeoutCounter++;
    }
    SPI_GetResponse(resp, respLen); // clear out response
    PC4 ^= 0x10; //Heartbeat

}
