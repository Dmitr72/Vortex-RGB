/*
 * system.h
 *
 *  Created on: Oct 29, 2025
 *      Author: Optik
 */

#ifndef SYSTEM_SYSTEM_H_
#define SYSTEM_SYSTEM_H_

#include "main.h"

#include "ee.h"
#include "dac8551.h"
#include "ad5160.h"
#include "rgb.h"

#define RX_BUFFER_SIZE 32
#define TX_BUFFER_SIZE 32

extern uint8_t tx_buffer[TX_BUFFER_SIZE];
extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern uint8_t usb_ready;

void systemInit();
void usbCallback();
void uartCallback();
void systemTask();
void colorEffect();
uint16_t crcCalculation(uint8_t* data);
bool checkCRC(uint8_t* data);
void BuildConfigData();
void BuildControlData();
void sendData();
void setData(uint8_t* data);
bool saveConfigToEE();

#endif /* SYSTEM_SYSTEM_H_ */
