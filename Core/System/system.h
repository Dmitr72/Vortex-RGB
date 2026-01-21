/*
 * system.h
 *
 *  Created on: Oct 29, 2025
 *      Author: Optik
 */

#ifndef SYSTEM_SYSTEM_H_
#define SYSTEM_SYSTEM_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "ee.h"
//#include "ds18b20.h"
#include "dac8551.h"
#include "ad5160.h"
#include "rgb.h"

#define RX_BUFFER_SIZE 32
#define TX_BUFFER_SIZE 32

extern uint8_t rx_buffer[TX_BUFFER_SIZE];
extern uint8_t rx_buffer[TX_BUFFER_SIZE];
extern uint8_t usb_ready;
extern RGB_channel drvR;
extern RGB_channel drvG;
extern RGB_channel drvB;
//uint8_t uart_rx_ready;
//uint8_t uart_tx_ready;
//AD5160_HandleTypeDef hadR, hadG, hadB;
//extern DAC8551_HandleTypeDef hdacR, hdacG, hdacB;
//extern bool config_saved;
//extern uint32_t adc2_buf[4];

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
void tempConversion();
bool saveConfigToEE();

#endif /* SYSTEM_SYSTEM_H_ */
