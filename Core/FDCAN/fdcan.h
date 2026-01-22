/*
 * fdcan.h
 *
 *  Created on: 22.01.2026
 *      Author: Optik
 */

#ifndef FDCAN_FDCAN_H_
#define FDCAN_FDCAN_H_

#include "main.h"

void FDCAN_Init(void);
void ProcessFDCANMessage(uint32_t id, uint8_t *data, uint8_t length);
void FDCAN_SendMessage(uint32_t id, uint8_t *data, uint8_t size);

#endif /* FDCAN_FDCAN_H_ */
