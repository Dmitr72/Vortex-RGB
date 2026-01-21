/*
 * ee.h
 *
 *  Created on: Sep 29, 2025
 *      Author: Dmitrij
 */

#ifndef EEPROM_EE_H_
#define EEPROM_EE_H_

//#include "stm32g4xx_hal.h"
#include "main.h"

#define EE_PAGE_SIZE 32

typedef enum {
    EE_OK,
    EE_ERROR,
    EE_NOT_FORMATTED
} EE_Init_State;

typedef struct {
	I2C_HandleTypeDef* i2c_handel;
	uint8_t address;	//chip I2C address
	uint16_t mem_size;	//in bytes
	uint8_t mem_address_size;	//1 or 2 bytes
	uint16_t page_size;	//chip page size

} EEprom_HandleTypeDef;

EE_Init_State eepromInit(EEprom_HandleTypeDef* EEprom_, I2C_HandleTypeDef* i2c_handel, uint8_t address,
		uint16_t mem_size, uint8_t mem_address_size, uint16_t page_size);

HAL_StatusTypeDef eepromWritePage(EEprom_HandleTypeDef *eeprom, uint8_t *pData, uint16_t page);
HAL_StatusTypeDef eepromReadPage(EEprom_HandleTypeDef *eeprom, uint8_t *pData, uint16_t page);
HAL_StatusTypeDef eepromWrite(EEprom_HandleTypeDef *eeprom, uint8_t *pData, uint16_t mem_address, uint16_t size);
HAL_StatusTypeDef eepromRead(EEprom_HandleTypeDef *eeprom, uint8_t *pData, uint16_t mem_address, uint16_t size);
HAL_StatusTypeDef eepromFormat(EEprom_HandleTypeDef *eeprom);

#endif /* EEPROM_EE_H_ */
