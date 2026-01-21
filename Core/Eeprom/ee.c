/*
 * ee.c
 *
 *  Created on: Sep 29, 2025
 *      Author: Dmitrij
 */

#include "ee.h"
#include <string.h>
#include <stdio.h>
EE_Init_State eepromInit(EEprom_HandleTypeDef* EEprom_, I2C_HandleTypeDef* i2c_handel, uint8_t address,
		uint16_t mem_size, uint8_t mem_address_size, uint16_t page_size)
{
	EEprom_->i2c_handel = i2c_handel;
	EEprom_->address = address;
	EEprom_->mem_size = mem_size;
	EEprom_->mem_address_size = mem_address_size;
	EEprom_->page_size = page_size;

	uint8_t page[page_size];
	uint16_t i;
	if(eepromReadPage(EEprom_, page, 0) != HAL_OK) return EE_ERROR;
	for(i = 0; i < page_size; i++)
		if(page[i] != 0) return EE_NOT_FORMATTED;

	return EE_OK;
}

HAL_StatusTypeDef eepromWritePage(EEprom_HandleTypeDef *eeprom, uint8_t *pData, uint16_t page) {
    uint16_t mem_address = page * eeprom->page_size;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(eeprom->i2c_handel, eeprom->address, mem_address,
            eeprom->mem_address_size, pData, eeprom->page_size, 100);

    if(status == HAL_OK) {
        HAL_Delay(10);  // Задержка для записи EEPROM
    }
    //printf("stat %d\n", status);
    return status;
}

HAL_StatusTypeDef eepromReadPage(EEprom_HandleTypeDef *eeprom, uint8_t *pData, uint16_t page) {
	uint16_t mem_address = page * eeprom->page_size;
	return HAL_I2C_Mem_Read(eeprom->i2c_handel, eeprom->address, mem_address,
			eeprom->mem_address_size, pData, eeprom->page_size, 20);
}

HAL_StatusTypeDef eepromWrite(EEprom_HandleTypeDef *eeprom, uint8_t *pData, uint16_t mem_address, uint16_t size) {
    if(size > eeprom->page_size) return HAL_ERROR;

    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(eeprom->i2c_handel, eeprom->address, mem_address,
            eeprom->mem_address_size, pData, size, 100);

    if(status == HAL_OK) {
        HAL_Delay(10);  // Задержка для записи EEPROM
    }
    return status;
}

HAL_StatusTypeDef eepromRead(EEprom_HandleTypeDef *eeprom, uint8_t *pData, uint16_t mem_address, uint16_t size) {
	if( size > eeprom->page_size) return HAL_ERROR;
	return HAL_I2C_Mem_Read(eeprom->i2c_handel, eeprom->address, mem_address,
			eeprom->mem_address_size, pData, size, 20);
}

HAL_StatusTypeDef eepromFormat(EEprom_HandleTypeDef *eeprom){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t data[eeprom->page_size];
	memset(data, 0, eeprom->page_size);
	for(uint16_t i = 0; i < eeprom->page_size; i++){
		status = eepromWritePage(eeprom, data, i);
		HAL_Delay(10);
		//printf("ee_stat "); printbyte(status);
		if(status != HAL_OK) return status;
	}
	return status;
}


