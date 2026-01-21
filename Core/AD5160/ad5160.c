/*
 * ad5160.c
 *
 *  Created on: Sep 18, 2025
 *      Author: Optik
 */

#include "ad5160.h"

/**
* @brief Initialize the AD5160 structure
* @param had: pointer to the AD5160 structure
* @param hspi: pointer to the SPI structure
* @param cs_port: CS line port
* @param cs_pin: CS line pin
*/
void AD5160_Init(AD5160_HandleTypeDef *had, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    had->hspi = hspi;
    had->cs_port = cs_port;
    had->cs_pin = cs_pin;
    had->current_value = 0;
    had->is_shutdown = 0;

    HAL_GPIO_WritePin(had->cs_port, had->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef AD5160_Write(AD5160_HandleTypeDef *had, uint8_t data) {
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(had->cs_port, had->cs_pin, GPIO_PIN_RESET);

    for(volatile int i = 0; i < 100; i++);

    status = HAL_SPI_Transmit(had->hspi, &data, 1, 100);

    HAL_GPIO_WritePin(had->cs_port, had->cs_pin, GPIO_PIN_SET);

    return status;
}

/**
* @brief Set the resistance value (0-255)
* @param had: pointer to AD5160 structure
* @param value: value (0-255)
* @retval Execution status
*/
HAL_StatusTypeDef AD5160_SetResistance(AD5160_HandleTypeDef *had, uint8_t value) {
    if (had->is_shutdown) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = AD5160_Write(had, value);
    if (status == HAL_OK) {
        had->current_value = value;
    }
    return status;
}

uint8_t AD5160_GetValue(AD5160_HandleTypeDef *had){
	return had->current_value;
}
