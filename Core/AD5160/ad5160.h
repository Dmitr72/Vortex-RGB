/*
 * ad5160.h
 *
 *  Created on: Sep 18, 2025
 *      Author: Optik
 */

#ifndef AD5160_H
#define AD5160_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint8_t current_value;    // Store the value locally
    uint8_t is_shutdown;
} AD5160_HandleTypeDef;

void AD5160_Init(AD5160_HandleTypeDef *had, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);

HAL_StatusTypeDef AD5160_SetResistance(AD5160_HandleTypeDef *had, uint8_t value);
uint8_t AD5160_GetValue(AD5160_HandleTypeDef *had);


#ifdef __cplusplus
}
#endif

#endif /* AD5160_H */
