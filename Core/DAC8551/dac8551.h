/*
 * dac8551.h
 *
 *  Created on: Sep 18, 2025
 *      Author: Optik
 */

#ifndef DAC8551_H
#define DAC8551_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DAC8551 Configuration */
#define DAC8551_RESOLUTION      16
#define DAC8551_MAX_VALUE       0xFFFF
#define DAC8551_SPI_TIMEOUT     100

/* DAC8551 Command bits */
#define DAC8551_CMD_WRITE_UPDATE    0x00  // Write to DAC register and update output
#define DAC8551_CMD_WRITE_NO_UPDATE 0x10  // Write to DAC register, don't update output
#define DAC8551_CMD_UPDATE_OUTPUT   0x20  // Update output from DAC register
#define DAC8551_CMD_POWER_DOWN      0x30  // Power down modes

/* Power down modes */
#define DAC8551_PD_NORMAL           0x00  // Normal operation
#define DAC8551_PD_1K_TO_GND        0x40  // 1kΩ to ground
#define DAC8551_PD_100K_TO_GND      0x80  // 100kΩ to ground
#define DAC8551_PD_HIGH_Z           0xC0  // High impedance

/* Error codes */
typedef enum {
    DAC8551_OK = 0,
    DAC8551_ERROR = 1,
    DAC8551_ERROR_INVALID_PARAM = 2,
    DAC8551_ERROR_SPI = 3,
    DAC8551_ERROR_VOLTAGE_OUT_OF_RANGE = 4
} DAC8551_StatusTypeDef;

/* Structure for DAC8551 instance */
typedef struct {
    SPI_HandleTypeDef *hspi;      // SPI handle
    GPIO_TypeDef *cs_port;        // Chip Select GPIO port
    uint16_t cs_pin;              // Chip Select GPIO pin
    float vref;                   // Reference voltage in volts
    uint16_t last_value;          // Last written DAC value
    bool initialized;             // Initialization flag
} DAC8551_HandleTypeDef;

/* Function prototypes */
DAC8551_StatusTypeDef DAC8551_Init(DAC8551_HandleTypeDef *hdac, SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin, float vref);
DAC8551_StatusTypeDef DAC8551_DeInit(DAC8551_HandleTypeDef *hdac);
DAC8551_StatusTypeDef DAC8551_WriteValue(DAC8551_HandleTypeDef *hdac, uint16_t value);
DAC8551_StatusTypeDef DAC8551_WriteVoltage(DAC8551_HandleTypeDef *hdac, float voltage);
DAC8551_StatusTypeDef DAC8551_WriteValueNoUpdate(DAC8551_HandleTypeDef *hdac, uint16_t value);
DAC8551_StatusTypeDef DAC8551_WriteVoltageNoUpdate(DAC8551_HandleTypeDef *hdac, float voltage);
DAC8551_StatusTypeDef DAC8551_UpdateOutput(DAC8551_HandleTypeDef *hdac);
DAC8551_StatusTypeDef DAC8551_PowerDown(DAC8551_HandleTypeDef *hdac, uint8_t pd_mode);
DAC8551_StatusTypeDef DAC8551_PowerUp(DAC8551_HandleTypeDef *hdac);
DAC8551_StatusTypeDef DAC8551_SetReferenceVoltage(DAC8551_HandleTypeDef *hdac, float vref);
float DAC8551_ValueToVoltage(DAC8551_HandleTypeDef *hdac, uint16_t value);
uint16_t DAC8551_VoltageToValue(DAC8551_HandleTypeDef *hdac, float voltage);
uint16_t DAC8551_GetLastValue(DAC8551_HandleTypeDef *hdac);
float DAC8551_GetLastVoltage(DAC8551_HandleTypeDef *hdac);
bool DAC8551_IsInitialized(DAC8551_HandleTypeDef *hdac);

#ifdef __cplusplus
}
#endif

#endif /* DAC8551_H */

