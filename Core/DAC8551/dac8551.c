/*
 * dac8551.c
 *
 *  Created on: Sep 18, 2025
 *      Author: Optik
 */

#include "dac8551.h"

/* Private function prototypes */
static DAC8551_StatusTypeDef DAC8551_WriteCommand(DAC8551_HandleTypeDef *hdac, uint8_t command, uint16_t data);
static bool DAC8551_ValidateHandle(DAC8551_HandleTypeDef *hdac);

/**
 * @brief Initialize DAC8551
 * @param hdac: Pointer to DAC8551 handle
 * @param hspi: Pointer to SPI handle
 * @param cs_port: Chip Select GPIO port
 * @param cs_pin: Chip Select GPIO pin
 * @param vref: Reference voltage in volts (must be > 0)
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_Init(DAC8551_HandleTypeDef *hdac, SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin, float vref)
{
    // Validate parameters
    if (hdac == NULL || hspi == NULL || cs_port == NULL || vref <= 0.0f) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    // Initialize structure
    hdac->hspi = hspi;
    hdac->cs_port = cs_port;
    hdac->cs_pin = cs_pin;
    hdac->vref = vref;
    hdac->last_value = 0;
    hdac->initialized = false;

    // Set CS pin high (inactive)
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    // Test SPI communication by powering up DAC
    if (DAC8551_PowerUp(hdac) != DAC8551_OK) {
        return DAC8551_ERROR_SPI;
    }

    hdac->initialized = true;

    // Set initial output to 0V
    if (DAC8551_WriteValue(hdac, 0) != DAC8551_OK) {
    	hdac->initialized = false;
        return DAC8551_ERROR_SPI;
    }

    return DAC8551_OK;
}

/**
 * @brief Deinitialize DAC8551
 * @param hdac: Pointer to DAC8551 handle
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_DeInit(DAC8551_HandleTypeDef *hdac)
{
    if (!DAC8551_ValidateHandle(hdac)) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    // Power down DAC to save power
    DAC8551_PowerDown(hdac, DAC8551_PD_HIGH_Z);

    // Clear structure
    hdac->hspi = NULL;
    hdac->cs_port = NULL;
    hdac->cs_pin = 0;
    hdac->vref = 0.0f;
    hdac->last_value = 0;
    hdac->initialized = false;

    return DAC8551_OK;
}

/**
 * @brief Write raw value to DAC8551 and update output immediately
 * @param hdac: Pointer to DAC8551 handle
 * @param value: 16-bit DAC value (0-65535)
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_WriteValue(DAC8551_HandleTypeDef *hdac, uint16_t value)
{
    if (!DAC8551_ValidateHandle(hdac)) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    DAC8551_StatusTypeDef status = DAC8551_WriteCommand(hdac, DAC8551_CMD_WRITE_UPDATE, value);

    if (status == DAC8551_OK) {
        hdac->last_value = value;
    }

    return status;
}

/**
 * @brief Write voltage to DAC8551 and update output immediately
 * @param hdac: Pointer to DAC8551 handle
 * @param voltage: Output voltage in volts (0 to Vref)
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_WriteVoltage(DAC8551_HandleTypeDef *hdac, float voltage)
{
    if (!DAC8551_ValidateHandle(hdac)) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    // Validate voltage range
    if (voltage < 0.0f || voltage > hdac->vref) {
        return DAC8551_ERROR_VOLTAGE_OUT_OF_RANGE;
    }

    uint16_t value = DAC8551_VoltageToValue(hdac, voltage);
    return DAC8551_WriteValue(hdac, value);
}

/**
 * @brief Write raw value to DAC8551 without updating output
 * @param hdac: Pointer to DAC8551 handle
 * @param value: 16-bit DAC value (0-65535)
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_WriteValueNoUpdate(DAC8551_HandleTypeDef *hdac, uint16_t value)
{
    if (!DAC8551_ValidateHandle(hdac)) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    DAC8551_StatusTypeDef status = DAC8551_WriteCommand(hdac, DAC8551_CMD_WRITE_NO_UPDATE, value);

    if (status == DAC8551_OK) {
        hdac->last_value = value;
    }

    return status;
}

/**
 * @brief Write voltage to DAC8551 without updating output
 * @param hdac: Pointer to DAC8551 handle
 * @param voltage: Output voltage in volts (0 to Vref)
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_WriteVoltageNoUpdate(DAC8551_HandleTypeDef *hdac, float voltage)
{
    if (!DAC8551_ValidateHandle(hdac)) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    if (voltage < 0.0f || voltage > hdac->vref) {
        return DAC8551_ERROR_VOLTAGE_OUT_OF_RANGE;
    }

    uint16_t value = DAC8551_VoltageToValue(hdac, voltage);
    return DAC8551_WriteValueNoUpdate(hdac, value);
}

/**
 * @brief Update DAC output from internal register
 * @param hdac: Pointer to DAC8551 handle
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_UpdateOutput(DAC8551_HandleTypeDef *hdac)
{
    if (!DAC8551_ValidateHandle(hdac)) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    return DAC8551_WriteCommand(hdac, DAC8551_CMD_UPDATE_OUTPUT, 0);
}

/**
 * @brief Power down DAC8551
 * @param hdac: Pointer to DAC8551 handle
 * @param pd_mode: Power down mode (DAC8551_PD_*)
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_PowerDown(DAC8551_HandleTypeDef *hdac, uint8_t pd_mode)
{
    if (!DAC8551_ValidateHandle(hdac)) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    uint8_t command = DAC8551_CMD_POWER_DOWN | pd_mode;
    return DAC8551_WriteCommand(hdac, command, 0);
}

/**
 * @brief Power up DAC8551 (normal operation)
 * @param hdac: Pointer to DAC8551 handle
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_PowerUp(DAC8551_HandleTypeDef *hdac)
{
    if (hdac == NULL) {
        return DAC8551_ERROR_INVALID_PARAM;
    }
    return DAC8551_WriteCommand(hdac, DAC8551_CMD_POWER_DOWN | DAC8551_PD_NORMAL, 0);
}

/**
 * @brief Set new reference voltage
 * @param hdac: Pointer to DAC8551 handle
 * @param vref: New reference voltage in volts (must be > 0)
 * @retval DAC8551 status
 */
DAC8551_StatusTypeDef DAC8551_SetReferenceVoltage(DAC8551_HandleTypeDef *hdac, float vref)
{
    if (!DAC8551_ValidateHandle(hdac) || vref <= 0.0f) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    hdac->vref = vref;
    return DAC8551_OK;
}

/**
 * @brief Convert DAC value to voltage
 * @param hdac: Pointer to DAC8551 handle
 * @param value: 16-bit DAC value
 * @retval Voltage in volts
 */
float DAC8551_ValueToVoltage(DAC8551_HandleTypeDef *hdac, uint16_t value)
{
    if (hdac == NULL || hdac->vref <= 0.0f) {
        return 0.0f;
    }

    return (float)value * hdac->vref / (float)DAC8551_MAX_VALUE;
}

/**
 * @brief Convert voltage to DAC value
 * @param hdac: Pointer to DAC8551 handle
 * @param voltage: Voltage in volts
 * @retval 16-bit DAC value
 */
uint16_t DAC8551_VoltageToValue(DAC8551_HandleTypeDef *hdac, float voltage)
{
    if (hdac == NULL || voltage < 0.0f || hdac->vref <= 0.0f) {
        return 0;
    }

    if (voltage >= hdac->vref) {
        return DAC8551_MAX_VALUE;
    }

    return (uint16_t)(voltage * (float)DAC8551_MAX_VALUE / hdac->vref + 0.5f);
}

/**
 * @brief Get last written DAC value
 * @param hdac: Pointer to DAC8551 handle
 * @retval Last DAC value
 */
uint16_t DAC8551_GetLastValue(DAC8551_HandleTypeDef *hdac)
{
    if (!DAC8551_ValidateHandle(hdac)) {
        return 0;
    }

    return hdac->last_value;
}

/**
 * @brief Get last written voltage
 * @param hdac: Pointer to DAC8551 handle
 * @retval Last voltage in volts
 */
float DAC8551_GetLastVoltage(DAC8551_HandleTypeDef *hdac)
{
    if (!DAC8551_ValidateHandle(hdac)) {
        return 0.0f;
    }

    return DAC8551_ValueToVoltage(hdac, hdac->last_value);
}

/**
 * @brief Check if DAC is initialized
 * @param hdac: Pointer to DAC8551 handle
 * @retval true if initialized, false otherwise
 */
bool DAC8551_IsInitialized(DAC8551_HandleTypeDef *hdac)
{
    return (hdac != NULL && hdac->initialized);
}

/* ====================================================================
 * PRIVATE FUNCTIONS
 * ====================================================================*/

/**
 * @brief Send command to DAC8551
 * @param hdac: Pointer to DAC8551 handle
 * @param command: Command byte
 * @param data: 16-bit data
 * @retval DAC8551 status
 */
static DAC8551_StatusTypeDef DAC8551_WriteCommand(DAC8551_HandleTypeDef *hdac, uint8_t command, uint16_t data)
{
    if (hdac == NULL || hdac->hspi == NULL || hdac->cs_port == NULL) {
        return DAC8551_ERROR_INVALID_PARAM;
    }

    uint8_t tx_data[3];

    // Prepare 24-bit command: 4-bit command + 4-bit don't care + 16-bit data
    tx_data[0] = command;              // Command byte
    tx_data[1] = (data >> 8) & 0xFF;   // Data high byte
    tx_data[2] = data & 0xFF;          // Data low byte

    // Pull CS low
    HAL_GPIO_WritePin(hdac->cs_port, hdac->cs_pin, GPIO_PIN_RESET);

    // Send data
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hdac->hspi, tx_data, 3, DAC8551_SPI_TIMEOUT);

    // Pull CS high
    HAL_GPIO_WritePin(hdac->cs_port, hdac->cs_pin, GPIO_PIN_SET);

    // Convert HAL status to DAC8551 status
    return (status == HAL_OK) ? DAC8551_OK : DAC8551_ERROR_SPI;
}

/**
 * @brief Validate DAC handle
 * @param hdac: Pointer to DAC8551 handle
 * @retval true if valid, false otherwise
 */
static bool DAC8551_ValidateHandle(DAC8551_HandleTypeDef *hdac)
{
    return (hdac != NULL && hdac->initialized && hdac->hspi != NULL &&
            hdac->cs_port != NULL && hdac->vref > 0.0f);
}
