/*
 * fdcan.c
 *
 *  Created on: 22.01.2026
 *      Author: Optik
 */

#include "fdcan.h"

void FDCAN_Init(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    // Убедимся, что модуль FDCAN1 успешно инициализируется
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler(); // Обработчик ошибки
    }

    // Настраиваем базовый фильтр, принимающий все сообщения
    sFilterConfig.IdType = FDCAN_STANDARD_ID;          // 11-битный ID
    sFilterConfig.FilterIndex = 0;                    // Индекс фильтра
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;     // Тип фильтра — маска
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Приём в FIFO0
    sFilterConfig.FilterID1 = 0x000;                  // Поле фильтра (ID 0x000)
    sFilterConfig.FilterID2 = 0x000;                  // Маска: принять все

    // Применить настройки фильтра
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler(); // Обработчик ошибки
    }
}

/**
 * @brief Sends an FDCAN message.
 * @param id: The 11-bit CAN ID for the message.
 * @param data: Pointer to the data buffer to be transmitted.
 * @param size: Size of the data buffer in bytes (maximum 8 bytes in classic CAN).
 * @retval None
 */
void FDCAN_SendMessage(uint32_t id, uint8_t *data, uint8_t size) {
    FDCAN_TxHeaderTypeDef TxHeader;

    // Configure the message header
    TxHeader.Identifier = id;                   // Message ID
    TxHeader.IdType = FDCAN_STANDARD_ID;        // Standard 11-bit ID type
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;    // Data frame
    TxHeader.DataLength = (size << 16);         // Data size in bytes
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;     // No bitrate switching
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;      // Classic CAN format
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Add the message to the Tx FIFO Queue
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK) {
        Error_Handler(); // Transmission error
    }
}

/**
 * @brief Processes received FDCAN messages.
 * @param id: The CAN ID of the received message.
 * @param data: Pointer to the received data.
 * @param length: Length of the received data in bytes.
 * @retval None
 */
void ProcessFDCANMessage(uint32_t id, uint8_t *data, uint8_t length) {
    // Example: Identify message sender by CAN ID
    switch (id) {
        case 0x01:
            // Process message from device 1
            break;
        case 0x02:
            // Process message from device 2
            break;
        case 0x03:
            // Process message from device 3
            break;
        case 0x04:
            // Process message from device 4
            break;
        default:
            // Handle unknown messages
            break;
    }
}
