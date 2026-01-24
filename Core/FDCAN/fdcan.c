/*
 * fdcan.c
 *
 *  Created on: 22.01.2026
 *      Author: Optik
 */

#include "fdcan.h"
#include <stdio.h>

// Definitions for FDCAN data handling
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t tx_data[CAN_PACKET_SIZE] = {0}; // Buffer for transmission
uint8_t rx_data[CAN_PACKET_SIZE] = {0}; // Buffer for reception

// Transmission manager initialized with default thresholds
CAN_TransmissionManager_t can_tx_manager = {
    .current_retries = 0,
    .tec_threshold = 50, // TEC threshold (Transmission Error Counter)
    .transmission_enabled = 1
};

// Macro for logging output
#if CAN_LOG_ENABLED
    #define CAN_LOG(...) printf(__VA_ARGS__); fflush(stdout)
#else
    #define CAN_LOG(...)
#endif

// Main FDCAN task execution
void can_task() {
	for(int i = 0; i<64; i++)	tx_data[i] = i;
    // Example of sending data
    CAN_Send(0x100, tx_data);

    FDCAN_ErrorCountersTypeDef ErrorCounters;

    // Получаем счетчики ошибок
    HAL_FDCAN_GetErrorCounters(&can_port, &ErrorCounters);

    // Анализируем
    CAN_LOG("TEC: %u\n", ErrorCounters.TxErrorCnt);         // Счетчик ошибок передачи
    CAN_LOG("REC: %u\n", ErrorCounters.RxErrorCnt);         // Счетчик ошибок приёма
    CAN_LOG("RxErrorPassive: %u\n", ErrorCounters.RxErrorPassive); // 1 если в Error Passive
    CAN_LOG("ErrorLogging: %u\n", ErrorCounters.ErrorLogging);     // Счетчик с последней ошибки
    // Check and recover FDCAN bus state after execution
    //CAN_CheckAndRecover();
}

// Callback for receiving data through FDCAN
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

    // Check for new message in FIFO0
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK) {
            if (RxHeader.DataLength == FDCAN_DLC_BYTES_64) {//HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);
                CAN_LOG("64-byte packet received: ID=0x%03X\n", RxHeader.Identifier);
            } else {
                CAN_LOG("Invalid packet length received\n");
            }
        }
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs){HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);

}

// Callback for notifying FDCAN transmission errors
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);
    uint32_t error_flags = HAL_FDCAN_GetError(hfdcan);

    // Logging detected errors
    CAN_LOG("FDCAN error callback triggered, flags: 0x%08X\n", error_flags);

    // Clear error flags to recover
    __HAL_FDCAN_CLEAR_FLAG(hfdcan, error_flags);

    // Retrieve and log the current error counters
    FDCAN_ErrorCountersTypeDef errors;
    HAL_FDCAN_GetErrorCounters(hfdcan, &errors);

    CAN_LOG("FDCAN Error Counters - Tx Errors: %d, Rx Errors: %d\n", errors.TxErrorCnt, errors.RxErrorCnt);

    // Check if Transmission Error Counter exceeds the threshold
    if (errors.TxErrorCnt >= can_tx_manager.tec_threshold) {
        CAN_LOG("Error threshold exceeded. Recovering CAN bus...\n");
        HAL_FDCAN_Stop(hfdcan);  // Stop the FDCAN module
        HAL_FDCAN_Start(hfdcan);  // Restart the FDCAN module
        can_tx_manager.transmission_enabled = 1;  // Re-enable transmission
        can_tx_manager.current_retries = 0;    // Reset retry attempts
    }
}

// Initialization of FDCAN with filters and notifications
void FDCAN_Init(void) {
    // Configure TX header with default parameters
    TxHeader.Identifier = 0x123;  // Example message ID
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_64;  // Fixed data length
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Set filters for receiving only specific IDs
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;              // Standard ID filtering
    sFilterConfig.FilterIndex = 0;                         // Filter index
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;          // Mask-based filter configuration
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // Route data to FIFO0
    sFilterConfig.FilterID1 = 0x123;                       // Example ID to filter
    sFilterConfig.FilterID2 = 0x7FF;                       // Mask allowing all IDs

    // Apply the filter configuration
    if (HAL_FDCAN_ConfigFilter(&can_port, &sFilterConfig) != HAL_OK) {
        CAN_LOG("Error configuring FDCAN filter\n");
        Error_Handler();
    }

    // Enable the FDCAN module
    if (HAL_FDCAN_Start(&can_port) != HAL_OK) {
        CAN_LOG("Error starting FDCAN\n");
        Error_Handler();
    }

    HAL_FDCAN_ConfigInterruptLines(&can_port, FDCAN_IT_BUS_OFF, FDCAN_INTERRUPT_LINE0);

    // Activate reception notification for FIFO0
    if (HAL_FDCAN_ActivateNotification(&can_port, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        CAN_LOG("Error enabling reception notifications\n");
        Error_Handler();
    }
//    HAL_FDCAN_ActivateNotification(&can_port, FDCAN_IT_BUS_OFF, 0);

    //HAL_FDCAN_ActivateNotification(&can_port, FDCAN_IT_BUS_OFF, 0);
}

// Send FDCAN message with retry control
HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t *data) {
	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&can_port, &TxHeader, data);
	return HAL_OK;
//    if (!can_tx_manager.transmission_enabled) {
//        return HAL_ERROR;  // Transmission disabled, exit early
//    }

    // Validate data buffer
    if (data == NULL) {
        CAN_LOG("Error: Data buffer is NULL!\n");
        return HAL_ERROR;
    }

    // Attempt message transmission within retry limits
    while (can_tx_manager.current_retries < MAX_RETRIES) {
        HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&can_port, &TxHeader, data);

        if (status == HAL_OK) {
            CAN_LOG("Message sent successfully\n");
            can_tx_manager.current_retries = 0; // Reset retry counter
            return HAL_OK;
        } else {
            // Log failed attempt
            can_tx_manager.current_retries++;
            CAN_LOG("Send attempt failed. Retry %d/%d\n",
                    can_tx_manager.current_retries, MAX_RETRIES);
        }
    }

    // Disable transmission if retries exceed limits
    can_tx_manager.transmission_enabled = 0;
    CAN_LOG("Transmission disabled due to excessive retries\n");
    return HAL_ERROR;
}

// Recover FDCAN module in error state
void CAN_CheckAndRecover(void) {
    // Retrieve current error counters
    FDCAN_ErrorCountersTypeDef errors;
    HAL_FDCAN_GetErrorCounters(&can_port, &errors);

    // Attempt to restart FDCAN under certain conditions
    if (errors.TxErrorCnt >= can_tx_manager.tec_threshold) {
        CAN_LOG("Attempting to recover FDCAN due to high TxErrorCnt\n");
        HAL_FDCAN_Stop(&can_port);
        HAL_FDCAN_Start(&can_port);
        can_tx_manager.transmission_enabled = 1;   // Re-adjust transmission flag
        can_tx_manager.current_retries = 0;        // Reset retry counter to zero
    }
}
