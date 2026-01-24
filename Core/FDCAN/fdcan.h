/*
 * fdcan.h
 *
 *  Created on: 22.01.2026
 *      Author: Optik
 */

#ifndef FDCAN_FDCAN_H_
#define FDCAN_FDCAN_H_

#include "main.h"
#define can_port hfdcan1
// Define packet size for FDCAN communication
#define CAN_PACKET_SIZE 64

// Define the maximum number of allowed retries for transmission
#define MAX_RETRIES 3

// Enable or disable logging functionality. Set to 0 to disable logging.
#ifndef CAN_LOG_ENABLED
#define CAN_LOG_ENABLED 1
#endif

// Data structure to manage the state of FDCAN transmissions
typedef struct {
    uint8_t current_retries;    // Current retry attempts
    uint32_t tec_threshold;     // Threshold value for Transmission Error Counter (TEC)
    uint8_t transmission_enabled; // Flag indicating whether transmission is enabled
} CAN_TransmissionManager_t;

// Function prototypes for FDCAN operations
void can_task();
void FDCAN_Init(void);
HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t *data);
void CAN_CheckAndRecover(void);
HAL_StatusTypeDef CAN_Send_Smart(uint32_t id, uint8_t *data, uint8_t length);

#endif /* FDCAN_FDCAN_H_ */
