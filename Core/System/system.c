/*
 * system.c
 *
 *  Created on: Oct 29, 2025
 *      Author: Optik
 */

#include "system.h"
#include "usbd_cdc_if.h"
#include "custom_bus.h"
#include "fdcan.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t usb_ready=0;
uint8_t uart_rx_ready=0;
uint8_t uart_tx_ready=0;

EEprom_HandleTypeDef eeprom;

AD5160_HandleTypeDef hadR;
AD5160_HandleTypeDef hadG;
AD5160_HandleTypeDef hadB;
DAC8551_HandleTypeDef hdacR;
DAC8551_HandleTypeDef hdacG;
DAC8551_HandleTypeDef hdacB;
RGB_channel drvR;
RGB_channel drvG;
RGB_channel drvB;

bool color_effect_mode = 0;
bool color_effect_mode_changed = 0;
uint8_t color_effect_count = 0;

uint32_t adc1_buf[7];
uint32_t adc2_buf[5];

bool config_saved = 0;

void systemTask(){
	HAL_IWDG_Refresh(&hiwdg);
	HAL_ADC_Start_DMA(&hadc1, adc1_buf, 7);
	HAL_ADC_Start_DMA(&hadc2, adc2_buf, 5);
}

void systemInit(){

	EE_Init_State ee = eepromInit(&eeprom, &hi2c2, 0xA0, 4096, I2C_MEMADD_SIZE_16BIT, EE_PAGE_SIZE);//ee = EE_ERROR;
	if(ee != EE_OK){
		if(ee == EE_ERROR){
			printf("EEPROM error, initialization process stopped.\n");
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(1000);
			Error_Handler();
		}
		if(ee == EE_NOT_FORMATTED){
			printf("Formatting EEPROM\n");
			HAL_Delay(100);
			if(eepromFormat(&eeprom) != HAL_OK){
				printf("Error formatting EEPROM, initialization process stopped.\n");
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(1000);
				Error_Handler();
			}
			else printf("EEPROM formatted\n");
		}

	}else printf("EEPROM initialization successful\n");

	uint32_t res = BSP_SPI1_Init();
	printf("BSP_SPI1_Init = %lu\n", res);

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_IWDG_Refresh(&hiwdg);

	//Initialization of Laser drivers
	AD5160_Init(&hadR, &hspi1, R_TH_CS_GPIO_Port, R_TH_CS_Pin);
	AD5160_Init(&hadG, &hspi1, G_TH_CS_GPIO_Port, G_TH_CS_Pin);
	AD5160_Init(&hadB, &hspi1, B_TH_CS_GPIO_Port, B_TH_CS_Pin);
	DAC8551_Init(&hdacR, &hspi1, R_DIV_CS_GPIO_Port, R_DIV_CS_Pin, 5);
	DAC8551_Init(&hdacG, &hspi1, G_DIV_CS_GPIO_Port, G_DIV_CS_Pin, 5);
	DAC8551_Init(&hdacB, &hspi1, B_DIV_CS_GPIO_Port, B_DIV_CS_Pin, 5);
//	RGB_Init(&drvR, 1150, &hadR, &hdacR, &hdac4, DAC_CHANNEL_1, &hcomp7, &htim2, TIM_CHANNEL_4, &hadc2, &adc2_buf[1], 200, &ts3, &ts2, &eeprom, RGB_FIRST_EE_PAGE_NUM);
//	RGB_Init(&drvG, 2200, &hadG, &hdacG, &hdac1, DAC_CHANNEL_2, &hcomp5, &htim2, TIM_CHANNEL_3, &hadc2, &adc2_buf[2], 100, &ts1, &ts2, &eeprom, RGB_FIRST_EE_PAGE_NUM + 1);
//	RGB_Init(&drvB, 3600, &hadB, &hdacB, &hdac3, DAC_CHANNEL_2, &hcomp4, &htim2, TIM_CHANNEL_1, &hadc2, &adc2_buf[3], 50, &ts1, &ts2, &eeprom, RGB_FIRST_EE_PAGE_NUM + 2);
	HAL_IWDG_Refresh(&hiwdg);

	HAL_Delay(500);
	HAL_IWDG_Refresh(&hiwdg);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc == &hadc1) HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);
}

/**
 * @brief FDCAN1 RX FIFO 0 message received callback.
 * @param hfdcan: pointer to a FDCAN_HandleTypeDef structure.
 * @param RxFifo0ITs: specifies the pending interrupt.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[64]; // Buffer for received data

        // Retrieve the message from RX FIFO 0
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            // Process the received message
            ProcessFDCANMessage(RxHeader.Identifier, RxData, RxHeader.DataLength >> 16);
        }
    }
}

/**
 * @brief FDCAN1 RX FIFO 1 message received callback.
 * @param hfdcan: pointer to a FDCAN_HandleTypeDef structure.
 * @param RxFifo1ITs: specifies the pending interrupt.
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0) {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[64]; // Buffer for received data

        // Retrieve the message from RX FIFO 1
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK) {
            // Process message
            ProcessFDCANMessage(RxHeader.Identifier, RxData, RxHeader.DataLength >> 16);
        }
    }
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	//if(huart==&huart2){
//		uart_rx_ready = 1;
//
//	}
//}
//
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	//if(huart==&huart2){
//		uart_tx_ready = 1;
//	}
//}
//
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
//    if(huart == &huart2) {
//        printf("Initial UART Error: 0x%lX\n", HAL_UART_GetError(huart));
//        // Только логируем первую ошибку, остальные будем ловить в systemTask
//        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF);
//        huart->ErrorCode = HAL_UART_ERROR_NONE;
//        HAL_UART_StateTypeDef state = HAL_UART_GetState(huart);
//        printf("Uart state %lu\n", state);
//        if(HAL_UART_Receive_IT(&huart2, rx_buffer, RX_BUFFER_SIZE) == HAL_OK) printf("The error has been fixed.\n");
//    }
//}

//void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart2) HAL_UART_Receive_IT(&huart2, rx_buffer, RX_BUFFER_SIZE);
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim == &htim3) colorEffect();
//}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	//HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);
//
//	TIM3->CNT = 0;
//	TIM3->ARR = TIM5->CNT/1360-5;
//	TIM5->CNT = 0;
//	color_effect_count = 0;
//	//colorEffect();
//}
//void printUartStatus() {
//    printf("UART State: %lu\n", HAL_UART_GetState(&huart2));
//    printf("UART Errors: 0x%lX\n", huart2.ErrorCode);
//}

void usbCallback(){
	uint8_t len = usb_ready;
	usb_ready=0;
	if(len!=32) return;
	HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);

	if(rx_buffer[0] == 'D' && rx_buffer[1] == 'a'){	//incoming data
		setData(rx_buffer);
		BuildControlData();
		CDC_Transmit_FS(tx_buffer, 32);
	}

	if(rx_buffer[0] == 'R' && rx_buffer[1] == 'C'){	//request config
		BuildConfigData();
		CDC_Transmit_FS(tx_buffer, 32);
		return;
	}
}

//void resyncUartReception(void) {
//    // 1. Останавливаем текущий прием
//    HAL_UART_AbortReceive_IT(&huart2);
//
//    // 2. Очищаем все флаги
//    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
//
//    // 3. Очищаем приемный регистр
//    volatile uint32_t tmp;
//    while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
//        tmp = huart2.Instance->RDR;
//        HAL_Delay(1);
//    }
//    (void)tmp;
//
//    // 4. Очищаем буфер
//    memset(rx_buffer, 0, RX_BUFFER_SIZE);
//
//    // 5. Перезапускаем прием
//    HAL_UART_Receive_IT(&huart2, rx_buffer, RX_BUFFER_SIZE);
//
//    printf("UART reception resynchronized\n");
//}

//void uartCallback(){
//	uart_rx_ready=0;
//	HAL_UART_Receive_IT(&huart2, rx_buffer, RX_BUFFER_SIZE);
//	HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);
//
//	if(rx_buffer[0] == 'D' && rx_buffer[1] == 'a'){	//incoming data
//		if(color_effect_mode) color_effect_mode = 0;
//		setData(rx_buffer);
//		BuildControlData();
//		HAL_UART_Transmit_IT(&huart2, tx_buffer, 32);
//		return;
//	}
//
//	if(rx_buffer[0] == 'C' && rx_buffer[1] == 'o')
//		if(!color_effect_mode){
//			color_effect_mode_changed = 1;
//			color_effect_mode = 1;
//		}
//
//	if(rx_buffer[0] == 'R' && rx_buffer[1] == 'C'){	//request config
//		BuildConfigData();
//		HAL_UART_Transmit_IT(&huart2, tx_buffer, 32);
//		return;
//	}
//
//	if(!checkCRC(rx_buffer)){
//		resyncUartReception();
//		//HAL_UART_Receive_IT(&huart2, rx_buffer, RX_BUFFER_SIZE);
//		printf("By uartCallback checkCRC rx_buffer error\r\n");
//	}
//}



//void colorEffect(){
//
//	setRGBthreshold(&drvR, rx_buffer[color_effect_count+2]);
//	setRGBthreshold(&drvG, rx_buffer[color_effect_count+3]);
//	setRGBthreshold(&drvB, rx_buffer[color_effect_count+4]);
////	setRGBthreshold(&drvR, 0);
////	setRGBthreshold(&drvG, 0);
////	setRGBthreshold(&drvB, 0);
//	if(color_effect_count < 21) color_effect_count += 3;
//
//	else color_effect_count = 0;
//}

uint16_t crcCalculation(uint8_t* data){
	uint16_t crc = 0;
	for(uint8_t i = 0; i < 32; i++)
		crc += data[i];
	return crc;
}

bool checkCRC(uint8_t* data)
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < 30; i++)
    {
        crc += data[i];
    }
    if (crc != (data[31]<<8 | data[30])) return false;
    return true; // Placeholder: implement CRC check if needed
}

//void BuildConfigData(){
//	memset(tx_buffer, 0, 32);
//	uint16_t val = 0;
//	uint32_t crc = 0;
//
//	tx_buffer[0] = 'C';
//	tx_buffer[1] = 'o';
//	tx_buffer[2] = getRGBthreshold(&drvR);
//	tx_buffer[3] = getRGBthreshold(&drvG);
//	tx_buffer[4] = getRGBthreshold(&drvB);
//	val = getRGBdivider(&drvR);
//	tx_buffer[5] = (uint8_t)(val & 0xFF);
//	tx_buffer[6] = (uint8_t)((val >> 8) & 0xFF);
//	val = getRGBdivider(&drvG);
//	tx_buffer[7] = (uint8_t)(val & 0xFF);
//	tx_buffer[8] = (uint8_t)((val >> 8) & 0xFF);
//	val = getRGBdivider(&drvB);
//	tx_buffer[9] = (uint8_t)(val & 0xFF);
//	tx_buffer[10] = (uint8_t)((val >> 8) & 0xFF);
//	val = getRGBsmolder(&drvR);
//	tx_buffer[11] = (uint8_t)(val & 0xFF);
//	tx_buffer[12] = (uint8_t)((val >> 8) & 0xFF);
//	val = getRGBsmolder(&drvG);
//	tx_buffer[13] = (uint8_t)(val & 0xFF);
//	tx_buffer[14] = (uint8_t)((val >> 8) & 0xFF);
//	val = getRGBsmolder(&drvB);
//	tx_buffer[15] = (uint8_t)(val & 0xFF);
//	tx_buffer[16] = (uint8_t)((val >> 8) & 0xFF);
//	tx_buffer[17] = getTECset_current(&tec1);
//	tx_buffer[18] = getTECset_current(&tec2);
//	tx_buffer[19] = getTECset_temp(&tec1);
//	tx_buffer[20] = getTECset_temp(&tec2);
//
//	crc = crcCalculation(tx_buffer);
//	tx_buffer[30] = (uint8_t)(crc & 0xFF);
//	tx_buffer[31] = (uint8_t)((crc >> 8) & 0xFF);
//}

//void BuildControlData(){
//	memset(tx_buffer, 0, 32);
//	uint32_t crc = 0;
//	uint16_t val = 0;
//
//	if(config_saved){
//		config_saved = 0;
//		tx_buffer[0] = 'c';
//		tx_buffer[1] = 'S';
//		//return;
//	}else{
//		tx_buffer[0] = 'D';
//		tx_buffer[1] = 'a';
//	}
//
//	tx_buffer[2] = getTECtemp(&tec1);
//	tx_buffer[3] = ds18b20_get_temp_int(&ts2);
//	tx_buffer[4] = getTECtemp(&tec2);
//	tx_buffer[5] = getTECcurrent(&tec1);
//	tx_buffer[6] = getTECcurrent(&tec2);
//	tx_buffer[7] = getTECstate(&tec1);
//	tx_buffer[8] = getTECstate(&tec2);
//	val = getRGBcurrent(&drvR);
//	tx_buffer[9] = (uint8_t)(val & 0xFF);
//	tx_buffer[10] = (uint8_t)((val >> 8) & 0xFF);
//	val = getRGBcurrent(&drvG);
//	tx_buffer[11] = (uint8_t)(val & 0xFF);
//	tx_buffer[12] = (uint8_t)((val >> 8) & 0xFF);
//	val = getRGBcurrent(&drvB);
//	tx_buffer[13] = (uint8_t)(val & 0xFF);
//	tx_buffer[14] = (uint8_t)((val >> 8) & 0xFF);
//
//	crc = crcCalculation(tx_buffer);
//	tx_buffer[30] = (uint8_t)(crc & 0xFF);
//	tx_buffer[31] = (uint8_t)((crc >> 8) & 0xFF);
//}

//void setData(uint8_t* data){
//	if(!checkCRC(data)){
//		printf("By setData checkCRC error\r\n");
//		return;
//	}
//	if(data[24]){
//		setRGBpwmPeriod(&drvR, data[17]);
//		setRGBpwmPeriod(&drvG, data[18]);
//		setRGBpwmPeriod(&drvB, data[19]);
//		HAL_Delay(10);
//		setRGBsmolder(&drvR, 0);
//		setRGBsmolder(&drvG, 0);
//		setRGBsmolder(&drvB, 0);
//		setRGBthreshold(&drvR, 255);
//		setRGBthreshold(&drvG, 255);
//		setRGBthreshold(&drvB, 255);
//
//	}
//	else{
//		setRGBthreshold(&drvR, data[2]);
//		setRGBthreshold(&drvG, data[3]);
//		setRGBthreshold(&drvB, data[4]);
//		setRGBsmolder(&drvR, (data[11]<<8) | data[12]);
//		setRGBsmolder(&drvG, (data[13]<<8) | data[14]);
//		setRGBsmolder(&drvB, (data[15]<<8) | data[16]);
//		setRGBpwmPeriod(&drvR, 100);
//		setRGBpwmPeriod(&drvG, 100);
//		setRGBpwmPeriod(&drvB, 100);
//	}
//	setRGBdivider(&drvR, (data[5]<<8) | data[6]);
//	setRGBdivider(&drvG, (data[7]<<8) | data[8]);
//	setRGBdivider(&drvB, (data[9]<<8) | data[10]);
//	setTECtemp(&tec1, data[20]);
//	setTECtemp(&tec2, data[21]);
//	setTECcurrent(&tec1, data[22]);
//	setTECcurrent(&tec2, data[23]);
//	//setRGBpwmMode(&drvR, data[24]);
//	//setRGBpwmMode(&drvG, data[24]);
//	//setRGBpwmMode(&drvB, data[24]);
//	if(data[29] > 0)
//		if(saveConfigToEE())
//			config_saved = 1;
//}


bool saveConfigToEE(){
	bool result = 1;
//	if(SaveRGBconfigToEEPROM(&eeprom, &drvR, RGB_FIRST_EE_PAGE_NUM) != HAL_OK) result = 0;
//	if(SaveRGBconfigToEEPROM(&eeprom, &drvG, RGB_FIRST_EE_PAGE_NUM + 1) != HAL_OK) result = 0;
//	if(SaveRGBconfigToEEPROM(&eeprom, &drvB, RGB_FIRST_EE_PAGE_NUM + 2) != HAL_OK) result = 0;
//	if(SaveTECconfigToEEPROM(&eeprom, &tec1, TEC_FIRST_EE_PAGE_NUM) != HAL_OK) result = 0;
//	if(SaveTECconfigToEEPROM(&eeprom, &tec2, TEC_FIRST_EE_PAGE_NUM + 1) != HAL_OK) result = 0;
	return result;
}
