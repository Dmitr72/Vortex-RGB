/*
 * rgb.c
 *
 *  Created on: Sep 23, 2025
 *      Author: Optik
 */

#include "rgb.h"
#include "system.h"
#include <string.h>


bool RGB_Init(RGB_channel* rgb,
			uint16_t max_current,
			AD5160_HandleTypeDef* threshold_had,
			DAC8551_HandleTypeDef* divider_dac,
			DAC_HandleTypeDef* smolder_dac, uint32_t smolder_dac_channel,
			COMP_HandleTypeDef* smolder_comp,
			uint32_t* adc_buffer_pwr,
			uint32_t* adc_buffer_shunt,
			float* mod_temp,
			float* rad_temp,
			uint16_t shunt,
			EEprom_HandleTypeDef* ee,
			uint16_t eeprom_page_num)
{
	uint8_t buf[EE_PAGE_SIZE] = {0};
	if(eepromReadPage(ee, buf, eeprom_page_num) != HAL_OK) return 0;
//	uint32_t check_digit = buf[EE_PAGE_SIZE-1] << 8 | buf[EE_PAGE_SIZE-2];
//	if(check_digit != RGB_EE_CHECK_DIGIT){
//		rgb->threshold = 0;
//		rgb->divider = 0;
//		rgb->smolder = 0;
//		rgb->ee_checked = 0;
//	}else{
//		rgb->threshold = buf[0];
//		rgb->divider = buf[2] <<8 | buf[1];
//		rgb->smolder = buf[4] <<8 | buf[3];
//		rgb->ee_checked = 1;
//	}
	rgb->threshold = buf[0];
	rgb->divider = buf[2] <<8 | buf[1];
	rgb->smolder = buf[4] <<8 | buf[3];

	rgb->max_current = max_current;
	rgb->threshold_had = threshold_had;
	rgb->divider_dac = divider_dac;
	rgb->smolder_dac = smolder_dac;
	rgb->smolder_dac_channel = smolder_dac_channel;
	rgb->smolder_comp = smolder_comp;
	rgb->adc_buffer_pwr = adc_buffer_pwr;
	rgb->adc_buffer_shunt = adc_buffer_shunt;
	rgb->shunt = shunt;

	DAC_ChannelConfTypeDef sConfig = {0};
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	HAL_DACEx_SelfCalibrate(rgb->smolder_dac, &sConfig, smolder_dac_channel);
	HAL_DAC_SetValue(rgb->smolder_dac, smolder_dac_channel, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(rgb->smolder_dac, rgb->smolder_dac_channel);
	HAL_COMP_Start(rgb->smolder_comp);
	setRGBsmolder(rgb, rgb->smolder);
	setRGBdivider(rgb, rgb->divider);
	setRGBthreshold(rgb, rgb->threshold);
	return 1;
}

//void RGB_Task(RGB_channel* rgb){
//	if(ds18b20_get_temp_int(rgb->ts_module) > 40 || ds18b20_get_temp_int(rgb->ts_radiator) > 55) rgb->temp_error = 1;//пока задаем ограничнгия внутри софта
//	else rgb->temp_error = 0;
//	if(rgb->temp_error){
//		if(rgb->divider) setRGBdivider(rgb, 0);
//	}
//	else{
//		if(!rgb->divider) setRGBdivider(rgb, rgb->divider);
//	}
//}

bool setRGBthreshold(RGB_channel* rgb, uint8_t val){
	rgb->last_threshold = val;
	if(AD5160_SetResistance(rgb->threshold_had, val)!=HAL_OK)	return 0;
	return 1;
}

bool setRGBdivider(RGB_channel* rgb, uint16_t val){
	rgb->last_divider = val;
	if(DAC8551_WriteValue(rgb->divider_dac, val)!=DAC8551_OK)	return 0;
	return 1;
}

bool setRGBsmolder(RGB_channel* rgb, uint16_t val){
	//printbyte(HAL_COMP_GetState(rgb->smolder_comp));
	rgb->last_smolder = val;
//	if(val == 0){
//		if(rgb->smolder_comp->Init.OutputPol == COMP_OUTPUTPOL_NONINVERTED){
//			HAL_COMP_Stop(rgb->smolder_comp);
//			rgb->smolder_comp->Init.OutputPol = COMP_OUTPUTPOL_INVERTED;
//			if (HAL_COMP_Init(&hcomp1) == HAL_OK) {
//				HAL_DAC_SetValue(rgb->smolder_dac, rgb->smolder_dac_channel, DAC_ALIGN_12B_R, 4095);
//				HAL_COMP_Start(rgb->smolder_comp);
//			}
//		}
//	}
//
//	else{
//		//if(HAL_COMP_GetState(rgb->smolder_comp) == HAL_COMP_STATE_READY) HAL_COMP_Start(rgb->smolder_comp);
//
//		if(rgb->smolder_comp->Init.OutputPol == COMP_OUTPUTPOL_INVERTED){
//			HAL_COMP_Stop(rgb->smolder_comp);
//			rgb->smolder_comp->Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
//			if (HAL_COMP_Init(&hcomp1) == HAL_OK) {
//
//				HAL_COMP_Start(rgb->smolder_comp);
//			}
//		}
//		if(HAL_DAC_SetValue(rgb->smolder_dac, rgb->smolder_dac_channel, DAC_ALIGN_12B_R, val)!=HAL_OK) return 0;
//
//	}

	return 1;
}

uint8_t getRGBthreshold(RGB_channel* rgb){
	return rgb->threshold;
}

uint16_t getRGBdivider(RGB_channel* rgb){
	return rgb->divider;
}

uint16_t getRGBsmolder(RGB_channel* rgb){
	return rgb->smolder;
}

//void setRGBpwmPeriod(RGB_channel* rgb, uint8_t val){
//	if(val > 100)	val = 100;
//	uint32_t ccr = 100 - val;
//	__HAL_TIM_SET_COMPARE(rgb->pwm_timer, rgb->pwm_timer_channel, (uint16_t)ccr);
//}

// Return value in milliampers
uint16_t getRGBcurrent(RGB_channel* rgb){
	rgb->actual_current = (uint32_t)(*rgb->adc_buffer_shunt * 25 / rgb->shunt);
	return rgb->actual_current;
}

//uint16_t getRGBsetCurrent(RGB_channel* rgb){
//	return rgb->set_current;
//}

void setRGBpwmMode(RGB_channel* rgb, uint8_t mode){

}

HAL_StatusTypeDef SaveRGBconfigToEEPROM(EEprom_HandleTypeDef *eeprom, RGB_channel *channel, uint16_t page) {
    uint8_t buffer[EE_PAGE_SIZE] = {0}; // Буфер размером с страницу EEPROM

    // Копируем данные в буфер в правильном порядке
    buffer[0] = channel->last_threshold; // 1 байт

    // Преобразуем divider в little-endian (2 байта)
    buffer[1] = (uint8_t)(channel->last_divider & 0xFF);
    buffer[2] = (uint8_t)((channel->last_divider >> 8) & 0xFF);

    // Преобразуем smolder в little-endian (2 байта)
    buffer[3] = (uint8_t)(channel->last_smolder & 0xFF);
    buffer[4] = (uint8_t)((channel->last_smolder >> 8) & 0xFF);

    // Преобразуем set_current в little-endian (2 байта)
    //buffer[5] = (uint8_t)(channel->set_current & 0xFF);
    //buffer[6] = (uint8_t)((channel->set_current >> 8) & 0xFF);

    // Копируем set_current (4 байта) через union для контроля представления
//    union {
//        float f;
//        uint8_t bytes[4];
//    } float_union;
//
//    float_union.f = channel->set_current;
//    for (int i = 0; i < 4; i++) {
//        buffer[5 + i] = float_union.bytes[i]; // Порядок little-endian
//    }

    // Вызываем функцию записи страницы
    return eepromWritePage(eeprom, buffer, page);
}
