/*
 * rgb.h
 *
 *  Created on: Sep 23, 2025
 *      Author: Optik
 */

#ifndef RGB_RGB_H_
#define RGB_RGB_H_

//#include "system.h"
#include <stdint.h>
#include <stdbool.h>
#include "ad5160.h"
#include "dac8551.h"
//#include "ds18b20.h"
#include "ee.h"

// Forward declarations для HAL структур
//typedef struct __DAC_HandleTypeDef DAC_HandleTypeDef;
//typedef struct __COMP_HandleTypeDef COMP_HandleTypeDef;
//typedef struct __TIM_HandleTypeDef TIM_HandleTypeDef;
//typedef struct __ADC_HandleTypeDef ADC_HandleTypeDef;

//#define RGB_PWM_TIMER htim2	//tim_arr = (HAL_RCC_GetHCLKFreq()/TEC_FREQUENZ)-1;
//#define RGB_PWM_TIMER_FREQUENZ 10000
#define RGB_SHUNT_MULTIPLIER 20	//shunt voltage amplifier multiplier
#define RGB_FIRST_EE_PAGE_NUM 1
#define RGB_EE_CHECK_DIGIT 0xAA55

typedef struct {
	uint8_t threshold;						//value for 8-bit digital resistor for threshold adjustment, loaded from EEPROM during initialization
	uint16_t divider;						//value for 16-bit DAC for setting maximum diode current, loaded from EEPROM during initialization
	uint16_t smolder;						//value for DAC for adjusting cutoff at minimum modulation, loaded from EEPROM during initialization
	uint16_t max_current;					//maximal current in milliamperes

	AD5160_HandleTypeDef* threshold_had;	//8-bit digital resistor for threshold adjustment
	DAC8551_HandleTypeDef* divider_dac;		//16-bit DAC for setting maximum diode current.
	DAC_HandleTypeDef* smolder_dac;			//peripheral DAC for adjusting cutoff at minimum modulation.
	uint32_t smolder_dac_channel;			//peripheral DAC channel
	COMP_HandleTypeDef* smolder_comp;		//peripheral Comparator, controlled by DAC for adjusting cutoff at minimum modulation
	GPIO_TypeDef* port_pen;     			// GPIO port for PWR enable
	uint16_t pin_pen;           			// GPIO pin for PWR enable
	uint32_t* adc_buffer_pwr;				//pointer to the ADC array element - pwr voltage
	uint32_t* adc_buffer_shunt;				//pointer to the ADC array element - voltage on the shunt
	float* mod_temp;						//module temperature
	float* rad_temp;						//radiator temperature
	uint16_t shunt;							//shunt value in milliohms
	uint16_t actual_current;				//current in milliamperes
	uint8_t last_threshold;
	uint16_t last_divider;
	uint16_t last_smolder;
	bool ee_checked;

} RGB_channel;

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
			uint16_t eeprom_page_num);

void RGB_Task(RGB_channel* rgb);
bool setRGBthreshold(RGB_channel* rgb, uint8_t val);
bool setRGBdivider(RGB_channel* rgb, uint16_t val);
bool setRGBsmolder(RGB_channel* rgb, uint16_t val);
uint8_t getRGBthreshold(RGB_channel* rgb);
uint16_t getRGBdivider(RGB_channel* rgb);
uint16_t getRGBsmolder(RGB_channel* rgb);
void setRGBpwmPeriod(RGB_channel* rgb, uint8_t val);
uint16_t getRGBcurrent(RGB_channel* rgb);
uint16_t getRGBsetCurrent(RGB_channel* rgb);
void setRGBpwmMode(RGB_channel* rgb, uint8_t mode);
HAL_StatusTypeDef SaveRGBconfigToEEPROM(EEprom_HandleTypeDef *eeprom, RGB_channel *channel, uint16_t page);

#endif /* RGB_RGB_H_ */
