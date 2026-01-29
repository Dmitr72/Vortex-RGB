/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim3;
//extern TIM_HandleTypeDef htim5;
//extern TIM_HandleTypeDef htim6;
//extern TIM_HandleTypeDef htim7;
//extern TIM_HandleTypeDef htim8;
//extern SPI_HandleTypeDef hspi3;
extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac3;
extern FDCAN_HandleTypeDef hfdcan1;
extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp4;
//extern OPAMP_HandleTypeDef hopamp1;
//extern OPAMP_HandleTypeDef hopamp2;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg;
extern SPI_HandleTypeDef hspi1;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LICHTSC_C_Pin GPIO_PIN_13
#define LICHTSC_C_GPIO_Port GPIOC
#define B_TH_CS_Pin GPIO_PIN_14
#define B_TH_CS_GPIO_Port GPIOC
#define B_DIV_CS_Pin GPIO_PIN_15
#define B_DIV_CS_GPIO_Port GPIOC
#define MSV_ADC_Pin GPIO_PIN_0
#define MSV_ADC_GPIO_Port GPIOC
#define R_ADC_Pin GPIO_PIN_1
#define R_ADC_GPIO_Port GPIOC
#define G_ADC_Pin GPIO_PIN_2
#define G_ADC_GPIO_Port GPIOC
#define B_ADC_Pin GPIO_PIN_3
#define B_ADC_GPIO_Port GPIOC
#define B_SHT_P_Pin GPIO_PIN_0
#define B_SHT_P_GPIO_Port GPIOA
#define B_SHT_N_Pin GPIO_PIN_1
#define B_SHT_N_GPIO_Port GPIOA
#define OPA_B_EN_Pin GPIO_PIN_2
#define OPA_B_EN_GPIO_Port GPIOA
#define BSV_ADC_Pin GPIO_PIN_4
#define BSV_ADC_GPIO_Port GPIOA
#define B_PEN_Pin GPIO_PIN_5
#define B_PEN_GPIO_Port GPIOA
#define OPA_G_EN_Pin GPIO_PIN_6
#define OPA_G_EN_GPIO_Port GPIOA
#define G_SHT_P_Pin GPIO_PIN_7
#define G_SHT_P_GPIO_Port GPIOA
#define G_SHT_N_Pin GPIO_PIN_4
#define G_SHT_N_GPIO_Port GPIOC
#define GSV_ADC_Pin GPIO_PIN_5
#define GSV_ADC_GPIO_Port GPIOC
#define RSV_ADC_Pin GPIO_PIN_2
#define RSV_ADC_GPIO_Port GPIOB
#define G_PEN_Pin GPIO_PIN_10
#define G_PEN_GPIO_Port GPIOB
#define R_SHT_P_Pin GPIO_PIN_11
#define R_SHT_P_GPIO_Port GPIOB
#define G_TH_CS_Pin GPIO_PIN_12
#define G_TH_CS_GPIO_Port GPIOB
#define G_DIV_CS_Pin GPIO_PIN_13
#define G_DIV_CS_GPIO_Port GPIOB
#define OPA_R_EN_Pin GPIO_PIN_14
#define OPA_R_EN_GPIO_Port GPIOB
#define R_SHT_N_Pin GPIO_PIN_15
#define R_SHT_N_GPIO_Port GPIOB
#define R_TH_CS_Pin GPIO_PIN_6
#define R_TH_CS_GPIO_Port GPIOC
#define R_DIV_CS_Pin GPIO_PIN_7
#define R_DIV_CS_GPIO_Port GPIOC
#define R_PEN_Pin GPIO_PIN_8
#define R_PEN_GPIO_Port GPIOC
#define GREEN_Pin GPIO_PIN_15
#define GREEN_GPIO_Port GPIOA
#define RED_Pin GPIO_PIN_10
#define RED_GPIO_Port GPIOC
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_2
#define GYRO_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define print_port	huart1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
