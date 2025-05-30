/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

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

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWMout_Pin GPIO_PIN_1
#define PWMout_GPIO_Port GPIOA
#define acsIN_Pin GPIO_PIN_2
#define acsIN_GPIO_Port GPIOA
#define ADXL_SCL_Pin GPIO_PIN_10
#define ADXL_SCL_GPIO_Port GPIOB
#define ADXL_SDA_Pin GPIO_PIN_11
#define ADXL_SDA_GPIO_Port GPIOB
#define enRED_Pin GPIO_PIN_12
#define enRED_GPIO_Port GPIOB
#define enBLACK_Pin GPIO_PIN_15
#define enBLACK_GPIO_Port GPIOB
#define pwmRED_Pin GPIO_PIN_8
#define pwmRED_GPIO_Port GPIOA
#define pwmBLACK_Pin GPIO_PIN_10
#define pwmBLACK_GPIO_Port GPIOA
#define ledA_Pin GPIO_PIN_3
#define ledA_GPIO_Port GPIOB
#define ledB_Pin GPIO_PIN_4
#define ledB_GPIO_Port GPIOB
#define ledC_Pin GPIO_PIN_5
#define ledC_GPIO_Port GPIOB
#define LORA_TX_Pin GPIO_PIN_6
#define LORA_TX_GPIO_Port GPIOB
#define LORA_RX_Pin GPIO_PIN_7
#define LORA_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
