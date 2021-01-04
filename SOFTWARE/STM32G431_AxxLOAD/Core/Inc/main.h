/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FANIN_Pin GPIO_PIN_0
#define FANIN_GPIO_Port GPIOA
#define FET1_TEMP_Pin GPIO_PIN_1
#define FET1_TEMP_GPIO_Port GPIOA
#define FET2_TEMP_Pin GPIO_PIN_4
#define FET2_TEMP_GPIO_Port GPIOA
#define PCB_TEMP_Pin GPIO_PIN_7
#define PCB_TEMP_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define FAN_PWM_Pin GPIO_PIN_8
#define FAN_PWM_GPIO_Port GPIOA
#define BILED_1_Pin GPIO_PIN_11
#define BILED_1_GPIO_Port GPIOA
#define DISCHARGE_LED_Pin GPIO_PIN_12
#define DISCHARGE_LED_GPIO_Port GPIOA
#define OVERTEMP_Pin GPIO_PIN_4
#define OVERTEMP_GPIO_Port GPIOB
#define BILED_2_Pin GPIO_PIN_5
#define BILED_2_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
