/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define L_SPEED_Pin GPIO_PIN_0
#define L_SPEED_GPIO_Port GPIOA
#define R_SPEED_Pin GPIO_PIN_1
#define R_SPEED_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOA
#define R_MOTOR1_Pin GPIO_PIN_0
#define R_MOTOR1_GPIO_Port GPIOB
#define R_MOTOR2_Pin GPIO_PIN_1
#define R_MOTOR2_GPIO_Port GPIOB
#define L_MOTOR2_Pin GPIO_PIN_2
#define L_MOTOR2_GPIO_Port GPIOB
#define L_MOTOR1_Pin GPIO_PIN_10
#define L_MOTOR1_GPIO_Port GPIOB
#define ECHO_Pin GPIO_PIN_8
#define ECHO_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_9
#define TRIG_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
