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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_A1_Pin GPIO_PIN_13
#define GPIO_A1_GPIO_Port GPIOC
#define GPIO_A2_Pin GPIO_PIN_14
#define GPIO_A2_GPIO_Port GPIOC
#define GPIO_A3_Pin GPIO_PIN_15
#define GPIO_A3_GPIO_Port GPIOC
#define GPIO_B1_Pin GPIO_PIN_0
#define GPIO_B1_GPIO_Port GPIOC
#define GPIO_B2_Pin GPIO_PIN_1
#define GPIO_B2_GPIO_Port GPIOC
#define GPIO_B3_Pin GPIO_PIN_2
#define GPIO_B3_GPIO_Port GPIOC
#define GPIO_C1_Pin GPIO_PIN_3
#define GPIO_C1_GPIO_Port GPIOC
#define GPIO_C2_Pin GPIO_PIN_0
#define GPIO_C2_GPIO_Port GPIOA
#define GPIO_C3_Pin GPIO_PIN_1
#define GPIO_C3_GPIO_Port GPIOA
#define GPIO_C4_Pin GPIO_PIN_2
#define GPIO_C4_GPIO_Port GPIOA
#define GPIO_D1_Pin GPIO_PIN_3
#define GPIO_D1_GPIO_Port GPIOA
#define GPIO_D2_Pin GPIO_PIN_4
#define GPIO_D2_GPIO_Port GPIOA
#define GPIO_D3_Pin GPIO_PIN_5
#define GPIO_D3_GPIO_Port GPIOA
#define GPIO_D4_Pin GPIO_PIN_6
#define GPIO_D4_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOC
#define LED3B0_Pin GPIO_PIN_0
#define LED3B0_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_1
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_2
#define SW2_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_12
#define SW3_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_13
#define SW4_GPIO_Port GPIOB
#define DIP4_Pin GPIO_PIN_14
#define DIP4_GPIO_Port GPIOB
#define DIP3_Pin GPIO_PIN_15
#define DIP3_GPIO_Port GPIOB
#define BZ_Pin GPIO_PIN_8
#define BZ_GPIO_Port GPIOC
#define DIP2_Pin GPIO_PIN_15
#define DIP2_GPIO_Port GPIOA
#define DIP1_Pin GPIO_PIN_2
#define DIP1_GPIO_Port GPIOD
#define CAN_LED2_Pin GPIO_PIN_4
#define CAN_LED2_GPIO_Port GPIOB
#define CAN_LED1_Pin GPIO_PIN_7
#define CAN_LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
