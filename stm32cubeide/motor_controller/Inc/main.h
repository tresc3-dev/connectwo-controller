/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
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
#define ENCODER3_CH1_Pin GPIO_PIN_0
#define ENCODER3_CH1_GPIO_Port GPIOA
#define ENCODER3_CH2_Pin GPIO_PIN_1
#define ENCODER3_CH2_GPIO_Port GPIOA
#define ID0_Pin GPIO_PIN_4
#define ID0_GPIO_Port GPIOA
#define ID1_Pin GPIO_PIN_5
#define ID1_GPIO_Port GPIOA
#define ID2_Pin GPIO_PIN_6
#define ID2_GPIO_Port GPIOA
#define ID3_Pin GPIO_PIN_7
#define ID3_GPIO_Port GPIOA
#define STATE_LED_Pin GPIO_PIN_4
#define STATE_LED_GPIO_Port GPIOC
#define DIR0_Pin GPIO_PIN_5
#define DIR0_GPIO_Port GPIOC
#define DIR1_Pin GPIO_PIN_0
#define DIR1_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_1
#define DIR2_GPIO_Port GPIOB
#define DIR3_Pin GPIO_PIN_2
#define DIR3_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_12
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define PWM0_Pin GPIO_PIN_6
#define PWM0_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_7
#define PWM1_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_8
#define PWM2_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_9
#define PWM3_GPIO_Port GPIOC
#define ENCODER0_CH1_Pin GPIO_PIN_8
#define ENCODER0_CH1_GPIO_Port GPIOA
#define ENCODER0_CH2_Pin GPIO_PIN_9
#define ENCODER0_CH2_GPIO_Port GPIOA
#define ENCODER1_CH1_Pin GPIO_PIN_4
#define ENCODER1_CH1_GPIO_Port GPIOB
#define ENCODER1_CH2_Pin GPIO_PIN_5
#define ENCODER1_CH2_GPIO_Port GPIOB
#define ENCODER2_CH1_Pin GPIO_PIN_6
#define ENCODER2_CH1_GPIO_Port GPIOB
#define ENCODER2_CH2_Pin GPIO_PIN_7
#define ENCODER2_CH2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
