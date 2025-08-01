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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RS_C13_Pin GPIO_PIN_13
#define RS_C13_GPIO_Port GPIOC
#define RALT_C14_Pin GPIO_PIN_14
#define RALT_C14_GPIO_Port GPIOC
#define LALT_C15_Pin GPIO_PIN_15
#define LALT_C15_GPIO_Port GPIOC
#define LS_A0_Pin GPIO_PIN_0
#define LS_A0_GPIO_Port GPIOA
#define RS_A1_Pin GPIO_PIN_1
#define RS_A1_GPIO_Port GPIOA
#define ALT_A2_Pin GPIO_PIN_2
#define ALT_A2_GPIO_Port GPIOA
#define LS_A3_Pin GPIO_PIN_3
#define LS_A3_GPIO_Port GPIOA
#define LS_A4_Pin GPIO_PIN_4
#define LS_A4_GPIO_Port GPIOA
#define LS_A5_Pin GPIO_PIN_5
#define LS_A5_GPIO_Port GPIOA
#define LD_A6_Pin GPIO_PIN_6
#define LD_A6_GPIO_Port GPIOA
#define LD_A7_Pin GPIO_PIN_7
#define LD_A7_GPIO_Port GPIOA
#define JOY_X_Pin GPIO_PIN_0
#define JOY_X_GPIO_Port GPIOB
#define JOY_Y_Pin GPIO_PIN_1
#define JOY_Y_GPIO_Port GPIOB
#define LD_B2_Pin GPIO_PIN_2
#define LD_B2_GPIO_Port GPIOB
#define LS_B10_Pin GPIO_PIN_10
#define LS_B10_GPIO_Port GPIOB
#define RS_B11_Pin GPIO_PIN_11
#define RS_B11_GPIO_Port GPIOB
#define LD_B12_Pin GPIO_PIN_12
#define LD_B12_GPIO_Port GPIOB
#define LS_B13_Pin GPIO_PIN_13
#define LS_B13_GPIO_Port GPIOB
#define JOY_BTN_Pin GPIO_PIN_14
#define JOY_BTN_GPIO_Port GPIOB
#define ENC_BTN_Pin GPIO_PIN_15
#define ENC_BTN_GPIO_Port GPIOB
#define RS_A12_Pin GPIO_PIN_12
#define RS_A12_GPIO_Port GPIOA
#define RD_A15_Pin GPIO_PIN_15
#define RD_A15_GPIO_Port GPIOA
#define RD_B3_Pin GPIO_PIN_3
#define RD_B3_GPIO_Port GPIOB
#define RD_B4_Pin GPIO_PIN_4
#define RD_B4_GPIO_Port GPIOB
#define RD_B5_Pin GPIO_PIN_5
#define RD_B5_GPIO_Port GPIOB
#define RS_B8_Pin GPIO_PIN_8
#define RS_B8_GPIO_Port GPIOB
#define RS_B9_Pin GPIO_PIN_9
#define RS_B9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
