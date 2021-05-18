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
#define MAX_nCS_Pin GPIO_PIN_4
#define MAX_nCS_GPIO_Port GPIOA
#define MAX_SCLK_Pin GPIO_PIN_5
#define MAX_SCLK_GPIO_Port GPIOA
#define MAX_DOUT_Pin GPIO_PIN_6
#define MAX_DOUT_GPIO_Port GPIOA
#define MAX_DIN_Pin GPIO_PIN_7
#define MAX_DIN_GPIO_Port GPIOA
#define TLC_BLANK_Pin GPIO_PIN_12
#define TLC_BLANK_GPIO_Port GPIOB
#define TLC_SCLK_Pin GPIO_PIN_13
#define TLC_SCLK_GPIO_Port GPIOB
#define TLC_SOUT_Pin GPIO_PIN_14
#define TLC_SOUT_GPIO_Port GPIOB
#define TLC_SIN_Pin GPIO_PIN_15
#define TLC_SIN_GPIO_Port GPIOB
#define TLC_MODE_Pin GPIO_PIN_8
#define TLC_MODE_GPIO_Port GPIOA
#define TLC_GSCLK_Pin GPIO_PIN_9
#define TLC_GSCLK_GPIO_Port GPIOA
#define TLC_XLAT_Pin GPIO_PIN_10
#define TLC_XLAT_GPIO_Port GPIOA
#define TLC_XERR_Pin GPIO_PIN_11
#define TLC_XERR_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define MAX_CS_GPIO_Port MAX_nCS_GPIO_Port
#define MAX_CS_Pin MAX_nCS_Pin

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
