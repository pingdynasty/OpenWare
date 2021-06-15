/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

/* #define MAGUS_USBH_MOD */

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
#define CS_SCK_Pin GPIO_PIN_2
#define CS_SCK_GPIO_Port GPIOE
#define CS_SDOUT_Pin GPIO_PIN_3
#define CS_SDOUT_GPIO_Port GPIOE
#define CS_LRCK_Pin GPIO_PIN_4
#define CS_LRCK_GPIO_Port GPIOE
#define CS_SCKE5_Pin GPIO_PIN_5
#define CS_SCKE5_GPIO_Port GPIOE
#define CS_MOSI_Pin GPIO_PIN_6
#define CS_MOSI_GPIO_Port GPIOE
#define SPI_A_SCK_Pin GPIO_PIN_7
#define SPI_A_SCK_GPIO_Port GPIOF
#define SPI_A_MISO_Pin GPIO_PIN_8
#define SPI_A_MISO_GPIO_Port GPIOF
#define SPI_A_MOSI_Pin GPIO_PIN_9
#define SPI_A_MOSI_GPIO_Port GPIOF
#define OSC1_Pin GPIO_PIN_0
#define OSC1_GPIO_Port GPIOH
#define OSC2_Pin GPIO_PIN_1
#define OSC2_GPIO_Port GPIOH
#define OLED_DC_Pin GPIO_PIN_0
#define OLED_DC_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_1
#define OLED_RST_GPIO_Port GPIOA
#define UART_MOSI_Pin GPIO_PIN_2
#define UART_MOSI_GPIO_Port GPIOA
#define UART_MISO_Pin GPIO_PIN_3
#define UART_MISO_GPIO_Port GPIOA
#define OLED_CS_Pin GPIO_PIN_4
#define OLED_CS_GPIO_Port GPIOA
#define PIXI_nCS_Pin GPIO_PIN_5
#define PIXI_nCS_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_6
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define FLASH_HOLD_Pin GPIO_PIN_4
#define FLASH_HOLD_GPIO_Port GPIOC
#define FLASH_nCS_Pin GPIO_PIN_5
#define FLASH_nCS_GPIO_Port GPIOC
#define FLASH_WP_Pin GPIO_PIN_0
#define FLASH_WP_GPIO_Port GPIOB
#define TLC_GSCLK_Pin GPIO_PIN_1
#define TLC_GSCLK_GPIO_Port GPIOB
#define ENC_CHGRDY_Pin GPIO_PIN_2
#define ENC_CHGRDY_GPIO_Port GPIOB
#define CS_CS_Pin GPIO_PIN_10
#define CS_CS_GPIO_Port GPIOB
#define CS_RST_Pin GPIO_PIN_11
#define CS_RST_GPIO_Port GPIOB
#define USB_HOST_PWR_EN_Pin GPIO_PIN_6
#define USB_HOST_PWR_EN_GPIO_Port GPIOC
#define USB_HOST_PWR_FAULT_Pin GPIO_PIN_8
#define USB_HOST_PWR_FAULT_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TLC_BLANK_Pin GPIO_PIN_15
#define TLC_BLANK_GPIO_Port GPIOA
#define ENC_NRST_Pin GPIO_PIN_10
#define ENC_NRST_GPIO_Port GPIOC
#define TLC_XLAT_Pin GPIO_PIN_11
#define TLC_XLAT_GPIO_Port GPIOC
#define CS_SDIN_Pin GPIO_PIN_6
#define CS_SDIN_GPIO_Port GPIOD
#define ENC_nCS_Pin GPIO_PIN_9
#define ENC_nCS_GPIO_Port GPIOG
#define TLC_MODE_Pin GPIO_PIN_11
#define TLC_MODE_GPIO_Port GPIOG
#define SPI_B_CLK_Pin GPIO_PIN_13
#define SPI_B_CLK_GPIO_Port GPIOG
#define SPI_B_MOSI_Pin GPIO_PIN_14
#define SPI_B_MOSI_GPIO_Port GPIOG
#define FLASH_SCK_Pin GPIO_PIN_3
#define FLASH_SCK_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
