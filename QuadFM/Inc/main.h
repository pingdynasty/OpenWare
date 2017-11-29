/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CS_SCK_Pin GPIO_PIN_2
#define CS_SCK_GPIO_Port GPIOE
#define CS_SDA_Pin GPIO_PIN_6
#define CS_SDA_GPIO_Port GPIOE
#define EXTSPI_nCS_Pin GPIO_PIN_15
#define EXTSPI_nCS_GPIO_Port GPIOC
#define EXTSPI_SCK_Pin GPIO_PIN_7
#define EXTSPI_SCK_GPIO_Port GPIOF
#define EXTSPI_MISO_Pin GPIO_PIN_8
#define EXTSPI_MISO_GPIO_Port GPIOF
#define EXTSPI_MOSI_Pin GPIO_PIN_9
#define EXTSPI_MOSI_GPIO_Port GPIOF
#define ADC4_Pin GPIO_PIN_10
#define ADC4_GPIO_Port GPIOF
#define ADC3_Pin GPIO_PIN_1
#define ADC3_GPIO_Port GPIOC
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOC
#define ADC1_Pin GPIO_PIN_3
#define ADC1_GPIO_Port GPIOC
#define ADC5_Pin GPIO_PIN_0
#define ADC5_GPIO_Port GPIOA
#define ADC6_Pin GPIO_PIN_1
#define ADC6_GPIO_Port GPIOA
#define ADC7_Pin GPIO_PIN_2
#define ADC7_GPIO_Port GPIOA
#define ADC8_Pin GPIO_PIN_3
#define ADC8_GPIO_Port GPIOA
#define SW1_L_Pin GPIO_PIN_4
#define SW1_L_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_6
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define FLASH_HOLD_Pin GPIO_PIN_4
#define FLASH_HOLD_GPIO_Port GPIOC
#define FLASH_nCS_Pin GPIO_PIN_5
#define FLASH_nCS_GPIO_Port GPIOC
#define SW6_L_Pin GPIO_PIN_1
#define SW6_L_GPIO_Port GPIOB
#define SW6_H_Pin GPIO_PIN_2
#define SW6_H_GPIO_Port GPIOB
#define CS_nCS_Pin GPIO_PIN_10
#define CS_nCS_GPIO_Port GPIOB
#define CS_nRST_Pin GPIO_PIN_11
#define CS_nRST_GPIO_Port GPIOB
#define USB_H_PWR_FAULT_Pin GPIO_PIN_9
#define USB_H_PWR_FAULT_GPIO_Port GPIOC
#define USB_H_PWR_FAULT_EXTI_IRQn EXTI9_5_IRQn
#define SW5_H_Pin GPIO_PIN_8
#define SW5_H_GPIO_Port GPIOA
#define USB_H_PWR_EN_Pin GPIO_PIN_9
#define USB_H_PWR_EN_GPIO_Port GPIOA
#define USB_H_ID_Pin GPIO_PIN_10
#define USB_H_ID_GPIO_Port GPIOA
#define USB_H_DM_Pin GPIO_PIN_11
#define USB_H_DM_GPIO_Port GPIOA
#define USB_H_DP_Pin GPIO_PIN_12
#define USB_H_DP_GPIO_Port GPIOA
#define SW5_L_Pin GPIO_PIN_15
#define SW5_L_GPIO_Port GPIOA
#define SW1_H_Pin GPIO_PIN_10
#define SW1_H_GPIO_Port GPIOC
#define SW2_H_Pin GPIO_PIN_11
#define SW2_H_GPIO_Port GPIOC
#define SW4_H_Pin GPIO_PIN_9
#define SW4_H_GPIO_Port GPIOG
#define SW4_L_Pin GPIO_PIN_10
#define SW4_L_GPIO_Port GPIOG
#define SW2_L_Pin GPIO_PIN_11
#define SW2_L_GPIO_Port GPIOG
#define SW3_H_Pin GPIO_PIN_8
#define SW3_H_GPIO_Port GPIOB
#define SW3_L_Pin GPIO_PIN_9
#define SW3_L_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
