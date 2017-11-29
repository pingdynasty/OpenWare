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
#define CS_SDOUT_Pin GPIO_PIN_3
#define CS_SDOUT_GPIO_Port GPIOE
#define CS_LRCK_Pin GPIO_PIN_4
#define CS_LRCK_GPIO_Port GPIOE
#define CS_SCKE5_Pin GPIO_PIN_5
#define CS_SCKE5_GPIO_Port GPIOE
#define CS_SDA_Pin GPIO_PIN_6
#define CS_SDA_GPIO_Port GPIOE
#define Flash_CS_Pin GPIO_PIN_6
#define Flash_CS_GPIO_Port GPIOF
#define Flash_CLK_Pin GPIO_PIN_7
#define Flash_CLK_GPIO_Port GPIOF
#define Flash_MISO_Pin GPIO_PIN_8
#define Flash_MISO_GPIO_Port GPIOF
#define Flash_MOSI_Pin GPIO_PIN_9
#define Flash_MOSI_GPIO_Port GPIOF
#define OLED_MOSI_Pin GPIO_PIN_1
#define OLED_MOSI_GPIO_Port GPIOC
#define OLED_RST_Pin GPIO_PIN_2
#define OLED_RST_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_3
#define OLED_DC_GPIO_Port GPIOC
#define CV_IN_A_Pin GPIO_PIN_0
#define CV_IN_A_GPIO_Port GPIOA
#define CV_IN_B_Pin GPIO_PIN_1
#define CV_IN_B_GPIO_Port GPIOA
#define ENC2_CH1_Pin GPIO_PIN_6
#define ENC2_CH1_GPIO_Port GPIOA
#define ENC2_CH2_Pin GPIO_PIN_7
#define ENC2_CH2_GPIO_Port GPIOA
#define CS_SDIN_Pin GPIO_PIN_2
#define CS_SDIN_GPIO_Port GPIOB
#define OLED_SCK_Pin GPIO_PIN_10
#define OLED_SCK_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_12
#define OLED_CS_GPIO_Port GPIOB
#define USB_HOST_PWR_EN_Pin GPIO_PIN_9
#define USB_HOST_PWR_EN_GPIO_Port GPIOC
#define USB_HOST_PWR_FAULT_Pin GPIO_PIN_10
#define USB_HOST_PWR_FAULT_GPIO_Port GPIOA
#define ENC1_CH1_Pin GPIO_PIN_15
#define ENC1_CH1_GPIO_Port GPIOA
#define TR_IN_B_Pin GPIO_PIN_10
#define TR_IN_B_GPIO_Port GPIOC
#define TR_IN_B_EXTI_IRQn EXTI15_10_IRQn
#define TR_IN_A_Pin GPIO_PIN_11
#define TR_IN_A_GPIO_Port GPIOC
#define TR_IN_A_EXTI_IRQn EXTI15_10_IRQn
#define TR_OUT_A_Pin GPIO_PIN_12
#define TR_OUT_A_GPIO_Port GPIOC
#define TR_OUT_B_Pin GPIO_PIN_2
#define TR_OUT_B_GPIO_Port GPIOD
#define CS_CS_Pin GPIO_PIN_9
#define CS_CS_GPIO_Port GPIOG
#define CS_RST_Pin GPIO_PIN_10
#define CS_RST_GPIO_Port GPIOG
#define ENC1_SW_Pin GPIO_PIN_14
#define ENC1_SW_GPIO_Port GPIOG
#define ENC1_SW_EXTI_IRQn EXTI15_10_IRQn
#define ENC1_CH2_Pin GPIO_PIN_3
#define ENC1_CH2_GPIO_Port GPIOB
#define ENC2_SW_Pin GPIO_PIN_4
#define ENC2_SW_GPIO_Port GPIOB
#define ENC2_SW_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */

#define USB_HOST_RX_BUFF_SIZE   64  /* Max Received data 64 bytes */

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
