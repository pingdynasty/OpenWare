#ifndef __HAL_TLC5946_H
#define __HAL_TLC5946_H

#ifdef __cplusplus
	extern "C" {
#endif

#include "device.h"

// LED Mapping
#define LED_1			15
#define LED_2			14
#define LED_3			13
#define LED_4			12
#define LED_5			11
#define LED_6			10
#define LED_7			9
#define LED_8			8
#define LED_9			0
#define LED_10		1
#define LED_11		2
#define LED_12		3
#define LED_13		4
#define LED_14		5
#define LED_15		6
#define LED_16		7

// Modes
#define Mode_GS 	0
#define Mode_DC		1

// Pin Control
#define pXLAT(state)		HAL_GPIO_WritePin(TLC_XLAT_GPIO_Port,  TLC_XLAT_Pin,  (GPIO_PinState)state)
#define pMODE(state)		HAL_GPIO_WritePin(TLC_MODE_GPIO_Port,  TLC_MODE_Pin,  (GPIO_PinState)state)
#define pBLANK(state)		HAL_GPIO_WritePin(TLC_BLANK_GPIO_Port, TLC_BLANK_Pin, (GPIO_PinState)state)
#define XERR						HAL_GPIO_ReadPin(TLC_XERR_GPIO_Port, 	 TLC_XERR_Pin)
 
// Prototypes
void TLC5946_init (SPI_HandleTypeDef *config);
void TLC5946_SetOutput_GS (unsigned char LED_ID, unsigned short value);
void TLC5946_SetOutput_DC (unsigned char LED_ID, unsigned char value);
void TLC5946_Refresh_GS(void);
void TLC5946_Refresh_DC(void);
void TLC5946_TxINTCallback(void);

#ifdef __cplusplus
}
#endif
#endif
