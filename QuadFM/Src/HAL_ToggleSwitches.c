#include "HAL_ToggleSwitches.h"

uint16_t usiTogCheck;

// 16-bit Result Format: [0|0|0|0|SW6H|SW6L|SW5H|SW5L|SW4H|SW4L|SW3H|SW3L|SW2H|SW2L|SW1H|SW1L]
uint16_t HAL_Toggles_read(void)
{
	uint16_t usiTogMask = 0;
	
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW1_L_GPIO_Port, SW1_L_Pin)) << 0;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW1_H_GPIO_Port, SW1_H_Pin)) << 1;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW2_L_GPIO_Port, SW2_L_Pin)) << 2;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW2_H_GPIO_Port, SW2_H_Pin)) << 3;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW3_L_GPIO_Port, SW3_L_Pin)) << 4;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW3_H_GPIO_Port, SW3_H_Pin)) << 5;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW4_L_GPIO_Port, SW4_L_Pin)) << 6;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW4_H_GPIO_Port, SW4_H_Pin)) << 7;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW5_L_GPIO_Port, SW5_L_Pin)) << 8;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW5_H_GPIO_Port, SW5_H_Pin)) << 9;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW6_L_GPIO_Port, SW6_L_Pin)) << 10;
	usiTogMask |= (1-HAL_GPIO_ReadPin(SW6_H_GPIO_Port, SW6_H_Pin)) << 11;
	
	// Return result mask
	return usiTogMask;
}

void HAL_Toggles_Scan(void)
{
	// Read current state
	uint16_t usiTogState = HAL_Toggles_read();
	
	// Compare to previous result and call callback if there has been a change
	if (usiTogCheck != usiTogState) HAL_Toggles_ChangeCallback(usiTogState);
	
	// Update check variable to compare to next check
	usiTogCheck = usiTogState;
}

void HAL_Toggles_ChangeCallback(uint16_t usiTogMask)
{
	
	__nop();
}

void HAL_Toggles_init(void)
{


}
