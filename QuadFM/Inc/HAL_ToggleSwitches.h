#ifndef __HAL_TOGGLESWITCHES_H
#define __HAL_TOGGLESWITCHES_H

#ifdef __cplusplus
	extern "C" {
#endif

#include "stm32f4xx_hal.h"


		
		
		
		
void HAL_Toggles_reset(void);
uint16_t HAL_Toggles_read(void);
void HAL_Toggles_Scan(void);
void HAL_Toggles_ChangeCallback(uint16_t usiTogMask);

		
#ifdef __cplusplus
}
#endif
#endif
