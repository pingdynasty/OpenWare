#include "stm32f7xx_hal.h"

// _____ Prototypes ______________________________________________________
void CV_IO_Config(void);
void ADC_Init(void);
void DAC_Init(void);

void CV_Out_A (DAC_HandleTypeDef* hdac, unsigned short millivolts);
void CV_Out_B (DAC_HandleTypeDef* hdac, unsigned short millivolts);
unsigned short CV_In_A(ADC_HandleTypeDef* hadc);
unsigned short CV_In_B(ADC_HandleTypeDef* hadc);

