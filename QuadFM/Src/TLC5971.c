#include "stm32f4xx_hal.h"
#include <string.h>
#include "TLC5971.h"

uint8_t rgTxBuffer[28];

SPI_HandleTypeDef* TLC5971_SPIConfig;
	
//_____ Functions _____________________________________________________________________________________________________
void TLC5971_SetOutput_GS (unsigned char LED_ID, unsigned char LED_Colour,  unsigned short value)
{
	unsigned char ucBuffLoc = LED_ID*6;
	
	switch (LED_Colour)
	{
		case 0:	// Green
			rgTxBuffer[27-(ucBuffLoc+0)] = (value&0x00FF)>>0;
			rgTxBuffer[27-(ucBuffLoc+1)] = (value&0xFF00)>>8;
			break;
		
		case 1:	// Red
			rgTxBuffer[27-(ucBuffLoc+4)] = (value&0x00FF)>>0;
			rgTxBuffer[27-(ucBuffLoc+5)] = (value&0xFF00)>>8;
			break;
		
		case 2:	// Blue
			rgTxBuffer[27-(ucBuffLoc+2)] = (value&0x00FF)>>0;
			rgTxBuffer[27-(ucBuffLoc+3)] = (value&0xFF00)>>8;
			break;
	}
}	

void TLC5971_SetOutput_BC(unsigned char LED_Colour,  unsigned char value)
{
	unsigned char ucBuffTemp;
		
	if (value < 128)
	{
		switch (LED_Colour)
		{		
			case 2:	// Blue
				ucBuffTemp 		 	= rgTxBuffer[3];	
				rgTxBuffer[3]  = (value<<7);
				rgTxBuffer[3] |= (ucBuffTemp&0x7F);
			
				ucBuffTemp 		 	= rgTxBuffer[2];
				rgTxBuffer[2]  = (value>>1);
				rgTxBuffer[2] |= (ucBuffTemp&0xC0);
				break;
			
			case 1:	// Red
				ucBuffTemp 		 	= rgTxBuffer[2];
				rgTxBuffer[2]  = (value<<6);
				rgTxBuffer[2] |= (ucBuffTemp&0x3F);
			
				ucBuffTemp 		 	= rgTxBuffer[1];
				rgTxBuffer[1]  = (value>>2);
				rgTxBuffer[1] |= (ucBuffTemp&0xE0);
				break;
			
			case 0:	// Green
				ucBuffTemp 		 	= rgTxBuffer[3];		
				rgTxBuffer[3]  = (value&0x7F);
				rgTxBuffer[3] |= (ucBuffTemp&0x80);
				break;
		}
	}		
}

void TLC5971_Settings (unsigned char value)
{
	unsigned char ucBuffTemp = rgTxBuffer[27-26];
	
	rgTxBuffer[1]  = (value<<5);
	rgTxBuffer[1] |= (ucBuffTemp&0x1F);
	rgTxBuffer[0]	 = (value>>3);
}
void TLC5971_Update(void)
{		
	rgTxBuffer[0] = (0x25<<2);	// Add write byte to start
	HAL_SPI_Transmit(TLC5971_SPIConfig, rgTxBuffer, sizeof rgTxBuffer, 100);
}

//_____ Initialisaion _________________________________________________________________________________________________
void TLC5971_init(SPI_HandleTypeDef *spiconfig)
{
	TLC5971_SPIConfig = spiconfig;
	memset(rgTxBuffer, 0, sizeof rgTxBuffer);
}

