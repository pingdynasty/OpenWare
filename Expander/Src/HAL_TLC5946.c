#include "device.h"
#include "HAL_TLC5946.h"


uint8_t rgGSbuf[24] = {0};
uint8_t rgDCbuf[12] = {255,255,255,255,255,255,255,255,255,255,255,255};
SPI_HandleTypeDef* TLC5946_SPIConfig;
	
//_____ Functions _____________________________________________________________________________________________________
void TLC5946_SetOutput_GS (unsigned char LED_ID, unsigned short value)
{
	unsigned char temp;
    uint8_t ucBuffLoc = LED_ID + (LED_ID>>1); // (uint8_t)(LED_ID*1.5);
	
	if (value < 4095)
	{
	  if(LED_ID & 0x01) // bbbbaaaa aaaaaaaa
		{ 
			temp                  = rgGSbuf[ucBuffLoc]; 
			rgGSbuf[ucBuffLoc] 	  = (value&0xF00)>>8; 
			rgGSbuf[ucBuffLoc]   |= (temp&0xF0); 
			rgGSbuf[ucBuffLoc+1]  = (value&0x0FF); 	
		}
	  else              // aaaaaaaa aaaabbbb
		{
			rgGSbuf[ucBuffLoc] 	  = (value&0xFF0)>>4; 
			temp                  = rgGSbuf[ucBuffLoc+1]; 
			rgGSbuf[ucBuffLoc+1]  = (value&0x00F)<<4; 
			rgGSbuf[ucBuffLoc+1] |= (temp&0x0F);
		}			
	}
}

void TLC5946_SetOutput_DC (unsigned char LED_ID, unsigned char value)
{
	unsigned char temp;
	unsigned char ucBuffLoc = (unsigned char)(LED_ID*0.75);
	
	if (value < 64)
	{
		if (LED_ID==0 || LED_ID==4 || LED_ID==8 || LED_ID==12)
		{
			temp 									= rgDCbuf[ucBuffLoc];
			rgDCbuf[ucBuffLoc] 		= (value&0x3F);
			rgDCbuf[ucBuffLoc] 	 |= (temp&0xC0);
		}
		else if (LED_ID==1 || LED_ID==5 ||LED_ID==9 || LED_ID==13)
		{
			temp 									= rgDCbuf[ucBuffLoc];
			rgDCbuf[ucBuffLoc] 		= (value&0x03)<<6;
			rgDCbuf[ucBuffLoc] 	 |= (temp&0x3F);
			
			temp 									= rgDCbuf[ucBuffLoc+1];
			rgDCbuf[ucBuffLoc+1] 	= (value&0x0F);
			rgDCbuf[ucBuffLoc+1] |= (temp&0xF0);
		}
		else if (LED_ID==2 || LED_ID==6 || LED_ID==10 || LED_ID==14)
		{
			temp 									= rgDCbuf[ucBuffLoc];
			rgDCbuf[ucBuffLoc] 		= (value&0x0F)<<4;
			rgDCbuf[ucBuffLoc] 	 |= (temp&0x0F);
			
			temp 									= rgDCbuf[ucBuffLoc+1];
			rgDCbuf[ucBuffLoc+1]	= (value&0xC0)>>6;
			rgDCbuf[ucBuffLoc+1] |= (temp&0xFC);
		}
		else if (LED_ID==3 || LED_ID==7 || LED_ID==11 || LED_ID==15)
		{
			temp 									= rgDCbuf[ucBuffLoc];
			rgDCbuf[ucBuffLoc] 		= value<<2;
			rgDCbuf[ucBuffLoc] 	 |= (temp&0x03);
		}
	}		
}


void TLC5946_TxINTCallback(void)
{
	pXLAT(1);
	pBLANK(1);
	pXLAT(0);

#ifdef TLC_CONTINUOUS
	pBLANK(0);
	HAL_SPI_Transmit_IT(TLC5946_SPIConfig, rgGSbuf, sizeof rgGSbuf);
#endif
}

void TLC5946_Refresh_GS(void)
{
	pXLAT(0);
	// Update Grayscale
	pMODE(Mode_GS);
	pBLANK(0);
#ifdef TLC_CONTINUOUS
	HAL_SPI_Transmit_IT(TLC5946_SPIConfig, rgGSbuf, sizeof rgGSbuf);
#else
	HAL_SPI_Transmit(TLC5946_SPIConfig, rgGSbuf, sizeof rgGSbuf, 100);
	pBLANK(1);
	pXLAT(1);
#endif
}

void TLC5946_Refresh_DC(void)
{
	pXLAT(0);
	// Update Dot Correction
	pMODE(Mode_DC);
	HAL_SPI_Transmit(TLC5946_SPIConfig, rgDCbuf, sizeof rgDCbuf, 100);	
	pXLAT(1);
}

//_____ Initialisaion _________________________________________________________________________________________________
void TLC5946_init (SPI_HandleTypeDef *spiconfig)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TLC_MODE_Pin|TLC_XLAT_Pin, GPIO_PIN_RESET);
	
  /*Configure GPIO pins : TLC_MODE_Pin TLC_XLAT_Pin */
  GPIO_InitStruct.Pin   = TLC_MODE_Pin|TLC_XLAT_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TLC_XERR_Pin */
  GPIO_InitStruct.Pin  = TLC_XERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TLC_XERR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TLC_BLANK_Pin */
  GPIO_InitStruct.Pin   = TLC_BLANK_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(TLC_BLANK_GPIO_Port, &GPIO_InitStruct);
	
	TLC5946_SPIConfig = spiconfig;
}
