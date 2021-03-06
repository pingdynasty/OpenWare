#include "HAL_TLC5946.h"
#include <string.h>
#include "device.h"
#include "main.h"

// #define TLC_CONTINUOUS
#ifndef TLC_DEVICES
#define TLC_DEVICES 	3
#endif

#define TLC_CHANNELS 16
#define TLC_DC_BYTES    (TLC_CHANNELS * 3 / 4)
#define TLC_GS_BYTES    (TLC_CHANNELS * 3 / 2)

static uint8_t rgGSbuf[TLC_DEVICES*TLC_GS_BYTES+1];
static uint8_t rgDCbuf[TLC_DEVICES*TLC_DC_BYTES+1] =
  {
   12, 48, 195, 12, 48, 195, 12, 48, 195, 12, 48, 195, // DC=3 Red
   12, 48, 195, 12, 48, 195, 12, 48, 195, 12, 48, 195, // DC=3 Blue
   12, 48, 195, 12, 48, 195, 12, 48, 195, 12, 48, 195, // DC=3 Green
  };
static SPI_HandleTypeDef* TLC5946_SPIConfig;

static const uint8_t rgLED_R[16] = {14,12,9,8,7,4,2,0,15,13,11,10,6,5,3,1};
static const uint8_t rgLED_G[16] = {14,12,10,8,7,4,2,0,15,13,11,9,6,5,3,1};
static const uint8_t rgLED_B[16] = {14,12,9,8,7,4,2,0,15,13,11,10,6,5,3,1};

// see also https://yurichev.com/blog/FAT12/ for optimised ASM

//_____ Functions _____________________________________________________________________________________________________
void TLC5946_SetOutput_GS(uint8_t IC, uint8_t led, uint16_t value)
{
  // 12-bit pwm value
  // aaaaaaaa aaaabbbb bbbbbbbb
  uint32_t bitshift = led*12;
  uint32_t word = bitshift/8;
  uint32_t pos = 8 - (bitshift+4) % 8;
  uint8_t* data = rgGSbuf+IC*TLC_GS_BYTES+word;
  uint8_t mask = 0xfffu >> pos;
  *data = (*data & ~mask) | ((value >> pos) & mask);
  pos = 8 - pos;
  mask = 0xfffu << pos;
  data++;
  *data = (*data & ~mask) | ((value << pos) & mask);
}

void TLC5946_SetOutput_DC(uint8_t ic, uint8_t led, uint8_t value)
{
  // 6-bit dot correction
  // aaaaaabb bbbbcccc ccdddddd
  value <<= 2;
  uint32_t bitshift = led*6;
  uint32_t word = bitshift/8;
  uint32_t pos = bitshift % 8;
  uint8_t mask = 0xfcu >> pos;
  uint8_t* data = rgDCbuf+ic*TLC_DC_BYTES+word;
  *data = (*data & ~mask) | ((value >> pos) & mask);
  pos = 8 - pos;
  mask = 0xfcu << pos;
  data++;
  *data = (*data & ~mask) | ((value << pos) & mask);
}

// Set the same value on one or more TLC clips. This is not called outside of this file.
static void TLC5946_SetOutput_DC_Many(uint8_t* data, uint8_t num_values, uint8_t value)
{
    uint8_t* last_data = data + num_values * 3 / 4;
    uint8_t byte1 = value << 2 | value >> 4;
    uint8_t byte2 = value << 4 | value >> 2;
    uint8_t byte3 = value << 6 | value;
	while (data < last_data) {
		*data++ = byte1;
		*data++ = byte2;
		*data++ = byte3;
	}
}

void TLC5946_TxINTCallback(void)
{
	// Latch pulse
	pXLAT(1);
	/* pBLANK(1); */
	pXLAT(0);

#ifdef TLC_CONTINUOUS
	/* pBLANK(0); */
	HAL_SPI_Transmit_IT(TLC5946_SPIConfig, rgGSbuf, TLC_GS_BYTES*TLC_DEVICES);
#endif
}

void TLC5946_Refresh_GS(void)
{
	// Update Grayscale
	pMODE(Mode_GS);
	pXLAT(0);
	/* pBLANK(1); */
	
#ifdef TLC_CONTINUOUS	
	HAL_SPI_Transmit_IT(TLC5946_SPIConfig, rgGSbuf, TLC_GS_BYTES*TLC_DEVICES);
#else
	HAL_SPI_Transmit(TLC5946_SPIConfig, rgGSbuf, TLC_GS_BYTES*TLC_DEVICES, 100);
	
	// Latch pulse
	pXLAT(1);
	/* HAL_Delay(1); */
	/* pBLANK(0); */
#endif
}

void TLC5946_Refresh_DC(void)
{
	pMODE(Mode_DC);
	pXLAT(0);
	
	// Update Dot Correction
	HAL_SPI_Transmit(TLC5946_SPIConfig, (uint8_t*)rgDCbuf, TLC_DC_BYTES*TLC_DEVICES, 100);
	
	// Latch pulse
	pXLAT(1);
}

// _____ Magus Functions _____
void TLC5946_setRGB(uint8_t LED_ID, uint16_t val_R, uint16_t val_G, uint16_t val_B)
{
	TLC5946_SetOutput_GS(0, rgLED_R[LED_ID-1], val_R);
	TLC5946_SetOutput_GS(2, rgLED_G[LED_ID-1], val_G);
	TLC5946_SetOutput_GS(1, rgLED_B[LED_ID-1], val_B);
}

void TLC5946_setRGB_DC(uint8_t val_R, uint8_t val_G, uint8_t val_B)
{
	TLC5946_SetOutput_DC_Many(rgDCbuf, TLC_CHANNELS, val_R);
	TLC5946_SetOutput_DC_Many(rgDCbuf + TLC_DC_BYTES, TLC_CHANNELS, val_G);
	TLC5946_SetOutput_DC_Many(rgDCbuf + TLC_DC_BYTES * 2, TLC_CHANNELS, val_B);
	
	TLC5946_Refresh_DC();
}

void TLC5946_setAll_DC(uint8_t value)
{
	TLC5946_SetOutput_DC_Many(rgDCbuf, TLC_CHANNELS * TLC_DEVICES, value);
	TLC5946_Refresh_DC();
}

void TLC5946_setAll(uint16_t val_R, uint16_t val_G, uint16_t val_B){
  for(int i=0; i<16; i++){
    TLC5946_SetOutput_GS(0, i, val_R);
    TLC5946_SetOutput_GS(2, i, val_G);
    TLC5946_SetOutput_GS(1, i, val_B);
  }
}

//_____ Initialisaion _________________________________________________________________________________________________
void TLC5946_init (SPI_HandleTypeDef *spiconfig)
{
	// Copy SPI handle to local variable
	TLC5946_SPIConfig = spiconfig;
	
	// 
}
