#include "device.h"
#include "HAL_MAX11300.h"

#include <string.h>
#define Pixi_SPIDMA
  
SPI_HandleTypeDef* MAX11300_SPIConfig;
 
// Variables
static uint8_t rgADCData_Rx[41];
static uint8_t rgDACData_Tx[41];
static volatile uint8_t ucPixiTask;
static volatile uint8_t ucPixiState;
static volatile uint8_t ucPixiBusy;

#define STATE_Idle				0
#define STATE_ContConv		1
#define STATE_Busy				1

#define TASK_readADC			0
#define TASK_setDAC				1


// Task and State Control
#define setPixiTask(task)		ucPixiTask = task
#define getPixiTask()				ucPixiTask
#define setPixiState(state)	ucPixiState = state
#define getPixiState()			ucPixiState
#define setPixiBusy(state)	ucPixiBusy = state
#define getPixiBusy()			ucPixiBusy

// SPI Read/Write bit
#define SPI_Read        	1
#define SPI_Write       	0

// Pin Control
#define pbarCS(state)		HAL_GPIO_WritePin(MAX_CS_GPIO_Port,  MAX_CS_Pin,  (GPIO_PinState)state)
 
//_____ Functions _____________________________________________________________________________________________________
// Port and Chip Setup
void MAX11300_setPortMode(uint8_t port, uint16_t config)
{
	uint8_t rgData[3] = "";
     
	// Split config into two bytes
	rgData[0] = (ADDR_CFGbase+port)<<1 | SPI_Write;
	rgData[1] = (config&0xFF00)>>8;
	rgData[2] = (config&0x00FF);
   
	while(getPixiBusy() == STATE_Busy); // wait until last transfer has finished
	pbarCS(0);
	#ifdef Pixi_SPIDMA_CTRL
		setPixiBusy(STATE_Busy);	
		HAL_SPI_Transmit_DMA(MAX11300_SPIConfig, rgData, sizeof rgData);
	#else
		HAL_SPI_Transmit(MAX11300_SPIConfig, rgData, sizeof rgData, 100);
		pbarCS(1);
	#endif
	
	Nop_delay(100000);
}

uint16_t MAX11300_readPortMode(uint8_t port)
{
	uint8_t ucAddress, rgRtnData[2] = "";
	uint16_t usiRtnValue = 0;
	 
	ucAddress = (ADDR_CFGbase+port)<<1 | SPI_Read;
	
	while(getPixiBusy() == STATE_Busy); // wait until last transfer has finished
	pbarCS(0);
	HAL_SPI_Transmit(MAX11300_SPIConfig, &ucAddress, 1, 100);
	HAL_SPI_Receive(MAX11300_SPIConfig, rgRtnData, 2, 100);
	pbarCS(1);
	
	usiRtnValue  = rgRtnData[0]<<8;
	usiRtnValue += rgRtnData[1];
	
	return usiRtnValue;
}

void MAX11300_setDeviceControl(uint16_t config)
{
	uint8_t rgData[3] = "";
	
	rgData[0] = (ADDR_DevCont)<<1 | SPI_Write;
	rgData[1] = (config&0xFF00)>>8;
	rgData[2] = (config&0x00FF);
	
	while(getPixiBusy() == STATE_Busy); // wait until last transfer has finished
	pbarCS(0);	
	#ifdef Pixi_SPIDMA_CTRL
		setPixiBusy(STATE_Busy);	
		HAL_SPI_Transmit_DMA(MAX11300_SPIConfig, rgData, sizeof rgData);
	#else
		HAL_SPI_Transmit(MAX11300_SPIConfig, rgData, sizeof rgData, 100);
		pbarCS(1);
	#endif
	
	Nop_delay(100000);
}

// ADC Functions
uint16_t MAX11300_readADC(uint8_t port)
{
	uint16_t usiRtnValue = 0;
	uint8_t rgData[3] = {((ADDR_ADCbase+port)<<1) | SPI_Read, 0, 0 };
	while(getPixiBusy() == STATE_Busy); // wait until last transfer has finished
	pbarCS(0);
	HAL_SPI_TransmitReceive(MAX11300_SPIConfig, rgData, rgData, 3, 100);
	pbarCS(1);
	usiRtnValue  = (rgData[1]<<8) | rgData[2];
	return usiRtnValue & 0xFFF;
}

void MAX11300_bulkreadADC(void)
{
	rgADCData_Rx[0] = (ADDR_ADCbase)<<1 | SPI_Read;
	while(getPixiBusy() == STATE_Busy); // wait until last transfer has finished
	pbarCS(0);
	setPixiTask(TASK_readADC);
	/* HAL_SPI_Transmit(MAX11300_SPIConfig, &ucAddress, 1, 100); */
	#ifdef Pixi_SPIDMA
	HAL_SPI_TransmitReceive_DMA(MAX11300_SPIConfig, rgADCData_Rx, rgADCData_Rx, 41);
        #else
	HAL_SPI_TransmitReceive(MAX11300_SPIConfig, rgADCData_Rx, rgADCData_Rx, 41, 100);
	pbarCS(1);
	#endif
}
 
// DAC Functions
void MAX11300_setDAC(uint8_t port, uint16_t value)
{
	uint8_t rgData[3] = "";
	 
	rgData[0] = (ADDR_DACbase+port)<<1 | SPI_Write;
	rgData[1] = (value&0xFF00)>>8;
	rgData[2] = (value&0x00FF);
	 	
	while(getPixiBusy() == STATE_Busy); // wait until last transfer has finished
	pbarCS(0);	
	#ifdef Pixi_SPIDMA_CTRL
		setPixiBusy(STATE_Busy);	
		HAL_SPI_Transmit_DMA(MAX11300_SPIConfig, rgData, sizeof rgData);
	#else
		HAL_SPI_Transmit(MAX11300_SPIConfig, rgData, sizeof rgData, 100);
		pbarCS(1);
	#endif
}

void MAX11300_setDACValue(uint8_t ucChannel, uint16_t value){
  rgDACData_Tx[(ucChannel*2)+1]	= (value&0x0F00)>>8;
  rgDACData_Tx[(ucChannel*2)+2] = (value&0x00FF);
}

void MAX11300_bulksetDAC(void)
{
	// Set address
	rgDACData_Tx[0] = ADDR_DACbase<<1 | SPI_Write;	
	while(getPixiBusy() == STATE_Busy); // wait until last transfer has finished
	pbarCS(0);  	
	setPixiTask(TASK_setDAC);
	#ifdef Pixi_SPIDMA
		setPixiBusy(STATE_Busy);	
		HAL_SPI_Transmit_DMA(MAX11300_SPIConfig, rgDACData_Tx, 41);
	#else
		HAL_SPI_Transmit(MAX11300_SPIConfig, rgDACData_Tx, 41, 100);
		pbarCS(1);
	#endif

}

uint16_t MAX11300_readDAC(uint8_t port)
{
	uint8_t ucAddress, rgRtnData[2] = "";
	uint16_t usiRtnValue = 0;
	 
	ucAddress = (ADDR_DACbase+port)<<1 | SPI_Read;
	
	while(getPixiBusy() == STATE_Busy); // wait until last transfer has finished
	pbarCS(0);
	HAL_SPI_Transmit(MAX11300_SPIConfig, &ucAddress, 1, 100);
	HAL_SPI_Receive(MAX11300_SPIConfig, rgRtnData, 2, 100);
	pbarCS(1);
	
	usiRtnValue  = rgRtnData[0]<<8;
	usiRtnValue += rgRtnData[1];
	
	return usiRtnValue & 0xFFF;
}
 
//_____ Initialisaion _________________________________________________________________________________________________
void MAX11300_init (SPI_HandleTypeDef *spiconfig)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	 
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, MAX_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : MAX_CS_Pin */
	GPIO_InitStruct.Pin = MAX_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(MAX_CS_GPIO_Port, &GPIO_InitStruct);
	 
	MAX11300_SPIConfig = spiconfig;
	setPixiState(STATE_Idle);
	setPixiBusy(STATE_Idle);
}

void MAX11300_startContinuous(void)
{
	setPixiState(STATE_ContConv);
	MAX11300_bulkreadADC();
}

void MAX11300_stopContinuous(void)
{
	setPixiState(STATE_Idle);
}

void MAX11300_TxINTCallback(void)
{
	pbarCS(1);
	setPixiBusy(STATE_Idle);	
	
	// Manage next task
	if (getPixiState() == STATE_ContConv)
	{
		if (getPixiTask() == TASK_setDAC) 	{MAX11300_bulkreadADC();}
		else  															{MAX11300_bulksetDAC();}
	}
}

uint16_t MAX11300_getADCValue(uint8_t ucChannel){
  uint16_t ret = rgADCData_Rx[(ucChannel*2)+1]<<8;
  ret += rgADCData_Rx[(ucChannel*2)+2];
  return ret;
}

void MAX11300_RxINTCallback(void)
{
	pbarCS(1);
	setPixiBusy(STATE_Idle);	
	
	// Manage next task
	if (getPixiState() == STATE_ContConv)
	{
		if (getPixiTask() == TASK_setDAC) 	{MAX11300_bulkreadADC();}
		else  															{MAX11300_bulksetDAC();}
	}
}

void MAX11300_TxRxINTCallback(void){
  pbarCS(1);
  setPixiBusy(STATE_Idle);	
  // Manage next task
  if (getPixiState() == STATE_ContConv){
    if(getPixiTask() == TASK_setDAC)
      MAX11300_bulkreadADC();
    else
      MAX11300_bulksetDAC();
  }
}

void Nop_delay(uint32_t nops){
  while (nops--)
    __asm("NOP");
}

