#include "device.h"
#include "Flash_S25FL.h"
#include <string.h>

#ifdef USE_SPI_FLASH

//_____ External Definitions _______________________________________________________________________
// Pin Mappings
#define Flash_Select()				HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port,  FLASH_nCS_Pin, 	GPIO_PIN_RESET)
#define Flash_Deselect()			HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port,  FLASH_nCS_Pin, 	GPIO_PIN_SET)	
#define Flash_Hold()					HAL_GPIO_WritePin(FLASH_HOLD_GPIO_Port, FLASH_HOLD_Pin, GPIO_PIN_RESET)
#define Flash_Release()				HAL_GPIO_WritePin(FLASH_HOLD_GPIO_Port, FLASH_HOLD_Pin, GPIO_PIN_SET)
#define Flash_WP_Disable()		HAL_GPIO_WritePin(FLASH_WP_GPIO_Port,  	FLASH_WP_Pin, 	GPIO_PIN_RESET)
#define Flash_WP_Enable()			HAL_GPIO_WritePin(FLASH_WP_GPIO_Port,  	FLASH_WP_Pin, 	GPIO_PIN_SET)

// Instruction Commands
#define INST_WRITE_STATREG			0x01
#define INST_PAGE_PROGRAM			0x02
#define INST_READ_EN				0x03
#define INST_FAST_READ_EN			0x0b
#define INST_WRITE_DIS				0x04
#define INST_READ_STATREG_1			0x05
#define INST_WRITE_EN				0x06
#define INST_READ_STATREG_3			0x33
#define INST_READ_STATREG_2			0x35
#define INST_WRITE_EN_VSTATREG			0x50
#define INST_BURSTWRAP_SET			0x77
#define INST_READ_RDID   			0x9F

#define __nop() __asm("NOP")

void _Flash_writeEN (void);
void _Flash_writeDIS (void);

// ____ SPI Config 
SPI_HandleTypeDef* FLASH_SPIConfig;
#ifdef DEBUG
uint32_t flash_rdid = 0;
#endif

void Flash_read(uint32_t address, uint8_t* data, size_t length){
  uint8_t rgAddress[3];
  uint8_t ucInstruction;

  Flash_Select();
	
  __nop();__nop();__nop();
/* The address can start at any byte location of the memory array. The address is automatically incremented to the next higher address */
/* in sequential order after each byte of data is shifted out. The entire memory can therefore be read out with one single read */
/* instruction and address 000000h provided. When the highest address is reached, the address counter will wrap around and roll back */
/* to 000000h, allowing the read sequence to be continued indefinitely. */

  /* There is also a FAST_READ 0xb instruction which requires dummy cycles after address, 
     default 8 cycles */

  // Build address array
  rgAddress[0] = (address & 0xFF0000) >> 16;
  rgAddress[1] = (address & 0x00FF00) >> 8;
  rgAddress[2] = (address & 0x0000FF) >> 0;

#if 1
  // READ 1-1-1, 0x03, no dummy cycles, up to 50Mhz
  ucInstruction = INST_READ_EN;
  Flash_WP_Disable(); // why?
  // Send and receive data
  HAL_SPI_Transmit(FLASH_SPIConfig, &ucInstruction, 1, 100);
  HAL_SPI_Transmit(FLASH_SPIConfig, rgAddress, 3, 100);
  HAL_SPI_Receive(FLASH_SPIConfig,  data, length, 100);
  Flash_WP_Enable(); // why?
#else // if this works, turn SPI speed up to max 108MHz
  // FAST_READ 1-1-1, 0x0b, up to 108Mhz
  ucInstruction = INST_FAST_READ_EN;
  HAL_SPI_Transmit(FLASH_SPIConfig, &ucInstruction, 1, 100);
  HAL_SPI_Transmit(FLASH_SPIConfig, rgAddress, 3, 100);
  HAL_SPI_Receive(FLASH_SPIConfig,  data, 1, 100); // 8 dummy cycles
  HAL_SPI_Receive(FLASH_SPIConfig,  data, length, 100);
#endif
  
  Flash_Deselect();
}

// address must be on a 256-byte boundary
/* "The Page Program command accepts from 1-byte up to 256 consecutive bytes of data (page) to be programmed in one operation. Programming means that bits can either be left at 1, or programmed from 1 to 0. Changing bits from 0 to 1 requires an erase operation." */  
void Flash_write(uint32_t address, uint8_t* data, size_t length){
  uint8_t rgAddress[3];

  // PP Page Program 1-1-1, 0x02, up to 108Mhz
  uint8_t ucInstruction = INST_PAGE_PROGRAM;

  while(length){

    _Flash_writeEN();
    Flash_WP_Disable();
	
    // wait for write enable latch WEL
    while (!(Flash_readStatusReg(INST_READ_STATREG_1) & 0x02)){}

    size_t len = length > 256 ? 256 : length;

    // Build address array
    rgAddress[0] = (address & 0xFF0000) >> 16;
    rgAddress[1] = (address & 0x00FF00) >> 8;
    rgAddress[2] = (address & 0x0000FF) >> 0;

    Flash_Select();
    __nop();__nop();__nop();

    // Send and receive data
    HAL_SPI_Transmit(FLASH_SPIConfig, &ucInstruction, 1, 100);
    HAL_SPI_Transmit(FLASH_SPIConfig, rgAddress, 3, 100);
    HAL_SPI_Transmit(FLASH_SPIConfig, data, len, 100);

    __nop();__nop();__nop();
    Flash_Deselect();

    length -= len;
    data += len;
    address += len;

    // Wait for write to finish
    while (Flash_readStatusReg(INST_READ_STATREG_1) & 0x01){}

    /* While not obvious from the data sheet, multiple page programming only works if enable command is also sent. */
    /* "After a programming command is issued, the programming operation status can be checked using the  */
    /*    Read Status Register 1 command. The WIP bit (SR1V[0]) will indicate  when the programming operation  */
    /*    is completed. The P_ERR bit (SR2V[5]) will indicate if an error occurs in the programming operation */
    /*    that prevents successful completion of programming" */
											   	
    Flash_WP_Enable();
		
    // Check that the write enable latch has been cleared
    while (Flash_readStatusReg(INST_READ_STATREG_1) & 0x02) {_Flash_writeDIS();}
  }
}

uint32_t Flash_readIdentification(){
  uint32_t ucData = 0;
  uint8_t ucInstruction = INST_READ_RDID;
  Flash_Select();
  Flash_WP_Disable();
  __nop();__nop();__nop();
	
  // Send and receive data
  HAL_SPI_Transmit(FLASH_SPIConfig, &ucInstruction, sizeof ucInstruction, 1000);
  HAL_SPI_Receive(FLASH_SPIConfig,  (uint8_t*)&ucData, sizeof ucData, 1000);
	
  __nop();__nop();__nop();
	
  Flash_WP_Enable();
  Flash_Deselect();
	
  return ucData;
}

//_____ Service Functions
uint8_t Flash_readStatusReg (uint8_t ucInstruction)
{
  uint8_t ucData = 0;
		
  Flash_Select();
  Flash_WP_Disable();
  __nop();__nop();__nop();
	
  // Send and receive data
  HAL_SPI_Transmit(FLASH_SPIConfig, &ucInstruction, sizeof ucInstruction, 1000);
  HAL_SPI_Receive(FLASH_SPIConfig,  &ucData, sizeof ucData, 1000);
	
  __nop();__nop();__nop();
	
  Flash_WP_Enable();
  Flash_Deselect();
	
  return ucData;
}

void Flash_writeStatusReg (uint8_t reg, uint8_t data)
{	
	uint8_t ucInstruction = INST_WRITE_STATREG;
	
	_Flash_writeEN();
	
	Flash_Select();								// Select device
	Flash_WP_Disable();						// Disable write protect	
	
	__nop();__nop();__nop();
	
	// Send and receive data
	HAL_SPI_Transmit(FLASH_SPIConfig, &ucInstruction, sizeof ucInstruction, 1000);
	HAL_SPI_Transmit(FLASH_SPIConfig, &data, sizeof data, 1000);
	
	__nop();__nop();__nop();
	
	Flash_WP_Enable();						// Enable write protect
	Flash_Deselect();							// Deselect device
	
	// Wait for write to finish
	while (Flash_readStatusReg(INST_READ_STATREG_1) & 0x01){}
	
	// Check that the write enable latch has been cleared
	while (Flash_readStatusReg(INST_READ_STATREG_1) & 0x02) {_Flash_writeDIS();}
}

void Flash_S25FL_init (SPI_HandleTypeDef *spiconfig)
{
	// Copy SPI configuration to local variable
	FLASH_SPIConfig = spiconfig;
		
	Flash_Deselect();		  			// Deselect chip
	Flash_Release();	  				// Disable Hold
	Flash_WP_Enable();		  		// Enable Write Protect

#ifdef DEBUG
	flash_rdid = Flash_readIdentification();
#endif
}

//_____ Erase Functions 
/* entire chip erase */
void Flash_BulkErase (void)
{	
	uint8_t ucInstruction = INST_ERASE_CHIP;
	
	_Flash_writeEN();								// Write enable sequence
	
	Flash_Select();									// Select device
	Flash_WP_Disable();							// Disable write protect	
	
	__nop();__nop();__nop();
	
	HAL_SPI_Transmit(FLASH_SPIConfig, &ucInstruction, 1, 100);
	
	__nop();__nop();__nop();
	
	Flash_WP_Enable();							// Enable write protect
	Flash_Deselect();								// Deselect device
	
	// Wait for write to finish
	while (Flash_readStatusReg(INST_READ_STATREG_1) & 0x01){__nop();__nop();__nop();}
	
	// Check that the write enable latch has been cleared
	while (Flash_readStatusReg(INST_READ_STATREG_1) & 0x02) {_Flash_writeDIS();__nop();}
}

/* individual 4 KB sector erase, 32 KB half block sector, 64 KB block sector erase */		
void Flash_erase(uint32_t address, uint8_t cmd){
  uint8_t data[4];
  data[0] = cmd;
  data[1] =  (address & 0xFF0000) >> 16;
  data[2] =  (address & 0x00FF00) >> 8;
  data[3] =  (address & 0x0000FF) >> 0;
	
  _Flash_writeEN();
  Flash_Select();
  Flash_WP_Disable();
	
  __nop();__nop();__nop();
  HAL_SPI_Transmit(FLASH_SPIConfig, data, sizeof(data), 100);
  __nop();__nop();__nop();
	
  Flash_WP_Enable();
  Flash_Deselect();
	
  // Wait for write to finish
  while (Flash_readStatusReg(INST_READ_STATREG_1) & 0x01){__nop();__nop();__nop();}
	
  // Check that the write enable latch has been cleared
  while (Flash_readStatusReg(INST_READ_STATREG_1) & 0x02) {_Flash_writeDIS();__nop();}
}

//_____ Sub Functions
void _Flash_writeEN (void)
{
	uint8_t ucInstruction = INST_WRITE_EN;
	
	Flash_Select();								// Select device
	Flash_WP_Disable();						// Disable write protect	
	
	__nop();__nop();__nop();
	
	// Send Command
	HAL_SPI_Transmit(FLASH_SPIConfig, &ucInstruction, sizeof ucInstruction, 100);
	
	__nop();__nop();__nop();
	
	Flash_WP_Enable();						// Enable write protect
	Flash_Deselect();							// Deselect device
}

void _Flash_writeDIS (void)
{	
	uint8_t ucInstruction = INST_WRITE_DIS;
	
	Flash_Select();								// Select device
	Flash_WP_Disable();						// Disable write protect	
	
	__nop();__nop();__nop();
	
	// Send Command
	HAL_SPI_Transmit(FLASH_SPIConfig, &ucInstruction, sizeof ucInstruction, 100);
	
	__nop();__nop();__nop();
	
	Flash_WP_Enable();						// Enable write protect
	Flash_Deselect();							// Deselect device
}

#endif
