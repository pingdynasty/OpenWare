#include "eepromcontrol.h"
#include <string.h> /* for memcpy */
#include "device.h"

extern void device_reset(void);

void eeprom_lock(){
  HAL_FLASH_Lock();
}

int eeprom_wait(){ 
#ifndef STM32H7xx
  return FLASH_WaitForLastOperation(5000);
#else
  /*
   * We're waiting for both banks here, but a better approach could be to track which one was erased / written
   */
  return FLASH_WaitForLastOperation(5000, FLASH_BANK_1) | FLASH_WaitForLastOperation(5000, FLASH_BANK_2);
#endif
}

int eeprom_get_error() {
  return HAL_FLASH_GetError();
}

int eeprom_erase_sector(uint32_t sector) {
  FLASH_EraseInitTypeDef cfg;
  cfg.TypeErase = FLASH_TYPEERASE_SECTORS;
#ifndef OWL_ARCH_F7
  cfg.Banks = FLASH_BANK_1;
#endif
  cfg.Sector = sector;
  cfg.NbSectors = 1;
  cfg.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  uint32_t error;
  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&cfg, &error);
  return status;
}

/*
 * Flash word is 32 bytes, so we'd have to pass pointer or address instead of raw object on H7
 */
int eeprom_write_word(uint32_t address, uint32_t data){
#ifndef STM32H7xx
  HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
  return status;
#else
  // TODO: not implemented, this is just a stub to allow compiling Genius firmware.
  // This is used to store patches, so we'd have to either change function signature
  // or use block writes for patch storage.
  return 0;
#endif
}
/*
 * This won't work on H7 and we don't use this function. Probably should be removed altogether.
 */
#ifndef STM32H7xx
int eeprom_write_byte(uint32_t address, uint8_t data){

  HAL_StatusTypeDef status =  HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, data);
  return status;
}
#endif

int eeprom_write_block(uint32_t address, void* data, uint32_t size){
  uint32_t* p32 = (uint32_t*)data;
  uint32_t i=0;
  HAL_StatusTypeDef status = HAL_OK;
#ifdef STM32H7xx
  // we are actually writing up to 31 bytes more than 'size'
  // which is okay assuming 'data' points to plenty of unused EXTRAM
  for(; i <= size; i += 32){
    status |= HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address + i, (uint32_t)p32);
    p32 += 32 / sizeof(uint32_t);
  }
#else
  for(;i+4<=size; i+=4)
    status |= eeprom_write_word(address+i, *p32++);
  uint8_t* p8 = (uint8_t*)p32;
  for(;i<size; i++)
    status |= eeprom_write_byte(address+i, *p8++);
#endif
  return eeprom_wait() == HAL_FLASH_ERROR_NONE ? (status != HAL_OK) : -1;
}

void eeprom_unlock(){
  HAL_FLASH_Unlock();
}

int eeprom_erase_address(uint32_t address){
  int ret = 0;
  if(address < ADDR_FLASH_SECTOR_1){
    eeprom_erase_sector(FLASH_SECTOR_0);
    ret = ADDR_FLASH_SECTOR_1 - ADDR_FLASH_SECTOR_0;
  }else if(address < ADDR_FLASH_SECTOR_2){
    eeprom_erase_sector(FLASH_SECTOR_1);
    ret = ADDR_FLASH_SECTOR_2 - ADDR_FLASH_SECTOR_1;
  }else if(address < ADDR_FLASH_SECTOR_3){
    eeprom_erase_sector(FLASH_SECTOR_2);
    ret = ADDR_FLASH_SECTOR_3 - ADDR_FLASH_SECTOR_2;
  }else if(address < ADDR_FLASH_SECTOR_4){
    eeprom_erase_sector(FLASH_SECTOR_3);
    ret = ADDR_FLASH_SECTOR_4 - ADDR_FLASH_SECTOR_3;
  }else if(address < ADDR_FLASH_SECTOR_5){
    eeprom_erase_sector(FLASH_SECTOR_4);
    ret = ADDR_FLASH_SECTOR_5 - ADDR_FLASH_SECTOR_4;
  }else if(address < ADDR_FLASH_SECTOR_6){
    eeprom_erase_sector(FLASH_SECTOR_5);
    ret = ADDR_FLASH_SECTOR_6 - ADDR_FLASH_SECTOR_5;
  }else if(address < ADDR_FLASH_SECTOR_7){
    eeprom_erase_sector(FLASH_SECTOR_6);
    ret = ADDR_FLASH_SECTOR_7 - ADDR_FLASH_SECTOR_6;
#ifdef FLASH_SECTOR_8
  }else if(address < ADDR_FLASH_SECTOR_8){
    eeprom_erase_sector(FLASH_SECTOR_7);
    ret = ADDR_FLASH_SECTOR_8 - ADDR_FLASH_SECTOR_7;
  }else if(address < ADDR_FLASH_SECTOR_9){
    eeprom_erase_sector(FLASH_SECTOR_8);
    ret = ADDR_FLASH_SECTOR_9 - ADDR_FLASH_SECTOR_8;
  }else if(address < ADDR_FLASH_SECTOR_10){
    eeprom_erase_sector(FLASH_SECTOR_9);
    ret = ADDR_FLASH_SECTOR_10 - ADDR_FLASH_SECTOR_9;
  }else if(address < ADDR_FLASH_SECTOR_11){
    eeprom_erase_sector(FLASH_SECTOR_10);
    ret = ADDR_FLASH_SECTOR_11 - ADDR_FLASH_SECTOR_10;
#ifdef ADDR_FLASH_SECTOR_12
  }else if(address < ADDR_FLASH_SECTOR_12){
    eeprom_erase_sector(FLASH_SECTOR_11);
    ret = ADDR_FLASH_SECTOR_12 - ADDR_FLASH_SECTOR_11;
  }else if(address < ADDR_FLASH_SECTOR_13){
    eeprom_erase_sector(FLASH_SECTOR_12);
    ret = ADDR_FLASH_SECTOR_13 - ADDR_FLASH_SECTOR_12;
  }else if(address < ADDR_FLASH_SECTOR_14){
    eeprom_erase_sector(FLASH_SECTOR_13);
    ret = ADDR_FLASH_SECTOR_14 - ADDR_FLASH_SECTOR_13;
  }else if(address < ADDR_FLASH_SECTOR_15){
    eeprom_erase_sector(FLASH_SECTOR_14);
    ret = ADDR_FLASH_SECTOR_15 - ADDR_FLASH_SECTOR_14;
  }else if(address < FLASH_END){
    eeprom_erase_sector(FLASH_SECTOR_15);
    ret = FLASH_END - ADDR_FLASH_SECTOR_15;
#else /* define FLASH_SECTOR_12 */
  }else if(address < FLASH_END){
    eeprom_erase_sector(FLASH_SECTOR_11);
    ret = FLASH_END - ADDR_FLASH_SECTOR_11;
#endif /* define FLASH_SECTOR_12 */
#else
  }else if(address < FLASH_END){
    eeprom_erase_sector(FLASH_SECTOR_7);
    ret = FLASH_END - ADDR_FLASH_SECTOR_7;
#endif /* define FLASH_SECTOR_8 */
  }
  return ret;
}

int eeprom_erase(uint32_t address, uint32_t size){
  int32_t remain = size;
  while(remain > 0){
    size_t len = eeprom_erase_address(address);
    remain -= len;
    address += len;
  }
  return eeprom_wait() == HAL_FLASH_ERROR_NONE ? 0 : -1;
}

int eeprom_write_unlock(uint32_t wrp_sectors){
  int ret = HAL_OK;
  FLASH_OBProgramInitTypeDef OptionBytes;
  /* Disable write protected pages */
  OptionBytes.OptionType = OPTIONBYTE_WRP;
  OptionBytes.WRPState = OB_WRPSTATE_DISABLE;
  OptionBytes.Banks = FLASH_BANK_1;
  OptionBytes.WRPSector = wrp_sectors;
  HAL_FLASH_OB_Unlock();
  ret = HAL_FLASHEx_OBProgram(&OptionBytes);
  /*
  * This would perform system reset here unless an error is returned
  */
  if (ret == HAL_OK) {
    ret |= HAL_FLASH_OB_Launch();
    device_reset();
  }
  HAL_FLASH_OB_Lock();
  return ret;
}

int eeprom_write_lock(uint32_t wrp_sectors){
  int ret = HAL_OK;
  FLASH_OBProgramInitTypeDef OptionBytes;
  /* Enable write protection */
  OptionBytes.OptionType = OPTIONBYTE_WRP;
  OptionBytes.WRPState = OB_WRPSTATE_ENABLE;
  OptionBytes.Banks = FLASH_BANK_1;
  OptionBytes.WRPSector = wrp_sectors;
  HAL_FLASH_OB_Unlock();
  ret = HAL_FLASHEx_OBProgram(&OptionBytes);
  /*
  * We perform system reset here unless an error is returned
  */
  if (ret == HAL_OK) {
    ret |= HAL_FLASH_OB_Launch();
    device_reset();
  }
  HAL_FLASH_OB_Lock();
    return ret;
}

/*
 * Checks which sectors have write protection option byte set
 */
uint32_t eeprom_write_protection(uint32_t wrp_sectors){
  FLASH_OBProgramInitTypeDef OptionBytes;
  HAL_FLASHEx_OBGetConfig(&OptionBytes);
  return ~OptionBytes.WRPSector & wrp_sectors;
}
