#ifndef __EEPROM_CONTROL_H
#define __EEPROM_CONTROL_H

#include <stdint.h>

/* Base address of the Flash sectors */
#ifdef STM32H7xx
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbyte */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbyte */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbyte */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbyte */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbyte */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x080a0000) /* Base @ of Sector 5, 128 Kbyte */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x080c0000) /* Base @ of Sector 6, 128 Kbyte */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x080e0000) /* Base @ of Sector 7, 128 Kbyte */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08100000) /* Base @ of Sector 8, 128 Kbyte */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x08120000) /* Base @ of Sector 9, 128 Kbyte */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x08140000) /* Base @ of Sector 10, 128 Kbyte */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x08160000) /* Base @ of Sector 11, 128 Kbyte */
#define ADDR_FLASH_SECTOR_12    ((uint32_t)0x08180000) /* Base @ of Sector 12, 128 Kbyte */
#define ADDR_FLASH_SECTOR_13    ((uint32_t)0x081a0000) /* Base @ of Sector 13, 128 Kbyte */
#define ADDR_FLASH_SECTOR_14    ((uint32_t)0x081c0000) /* Base @ of Sector 14, 128 Kbyte */
#define ADDR_FLASH_SECTOR_15    ((uint32_t)0x081e0000) /* Base @ of Sector 15, 128 Kbyte */
#else
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbyte */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbyte */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbyte */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbyte */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbyte */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbyte */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbyte */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbyte */
#endif

#ifdef __cplusplus
 extern "C" {
#endif

   void eeprom_lock();
   void eeprom_unlock();
   int eeprom_write_block(uint32_t address, void* data, uint32_t size);
   int eeprom_write_word(uint32_t address, uint32_t data);
   int eeprom_write_byte(uint32_t address, uint8_t data);
   int eeprom_erase(uint32_t address, uint32_t size);
   int eeprom_wait();
   int eeprom_erase_sector(uint32_t sector);
   /*
    * Functions to lock/unlock pages will perform system reset unless flash is already in desired state
    */
   int eeprom_write_unlock(uint32_t wrp_sectors);
   int eeprom_write_lock(uint32_t wrp_sectors);
   uint32_t eeprom_write_protection(uint32_t wrp_sectors);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_CONTROL_H */
