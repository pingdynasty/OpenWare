#ifndef __Resource_h__
#define __Resource_h__

#include <stdint.h>
#include <cstddef>
#include <cstring>
#include "device.h"
#include "ResourceHeader.h"

#define RESOURCE_SYSTEM_RESOURCE 0x0100 // System settings
#define RESOURCE_USER_PATCH      0x0200 // User patch
#define RESOURCE_MEMORY_MAPPED   0x0400 // Internal MCU flash memory
#define RESOURCE_PORT_MAPPED     0x0800 // External NOR flash

#define RESOURCE_VALID_MAGIC     0xDADADEED
#define RESOURCE_ERASED_MAGIC    0xDADA0000
#define RESOURCE_FREE_MAGIC      0xffffffff

#define INTERNAL_STORAGE_BEGIN ((uint32_t)&_FLASH_STORAGE_BEGIN)
#define INTERNAL_STORAGE_END   ((uint32_t)&_FLASH_STORAGE_END)
#define INTERNAL_STORAGE_SIZE  (INTERNAL_STORAGE_END - INTERNAL_STORAGE_BEGIN)
extern char _FLASH_STORAGE_BEGIN;
extern char _FLASH_STORAGE_END;

class Resource {
private:
  ResourceHeader* header;
public:
  Resource() : header(NULL){}
  Resource(ResourceHeader* header) : header(header){}
  bool isFree(){
    return header && header->magic == RESOURCE_FREE_MAGIC;
  }
  /**
   * A valid resource is not erased and not free.
   */
  bool isValid(){
    return isValidSize() && header->magic == RESOURCE_VALID_MAGIC;
  }
  /**
   * A used resource may be erased but always has a correct size. It is not free.
   */
  bool isUsed(){
    return isValid() || isErased();
  }
  bool isErased(){
    return isValidSize() && header->magic == RESOURCE_ERASED_MAGIC;
  }
  /**
   * The resource size is valid if it is within the boundaries of its storage.
   * @return false if the resource is null, free, or corrupt.
   */
  bool isValidSize(){
    if(isMemoryMapped()){
      return uint32_t(header) + getTotalSize() < INTERNAL_STORAGE_END;
#ifdef USE_SPI_FLASH
    }else{
      return getAddress() < EXTERNAL_STORAGE_SIZE;
#endif
    }
    return false;
  }
  bool isPatch(){
    return isValid() && (header->flags & RESOURCE_USER_PATCH);
  }
  bool isSystemResource(){
    return isValid() && (header->flags & RESOURCE_SYSTEM_RESOURCE);
  }
  bool isMemoryMapped(){
    if(isFree()) // we can't look at the flags because they will be all ones 
      return uint32_t(header) >= INTERNAL_STORAGE_BEGIN && uint32_t(header) < INTERNAL_STORAGE_END;
    else
      return header && (header->flags & RESOURCE_MEMORY_MAPPED);
  }
  /**
   * Returns true if resource only has flags that are set in @param mask
   */
  bool flagsMatch(uint32_t mask){
    return isUsed() && (header->flags & mask) == header->flags;
  }
  /**
   * Returns true if resource has at least those flags that are set in @param mask
   */
  bool flagsContain(uint32_t mask){
    return isUsed() && (header->flags & mask);
  }
  /**
   * Get slot number (stored in the flags), or 0 if not applicable.
   */
  uint8_t getSlot(){
    if(isValid())
      return header->flags & 0xff;
    // todo: if we use MSB instead of LSB for slot then name will always have a terminating 0
    return 0;
  }  
  /**
   * Get data pointer to memory-mapped resource
   */
  uint8_t* getData(){
    return ((uint8_t*)header)+sizeof(ResourceHeader);
  }
#ifdef USE_SPI_FLASH
  /**
   * Get address of port-mapped resource. 
   * Assumes header is immediately followed by a 32-bit address value.
   */
  uint32_t getAddress(){
    return *(uint32_t*)getData();
  }
#endif
  size_t getDataSize(){
    return header->size;
  }
  /**
   * Get size, including header, aligned to 32 or 256 bytes
   * @return 0 if resource is null or free
   */
  size_t getTotalSize(){
    if(!header || header->magic == RESOURCE_FREE_MAGIC)
      return 0;
    if(header->flags & RESOURCE_MEMORY_MAPPED)
      return (header->size+sizeof(ResourceHeader)+31) & ~31;
    else
      return (header->size+sizeof(ResourceHeader)+255) & ~255;
  }
  const char* getName(){
    return header->name;
  }
  void setName(const char* name){
    strncpy(header->name, name, sizeof(header->name));
  }
  void setHeader(ResourceHeader* header){
    this->header = header;
  }
  uint32_t getFlags(){
    return header->flags;
  }
  ResourceHeader* getHeader(){
    return header;
  }
  ResourceHeader* getNextHeader(){
    return (ResourceHeader*)(((uint8_t*)header)+getTotalSize());
  }
};

#endif // __Resource_h__
