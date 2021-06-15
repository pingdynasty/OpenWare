#ifndef __Storage_h__
#define __Storage_h__

#include <stdint.h>
#include <cstddef>
#include "device.h"
#include "Resource.h"

class Storage {
private:
  Resource resources[MAX_RESOURCE_HEADERS];
  size_t resource_count = 0;
  Resource* getFreeResource(uint32_t flags);
  bool verifyData(Resource* resource, void* data, size_t length);
public:
  Storage(){}
  void init();
  void index();
  void defrag(void* buffer, size_t size, uint32_t flags);
  size_t getNumberOfResources(){
    return resource_count;
  }
  Resource* getResource(size_t index){
    if(index < resource_count)
      return &resources[index];
    return NULL;
  }
  Resource* getResourceByName(const char* name);  
  Resource* getResourceBySlot(uint8_t slot);  
  size_t readResource(Resource* resource, void* data, size_t offset, size_t length);
  size_t writeResource(ResourceHeader* header);
  size_t writeResource(const char* name, uint8_t* data, size_t datasize, uint32_t flags);
  size_t writeResourceHeader(uint8_t* dest, const char* name, size_t datasize, uint32_t flags);
  bool eraseResource(uint8_t slot);
  bool eraseResource(const char* name);
  bool eraseResource(Resource* resource);
  void erase(uint32_t flags = FLASH_DEFAULT_FLAGS);

  /** 
   * returns number of bytes in use by valid resources.
   */
  size_t getUsedSize(uint32_t flags = FLASH_DEFAULT_FLAGS){
    size_t size = 0;
    for(size_t i=0; i<resource_count; ++i)
      if(resources[i].isValid() && resources[i].flagsContain(flags))
	size += resources[i].getTotalSize();
    return size;
  }
  /**
   * returns number of bytes available to write without erasing or defragmenting.
   */
  size_t getFreeSize(uint32_t flags = FLASH_DEFAULT_FLAGS);
  /**
   * returns number of bytes of storage there are in total.
   */
  size_t getTotalCapacity(uint32_t flags = FLASH_DEFAULT_FLAGS);
};

extern Storage storage;

#endif // __Storage_h__
