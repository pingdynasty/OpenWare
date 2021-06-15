#include <cstring>
#include <algorithm>
#include "cmsis_os.h"
#include "device.h"
#include "message.h"
#include "Storage.h"
#include "eepromcontrol.h"
#include "callbacks.h"

#ifdef USE_SPI_FLASH
#include "Flash_S25FL.h"

extern char _FLASH_STORAGE_BEGIN;
extern char _FLASH_STORAGE_END;

struct NorHeader : ResourceHeader {
  uint32_t address;
};

static NorHeader nor_index[MAX_SPI_FLASH_HEADERS];
#endif

Storage storage;

void* findFirstFreeBlock(void* begin, void* end, uint32_t align){
  // search backwards for last block that is not written
  uint32_t* p = (uint32_t*)end;
  p -= 2*align/sizeof(uint32_t); // start at two alignment units from end
  while(p > begin && *p == RESOURCE_FREE_MAGIC){
    setProgress(((uint32_t)end-(uint32_t)p)*4095/((uint32_t)end-(uint32_t)begin), "Index");
    p--;
  }
  p += align/sizeof(uint32_t);
  p = (uint32_t*)((uint32_t)p & ~(align-1));
  if(*p == RESOURCE_FREE_MAGIC)
    return p;
  return NULL;
}

#ifdef USE_SPI_FLASH
uint32_t findFirstFreePage(uint32_t begin, uint32_t end, size_t align){
  uint32_t quad[4]; // read 16 bytes at a time (slow but memory efficient)
  uint32_t address = end-align;
  while(address > begin){
    setProgress((end-address)*4095/(end-begin), "Index");
    Flash_read(address, (uint8_t*)quad, sizeof(quad));
    if(RESOURCE_FREE_MAGIC != (quad[0] & quad[1] & quad[2] & quad[3]))
      break;
    address -= sizeof(quad);
  }
  if(address > begin)
    return (address+sizeof(quad) + (align-1)) & ~(align-1) ;
  return end;
}
#endif
  
void Storage::init(){
#ifdef USE_SPI_FLASH
  extern SPI_HandleTypeDef SPI_FLASH_HSPI;
  Flash_S25FL_init(&SPI_FLASH_HSPI);
  memset(nor_index, 0, sizeof(nor_index));
#endif
  index();
}

void Storage::index(){
  memset(resources, 0, sizeof(resources));
  size_t i = 0;
  size_t progress = 0;
#ifdef USE_FLASH
  resources[i].setHeader((ResourceHeader*)(INTERNAL_STORAGE_BEGIN));
  while(i<MAX_RESOURCE_HEADERS && resources[i].isUsed()){
    ResourceHeader* next = resources[i].getNextHeader();
    if(resources[i].isValid())
      i++;
    resources[i].setHeader(next);
    setProgress(progress += 4095/MAX_RESOURCE_HEADERS, "Indexing");
  }
  if(!resources[i].isFree()){
    error(FLASH_ERROR, "Invalid flash resource");
    // set resource to point to first free block, or NULL
    void* p = findFirstFreeBlock(resources[i].getHeader(), (void*)INTERNAL_STORAGE_END, 32);
    resources[i].setHeader((ResourceHeader*)p);
  }
#endif
#ifdef USE_SPI_FLASH
  uint32_t address = 0;
  NorHeader* header = &nor_index[0];
  header->address = address;
  resources[++i].setHeader(header);
  Flash_read(address, (uint8_t*)header, sizeof(ResourceHeader));
  while(i<MAX_RESOURCE_HEADERS && resources[i].isUsed() &&
	address < EXTERNAL_STORAGE_SIZE &&
	header < &nor_index[MAX_SPI_FLASH_HEADERS]){
    address += resources[i].getTotalSize();
    if(resources[i].isValid())
      resources[++i].setHeader(++header);
    header->address = address;
    Flash_read(address, (uint8_t*)header, sizeof(ResourceHeader));
    setProgress(progress += 4095/MAX_RESOURCE_HEADERS, "Indexing");
  }
  if(!resources[i].isFree()){
    error(FLASH_ERROR, "Invalid SPI resource");
    // set resource to point to first free block, or NULL
    address = findFirstFreePage(resources[i].getAddress(), EXTERNAL_STORAGE_SIZE, 256);
    if(address < EXTERNAL_STORAGE_SIZE)
      ((NorHeader*)resources[i].getHeader())->address = address;
    else
      resources[i].setHeader(NULL);
  }
#endif
  setProgress(4095, "Indexing");
  resource_count = i+1;
}

size_t Storage::readResource(Resource* resource, void* data, size_t offset, size_t length){
  size_t ret = 0;
  if(resource){
    if(resource->isMemoryMapped()){
      size_t len = std::min(resource->getDataSize()-offset, length);
      memcpy(data, resource->getData()+offset, len);
      ret = len;
    }else{
#ifdef USE_SPI_FLASH
      uint32_t address = resource->getAddress();
      size_t len = std::min(resource->getDataSize()-offset, length);
      Flash_read(address+sizeof(ResourceHeader)+offset, (uint8_t*)data, len);
      ret = len;
#endif
    }
  }
  return ret;    
}

bool Storage::eraseResource(uint8_t slot){
  Resource* resource = getResourceBySlot(slot);
  if(resource)
    return eraseResource(resource);
  return false;
}

bool Storage::eraseResource(const char* name){
  Resource* resource = getResourceByName(name);
  if(resource)
    return eraseResource(resource);
  return false;
}

// mark as deleted
bool Storage::eraseResource(Resource* resource){
  bool status = false;  
  if(resource->isMemoryMapped()){
#ifdef USE_FLASH
    ResourceHeader* header = resource->getHeader();
    eeprom_unlock();
    eeprom_wait(); // clear flash flags
#ifdef STM32H743xx
    ResourceHeader copy = *header;
    copy.magic = RESOURCE_ERASED_MAGIC;
    status = eeprom_write_block((uint32_t)header, &copy, sizeof(ResourceHeader));
#else
    status = eeprom_write_word((uint32_t)header, RESOURCE_ERASED_MAGIC);
#endif
    eeprom_lock();
#endif
  }else{
#ifdef USE_SPI_FLASH
    uint32_t address = resource->getAddress();
    resource->getHeader()->magic = RESOURCE_ERASED_MAGIC;
    Flash_write(address, (uint8_t*)resource->getHeader(), 4); // write new magic
#endif
  }
  return status;
}

/**
 * Get patch or resource with the given name
 */
Resource* Storage::getResourceByName(const char* name){
  for(size_t i=0; i<resource_count; ++i){
    if(resources[i].isValid()
       && strcmp(name, resources[i].getName()) == 0)
      return &resources[i];
  }
  return NULL;
}

/**
 * Get patch or resource with the given slot number
 */
Resource* Storage::getResourceBySlot(uint8_t slot){
  for(size_t i=0; i<resource_count; ++i){
    if(resources[i].isValid() &&
       (resources[i].getFlags() & 0xff) == slot)
      return &resources[i];
  }
  return NULL;
}

/**
 * Returns first free resource in either memory-mapped or port-mapped storage,
 * depending on @param flags.
 * @return NULL if no free resource found.
 */
Resource* Storage::getFreeResource(uint32_t flags){
  bool mapped = flags & RESOURCE_MEMORY_MAPPED;
  for(size_t i=0; i<MAX_RESOURCE_HEADERS; ++i)
    if(resources[i].isFree() && resources[i].isMemoryMapped() == mapped)
      return &resources[i];
  return NULL;
}

// erase entire FLASH memory
void Storage::erase(uint32_t flags){
#ifdef USE_FLASH
  // tbd: keep system resources
  if(flags & RESOURCE_MEMORY_MAPPED){
    eeprom_unlock();
    eeprom_erase(INTERNAL_STORAGE_BEGIN, INTERNAL_STORAGE_SIZE);
    eeprom_lock();
  }
#endif
#ifdef USE_SPI_FLASH
  if(flags & RESOURCE_PORT_MAPPED){
    const size_t blocksize = (64*1024);
    uint32_t endaddress = EXTERNAL_STORAGE_SIZE;
    Resource* last = getFreeResource(RESOURCE_PORT_MAPPED);
    if(last)
      endaddress = last->getAddress();
    for(uint32_t address=0; address < endaddress; address += blocksize){
      setProgress(address*4095/endaddress, "Erasing");
      Flash_erase(address, ERASE_64KB); // 450 to 1150 mS each
      vTaskDelay(1); // delay for 1 tick
    }
    setProgress(4095, "Erasing");
  }
#endif
  index();
}

void Storage::defrag(void* buffer, size_t size, uint32_t flags){
  uint8_t* ptr = (uint8_t*)buffer;
  if(getUsedSize(flags) > size)
    debugMessage("Not enough RAM to defrag");

  uint32_t offset = 0;
  for(uint8_t i=0; i<resource_count; ++i){
    if(resources[i].isValid() && resources[i].flagsContain(flags)){
      if(offset+resources[i].getTotalSize() < size){
	readResource(&resources[i], ptr+offset,  0, resources[i].getTotalSize());
	offset += resources[i].getTotalSize();
      }
    }
  }
  erase(flags);
  if(flags & RESOURCE_MEMORY_MAPPED){
#ifdef USE_FLASH
    eeprom_unlock();
    eeprom_write_block(INTERNAL_STORAGE_BEGIN, buffer, offset);
    eeprom_lock();
#endif
  }else{
#ifdef USE_SPI_FLASH
    Flash_write(0, (uint8_t*)buffer, offset);
#endif
  }
  index();
}

size_t Storage::writeResourceHeader(uint8_t* dest, const char* name, size_t datasize, uint32_t flags){
  ResourceHeader header;
  header.magic = RESOURCE_VALID_MAGIC;
  header.size = datasize;
  strncpy(header.name, name, sizeof(header.name));
  header.flags = flags;
  memcpy(dest, &header, sizeof(header));
  return sizeof(header);
}

// data length must already include space for ResourceHeader
size_t Storage::writeResource(const char* name, uint8_t* data, size_t datasize, uint32_t flags){
  writeResourceHeader(data, name, datasize, flags);  
  return writeResource((ResourceHeader*)data);
}

/**
 * Write new or overwrite existing resource. Steps:
 * 1. if it exists, erase (indexed by slot number, if present, or name)
 * 2. if there is not enough remaining storage space, defrag
 * 3. flash header and data
 * 4. verify written data
 * 5. rebuild index
 */
size_t Storage::writeResource(ResourceHeader* header){
#ifndef USE_SPI_FLASH
  header->flags |= RESOURCE_MEMORY_MAPPED; // save everything mem mapped
#endif
  size_t length = header->size+sizeof(ResourceHeader);
  uint32_t flags = header->flags;
  uint8_t slot = flags & 0xff;
  if(slot){ // if there is a slot number
    eraseResource(slot); // mark as deleted if it exists
  }else{
    eraseResource(header->name); // mark as deleted if it exists
  }
  uint8_t* data = (uint8_t*)header;
  size_t capacity = getFreeSize(flags);
  if(length > capacity){ // assuming 'header' is located in extram
#ifdef USE_EXTERNAL_RAM
    extern char _EXTRAM_SIZE;
    defrag(data + length, _EXTRAM_SIZE - length, flags);
#else
    // required by devices with no ext mem
    extern char _PATCHRAM_SIZE;
    defrag(data + length, _PATCHRAM_SIZE - length, flags);
#endif
  }
  Resource* dest = getFreeResource(flags);
  if(!dest){
    error(FLASH_ERROR, "No free resources");
    return 0;
  }

  int status = -1;
  if(dest->isMemoryMapped()){
#ifdef USE_FLASH
    eeprom_unlock();
    eeprom_wait();
    status = eeprom_write_block((uint32_t)dest->getHeader(), data, length);
    eeprom_lock();
#endif
  }else{
#ifdef USE_SPI_FLASH
    uint32_t address = dest->getAddress();
    Flash_write(address, data, length);
    Flash_read(address, (uint8_t*)dest->getHeader(), sizeof(ResourceHeader)); // read back resource header
    status = 0;
#endif
  }
  if(status){
    error(FLASH_ERROR, "Flash write failed");
  }else if(dest->getTotalSize() < length){ // allow for storage-specific alignment
    error(FLASH_ERROR, "Size verification failed");
  }else if(!verifyData(dest, data, length)){
    error(FLASH_ERROR, "Data verification failed");
  }
  index(); // rebuild index
  return length;
}

bool Storage::verifyData(Resource* resource, void* data, size_t length){
  if(resource->isMemoryMapped()){
    return memcmp(data, resource->getHeader(), length) == 0;
#ifdef USE_SPI_FLASH
  }else{
    uint32_t quad[4]; // read 16 bytes at a time (slow but memory efficient)
    uint32_t address = resource->getAddress();
    size_t blocks = length/sizeof(quad);
    uint32_t* src = (uint32_t*)data;
    while(blocks--){
      Flash_read(address, (uint8_t*)quad, sizeof(quad));
      if(quad[0] != *src++ || quad[1] != *src++ ||
	 quad[2] != *src++ || quad[3] != *src++)
	return false;
      address += sizeof(quad);
    }
    return true;
#endif
  }
  return false;
}

size_t Storage::getFreeSize(uint32_t flags){
  size_t total = 0;
#ifdef USE_FLASH
  if(flags & RESOURCE_MEMORY_MAPPED){
    Resource* resource = getFreeResource(RESOURCE_MEMORY_MAPPED);
    if(resource)
      total += INTERNAL_STORAGE_END - (uint32_t)resource->getHeader();
  }
#endif
#ifdef USE_SPI_FLASH
  if(flags & RESOURCE_PORT_MAPPED){
    Resource* resource = getFreeResource(RESOURCE_PORT_MAPPED);
    if(resource)
      total += EXTERNAL_STORAGE_SIZE - resource->getAddress();
  }
#endif
  return total;
}

size_t Storage::getTotalCapacity(uint32_t flags){
  size_t total = 0;
#ifdef USE_FLASH
  if(flags & RESOURCE_MEMORY_MAPPED)
    total += INTERNAL_STORAGE_SIZE;
#endif
#ifdef USE_SPI_FLASH
  if(flags & RESOURCE_PORT_MAPPED)
    total += EXTERNAL_STORAGE_SIZE;
#endif
  return total;
}
