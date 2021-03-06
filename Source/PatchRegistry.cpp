#include "PatchRegistry.h"
#include "FlashStorage.h"
#include "ProgramManager.h"
#include "ResourceHeader.h"
#include "ProgramHeader.h"
#include "DynamicPatchDefinition.hpp"
#include "message.h"

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

static PatchDefinition emptyPatch("---", 0, 0);

PatchRegistry::PatchRegistry() {}

void PatchRegistry::init() {
  patchCount = 0;
  resourceCount = 0;
  for(int i=0; i<storage.getBlocksTotal(); ++i){
    StorageBlock block = storage.getBlock(i);
    if(block.verify() && block.getDataSize() > 4){
      uint32_t magic = *(uint32_t*)block.getData();
      int id = magic&0x00ff;
      if(id > 0 && id <= MAX_NUMBER_OF_PATCHES){
        patchblocks[id-1] = block;
        patchCount = max(patchCount, id);
      }else if(id > MAX_NUMBER_OF_PATCHES && id <= MAX_NUMBER_OF_PATCHES+MAX_NUMBER_OF_RESOURCES){
        resourceblocks[id-1-MAX_NUMBER_OF_PATCHES] = block;
        resourceCount = max(resourceCount, id - MAX_NUMBER_OF_PATCHES);
      }
    }
  }
}

ResourceHeader* PatchRegistry::getResource(uint8_t index){
  index = index - 1 - MAX_NUMBER_OF_PATCHES;
  if(index < MAX_NUMBER_OF_RESOURCES && resourceblocks[index].verify())
    return (ResourceHeader*)resourceblocks[index].getData();
  return NULL;
}

ResourceHeader* PatchRegistry::getResource(const char* name){
  for(int i=0; i<MAX_NUMBER_OF_RESOURCES; ++i){
    if(resourceblocks[i].verify()){
      ResourceHeader* hdr = (ResourceHeader*)resourceblocks[i].getData();
      if(strcmp(name, hdr->name) == 0)
        return hdr;
    }
  }
  return NULL;
}

unsigned int PatchRegistry::getSlot(ResourceHeader* resource){
  const char* name = resource->name;
  for(int i=0; i<MAX_NUMBER_OF_RESOURCES; ++i){
    if(resourceblocks[i].verify()){
      ResourceHeader* hdr = (ResourceHeader*)resourceblocks[i].getData();
      if(strcmp(name, hdr->name) == 0)
        return i+MAX_NUMBER_OF_PATCHES+1;
    }
  }
  return 0;
}

void* PatchRegistry::getData(ResourceHeader* resource){
  return (uint8_t*)resource + sizeof(ResourceHeader);
}

void PatchRegistry::store(uint8_t index, uint8_t* data, size_t size){
  if(size > storage.getFreeSize() + storage.getDeletedSize())
    return error(FLASH_ERROR, "Insufficient flash available");
  if(size < 4)
    return error(FLASH_ERROR, "Invalid resource size");
#ifdef USE_EXTERNAL_RAM
  extern char _EXTRAM_END, _FLASH_STORAGE_SIZE;
  if(size > storage.getFreeSize())
    storage.defrag(
      (uint8_t*)&_EXTRAM_END - (uint32_t)(&_FLASH_STORAGE_SIZE),
      (uint32_t)(&_FLASH_STORAGE_SIZE));
#endif
  uint32_t* magic = (uint32_t*)data;
  if(*magic == 0xDADAC0DE && index > 0 && index <= MAX_NUMBER_OF_PATCHES){
    // if it is a patch, set the program id
    *magic = (*magic&0xffffff00) | (index&0xff);
    StorageBlock block = storage.append(data, size);
    if(block.verify()){
      debugMessage("Patch stored to flash");
      index = index - 1;
      if(patchblocks[index].verify())
	patchblocks[index].setDeleted(); // delete old patch
      patchblocks[index] = block;
      patchCount = max(patchCount, index+1);
    }else{
      error(FLASH_ERROR, "Failed to verify patch");
    }
  }else if(*magic == 0xDADADEED && index > MAX_NUMBER_OF_PATCHES &&
	   index <= MAX_NUMBER_OF_PATCHES+MAX_NUMBER_OF_RESOURCES){
    // if it is data, set the resource id
    *magic = (*magic&0xffffff00) | (index&0xff);
    StorageBlock block = storage.append(data, size);
    if(block.verify()){
      debugMessage("Resource stored to\nflash"); // Do we want to show this to users?
      index = index - 1 - MAX_NUMBER_OF_PATCHES;
      if(resourceblocks[index].verify())
        resourceblocks[index].setDeleted();
      resourceblocks[index] = block;
      resourceCount = max(resourceCount, index + 1);
    }else{
      error(FLASH_ERROR, "failed to verify resource");
    }
  }else{
    error(PROGRAM_ERROR, "Invalid magic");
  }
}

void PatchRegistry::setDeleted(uint8_t index) {
  if (!index) {
    // 0 is dynamic patch, nothing to delete from storage
    error(PROGRAM_ERROR, "Invalid ID");
  }
  else {
    if (--index < MAX_NUMBER_OF_PATCHES){
      StorageBlock* block = &patchblocks[index];
      if (block->isValidSize()){
        block->setDeleted();
        init();
        debugMessage("Deleted patch", index);
      }
      else {
        error(PROGRAM_ERROR, "Invalid patch");
      }
    }
    else {
      index -= MAX_NUMBER_OF_PATCHES;
      if (index < MAX_NUMBER_OF_RESOURCES) {
        StorageBlock* block = &resourceblocks[index];
        if (block->isValidSize()){
          block->setDeleted();
          init();
          debugMessage("Deleted resource", index);
          onResourceUpdate();
        }
        else {
          error(PROGRAM_ERROR, "Invalid resource");
        }
      }
      else {
          error(PROGRAM_ERROR, "Invalid ID");
      }
    }
  }
}

const char* PatchRegistry::getResourceName(unsigned int index){
  ResourceHeader* hdr = getResource(index);
  if(hdr == NULL)
    return emptyPatch.getName();
  return hdr->name;
}

const char* PatchRegistry::getPatchName(unsigned int index){
  PatchDefinition* def = getPatchDefinition(index);
  if(def == NULL)
    return emptyPatch.getName();
  return def->getName();
}

unsigned int PatchRegistry::getNumberOfPatches(){
  // +1 for the current / dynamic patch in slot 0
  // return nofPatches+1;
  // return MAX_NUMBER_OF_PATCHES+1;
  return patchCount+1;
}

unsigned int PatchRegistry::getNumberOfResources(){
  return resourceCount;
}

bool PatchRegistry::hasPatches(){
  return patchCount > 0 || dynamicPatchDefinition != NULL;
}

PatchDefinition* PatchRegistry::getPatchDefinition(unsigned int index){
  PatchDefinition *def = NULL;
  if(index == 0)
    def = dynamicPatchDefinition;
  else if(--index < MAX_NUMBER_OF_PATCHES){
    static DynamicPatchDefinition flashPatch;
    if(patchblocks[index].verify()){
      flashPatch.load(patchblocks[index].getData(), patchblocks[index].getDataSize());
      if(flashPatch.verify())
        def = &flashPatch;
    }
  }
  if(def == &emptyPatch)
    def = NULL;
  return def;
}

void PatchRegistry::registerPatch(PatchDefinition* def){
  if(patchCount < MAX_NUMBER_OF_PATCHES)
    defs[patchCount++] = def;
}

// void PatchRegistry::registerPatch(const char* name, uint8_t inputChannels, uint8_t outputChannels){
//   if(nofPatches < MAX_NUMBER_OF_PATCHES){
//     names[nofPatches] = name;
//     nofPatches++;
//   }
// }

void delete_resource(uint8_t index){
  registry.setDeleted(index + MAX_NUMBER_OF_PATCHES + 1);
}
