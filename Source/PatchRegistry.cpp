#include "PatchRegistry.h"
#include "ProgramManager.h"
#include "ResourceHeader.h"
#include "ProgramHeader.h"
#include "PatchDefinition.hpp"
#include "Storage.h"
#include "message.h"

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

PatchRegistry::PatchRegistry() {}

void PatchRegistry::init() {
  patchCount = 0;
  resourceCount = 0;

  memset(patches, 0, sizeof(patches));
  memset(resources, 0, sizeof(resources));

  for(size_t i=0; i<storage.getNumberOfResources(); ++i){
    Resource* resource = storage.getResource(i);
    if(resource->isValid()){
      uint8_t id = resource->getSlot();
      if(resource->isPatch() && id <= MAX_NUMBER_OF_PATCHES){
        patches[id-1] = resource;
        patchCount = max(patchCount, id);
      }else if(!resource->isSystemResource() && resourceCount < MAX_NUMBER_OF_RESOURCES){
        resources[resourceCount++] = resource;
      }
    }
  }
}

Resource* PatchRegistry::getResource(uint8_t index){
  if(index < MAX_NUMBER_OF_RESOURCES)
    return resources[index];
  return NULL;
}

Resource* PatchRegistry::getPatch(uint8_t index){
  if(index < MAX_NUMBER_OF_PATCHES)
    return patches[index];
  return NULL;
}

const char* PatchRegistry::getResourceName(unsigned int index){
  Resource* hdr = getResource(index);
  if(hdr)
    return hdr->getName();
  return "---";
}

const char* PatchRegistry::getPatchName(unsigned int index){
  if(index == 0){
    patchDefinition.getName();
  }else{
    Resource* resource = getPatch(index-1);
    if(resource)
      return resource->getName();
  }
  return "---";
}

unsigned int PatchRegistry::getNumberOfPatches(){
  // +1 for the current / dynamic patch in slot 0
  return patchCount+1;
}

unsigned int PatchRegistry::getNumberOfResources(){
  return resourceCount;
}

bool PatchRegistry::hasPatch(uint8_t index){
  return getPatch(index - 1) != NULL;
}

bool PatchRegistry::hasPatches(){
  return patchCount > 0;
}

bool PatchRegistry::loadProgram(uint8_t index){
  Resource* resource = getPatch(index-1);  
  if(resource && resource->isValid())
    return patchDefinition.load(resource) && patchDefinition.isValid();
  return false;
}

bool PatchRegistry::loadProgram(void* address, uint32_t length){
  return patchDefinition.load(address, length) && patchDefinition.isValid();  
}
