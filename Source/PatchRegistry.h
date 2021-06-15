#ifndef __PatchRegistry_h__
#define __PatchRegistry_h__

#include "device.h"
#include "PatchDefinition.hpp"
#include "Resource.h"
// #include "StorageBlock.h"
// #include "ResourceHeader.h"

class PatchRegistry;
extern PatchRegistry registry;

class PatchRegistry {
public:
  PatchRegistry();
  void init();
  /* const char* getName(unsigned int index); */
  const char* getPatchName(unsigned int index);
  const char* getResourceName(unsigned int index);
  PatchDefinition* getPatchDefinition(){
    return &patchDefinition;
  }
  unsigned int getNumberOfPatches();
  unsigned int getNumberOfResources();
  bool hasPatches();
  /**
   * @return true if registry has a patch with given 1-based index.
   */
  bool hasPatch(uint8_t index);
  Resource* getPatch(uint8_t index);
  Resource* getResource(uint8_t index);
  bool loadProgram(uint8_t slot); // load program from storage
  bool loadProgram(void* address, uint32_t length); // load program from RAM
private:
  Resource* patches[MAX_NUMBER_OF_PATCHES];
  Resource* resources[MAX_NUMBER_OF_RESOURCES];
  uint8_t patchCount, resourceCount;
  PatchDefinition patchDefinition;
};

// // Wrappers to be used as callbacks for service calls
// void delete_resource(uint8_t index);

#endif // __PatchRegistry_h__
