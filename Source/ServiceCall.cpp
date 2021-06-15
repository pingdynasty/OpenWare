#include <inttypes.h>
#include "arm_math.h"
#include "device.h"
#include "ServiceCall.h"
#include "ApplicationSettings.h"
#include "OpenWareMidiControl.h"
#include "Storage.h"
#include "Owl.h"

#ifdef USE_FFT_TABLES
#include "arm_const_structs.h"
#endif /* USE_FFT_TABLES */
#ifdef USE_FAST_POW
#include "FastLogTable.h"
#include "FastPowTable.h"
#endif /* USE_FAST_POW */
#ifdef USE_SCREEN
#include "Graphics.h"
#endif
#ifdef USE_MIDI_CALLBACK
#include "MidiReceiver.h"
#endif /* USE_MIDI_CALLBACK */
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifdef USE_FFT_TABLES
int SERVICE_ARM_CFFT_INIT_F32(arm_cfft_instance_f32* instance, int len){
  switch(len) { 
  case 16:
    *instance = arm_cfft_sR_f32_len16;
    break;
  case 32:
    *instance = arm_cfft_sR_f32_len32;
    break;
  case 64:
    *instance = arm_cfft_sR_f32_len64;
    break;
  case 128:
    *instance = arm_cfft_sR_f32_len128;
    break;
  case 256:
    *instance = arm_cfft_sR_f32_len256;
    break;
  case 512:
    *instance = arm_cfft_sR_f32_len512;
    break;
  case 1024:
    *instance = arm_cfft_sR_f32_len1024;
    break;      
  case 2048:
    *instance = arm_cfft_sR_f32_len2048;
    break;
  case 4096:
    *instance = arm_cfft_sR_f32_len4096;
    break;
  default:
    return OWL_SERVICE_INVALID_ARGS;
  }
  return OWL_SERVICE_OK;
}
#endif /* USE_FFT_TABLES */

static int handleVersion(void** params, int len){
  int ret = OWL_SERVICE_INVALID_ARGS;
  if(len > 0){
    int* value = (int*)params[0];
    *value = OWL_SERVICE_VERSION_V1;
    ret = OWL_SERVICE_OK;
  }
  return ret;
}
#ifdef USE_FFT_TABLES
static int handleRFFT(void** params, int len){
  int ret = OWL_SERVICE_INVALID_ARGS;
  if(len == 2){
    arm_rfft_fast_instance_f32* instance = (arm_rfft_fast_instance_f32*)params[0];
    int fftlen = *(int*)params[1];
    arm_rfft_fast_init_f32(instance, fftlen);
    ret = OWL_SERVICE_OK;
  }
  return ret;
}

static int handleCFFT(void** params, int len){
  int ret = OWL_SERVICE_INVALID_ARGS;
    if(len == 2){
      arm_cfft_instance_f32* instance = (arm_cfft_instance_f32*)params[0];
      int fftlen = *(int*)params[1];
      ret = SERVICE_ARM_CFFT_INIT_F32(instance, fftlen);
    }
  return ret;
}
#endif

static int handleGetParameters(void** params, int len){
  int ret = OWL_SERVICE_OK;
  int index = 0;
  while(len >= index+2){
    char* p = (char*)params[index++];
    int32_t* value = (int32_t*)params[index++];
    if(strncmp(SYSEX_CONFIGURATION_INPUT_OFFSET, p, 2) == 0){
      *value = settings.input_offset;
    }else if(strncmp(SYSEX_CONFIGURATION_INPUT_SCALAR, p, 2) == 0){
      *value = settings.input_scalar;
    }else if(strncmp(SYSEX_CONFIGURATION_OUTPUT_OFFSET, p, 2) == 0){
      *value = settings.output_offset;
    }else if(strncmp(SYSEX_CONFIGURATION_OUTPUT_SCALAR, p, 2) == 0){
      *value = settings.output_scalar;
    }else{
      ret = OWL_SERVICE_INVALID_ARGS;
    }
  }
  return ret;
}

/*
 * Copy resource contents to preallocated buffer in memory
 * 
 * 4 parameters are expected:
 *  - resource name
 *  - buffer address
 *  - offset in bytes
 *  - max_length in bytes
 * 
 * When buffer address is not given, we will update max_length based on resource size. If resource
 * is using memory mapped storage, buffer address will be set to resource start + offset. If storage
 * is not memory mapped, we can't set this pointer.
 * 
 * When address is given, we're assuming max_length to be its size. So resource contents up to that
 * length in bytes would be copied, starting after offset specified.
 */
static int handleLoadResource(void** params, int len){
  int ret = OWL_SERVICE_INVALID_ARGS;
  if(len == 4){
    const char* name = (const char*)params[0];
    uint8_t** buffer = (uint8_t**)params[1];
    uint32_t offset = *(uint32_t*)params[2];
    uint32_t* max_size = (uint32_t*)params[3];
    Resource* res = storage.getResourceByName(name);
    // We require offset to be aligned to 4 bytes
    if (res != NULL && !(offset & 0b11)) {
      if (*buffer == NULL) {
        // Buffer pointer not given, so we will update value refenced by max_size with
        // actual resource size here
        *max_size = res->getDataSize() - offset;
	if(res->isMemoryMapped())
	  *buffer = (uint8_t*)res->getData() + offset;
      }else{
	uint32_t copy_size = min(*max_size, res->getDataSize() - offset);
        // Buffer pointer is given. We'll copy no more than max_size data into it.
	storage.readResource(res, *buffer, offset, copy_size);
	*max_size = copy_size; // update max_size parameter with amount of data actually copied
      }
      ret = OWL_SERVICE_OK;
    }
  }
  return ret;
}

static int handleGetArray(void** params, int len){
  int ret = OWL_SERVICE_INVALID_ARGS;
  // get array and array size
  // expects three parameters: name, &array and &size
#ifdef USE_FAST_POW
  int index = 0;
  ret = OWL_SERVICE_OK;
  if(len >= index+3){
    char* p = (char*)params[index++];
    void** array = (void**)params[index++];
    int* size = (int*)params[index++];
    if(strncmp(SYSTEM_TABLE_LOG, p, 3) == 0){
      *array = (void*)fast_log_table;
      *size = fast_log_table_size;
    }else if(strncmp(SYSTEM_TABLE_POW, p, 3) == 0){
      *array = (void*)fast_pow_table;
      *size = fast_pow_table_size;
    }else{
      *array = NULL;
      *size = 0;
      ret = OWL_SERVICE_INVALID_ARGS;
    }
  }
#else
  ret = OWL_SERVICE_INVALID_ARGS;    
#endif /* USE_FAST_POW */
  return ret;
}

static int handleRequestCallback(void** params, int len){
  int ret = OWL_SERVICE_INVALID_ARGS;
  int index = 0;
  if(len >= index+2){
    char* name = (char*)params[index++];
    void** callback = (void**)params[index++];
#ifdef USE_MIDI_CALLBACK
    if(strncmp(SYSTEM_FUNCTION_MIDI, name, 3) == 0){
      *callback = (void*)midi_send;
      ret = OWL_SERVICE_OK;
    }
#endif /* USE_MIDI_CALLBACK */
  }
  return ret;
}

static int handleRegisterCallback(void** params, int len){
  int ret = OWL_SERVICE_INVALID_ARGS;
  int index = 0;
  if(len >= index+2){
    char* name = (char*)params[index++];
    void* callback = (void*)params[index++];
#ifdef USE_SCREEN
    if(strncmp(SYSTEM_FUNCTION_DRAW, name, 3) == 0){
      // void (*drawCallback)(uint8_t*, uint16_t, uint16_t);
      graphics.setCallback(callback);
      ret = OWL_SERVICE_OK;
    }
#endif /* USE_SCREEN */
#ifdef USE_MIDI_CALLBACK
    if(strncmp(SYSTEM_FUNCTION_MIDI, name, 3) == 0){
      // void (*midiCallback)(uint8_t port, uint8_t status, uint8_t, uint8_t);
      midi_rx.setCallback(callback);
      ret = OWL_SERVICE_OK;
    }
#endif /* USE_MIDI_CALLBACK */
  }
  return ret;
}

int serviceCall(int service, void** params, int len){
  int ret = OWL_SERVICE_INVALID_ARGS;
  switch(service){
  case OWL_SERVICE_VERSION:
    ret = handleVersion(params, len);
    break;
#ifdef USE_FFT_TABLES
  case OWL_SERVICE_ARM_RFFT_FAST_INIT_F32:
    ret = handleRFFT(params, len);    
    break;
  case OWL_SERVICE_ARM_CFFT_INIT_F32:
    ret = handleCFFT(params, len);
    break;
#endif /* USE_FFT_TABLES */
  case OWL_SERVICE_GET_PARAMETERS:
    ret = handleGetParameters(params, len);
    break;
  case OWL_SERVICE_GET_ARRAY:
    ret = handleGetArray(params, len);
    break;
  case OWL_SERVICE_LOAD_RESOURCE:
    ret = handleLoadResource(params, len);
    break;
  case OWL_SERVICE_REQUEST_CALLBACK:
    ret = handleRequestCallback(params, len);
    break;
  case OWL_SERVICE_REGISTER_CALLBACK:
    ret = handleRegisterCallback(params, len);
    break;
  }
  return ret;
}
