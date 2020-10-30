#ifndef __PROGRAM_VECTOR_V13_H
#define __PROGRAM_VECTOR_V13_H

#define PROGRAM_VECTOR_V13
#define PROGRAM_VECTOR_CHECKSUM     0x51

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define OWL_PEDAL_HARDWARE          0x11
#define OWL_MODULAR_LEGACY_HARDWARE 0x12
#define OWL_RACK_HARDWARE           0x13
#define PRISM_HARDWARE              0x14
#define PLAYER_HARDWARE             0x15
#define TESSERACT_HARDWARE          0x16
#define ALCHEMIST_HARDWARE          0x17
#define WIZARD_HARDWARE             0x18
#define MAGUS_HARDWARE              0x19
#define EFFECTSBOX_HARDWARE         0x1a
#define WAVETABLE_HARDWARE          0x1b
#define NOCTUA_HARDWARE             0x1c
#define BIOSIGNALS_HARDWARE         0x1d
#define LICH_HARDWARE               0x1e
#define WITCH_HARDWARE              0x1f
#define OWL_MODULAR_HARDWARE        0x20 /* This is not a new device, but a way to distinguish firmware */

#define PROGRAM_VECTOR_CHECKSUM_V11 0x40
#define PROGRAM_VECTOR_CHECKSUM_V12 0x50
#define PROGRAM_VECTOR_CHECKSUM_V13 0x51

#define CHECKSUM_ERROR_STATUS        -10
#define OUT_OF_MEMORY_ERROR_STATUS   -20
#define CONFIGURATION_ERROR_STATUS   -30

#define AUDIO_FORMAT_24B16_2X       0x10
#define AUDIO_FORMAT_24B24_2X       0x18
#define AUDIO_FORMAT_24B32          0x20
#define AUDIO_FORMAT_24B32_2X       0x22
#define AUDIO_FORMAT_24B32_4X       0x24
#define AUDIO_FORMAT_24B32_8X       0x28

  typedef enum { 
    AUDIO_IDLE_STATUS = 0, 
    AUDIO_READY_STATUS, 
    AUDIO_PROCESSING_STATUS, 
    AUDIO_EXIT_STATUS, 
    AUDIO_ERROR_STATUS 
  } ProgramVectorAudioStatus;

  typedef struct {
    uint8_t* location;
    uint32_t size;
  } MemorySegment;

   typedef struct {
     uint8_t checksum;
     uint8_t hardware_version;
     int32_t* audio_input;
     int32_t* audio_output;
     uint8_t audio_format;
     uint16_t audio_blocksize;
     uint32_t audio_samplingrate;
     int16_t* parameters;
     uint8_t parameters_size;
     uint16_t buttons;
     int8_t error;
     void (*registerPatch)(const char* name, uint8_t inputChannels, uint8_t outputChannels);
     void (*registerPatchParameter)(uint8_t id, const char* name);
     void (*programReady)(void);
     void (*programStatus)(ProgramVectorAudioStatus status);
     int (*serviceCall)(int service, void** params, int len);
     uint32_t cycles_per_block;
     uint32_t heap_bytes_used;
     char* message;
     void (*setButton)(uint8_t id, uint16_t state, uint16_t samples);
     void (*setPatchParameter)(uint8_t id, int16_t value);
     void (*buttonChangedCallback)(uint8_t bid, uint16_t state, uint16_t samples);
     MemorySegment* heapSegments;
   } ProgramVector;

   ProgramVector* getProgramVector();

#ifdef __cplusplus
}
#endif

#endif /* __PROGRAM_VECTOR_V13_H */
