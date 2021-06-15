#ifndef __FirmwareLoader_H__
#define __FirmwareLoader_H__

#include <math.h>
#include "crc32.h"
#include "sysex.h"
#include "device.h"
#include <stdint.h>
// #include "owlcontrol.h"
#include "errorhandlers.h"
#include "ProgramManager.h"

class FirmwareLoader {
private:
  // enum SysExFirmwareStatus {
  //   NORMAL = 0,
  //   UPLOADING,
  //   ERROR = 0xff
  // };
  // SysExFirmwareStatus status = NORMAL;
public:
  uint16_t packageIndex = 0;
  uint8_t* buffer = NULL;
  uint32_t size;
  uint32_t index;
  uint32_t crc;
  bool ready;
public:
  void clear(){
    buffer = NULL;
    index = 0;
    packageIndex = 0;
    ready = false;
    crc = -1;
  }

  uint32_t getChecksum(){
    return crc;
  }

  bool isReady(){
    return ready;
  }

  int setError(const char* msg){
    error(PROGRAM_ERROR, msg);
    clear();
    return -1;
  }

  uint8_t* getData(){
    return buffer;
  }

  uint32_t getSize(){
    return size;
  }

  /* decode a 32-bit unsigned integer from 5 bytes of sysex encoded data */
  uint32_t decodeInt(uint8_t *data){
    uint8_t buf[4];
    sysex_to_data(data, buf, 5);
    uint32_t result = buf[3] | (buf[2] << 8L) | (buf[1] << 16L) | (buf[0] << 24L);
    return result;
  }

  int32_t beginFirmwareUpload(uint8_t* data, uint16_t length, uint16_t offset){
    clear();
    setErrorStatus(NO_ERROR);
    // first package
    if(length < 3+5+5)
      return setError("Invalid SysEx package");
    // stop running program and free its memory
    program.exitProgram(true);
    owl.setOperationMode(LOAD_MODE);
    // program.loadProgram(2); // load progress bar
    // program.resetProgram(true);
    // get firmware data size (decoded)
    size = decodeInt(data+offset);
    offset += 5; // it takes five 7-bit values to encode four bytes
    // allocate memory
    if(size > MAX_SYSEX_PAYLOAD_SIZE)
      return setError("SysEx too big");
#ifdef USE_EXTERNAL_RAM
    extern char _EXTRAM; // defined in link script
    buffer = (uint8_t*)&_EXTRAM;
#else
    // required by devices with no ext mem
    extern char _PATCHRAM;
    buffer = (uint8_t*)&_PATCHRAM; 
#endif
    packageIndex = 1;
    return 0;
  }

  int32_t receiveFirmwarePackage(uint8_t* data, uint16_t length, uint16_t offset){
    int len = sysex_to_data(data+offset, buffer+index, length-offset);
    index += len;
    packageIndex++;
    return 0;
  }

  int32_t finishFirmwareUpload(uint8_t* data, uint16_t length, uint16_t offset){
    // last package: package index and checksum
    // check crc
    crc = crc32(buffer, size, 0);
    // get checksum: last 4 bytes of buffer
    uint32_t checksum = decodeInt(data+length-5);
    if(crc != checksum)
      return setError("Invalid SysEx checksum");
    ready = true;
    return index;
  }

  int32_t handleFirmwareUpload(uint8_t* data, uint16_t length){
    uint16_t offset = 3;
    uint16_t idx = decodeInt(data+offset);
    offset += 5;
    if(idx == 0)
      return beginFirmwareUpload(data, length, offset);
    if(packageIndex != idx)
      return setError("SysEx package out of sequence"); // out of sequence package
    int len = floor((length-offset)*7/8.0f);
    // wait for program to exit before writing to buffer
    if(index+len <= size)
      // mid package
      return receiveFirmwarePackage(data, length, offset);
    else if(index == size)
      return finishFirmwareUpload(data, length, offset);
    return setError("Invalid SysEx size"); // wrong size
  }
};

#endif // __FirmwareLoader_H__
