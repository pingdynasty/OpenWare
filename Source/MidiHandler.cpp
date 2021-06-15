#include <string.h>
#include "MidiHandler.h"
#include "MidiStatus.h"
#include "MidiController.h"
#include "OpenWareMidiControl.h"
#include "FirmwareLoader.hpp"
#include "ApplicationSettings.h"
#include "errorhandlers.h"
#include "VersionToken.h"
#include "ProgramHeader.h"
#include "PatchRegistry.h"
#ifdef USE_CODEC
#include "Codec.h"
#endif
#include "Owl.h"
#include "Storage.h"
#ifndef USE_BOOTLOADER_MODE
#include "BootloaderStorage.h"
#endif
#ifdef USE_DIGITALBUS
#include "bus.h"
#endif

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

static FirmwareLoader loader;

MidiHandler::MidiHandler() : channel(MIDI_OMNI_CHANNEL) {
  // memset(midi_values, 0, NOF_PARAMETERS*sizeof(uint16_t));
}

void MidiHandler::handlePitchBend(uint8_t status, uint16_t value){
}

void MidiHandler::handleNoteOn(uint8_t status, uint8_t note, uint8_t velocity){
  if(channel != MIDI_OMNI_CHANNEL && channel != getChannel(status))
    return;
  if(getProgramVector()->buttonChangedCallback != NULL)
    getProgramVector()->buttonChangedCallback(MIDI_NOTE_BUTTON+note, velocity<<5, getSampleCounter());
}

void MidiHandler::handleNoteOff(uint8_t status, uint8_t note, uint8_t velocity){
  if(channel != MIDI_OMNI_CHANNEL && channel != getChannel(status))
    return;
  if(getProgramVector()->buttonChangedCallback != NULL)
    getProgramVector()->buttonChangedCallback(MIDI_NOTE_BUTTON+note, 0, getSampleCounter());
}

void MidiHandler::handleProgramChange(uint8_t status, uint8_t pid){
  if(channel != MIDI_OMNI_CHANNEL && channel != getChannel(status))
    return;
  if(pid == 0){
    runProgram();
  }else{
    program.loadProgram(pid);
    program.resetProgram(true);
  }
}

void MidiHandler::handleControlChange(uint8_t status, uint8_t cc, uint8_t value){
  if(channel != MIDI_OMNI_CHANNEL && channel != getChannel(status))
    return;
  switch(cc){
#ifdef USE_CODEC
  case MIDI_CC_VOLUME:
    codec.setOutputGain(value);
    break;
#endif
  case PATCH_PARAMETER_A:
    setParameterValue(PARAMETER_A, value<<5); // scale from 7bit to 12bit value
    break;
  case PATCH_PARAMETER_B:
    setParameterValue(PARAMETER_B, value<<5);
    break;
  case PATCH_PARAMETER_C:
    setParameterValue(PARAMETER_C, value<<5);
    break;
  case PATCH_PARAMETER_D:
    setParameterValue(PARAMETER_D, value<<5);
    break;
  case PATCH_PARAMETER_E:
    setParameterValue(PARAMETER_E, value<<5);
    break;
  case MIDI_CC_MODULATION:
    setParameterValue(PARAMETER_F, value<<5);
    break;
  case MIDI_CC_EFFECT_CTRL_1:
    setParameterValue(PARAMETER_G, value<<5);
    break;
  case MIDI_CC_EFFECT_CTRL_2:
    setParameterValue(PARAMETER_H, value<<5);
    break;
  case PATCH_BUTTON:
    setButtonValue(PUSHBUTTON, value < 64 ? 0 : 255);
    break;
  case PATCH_BUTTON_ON:
    setButtonValue(value, 255);
    break;
  case PATCH_BUTTON_OFF:
    setButtonValue(value, 0);
    break;
  case REQUEST_SETTINGS:
    switch(value){
    case 0:
    case 127:
      midi_tx.sendDeviceInfo();
      break;
    case SYSEX_PRESET_NAME_COMMAND:
      midi_tx.sendPatchNames();
      break;
    case SYSEX_CONFIGURATION_COMMAND:
      midi_tx.sendSettings();
      break;
    case SYSEX_RESOURCE_NAME_COMMAND:
      midi_tx.sendResourceNames();
      break;
    case SYSEX_FIRMWARE_VERSION:
      midi_tx.sendFirmwareVersion();
      break;
    case SYSEX_DEVICE_ID:
      midi_tx.sendDeviceId();
      break;
    case SYSEX_DEVICE_STATS:
      midi_tx.sendDeviceStats();
      break;
    case SYSEX_BOOTLOADER_VERSION:
      midi_tx.sendBootloaderVersion();
      break;
    case SYSEX_PROGRAM_MESSAGE:
      midi_tx.sendProgramMessage();
      break;
    case SYSEX_PROGRAM_STATS:
      midi_tx.sendStatus();
      break;
    case PATCH_BUTTON:
      midi_tx.sendCc(PUSHBUTTON, getButtonValue(PUSHBUTTON) ? 127 : 0);
      break;
    }
    break;
  default:
    if(cc >= PATCH_PARAMETER_AA && cc <= PATCH_PARAMETER_DH)
      setParameterValue(PARAMETER_AA+(cc-PATCH_PARAMETER_AA), value<<5);
    break;
  }
}

void MidiHandler::handleSystemRealTime(uint8_t cmd){
}

void MidiHandler::handleSystemCommon(uint8_t cmd1, uint8_t cmd2){
}

void MidiHandler::handleSystemCommon(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3){
}

void MidiHandler::handleChannelPressure(uint8_t status, uint8_t value){
}

void MidiHandler::handlePolyKeyPressure(uint8_t status, uint8_t note, uint8_t value){
}

void MidiHandler::updateCodecSettings(){
#ifdef USE_CODEC
  codec.reset();
#endif
  program.resetProgram(true);
}

void MidiHandler::handleConfigurationCommand(uint8_t* data, uint16_t size){
  if(size < 3) // size may be 3 or more depending on number of digits in value
    return;
  char* p = (char*)data;
  int32_t value = strtol(p+2, NULL, 16);
  if(strncmp(SYSEX_CONFIGURATION_AUDIO_RATE, p, 2) == 0){
    settings.audio_samplingrate = value;
  }else if(strncmp(SYSEX_CONFIGURATION_AUDIO_BLOCKSIZE, p, 2) == 0){
    settings.audio_blocksize = value;
    updateCodecSettings();
  }else if(strncmp(SYSEX_CONFIGURATION_AUDIO_BITDEPTH, p, 2) == 0){
    settings.audio_bitdepth = value;
  }else if(strncmp(SYSEX_CONFIGURATION_AUDIO_DATAFORMAT, p, 2) == 0){
    settings.audio_dataformat = value;
#ifdef USE_CODEC
  }else if(strncmp(SYSEX_CONFIGURATION_CODEC_SWAP, p, 2) == 0){
    settings.audio_codec_swaplr = value;
  }else if(strncmp(SYSEX_CONFIGURATION_CODEC_BYPASS, p, 2) == 0){
    settings.audio_codec_bypass = value;
    codec.bypass(value);
  }else if(strncmp(SYSEX_CONFIGURATION_CODEC_INPUT_GAIN, p, 2) == 0){
    settings.audio_input_gain = value;  
    codec.setInputGain(settings.audio_input_gain);
  }else if(strncmp(SYSEX_CONFIGURATION_CODEC_OUTPUT_GAIN, p, 2) == 0){
    settings.audio_output_gain = value;  
    codec.setOutputGain(settings.audio_output_gain);
  }else if(strncmp(SYSEX_CONFIGURATION_CODEC_HIGHPASS, p, 2) == 0){
    codec.setHighPass(value);
#endif
  }else if(strncmp(SYSEX_CONFIGURATION_PC_BUTTON, p, 2) == 0){
    settings.program_change_button = value;
  }else if(strncmp(SYSEX_CONFIGURATION_INPUT_OFFSET, p, 2) == 0){
    settings.input_offset = value;
  }else if(strncmp(SYSEX_CONFIGURATION_INPUT_SCALAR, p, 2) == 0){
    settings.input_scalar = value;
  }else if(strncmp(SYSEX_CONFIGURATION_OUTPUT_OFFSET, p, 2) == 0){
    settings.output_offset = value;
  }else if(strncmp(SYSEX_CONFIGURATION_OUTPUT_SCALAR, p, 2) == 0){
    settings.output_scalar = value;
  }else if(strncmp(SYSEX_CONFIGURATION_MIDI_INPUT_CHANNEL, p, 2) == 0){
    midiSetInputChannel(max(-1, min(15, value)));
  }else if(strncmp(SYSEX_CONFIGURATION_MIDI_OUTPUT_CHANNEL, p, 2) == 0){
    midiSetOutputChannel(max(-1, min(15, value)));
#ifndef USE_BOOTLOADER_MODE
  }else if(strncmp(SYSEX_CONFIGURATION_BOOTLOADER_LOCK, p, 2) == 0){
    if (value)
      bootloader.lock();
    else
      bootloader.unlock();
#endif
#ifdef USE_DIGITALBUS
  }else if(strncmp(SYSEX_CONFIGURATION_BUS_ENABLE, p, 2) == 0){
    settings.bus_enabled = value;
  }else if(strncmp(SYSEX_CONFIGURATION_BUS_FORWARD_MIDI, p, 2) == 0){
    settings.bus_forward_midi = value;
#endif
  }
}

void MidiHandler::handleSettingsResetCommand(uint8_t* data, uint16_t size){
  settings.reset();
  updateCodecSettings();
}

void MidiHandler::handleSettingsStoreCommand(uint8_t* data, uint16_t size){
  settings.saveToFlash();
}

void MidiHandler::handleFirmwareUploadCommand(uint8_t* data, uint16_t size){
  int32_t ret = loader.handleFirmwareUpload(data, size);
  if(ret == 0){
    owl.setOperationMode(LOAD_MODE);
    setProgress(loader.index*4095/loader.size, "Loading");
  }
}

void MidiHandler::handleFirmwareRunCommand(uint8_t* data, uint16_t size){
  runProgram();
}

void MidiHandler::runProgram(){
  if(loader.isReady()){
    program.loadDynamicProgram(loader.getData(), loader.getSize());
    loader.clear();
    // program.startProgram(true);
    program.resetProgram(true);
  }else{
    error(PROGRAM_ERROR, "No program to run");
  }      
}

void MidiHandler::handleFlashEraseCommand(uint8_t* data, uint16_t size){
  if(size == 5){
    uint32_t slot = loader.decodeInt(data);
    program.eraseFromFlash(slot);
  }else if(size == 0){
    program.eraseFromFlash(0xff);
  }else{
    error(PROGRAM_ERROR, "Invalid FLASH ERASE command");
  }
}

void MidiHandler::handleFirmwareFlashCommand(uint8_t* data, uint16_t size){
  if(loader.isReady() && size == 5){
    uint32_t checksum = loader.decodeInt(data);
    if(checksum == loader.getChecksum()){
      // Bootloader size would be exactly 32k/64k due to token added in its end. So
      // alignment is not expected to become an issue here (unless >16 bytes would be necessary).
      extern char _ISR_VECTOR_SIZE;
      VersionToken* token = reinterpret_cast<VersionToken*>(
        loader.getData() + (uint32_t)&_ISR_VECTOR_SIZE);
      if (token->magic != BOOTLOADER_MAGIC) {
        error(PROGRAM_ERROR, "Invalid bootloader");
      }
      else if (token->hardware_id != HARDWARE_ID) {
        error(PROGRAM_ERROR, "Invalid hardware ID");
      }
      else {
        //program.eraseFromFlash(-2);
        program.saveToFlash(-2, loader.getData(), loader.getSize());
        loader.clear();
        program.resetProgram(true);
      }
    }else{
      error(PROGRAM_ERROR, "Invalid FLASH checksum");
    }
  }else{
    error(PROGRAM_ERROR, "Invalid FLASH command");
  }
}

void MidiHandler::handleFirmwareSendCommand(uint8_t* data, uint16_t size){
  uint32_t slot = loader.decodeInt(data);
  Resource* resource = NULL;
  if(slot-1 < MAX_NUMBER_OF_PATCHES)
    resource = registry.getPatch(slot-1);
  else if(slot-MAX_NUMBER_OF_PATCHES < MAX_NUMBER_OF_RESOURCES)
    resource = registry.getResource(slot-MAX_NUMBER_OF_PATCHES);
  if(resource != NULL)
    program.sendResource(resource);
  else
    error(PROGRAM_ERROR, "Invalid SEND command");
}

void MidiHandler::handleFirmwareStoreCommand(uint8_t* data, uint16_t size){
  if(loader.isReady() && size == 5){
    uint32_t slot = loader.decodeInt(data);
    if(slot > 0 && slot <= MAX_NUMBER_OF_PATCHES){
      data = loader.getData();
      size_t datasize = loader.getSize();
      // char name[] = "patch00";
      // name[5] = '0'+((slot*10)%10);
      // name[6] = '0'+(slot%10);
      memmove(data+sizeof(ResourceHeader), data, datasize); // make space for resource header
      ProgramHeader* header = (ProgramHeader*)(data+sizeof(ResourceHeader));
      if(header->magic == 0XDADAC0DE){	
	storage.writeResourceHeader(data, header->programName, datasize,
				    RESOURCE_PORT_MAPPED|RESOURCE_USER_PATCH|slot);
	program.saveToFlash(slot, data, datasize+sizeof(ResourceHeader));
      }else{
	error(PROGRAM_ERROR, "Invalid patch magic");
      }
    }else{
      error(PROGRAM_ERROR, "Invalid STORE slot");
    }
  }else{
    error(PROGRAM_ERROR, "Invalid STORE command");
  }
  loader.clear();
}

void MidiHandler::handleFirmwareSaveCommand(uint8_t* data, uint16_t size){
  if(loader.isReady() && size > 1){
    const char* name = (const char*)data;
    size_t len = strnlen(name, 20);
    if(len > 0 && len < 20){
      // todo: create ResourceHeader in FirmwareLoader::beginFirmwareUpload()
      // stop patch or check if running
      // flash in background task
      data = loader.getData();
      size_t datasize = loader.getSize();
      memmove(data+sizeof(ResourceHeader), data, datasize); // make space for resource header
      storage.writeResourceHeader(data, name, datasize, RESOURCE_PORT_MAPPED);
      program.saveToFlash(0, data, datasize+sizeof(ResourceHeader));
      loader.clear();
    }else{
      error(PROGRAM_ERROR, "Invalid SAVE name");
    }
  }else{
    error(PROGRAM_ERROR, "Invalid SAVE command");
  }
}

void MidiHandler::handleSysEx(uint8_t* data, uint16_t size){
  if(size < 5 || data[1] != MIDI_SYSEX_MANUFACTURER)     
    return;
  if(data[2] != MIDI_SYSEX_OMNI_DEVICE && data[2] != (MIDI_SYSEX_OWL_DEVICE | channel))
    // not for us
    return;
  switch(data[3]){
  case SYSEX_CONFIGURATION_COMMAND:
    handleConfigurationCommand(data+4, size-5);
    break;
  case SYSEX_DEVICE_RESET_COMMAND:
    device_reset();
    break;
  case SYSEX_BOOTLOADER_COMMAND:
    jump_to_bootloader();
    break;
  case SYSEX_FIRMWARE_UPLOAD:
    handleFirmwareUploadCommand(data+1, size-2);
    break;
  case SYSEX_FIRMWARE_RUN:
    handleFirmwareRunCommand(data+4, size-5);
    break;
  case SYSEX_FIRMWARE_STORE:
    handleFirmwareStoreCommand(data+4, size-5);
    break;
  case SYSEX_FIRMWARE_SEND:
    handleFirmwareSendCommand(data+4, size-5);
    break;
  case SYSEX_FIRMWARE_FLASH:
    handleFirmwareFlashCommand(data+4, size-5);
    break;
  case SYSEX_FIRMWARE_SAVE:
    handleFirmwareSaveCommand(data+4, size-5);
    break;
  case SYSEX_FLASH_ERASE:
    handleFlashEraseCommand(data+4, size-5);
    break;
  case SYSEX_SETTINGS_RESET:
    handleSettingsResetCommand(data+4, size-5);
    break;
  case SYSEX_SETTINGS_STORE:
    handleSettingsStoreCommand(data+4, size-5);
    break;
  default:
    error(PROGRAM_ERROR, "Invalid SysEx Message");
    break;
  }
}

int8_t MidiHandler::getChannel(uint8_t status){
  return status & MIDI_CHANNEL_MASK;
}
