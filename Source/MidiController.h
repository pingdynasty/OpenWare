#ifndef __MIDI_CONTROLLER_H
#define __MIDI_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "device.h"
#include "Resource.h"
#include "MidiMessage.h"
#include "MidiWriter.h"
#include "OpenWareMidiControl.h"

class MidiController;
extern MidiController midi_tx;

class MidiController : public MidiWriter {
private:
  uint8_t channel;
public:
  void setOutputChannel(uint8_t ch){
    channel = ch;
  }
  void sendSettings();
  void sendConfigurationSetting(const char* name, uint32_t value);
  void sendPatchParameterName(PatchParameterId pid, const char* name);
  void sendPatchParameterValues();
  void sendPatchName(uint8_t slot);
  void sendPatchNames();
  void sendResourceNames();
  void sendResource(Resource* resource);
  void sendName(uint8_t cmd, uint8_t index, const char* name, size_t size);
  void sendDeviceInfo();
  void sendDeviceStats();
  void sendProgramStats();
  void sendStatus();
  void sendFirmwareVersion();
  void sendBootloaderVersion();
  void sendDeviceId();
  void sendProgramMessage();
  void sendErrorMessage();
};

#endif /* __MIDI_CONTROLLER_H */
