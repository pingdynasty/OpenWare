#ifndef __ParameterController_hpp__
#define __ParameterController_hpp__

#include "device.h"
#include "errorhandlers.h"
#include "ProgramVector.h"
// #include "HAL_Encoders.h"
#include "Owl.h"
#include "OpenWareMidiControl.h"
#include "PatchRegistry.h"
#include "FlashStorage.h"
#include "ApplicationSettings.h"
#include "ProgramManager.h"
#include "Codec.h"
#include "message.h"
#include "VersionToken.h"
#include "ScreenBuffer.h"
#include "HAL_TLC5946.h"

void defaultDrawCallback(uint8_t* pixels, uint16_t width, uint16_t height);

#define NOF_ENCODERS 6
#define ENC_MULTIPLIER 6 // shift left by this many steps
#define SHOW_CALIBRATION_INFO  // This flag renders current values in calibration menu
#define CALIBRATION_INFO_FLOAT // Display float values instead of raw integers

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif

#include "calibration.hpp"

extern VersionToken* bootloader_token;

/*    
screen 128 x 64, font 5x7
4 blocks, 32px per each, 3-4 letters each

4 bottom encoders
press once to toggle mode: update > select
turn to scroll through 4 functions
press again to select parameter: select > update


todo:
- update parameter / encoderChanged
- select parameter
- select global parameter
- select preset mode
*/
template<uint8_t SIZE>
class ParameterController {
public:
  char title[11] = "Magus";
  
  enum EncoderSensitivity {
      SENS_SUPER_FINE = 0,
      SENS_FINE = (ENC_MULTIPLIER / 2),
      SENS_STANDARD = ENC_MULTIPLIER,
      SENS_COARSE = (3 * ENC_MULTIPLIER / 2),
      SENS_SUPER_COARSE = (ENC_MULTIPLIER * 2)
  };
  EncoderSensitivity encoderSensitivity = SENS_STANDARD;
  // Sensitivity is currently not stored in settings, but it's not reset either.
  bool sensitivitySelected;
  
  int16_t parameters[SIZE];
  int16_t encoders[NOF_ENCODERS]; // last seen encoder values
  int16_t offsets[NOF_ENCODERS]; // last seen encoder values
  int16_t user[SIZE]; // user set values (ie by encoder or MIDI)
  char names[SIZE][12];
  // char blocknames[4][NOF_ENCODERS] = {"OSC", "FLT", "ENV", "LFO"} ; // 4 times up to 5 letters/32px
  uint8_t selectedBlock;
  uint8_t selectedPid[NOF_ENCODERS];
  enum DisplayMode {
    STANDARD, SELECTBLOCKPARAMETER, SELECTGLOBALPARAMETER, CONTROL, ERROR
  };
  DisplayMode displayMode;
  
  enum ControlMode {
    PLAY, STATUS, PRESET, DATA, VOLUME, LEDS, CALIBRATE, EXIT, NOF_CONTROL_MODES
  };
  ControlMode controlMode = PLAY;
  bool saveSettings;

  bool resourceDelete;
  bool resourceDeletePressed; // This is used to ensure that we don't delete current resourse on menu enter

  InputCalibration input_cal;
  OutputCalibration output_cal;
  BaseCalibration *current_cal;
  BaseCalibration::CalibrationMode calibrationMode;
  bool isCalibrationRunning;
  bool isCalibrationModeSelected;
  bool calibrationConfirm = false; //A flag used to track encoder presses.
  bool continueCalibration = false; // Runs output calibration for several buffer

  const char controlModeNames[NOF_CONTROL_MODES][12] = {
    "  Play   >",
    "< Status >",
    "< Preset >",
    "< Data   >",
    "< Volume >",
    "< LEDs   >",
    "< V/Oct   " };

  ParameterController(){
    reset();
  }
  void reset(){
    saveSettings = false;
    resourceDelete = false;
    drawCallback = defaultDrawCallback;
    for(int i=0; i<SIZE; ++i){
      strcpy(names[i], "Parameter ");
      names[i][9] = 'A'+i;
      user[i] = 0;
      parameters[i] = 0;
    }
    for(int i=0; i<NOF_ENCODERS; ++i){
      // encoders[i] = 0;
      offsets[i] = 0;
    }
    selectedBlock = 0;
    selectedPid[0] = PARAMETER_BA;
    selectedPid[1] = 0;
    selectedPid[2] = PARAMETER_A;
    selectedPid[3] = PARAMETER_C;
    selectedPid[4] = PARAMETER_E;
    selectedPid[5] = PARAMETER_G;
    displayMode = STANDARD;

#ifdef OWL_MAGUS
    for(int i=0; i<20; ++i)
      setPortMode(i, PORT_UNI_INPUT);
#endif
  }
 
  int16_t getEncoderValue(uint8_t eid){
    return (encoders[eid] - offsets[eid]) << encoderSensitivity;
  }

  void setEncoderValue(uint8_t eid, int16_t value){
    offsets[eid] = encoders[eid] - (value >> encoderSensitivity);
  }

  void draw(uint8_t* pixels, uint16_t width, uint16_t height){
    ScreenBuffer screen(width, height);
    screen.setBuffer(pixels);
    draw(screen);
  }

  void drawLoadProgress(uint8_t progress, ScreenBuffer &screen){
    // progress should be 0 - 127
    screen.drawRectangle(0, 30, 128, 20, WHITE);
    screen.setCursor(32, 40);
    screen.setTextSize(1);
    screen.print("Uploading...");
    screen.fillRectangle(0, 44, progress, 5, WHITE);
  }

  void drawParameter(int pid, int y, ScreenBuffer& screen){
    int x = 0;
#if 0
    // stacked
    // 6px high by up to 128px long rectangle
    screen.drawRectangle(x, y, max(1, min(128, parameters[pid]/32)), 6, WHITE);
    // y -= 7;
    screen.setTextSize(1);
    screen.print(x, y, names[pid]);
#endif
    screen.setTextSize(1);
    screen.print(x, y, names[pid]);
    // 6px high by up to 64px long rectangle
    y -= 7;
    x += 64;
    screen.drawRectangle(x, y, max(1, min(64, parameters[pid]/64)), 6, WHITE);
}

  void drawBlocks(ScreenBuffer& screen){
    drawBlockValues(screen);
    int x = 0;
    int y = 63-8;
    for(int i=2; i<NOF_ENCODERS; ++i){
      // screen.print(x+1, y, blocknames[i-1]);
      if(selectedBlock == i)
        screen.drawHorizontalLine(x, y, 32, WHITE);
      // screen.invert(x, 63-8, 32, 8);
      // screen.invert(x, y-10, 32, 10);
      x += 32;
    }
  }

  void drawGlobalParameterNames(ScreenBuffer& screen){
    screen.setTextSize(1);
    if(selectedPid[0] > 0)
      screen.print(1, 24, names[selectedPid[0]-1]);
    screen.print(1, 24+10, names[selectedPid[0]]);
    if(selectedPid[0] < SIZE-1)
      screen.print(1, 24+20, names[selectedPid[0]+1]);
    screen.invert(0, 25, 64, 10);
  }

  void drawBlockParameterNames(ScreenBuffer& screen){
    int y = 29;
    screen.setTextSize(1);
    int selected = selectedPid[selectedBlock];
    int i = selected & 0x06;
    screen.print(1, y, names[i]);
    if(selected == i)
      screen.invert(0, y-10, 64, 10);
    i += 1;
    screen.print(65, y, names[i]);
    if(selected == i)
      screen.invert(64, y-10, 64, 10);
    y += 12;
    i += 7;
    screen.print(1, y, names[i]);
    if(selected == i)
      screen.invert(0, y-10, 64, 10);
    i += 1;
    screen.print(65, y, names[i]);
    if(selected == i)
      screen.invert(64, y-10, 64, 10);
  }

  void drawBlockValues(ScreenBuffer& screen){
    // draw 4x2x2 levels on bottom 8px row
    int x = 0;
    int y = 63-7;
    int block = 2;
    for(int i=0; i<16; ++i){
      // 4px high by up to 16px long rectangle, filled if selected
      if(i == selectedPid[block])
        screen.fillRectangle(x, y, max(1, min(16, parameters[i]/255)), 4, WHITE);
      else
        screen.drawRectangle(x, y, max(1, min(16, parameters[i]/255)), 4, WHITE);
      x += 16;
      if(i & 0x01)
        block++;
      if(i == 7){
        x = 0;
        y += 3;
        block = 0;
      }
    }
  }
  
  void drawPlay(ScreenBuffer& screen) {
    int offset = 16;
    screen.setTextSize(1);
    screen.print(1, offset + 33, "Encoder sensitivity");

    screen.drawCircle(24, offset + 15, 8, WHITE);
    screen.drawCircle(44, offset + 15, 8, WHITE);
    screen.drawCircle(64, offset + 15, 8, WHITE);
    screen.drawCircle(84, offset + 15, 8, WHITE);
    screen.drawCircle(104, offset + 15, 8, WHITE);
    
    screen.fillCircle(24, offset + 15, 6, WHITE);
    if (encoderSensitivity >= SENS_FINE)
      screen.fillCircle(44, offset + 15, 6, WHITE);
    if (encoderSensitivity >= SENS_STANDARD)
      screen.fillCircle(64, offset + 15, 6, WHITE);
    if (encoderSensitivity >= SENS_COARSE)
      screen.fillCircle(84, offset + 15, 6, WHITE);
    if (encoderSensitivity >= SENS_SUPER_COARSE)
      screen.fillCircle(104, offset + 15, 6, WHITE);
}

  void drawStatus(ScreenBuffer& screen){
    int offset = 16;
    screen.setTextSize(1);
    // single row
    screen.print(1, offset+8, "mem ");
    ProgramVector* pv = getProgramVector();
    int mem = (int)(pv->heap_bytes_used)/1024;
    if(mem > 999){
      screen.print(mem/1024);
      screen.print("M");
    }else{
      screen.print(mem);
      screen.print("k");
    }

    // draw flash usage
    int flash_used = storage.getWrittenSize() / 1024;
    int flash_total = storage.getTotalAllocatedSize() / 1024;
    screen.print(64, offset + 8, "flash ");
    screen.print(flash_used * 100 / flash_total);
    screen.print("%");
    screen.setCursor(64, offset + 17);
    if (flash_used > 999) {
      screen.print(flash_used / 1024);
      screen.print(".");
      screen.print((int)((flash_used  % 1024) * 10 / 1024));
      screen.print("M/");
    }
    else {
      screen.print(flash_used);
      screen.print("k/");
    }
    if (flash_total > 999) {
      screen.print(flash_total / 1024);
      screen.print(".");
      screen.print((int)((flash_total  % 1024) * 10 / 1024));
      screen.print("M");
    }
    else {
      screen.print(flash_total);
      screen.print("k");
    }

    // draw CPU load
    screen.print(1, offset + 17, "cpu ");
    screen.print((int)((pv->cycles_per_block) / pv->audio_blocksize) / 35);
    screen.print("%");
    
    // draw firmware version
    screen.print(1, offset+26, getFirmwareVersion());
    if (bootloader_token->magic == BOOTLOADER_MAGIC){
      screen.print(" (bt.");
      screen.print(getBootloaderVersion());
      screen.print(")");
    }
  }
  
  void drawStats(ScreenBuffer& screen){
    int offset = 0;
    screen.setTextSize(1);
    // screen.clear(86, 0, 128-86, 16);
    // draw memory use

    // two columns
    screen.print(80, offset+8, "mem");
    ProgramVector* pv = getProgramVector();
    screen.setCursor(80, offset+17);
    int mem = (int)(pv->heap_bytes_used)/1024;
    if(mem > 999){
      screen.print(mem/1024);
      screen.print("M");
    }else{
      screen.print(mem);
      screen.print("k");
    }
    // draw CPU load
    screen.print(110, offset+8, "cpu");
    screen.setCursor(110, offset+17);
    screen.print((int)((pv->cycles_per_block)/pv->audio_blocksize)/35);
    screen.print("%");
  }

  void drawError(ScreenBuffer& screen){
    if(getErrorMessage() != NULL){
      screen.setTextSize(1);
      screen.setTextWrap(true);
      screen.print(0, 26, getErrorMessage());
      screen.setTextWrap(false);
    }
    drawMessage(51, screen);
  }

  void drawTitle(ScreenBuffer& screen){
    drawTitle(title, screen);
  }

  void drawTitle(const char* title, ScreenBuffer& screen){
    // draw title
    screen.setTextSize(2);
    screen.print(0, 16, title);
  }

  void drawMessage(int16_t y, ScreenBuffer& screen){
    ProgramVector* pv = getProgramVector();
    if(pv->message != NULL){
      screen.setTextSize(1);
      screen.setTextWrap(true);
      screen.print(0, y, pv->message);
      screen.setTextWrap(false);
    }    
  }

  void drawPresetNames(uint8_t selected, ScreenBuffer& screen){
    screen.setTextSize(1);
    selected = min(selected, registry.getNumberOfPatches()-1);
    if(selected > 1) {
      screen.setCursor(1, 24);
      screen.print((int)selected - 1);
      screen.print(".");
      screen.print(registry.getPatchName(selected - 1));
    };
    screen.setCursor(1, 24+10);
    screen.print((int)selected);
    screen.print(".");
    screen.print(registry.getPatchName(selected));
    if(selected+1 < (int)registry.getNumberOfPatches()) {
      screen.setCursor(1, 24+20);
      screen.print((int)selected + 1);
      screen.print(".");
      screen.print(registry.getPatchName(selected+1));
    }
    screen.invert(0, 25, 128, 10);
  }

  void drawResourceNames(int selected, ScreenBuffer &screen) {
    screen.setTextSize(1);
    if (resourceDelete)
      selected = min(selected, registry.getNumberOfResources());
    else
      selected = min(selected, registry.getNumberOfResources() - 1);
    if (resourceDelete && selected == 0)
      screen.print(18, 24, "Delete:");
    if (selected > 0 && registry.getNumberOfResources() > 0) {
      screen.setCursor(1, 24);
      screen.print((int)selected + MAX_NUMBER_OF_PATCHES);
      screen.print(".");
      screen.print(registry.getResourceName(MAX_NUMBER_OF_PATCHES + selected));
    };
    if (selected < (int)registry.getNumberOfResources()) {
      screen.setCursor(1, 24 + 10);
      screen.print((int)selected + 1 + MAX_NUMBER_OF_PATCHES);
      screen.print(".");
      screen.print(registry.getResourceName(MAX_NUMBER_OF_PATCHES + 1 + selected));
    }
    else if (resourceDelete)
      screen.print(18, 24 + 10, "Exit");
    if (selected + 1 < (int)registry.getNumberOfResources()) {
      screen.setCursor(1, 24 + 20);
      screen.print((int)selected + 2 + MAX_NUMBER_OF_PATCHES);
      screen.print(".");
      screen.print(registry.getResourceName(MAX_NUMBER_OF_PATCHES + 2 + selected));
    }
    if (resourceDelete)
      screen.drawRectangle(0, 25, 128, 10, WHITE);
    else
      screen.invert(0, 25, 128, 10);
  }

  void drawVolume(uint8_t selected, ScreenBuffer& screen){
    screen.setTextSize(1);
    screen.print(1, 24 + 10, "Volume ");
    screen.print((int)settings.audio_output_gain);
    screen.drawRectangle(64, 24 + 1, 64, 8, WHITE);
    screen.fillRectangle(64, 24 + 1 + 2, (int)settings.audio_output_gain >> 1, 4, WHITE);
    screen.invert(0, 24, 40, 10);
  }

  void drawLeds(uint8_t selected, ScreenBuffer& screen){
    screen.setTextSize(1);
    screen.print(1, 24 + 10, "Level  ");
    screen.print((int)settings.leds_brightness);
    screen.drawRectangle(64, 24 + 1, 64, 8, WHITE);
    screen.fillRectangle(64, 24 + 1 + 2, (int)settings.leds_brightness, 4, WHITE);
    screen.invert(0, 24, 40, 10);
  }  

  void drawCalibration(uint8_t selected, ScreenBuffer& screen){
    screen.setTextSize(1);
    if (isCalibrationRunning) {
      if (isCalibrationModeSelected) {
        float input_multiplier = (float)(int32_t)settings.input_scalar / UINT16_MAX;
        float input_offset = (float)(int32_t)settings.input_offset / UINT16_MAX;
        float input_sample = current_cal->getInput();
        float input_voltage = (input_sample - input_offset) * input_multiplier;

        switch (current_cal->state){
        case BaseCalibration::CAL_LO:
          if (calibrationMode == BaseCalibration::CAL_INPUT) {
            screen.print(1, 24 + 10, "1V to IN1");
          }
          else {
            screen.print(1, 24 + 10, "OUT1 to IN1");
            screen.print(1, 24 + 20, "Voltage ");
            screen.print(input_voltage);
          }
          screen.print(1, 24 + 30, "Sample  ");
          screen.print(input_sample);
          break;
        case BaseCalibration::CAL_HI:
          if (calibrationMode == BaseCalibration::CAL_INPUT) {
            screen.print(1, 24 + 10, "3V to IN1");
          }
          else {
            screen.print(1, 24 + 10, "OUT1 to IN1");
            screen.print(1, 24 + 20, "Voltage ");
            screen.print(input_voltage);
          }
          screen.print(1, 24 + 30, "Sample  ");
          screen.print(input_sample);
          break;	  
        case BaseCalibration::CAL_DONE:
          // Save or discard results
          screen.print(1, 24, "Calibration results");
          screen.print(1, 24 + 10, "Scalar:");
          screen.print((float)current_cal->getScalar() / UINT16_MAX);
          screen.print(1, 24 + 20, "Offset:");
          screen.print((float)current_cal->getOffset() / UINT16_MAX);

          screen.print(1, 24 + 30, "Save");
          screen.print(65, 24 + 30, "Discard");
          if (selected == BaseCalibration::CAL_SAVE) {
            screen.invert(0, 24 + 20, 64, 10);
          }
          else {
            screen.invert(64, 24 + 20, 64, 10);
          }
          break;
        }
      }
      else {
        // Select calibration mode
        screen.print(1, 24, "Input");
        screen.print(49, 24, "Output");
        screen.print(97, 24, "Exit");
        switch (selected) {
        case BaseCalibration::CAL_INPUT:
          screen.invert(0, 14, 48, 10);
          break;
        case BaseCalibration::CAL_OUTPUT:
          screen.invert(48, 14, 48, 10);
          break;
        default:
          screen.invert(96, 14, 48, 10);
          break;
        }
      }
    }
    else {
      screen.print(1, 24, "Start calibration");
      #ifdef SHOW_CALIBRATION_INFO
      // Not sure if this should be visible by default, but it helps while debugging
      screen.invert(0, 14, 128, 10);
      screen.print(29, 24 + 10, "Input");
      screen.print(78, 24 + 10, "Output");
      screen.print(1, 24 + 20, "Scl");
      
      #ifdef CALIBRATION_INFO_FLOAT
      screen.print(29, 24 + 20, msg_ftoa((float)((int32_t)settings.input_scalar) / UINT16_MAX, 10));
      screen.print(78, 24 + 20, msg_ftoa((float)((int32_t)settings.output_scalar) / UINT16_MAX, 10));
      #else
      screen.print(29, 24 + 20, msg_itoa(settings.input_scalar, 10));
      screen.print(78, 24 + 20, msg_itoa(settings.output_scalar, 10));
      #endif // CALIBRATION_INFO_FLOAT
      
      screen.print(1, 24 + 30, "Off");
      
      #ifdef CALIBRATION_INFO_FLOAT
      screen.print(29, 24 + 30, msg_ftoa((float)((int32_t)settings.input_offset) / UINT16_MAX, 10));
      screen.print(78, 24 + 30, msg_ftoa((float)((int32_t)settings.output_offset) / UINT16_MAX, 10));
      #else
      screen.print(29, 24 + 30, msg_itoa(settings.input_offset, 10));
      screen.print(78, 24 + 30, msg_itoa(settings.output_offset, 10));
      #endif // CALIBRATION_INFO_FLOAT
      
      screen.drawHorizontalLine(0, 24 + 10, 128, WHITE);
      screen.drawHorizontalLine(0, 24 + 20, 128, WHITE);
      screen.drawVerticalLine(27, 24, 30, WHITE);
      screen.drawVerticalLine(76, 24, 30, WHITE);
      #endif // SHOW_CALIBRATION_INFO
    }
  }

  void drawControlMode(ScreenBuffer& screen){
    switch(controlMode){
    case PLAY:
      // drawMessage("push to exit ->", screen);
      drawTitle(controlModeNames[controlMode], screen);    
      drawPlay(screen);
      break;
    case STATUS:
      drawTitle(controlModeNames[controlMode], screen);    
      drawStatus(screen);
      drawMessage(51, screen);
      break;
    case PRESET:
      drawTitle(controlModeNames[controlMode], screen);    
      drawPresetNames(selectedPid[1], screen);
      break;
    case DATA:
      drawTitle(controlModeNames[controlMode], screen);
      drawResourceNames(selectedPid[1], screen);
      break;
    case VOLUME:
      drawTitle(controlModeNames[controlMode], screen);    
      drawVolume(selectedPid[1], screen);
      break;
    case LEDS:
      drawTitle(controlModeNames[controlMode], screen);    
      drawLeds(selectedPid[1], screen);
      break;
    case CALIBRATE:
      drawTitle(controlModeNames[controlMode], screen);
      drawCalibration(selectedPid[1], screen);
      break;
    case EXIT:
      drawTitle("done", screen);
      break;
    default:
      break;
    }
    // todo!
    // select: Scope, VU Meter, Patch Stats, Set Gain, Show MIDI, Reset Patch, Select Patch...
  }

  void draw(ScreenBuffer& screen){
    screen.clear();
    screen.setTextWrap(false);
    switch(displayMode){
    case STANDARD:
      // draw most recently changed parameter
      // drawParameter(selectedPid[selectedBlock], 44, screen);
      if (owl.getOperationMode() == LOAD_MODE){
        drawLoadProgress(user[LOAD_INDICATOR_PARAMETER] * 127 / 4095, screen);
      }
      else {
        drawParameter(selectedPid[selectedBlock], 54, screen);
      }
      // use callback to draw title and message
      drawCallback(screen.getBuffer(), screen.getWidth(), screen.getHeight());
      break;
    case SELECTBLOCKPARAMETER:
      drawTitle(screen);
      drawBlockParameterNames(screen);
      break;
    case SELECTGLOBALPARAMETER:
      drawTitle(screen);
      drawGlobalParameterNames(screen);
      break;
    case CONTROL:
      drawControlMode(screen);
      break;
    case ERROR:
      drawTitle("ERROR", screen);
      drawError(screen);
      drawStats(screen);
      break;
    }
    drawBlocks(screen);
  }

  void setName(uint8_t pid, const char* name){
    if(pid < SIZE){
      strncpy(names[pid], name, 11);
#ifdef OWL_MAGUS
      if(names[pid][strnlen(names[pid], 11)-1] == '>')
        setPortMode(pid, PORT_UNI_OUTPUT);
      else
        setPortMode(pid, PORT_UNI_INPUT);
#endif
    }
  }

  void setTitle(const char* str){
    strncpy(title, str, 10);    
  }

  uint8_t getSize(){
    return SIZE;
  }

  void selectBlockParameter(uint8_t enc, int8_t pid){
    uint8_t i = enc-2;
    if(pid == i*2+2)
      pid = i*2+8; // skip up
    if(pid == i*2+7)
      pid = i*2+1; // skip down
    selectedPid[enc] = max(i*2, min(i*2+9, pid));
    setEncoderValue(enc, user[selectedPid[enc]]);
  }

  void selectGlobalParameter(int8_t pid){
    selectedPid[0] = max(0, min(SIZE-1, pid));
    setEncoderValue(0, user[selectedPid[0]]);
  }

  void setControlMode(uint8_t value){
    controlMode = (ControlMode)value;
    switch(controlMode){
    case PLAY:
    case STATUS:
      break;
    case PRESET:
      selectedPid[1] = settings.program_index;
      break;
    case DATA:
      selectedPid[1] = 0; // Go to beginning of resource list
      resourceDelete = false;
      break;
    case VOLUME:
      selectedPid[1] = settings.audio_output_gain; // todo: get current
      break;
    case LEDS:
      selectedPid[1] = settings.leds_brightness;
      break;
    case CALIBRATE:
      selectedPid[1] = 2;
      resetCalibration();
      break;
    default:
      break;
    }
  }

  void selectControlMode(int16_t value, bool pressed){
    if(pressed){
      switch(controlMode){
      case PLAY:
        if (sensitivitySelected) {
          controlMode = EXIT;
        }
        break;
      case STATUS:
        setErrorStatus(NO_ERROR);
        break;
      case PRESET:
        // load preset
        settings.program_index = selectedPid[1];
        program.loadProgram(settings.program_index);
        program.resetProgram(false);
        controlMode = EXIT;
        break;
      case DATA: {
        if (resourceDelete) {
          if (selectedPid[1] == registry.getNumberOfResources()){
            // Exit on last menu item (exit link after resources list)
            resourceDelete = false;
            resourceDeletePressed = false;
            controlMode = EXIT;
          }
          else if (!resourceDeletePressed) {
            // Delete resource unless it's protected by "__" prefix
            resourceDeletePressed = true;
            ResourceHeader* res = registry.getResource(selectedPid[1] + MAX_NUMBER_OF_PATCHES + 1);
            if (res != NULL) {
              if(res->name[0] == '_' && res->name[1] == '_'){
                debugMessage("Resource protected");
              }
              else {
                registry.setDeleted(selectedPid[1] + MAX_NUMBER_OF_PATCHES + 1);
              }
            }
          }
        }
        else {
          resourceDelete = true;
          resourceDeletePressed = true;
        }
        break;
      }
      case VOLUME:
        controlMode = EXIT;
        break;
      case LEDS:
        controlMode = EXIT;
        break;
      case CALIBRATE:
        if (!calibrationConfirm) {
          calibrationConfirm = true;
          updateCalibration();
        }
        break;
      default:
        break;
      }
    }
    else{
      if(controlMode == EXIT){
        displayMode = STANDARD;
        //selectedBlock = 0;
        sensitivitySelected = false;
        if(saveSettings)
          settings.saveToFlash();
      }
      else{
        int16_t delta = value - encoders[1];
        if(delta > 0 && controlMode+1 < NOF_CONTROL_MODES){
          setControlMode(controlMode+1);
        }
        else if(delta < 0 && controlMode > 0){
          setControlMode(controlMode-1);
        }
        if (controlMode == CALIBRATE) {
          if (continueCalibration)
            updateCalibration();
          else
            calibrationConfirm = false;
        }
        else if (controlMode == DATA && resourceDeletePressed) {
          resourceDeletePressed = false;
        }
        encoders[1] = value;
      }
    }
  }

  void resetCalibration(){
    //selectedPid[1] = 0;
    isCalibrationRunning = false;
    isCalibrationModeSelected = false;
    input_cal.reset();
    output_cal.reset();
  }

  void updateCalibration(){
    continueCalibration = false;
    // This function runs once every time when encoder is pressed.
    if (isCalibrationRunning){
      if (isCalibrationModeSelected) {
        if (current_cal->state == BaseCalibration::CAL_DONE) {
          current_cal->results = (BaseCalibration::CalibrationResults)selectedPid[1];
          if (current_cal->results == BaseCalibration::CAL_SAVE)
            current_cal->storeResults();
          resetCalibration();
          program.loadProgram(settings.program_index);
          program.resetProgram(false);
        } else{        
          if (current_cal->readSample()) {
            current_cal->nextState();
          }
          else {
            continueCalibration = true;
          }
          if (current_cal->isDone())
            current_cal->calibrate();
        }
      } else {
        isCalibrationModeSelected = true;
        switch (selectedPid[1]){
        case 0:
          program.exitProgram(false);
          input_cal.reset();
          break;
        case 1:
          program.exitProgram(false);
          output_cal.reset();
          break;
	  //case 2:
	  //  controlMode = EXIT;
	  //  break;
        }
      }
    }
    else{
      isCalibrationRunning = true;
    }
  }

  void setControlModeValue(uint8_t value){
    bool sensitivityChanged = false;
    switch(controlMode){
    case PLAY:
        sensitivitySelected = true;
        value = max((uint8_t)SENS_SUPER_FINE, min((uint8_t)SENS_SUPER_COARSE, value));
        if (value > (uint8_t)encoderSensitivity){
          encoderSensitivity = (EncoderSensitivity)((uint8_t)encoderSensitivity + ENC_MULTIPLIER / 2);
          value = (uint8_t)encoderSensitivity;
          sensitivityChanged = true;
        }
        else {
            if (selectedPid[1] < encoderSensitivity) {
                encoderSensitivity = (EncoderSensitivity)((uint8_t)encoderSensitivity - ENC_MULTIPLIER / 2);
                value = (uint8_t)encoderSensitivity;
                sensitivityChanged = true;
            }
        }
        selectedPid[1] = value;
        if (sensitivityChanged) {
          for (int eid = 0; eid < NOF_ENCODERS; eid++) {
              // We update encoders with previous values recalculated with different sensitivity
              if (eid != 1)
                setEncoderValue(eid, user[selectedPid[eid]]);
          }
        }
        break;
    case VOLUME:
      selectedPid[1] = max(0, min(127, value));
      codec.setOutputGain(selectedPid[1]);
      settings.audio_output_gain = selectedPid[1];
      saveSettings = true;
      break;
    case LEDS:
      selectedPid[1] = max(0, min(63, value));
      TLC5946_setAll_DC(selectedPid[1]);
      TLC5946_Refresh_DC();      
      settings.leds_brightness = selectedPid[1];
      saveSettings = true;
      break;
    case PRESET:
      selectedPid[1] = max(1, min(registry.getNumberOfPatches()-1, value));
      break;
    case DATA:
      if (resourceDelete)
        selectedPid[1] = max(0, min(registry.getNumberOfResources(), value));
      else
        selectedPid[1] = max(0, min(registry.getNumberOfResources() - 1, value));
      break;
    case CALIBRATE:
      if (isCalibrationRunning && !isCalibrationModeSelected) {
        selectedPid[1] = max(0, min(value, 2));
        // calibration process is not running yet.
        switch ((BaseCalibration::CalibrationMode)selectedPid[1]){
        case BaseCalibration::CAL_INPUT:
          calibrationMode = BaseCalibration::CAL_INPUT;
          current_cal = &input_cal;
          break;
        case BaseCalibration::CAL_OUTPUT:
          calibrationMode = BaseCalibration::CAL_OUTPUT;
          current_cal = &output_cal;
          break;
        }
      }
      else if (isCalibrationRunning && isCalibrationModeSelected && current_cal->state == BaseCalibration::CAL_DONE) {
        selectedPid[1] = max(0, min(value, 1));
      }
      break;
    default:
      break;
    }
  }

  void updateEncoders(int16_t* data, uint8_t size){
    uint16_t pressed = data[0];

    // update encoder 1 top right
    int16_t value = data[2];
    int16_t right_enc = encoders[1]; // Save old value for encoder scrolling in main menu
    if(displayMode == CONTROL){
      selectControlMode(value, pressed&0x3); // action if either left or right encoder pushed
      if(pressed&0x3c) // exit status mode if any other encoder is pressed
        controlMode = EXIT;
      // use delta value from encoder 0 top left, store in selectedPid[1]
      int16_t delta = data[1] - encoders[0];
      if(delta > 0 && selectedPid[1] < 127) {
        setControlModeValue(selectedPid[1]+1);
      }
      else {
          if(delta < 0 && selectedPid[1] > 0) {
            setControlModeValue(selectedPid[1]-1);
          }
      }
      encoders[0] = data[1];
      return; // skip normal encoder processing
      // todo: should update offsets so values aren't changed on exit
    }
    else {
      if(pressed & (1 << 1)){
        displayMode = CONTROL;
        controlMode = PLAY;
        selectedPid[1] = encoderSensitivity;
      }
      encoders[1] = value;

      // update encoder 0 top left
      value = data[1];
      if(pressed & (1<<0)){
        // update selected global parameter
        // TODO: add 'special' parameters: Volume, Freq, Gain, Gate
        displayMode = SELECTGLOBALPARAMETER;
        int16_t delta = value - encoders[0];
        selectedPid[0] = selectedPid[selectedBlock];
        selectedBlock = 0;
        if(delta < 0) {
          selectGlobalParameter(selectedPid[selectedBlock] - 1);
      }
        else {
          if(delta > 0) {              
            selectGlobalParameter(selectedPid[selectedBlock] + 1);
          }
        }
      }
      else{
        if (displayMode == STANDARD) {
          // Quick selection of global parameter by right encoder - without popover menu
          int16_t delta = data[2] - right_enc;
          encoders[1] = data[2];
          if(delta < 0) {
            selectGlobalParameter(selectedPid[selectedBlock] - 1);
            selectedBlock = 0;
          }
          else
            if(delta > 0) {
              selectGlobalParameter(selectedPid[selectedBlock] + 1);
              selectedBlock = 0;
            }
        }
        if(encoders[0] != value){
          encoders[0] = value;
          // We must update encoder value before calculating user value, otherwise
          // previous value would be displayed
          user[selectedPid[0]] = getEncoderValue(0);
        }

        if(displayMode == SELECTGLOBALPARAMETER)
          displayMode = STANDARD;
      }
      encoders[0] = value;

      // update encoders 2-6 bottom row
      for(uint8_t i=2; i<NOF_ENCODERS; ++i){
        value = data[i+1]; // +1 for buttons
        if(pressed&(1<<i)){
          // update selected block parameter
          selectedBlock = i;
          displayMode = SELECTBLOCKPARAMETER;
          int16_t delta = value - encoders[i];
          if(delta < 0) {
            selectBlockParameter(i, selectedPid[i]-1);
          }
          else
            if(delta > 0)
              selectBlockParameter(i, selectedPid[i]+1);
          }
        else{
          if(encoders[i] != value){
            selectedBlock = i;
            encoders[i] = value;
            // We must update encoder value before calculating user value, otherwise
            // previous value would be displayed
            user[selectedPid[i]] = getEncoderValue(i);
          }
          if(displayMode == SELECTBLOCKPARAMETER && selectedBlock == i)
            displayMode = STANDARD;
        }
        encoders[i] = value;
      }
      if(displayMode == STANDARD && getErrorStatus() && getErrorMessage() != NULL)
        displayMode = ERROR;    
    }
  }

  // called by MIDI cc and/or from patch
  void setValue(uint8_t pid, int16_t value){
    user[pid] = value;
    // reset encoder value if associated through selectedPid to avoid skipping
    for(int i=0; i<NOF_ENCODERS; ++i)
      if(selectedPid[i] == pid)
        setEncoderValue(i, value);
    // TODO: store values set from patch somewhere and multiply with user[] value for outputs
    // graphics.params.updateOutput(i, getOutputValue(i));
  }

  // @param value is the modulation ADC value
  void updateValue(uint8_t pid, int16_t value){
    // smoothing at apprx 50Hz
    parameters[pid] = max(0, min(4095, (parameters[pid] + user[pid] + value)>>1));
  }

  void updateOutput(uint8_t pid, int16_t value){
    parameters[pid] = max(0, min(4095, (((parameters[pid] + (user[pid]*value))>>12)>>1)));
  }

  // void updateValues(uint16_t* data, uint8_t size){
    // for(int i=0; i<16; ++i)
    //   parameters[selectedPid[i]] = encoders[i] + data[i];
  // }

  void encoderChanged(uint8_t encoder, int32_t delta){
  }


  void setCallback(void *callback){
    if(callback == NULL)
      drawCallback = defaultDrawCallback;
    else
      drawCallback = (void (*)(uint8_t*, uint16_t, uint16_t))callback;
  }
  void (*drawCallback)(uint8_t*, uint16_t, uint16_t);
};

#endif // __ParameterController_hpp__
