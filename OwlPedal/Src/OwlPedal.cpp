#include "Owl.h"
#include "device.h"
#include "errorhandlers.h"
#include "MidiController.h"
#include "OpenWareMidiControl.h"
#include "ApplicationSettings.h"
#include "PatchRegistry.h"
#include "ProgramManager.h"

#define PROGRAM_CHANGE_PUSHBUTTON_TICKS 2000
// Number of buffers before patch switching kicks in, with 2 ms loop delay this should take 5.23 seconds

#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

extern uint32_t ledstatus;

void initLed(){
}

static bool pushButtonPressed = 0;
static uint16_t pushButtonPressDuration = 0;
static uint16_t pushButtonAnimation = 0;
static uint8_t owlSelectedProgramId = 0;
static uint16_t bank = 0;
static uint16_t prog = 0;

void setLed(uint8_t led, uint32_t rgb){
  // rgb should be a 3x 10 bit value
  switch(rgb){
  case RED_COLOUR:
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    break;
  case GREEN_COLOUR:
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    break;
  case NO_COLOUR:
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    break;
  }
}

void onChangePin(uint16_t pin){
  switch(pin){
  case PUSHBUTTON_Pin: {
    pushButtonPressed = !(PUSHBUTTON_GPIO_Port->IDR & PUSHBUTTON_Pin);
    if (pushButtonPressed) {
      // Rising edge
      pushButtonAnimation = 0x96;
    }
    else {
      // Falling edge
      if (pushButtonPressDuration > PROGRAM_CHANGE_PUSHBUTTON_TICKS && owlSelectedProgramId > 0) {
        // This runs unless button was depressed too early or the same
        // patch as before was selected.
        program.loadProgram(owlSelectedProgramId);
        program.resetProgram(true);
        pushButtonPressDuration = 0;
      }
    }
    setButtonValue(BUTTON_A, pushButtonPressed);
    setButtonValue(PUSHBUTTON, pushButtonPressed);
    midi_tx.sendCc(PUSHBUTTON, pushButtonPressed ? 127 : 0);
    setLed(0, pushButtonPressed ? RED_COLOUR : GREEN_COLOUR);
    break;
  }
#ifdef OWL_PEDAL
  case BYPASS_Pin: {
    bool isSet = BYPASS_GPIO_Port->IDR & BYPASS_Pin;
    setLed(0, isSet ? NO_COLOUR : GREEN_COLOUR);
    break;
  }
#endif
#ifdef OWL_MODULAR
  case PUSH_GATE_IN_Pin: {
    bool isSet = !(PUSH_GATE_IN_GPIO_Port->IDR & PUSH_GATE_IN_Pin);
    setButtonValue(PUSHBUTTON, isSet);
    setLed(0, isSet ? RED_COLOUR : GREEN_COLOUR);
    break;
  }
#endif
  }
}

inline uint16_t getProgramSelection(){
  // This code is copied from original firmware. The only difference is that
  // it won't be running in a separate OS task. So we won't be waiting for
  // usable patch being selected. Instead of that, we return -1 for invalid
  // selection - it should be ignored by the caller.
  #ifdef OWL_MODULAR
  float a = (4095 - getAnalogValue(PARAMETER_A)) * 5 / 4096.0;// - 0.5 / 5;
  float b = (4095 - getAnalogValue(PARAMETER_B)) * 8 / 4096.0;// - 0.5 / 8;
  #else
  float a = getAnalogValue(PARAMETER_A) * 5 / 4096.0;// - 0.5 / 5;
  float b = getAnalogValue(PARAMETER_B) * 8 / 4096.0;// - 0.5 / 8;
  #endif
  // deadband each segment: [0.9-1.1]
  if(a > 0.f && abs(a - (uint16_t)(a + 0.1f)) > 0.1f)
    bank = (uint16_t)a;
  else if (a <= 0.1f)
    bank = 0; // Ignore bottom dead zone
  else if (a >= 5.0f - 0.1f)
    bank = 4; // Ignore top dead zone

  // deadband each segment: [0.85-1.15]
  // larger deadband because we have the same amount of noise spread over more zones
  if(b > 0.f && abs(b - (uint16_t)(b + 0.15f)) > 0.15f)
    prog = (uint16_t)b + 1;
  else if (b <= 0.15f)
    prog = 1;  // Ignore bottom dead zone
  else if (b >= 8.0f - 0.15f)
    prog = 8; // Ignore top dead zone

  uint16_t pc = max(bank * 8 + prog, 1);
  // We must check that patch exists to avoid loading from a gap in the patches list
  if (registry.hasPatch(pc))
    return pc;
  else
    return 0;
}

void setGateValue(uint8_t ch, int16_t value){
  switch (ch) {
  case BUTTON_A:
    setLed(0, value ? RED_COLOUR : GREEN_COLOUR);
    break;
  case PUSHBUTTON:
    setLed(0, value ? RED_COLOUR : GREEN_COLOUR);
    // deliberate fall-through - this synchronizes LED to pushbutton value
  case BUTTON_B:
#ifdef OWL_MODULAR
    HAL_GPIO_WritePin(PUSH_GATE_OUT_GPIO_Port, PUSH_GATE_OUT_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
    break;
  case GREEN_BUTTON:
    setLed(0, value ? GREEN_COLOUR : NO_COLOUR);
    break;
  case RED_BUTTON:
    setLed(0, value ? RED_COLOUR : NO_COLOUR);
  }
}

void setup(){  
#ifdef OWL_PEDAL
  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  // if (HAL_GetREVID() == 0x1001)
  // {
  //   /* Enable the Flash prefetch */
  //   __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  // }
  // enable expression pedal reference voltage
  HAL_GPIO_WritePin(EXPRESSION_PEDAL_TIP_GPIO_Port, EXPRESSION_PEDAL_TIP_Pin, GPIO_PIN_SET);
  // todo: on OWL Modular the ADC should read Exp pin PA2 instead of PA3
#endif
  owl.setup();
 }
 
#define PATCH_RESET_COUNTER (1000/MAIN_LOOP_SLEEP_MS)

void loop(){
  static uint32_t counter = PATCH_RESET_COUNTER;
  switch(owl.getOperationMode()){
  case STARTUP_MODE:
  case LOAD_MODE:
    setLed(0, counter > PATCH_RESET_COUNTER/2 ? GREEN_COLOUR : NO_COLOUR);
    break;
  case RUN_MODE:
    if(getErrorStatus() != NO_ERROR)
      owl.setOperationMode(ERROR_MODE);
    else if (pushButtonPressed) {
      if (pushButtonPressDuration++> PROGRAM_CHANGE_PUSHBUTTON_TICKS){
        // Pushbutton kept pressed above threshold, overflow is prevented
        pushButtonPressDuration = PROGRAM_CHANGE_PUSHBUTTON_TICKS + 1;
        if (pushButtonAnimation){
          // Pulse as we unlock patch selection
          if ((pushButtonAnimation-- & 0xf) == 0){
            HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
          }
          if (!pushButtonAnimation){
            // Animation just finished and we're ready to select patches on next cycle
            owlSelectedProgramId = 0;
#ifdef OWL_MODULAR
            bank = (4095 - getAnalogValue(PARAMETER_A)) * 5 / 4096;
            prog = (4095 - getAnalogValue(PARAMETER_B)) * 8 / 4096 + 1;
#else
            bank = getAnalogValue(PARAMETER_A) * 5 / 4096;
            prog = getAnalogValue(PARAMETER_B) * 8 / 4096 + 1;
#endif
          }
        }
        else {
          // Ready to check for patch selection
          uint16_t newOwlProgramId = getProgramSelection();
          if (!owlSelectedProgramId) {
            // No program selected yet
            if (!newOwlProgramId) {
              // No color if nothing is selected
              setLed(0, NO_COLOUR);
            }
            else {
              // First selection, color chosen based on patch number
              owlSelectedProgramId = newOwlProgramId;
              setLed(0, (owlSelectedProgramId & 1) ? GREEN_COLOUR : RED_COLOUR);
            }
          }
          else if (newOwlProgramId != owlSelectedProgramId && newOwlProgramId > 0){
            // Toggle pushbutton LEDs if patch number was changed
            owlSelectedProgramId = newOwlProgramId;
            HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
          }
        }
      }
    }
    break;
  case CONFIGURE_MODE:
    owl.setOperationMode(RUN_MODE);
    break;
  case STREAM_MODE:
  case ERROR_MODE:
    setLed(0, counter > PATCH_RESET_COUNTER/2 ? RED_COLOUR : NO_COLOUR);
    break;
  }
  if(--counter == 0)
    counter = PATCH_RESET_COUNTER;

  owl.loop();
}
