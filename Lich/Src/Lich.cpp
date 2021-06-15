#include <string.h>
#include <stdlib.h>
#include "device.h"
#include "Owl.h"
#include "errorhandlers.h"
#include "message.h"
#include "ProgramManager.h"
#include "PatchRegistry.h"
#include "OpenWareMidiControl.h"
#include "ApplicationSettings.h"
#include "Pin.h"

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif

extern TIM_HandleTypeDef htim2;

static uint16_t progress = 0;
static int patchselect = 0;

#define SEG_DISPLAY_A     10
#define SEG_DISPLAY_B     11
#define SEG_DISPLAY_C     12
#define SEG_DISPLAY_D     13
#define SEG_DISPLAY_E     14
#define SEG_DISPLAY_F     15
#define SEG_DISPLAY_BLANK 16
#define SEG_DISPLAY_U     17
#define SEG_DISPLAY_L     18
const uint8_t seg_bits[] =
  {
/*    0,    1,    2,    3,    4,    5,    6,    7,    8,    9 */
   0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x67,
/*          A,          b,          C,          d,          E,          F */
   0b01110111, 0b01111100, 0b00111001, 0b01011110, 0b01111001, 0b01110001,
/*      blank,          U,          L */
   0b00000000, 0b00111110, 0b00111000
  };

Pin seg_pins[8] =
  {
   Pin(DISPLAY_A_GPIO_Port, DISPLAY_A_Pin), 
   Pin(DISPLAY_B_GPIO_Port, DISPLAY_B_Pin), 
   Pin(DISPLAY_C_GPIO_Port, DISPLAY_C_Pin), 
   Pin(DISPLAY_D_GPIO_Port, DISPLAY_D_Pin), 
   Pin(DISPLAY_E_GPIO_Port, DISPLAY_E_Pin), 
   Pin(DISPLAY_F_GPIO_Port, DISPLAY_F_Pin), 
   Pin(DISPLAY_G_GPIO_Port, DISPLAY_G_Pin), 
   Pin(DISPLAY_DP_GPIO_Port, DISPLAY_DP_Pin), 
  };

// set value to 10 for no display
static void setSegmentDisplay(int value, bool dot=false){
  seg_pins[7].set(!dot);
  // HAL_GPIO_WritePin(seg_ports[7], seg_pins[7], dot ? GPIO_PIN_RESET : GPIO_PIN_SET);
  uint8_t bits = seg_bits[value % sizeof(seg_bits)];
  for(int i=0; i<7; ++i)
    seg_pins[i].set(!(bits & (1<<i)));
    // HAL_GPIO_WritePin(seg_ports[i], seg_pins[i], (bits & (1<<i)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void onChangePin(uint16_t pin){
  switch(pin){
  case SW1_Pin:
  case GATE_IN1_Pin: {
    bool state = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET ||
      HAL_GPIO_ReadPin(GATE_IN1_GPIO_Port, GATE_IN1_Pin) == GPIO_PIN_RESET;
    setButtonValue(BUTTON_A, state);
    setButtonValue(PUSHBUTTON, state);
    setLed(1, state ? RED_COLOUR : NO_COLOUR);
    break;
  }
  case SW2_Pin:
  case GATE_IN2_Pin: {
    bool state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET ||
      HAL_GPIO_ReadPin(GATE_IN2_GPIO_Port, GATE_IN2_Pin) == GPIO_PIN_RESET;
    setButtonValue(BUTTON_B, state);
    setLed(2, state ? RED_COLOUR : NO_COLOUR);
    break;
  }
  }
}

void setAnalogValue(uint8_t ch, int16_t value){
extern DAC_HandleTypeDef hdac;
  switch(ch){
  case PARAMETER_F:
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, __USAT(value, 12));
    break;
  case PARAMETER_G:
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, __USAT(value, 12));
    break;
  }
}

void setGateValue(uint8_t ch, int16_t value){
  switch(ch){
  case BUTTON_A:
    setLed(1, value);
    break;
  case BUTTON_B:
    setLed(2, value);
    break;
  case PUSHBUTTON:
  case BUTTON_C:
    HAL_GPIO_WritePin(GATE_OUT_GPIO_Port, GATE_OUT_Pin, value ? GPIO_PIN_RESET :  GPIO_PIN_SET);
    break;
  }
}

void setLed(uint8_t led, uint32_t rgb){
  switch(led){
  case 1:
    HAL_GPIO_WritePin(LED_SW1_GPIO_Port, LED_SW1_Pin, rgb == NO_COLOUR ? GPIO_PIN_SET : GPIO_PIN_RESET);
    break;
  case 2:
    HAL_GPIO_WritePin(LED_SW2_GPIO_Port, LED_SW2_Pin, rgb == NO_COLOUR ? GPIO_PIN_SET : GPIO_PIN_RESET);
    break;
  }
}

bool isModeButtonPressed(){
  return HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == GPIO_PIN_RESET;
}

int getEncoderValue(){
  return __HAL_TIM_GET_COUNTER(&htim2)>>2;
}

void setEncoderValue(int value){
  __HAL_TIM_SET_COUNTER(&htim2, value<<2);
}

void setup(){
  // __HAL_TIM_SET_COUNTER(&htim2, INT16_MAX/2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  setSegmentDisplay(SEG_DISPLAY_L, true);
  owl.setup();
  setEncoderValue(program.getProgramIndex());
  setLed(1, 0);
  setLed(2, 0);
  setGateValue(PUSHBUTTON, 0);
}

#define PATCH_RESET_COUNTER (4000/MAIN_LOOP_SLEEP_MS)

void setProgress(uint16_t value, const char* msg){
  debugMessage(msg, (int)(100*value/4095));
  progress = value;
}

void onChangeMode(OperationMode new_mode, OperationMode old_mode){
  progress = 0;
  patchselect = program.getProgramIndex();
  setEncoderValue(patchselect);
}

static void update_preset(){
  static uint32_t counter = PATCH_RESET_COUNTER;
  switch(owl.getOperationMode()){
  case STARTUP_MODE:
    setSegmentDisplay(SEG_DISPLAY_BLANK, true);
    break;
  case LOAD_MODE:
    setSegmentDisplay(SEG_DISPLAY_L); // todo: spin with progress, or just spin
    break;
  case RUN_MODE:
    if(getErrorStatus() != NO_ERROR){
      owl.setOperationMode(ERROR_MODE);
    }else if(getEncoderValue() != patchselect){
      // encoder has changed
      patchselect = max(1, min((int)registry.getNumberOfPatches()-1, getEncoderValue()));
      if(getEncoderValue() != patchselect)
	setEncoderValue(patchselect);
    }
    if(program.getProgramIndex() != patchselect){
      setSegmentDisplay(patchselect % 16, false);
      if(isModeButtonPressed()){
	// switch patches
	program.loadProgram(patchselect);
	program.resetProgram(false);
      }else if(--counter == 0){
	counter = PATCH_RESET_COUNTER;
	patchselect = program.getProgramIndex();
	setEncoderValue(patchselect);	
      }
    }else{
      // encoder hasn't changed and the current patch is selected
      if(isModeButtonPressed() && patchselect != 0){
	// press and hold to store settings
	if(--counter == 0){
	  counter = PATCH_RESET_COUNTER;
	  settings.program_index = patchselect;
	  settings.saveToFlash();
	}else{
	  setSegmentDisplay(patchselect % 16, counter*MAIN_LOOP_SLEEP_MS > 2000);
	}
      }else{
	setSegmentDisplay(patchselect % 16, true);
	counter = PATCH_RESET_COUNTER;
      }
    }
    break;
  case CONFIGURE_MODE:
    owl.setOperationMode(RUN_MODE);
    break;
  case STREAM_MODE:
    setSegmentDisplay(SEG_DISPLAY_U);
    break;
  case ERROR_MODE:
    setSegmentDisplay(SEG_DISPLAY_E, counter > PATCH_RESET_COUNTER/2);
    if(--counter == 0)
      counter = PATCH_RESET_COUNTER;
    if(isModeButtonPressed()){
      setErrorStatus(NO_ERROR);
      owl.setOperationMode(RUN_MODE); // allows new patch selection if patch doesn't load
      program.resetProgram(false);
    }
    break;
  }
}

extern "C" {
  void usbh_midi_reset(void){
    HAL_GPIO_WritePin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(100); // wait 100mS
    HAL_GPIO_WritePin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin, GPIO_PIN_SET);
    // extern USBH_HandleTypeDef USBH_HANDLE; // defined in usb_host.c
    // USBH_LL_ResetPort(&USBH_HANDLE);
  }
}

void loop(void){
  if(HAL_GPIO_ReadPin(USB_HOST_PWR_FAULT_GPIO_Port, USB_HOST_PWR_FAULT_Pin) == GPIO_PIN_RESET){
    if(HAL_GPIO_ReadPin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin) == GPIO_PIN_SET){
      HAL_GPIO_WritePin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin, GPIO_PIN_RESET);
      error(USB_ERROR, "USBH PWR Fault");
    }
  }else{
    MX_USB_HOST_Process();
  }
  update_preset();
  owl.loop();
}
