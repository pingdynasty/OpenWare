#include <string.h>
#include <stdlib.h>
#include "callbacks.h"
#include "device.h"
#include "Owl.h"
#include "MidiReceiver.h"
#include "MidiStreamReader.h"
#include "MidiController.h"
#include "ProgramVector.h"
#include "ProgramManager.h"
#include "ApplicationSettings.h"
#include "VersionToken.h"
#include "cmsis_os.h"
#include "BitState.hpp"
#include "errorhandlers.h"
#include "message.h"
#include "PatchRegistry.h"
#ifdef USE_SCREEN
#include "Graphics.h"
#endif
#ifdef USE_CODEC
#include "Codec.h"
#endif
#ifdef OWL_BIOSIGNALS
#include "ads.h"
#ifdef USE_KX122
#include "kx122.h"
#endif
#include "ble_midi.h"
#endif

#if defined USE_RGB_LED
#include "rainbow.h"
#endif /* USE_RGB_LED */

#ifdef USE_USB_HOST
#include "usbh_core.h"
extern "C"{
  void MX_USB_HOST_Process(void);
}
#endif /* USE_USB_HOST */

#ifdef USE_ENCODERS
extern TIM_HandleTypeDef ENCODER_TIM1;
extern TIM_HandleTypeDef ENCODER_TIM2;
#endif

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif

#ifdef USE_ADC
extern uint16_t adc_values[NOF_ADC_VALUES] DMA_RAM;
#endif

#ifdef USE_DAC
extern DAC_HandleTypeDef hdac;
#endif

#ifdef USE_RGB_LED
void updateLed();
#endif

#ifdef USE_MODE_BUTTON
bool isModeButtonPressed(){
  return HAL_GPIO_ReadPin(MODE_BUTTON_PORT, MODE_BUTTON_PIN) == GPIO_PIN_RESET;
}
int getGainSelectionValue(){
  return adc_values[MODE_BUTTON_GAIN]*128*4/4096;
}
int getPatchSelectionValue(){
  return adc_values[MODE_BUTTON_PATCH]*(registry.getNumberOfPatches()-1)*4/4095;
}

void owl_mode_button(void){
  static int patchselect = 0;
  static int gainselect = 0;
  switch(owl.getOperationMode()){
  case STARTUP_MODE:
    owl.setOperationMode(RUN_MODE);
    break;
  case LOAD_MODE:
    setLed(0, getParameterValue(PARAMETER_A)*BLUE_COLOUR/4095);
    break;
  case RUN_MODE:
    if(isModeButtonPressed()){
      patchselect = getPatchSelectionValue();
      gainselect = getGainSelectionValue();
      owl.setOperationMode(CONFIGURE_MODE);
      setLed(0, NO_COLOUR);
    }else if(getErrorStatus() != NO_ERROR){
      owl.setOperationMode(ERROR_MODE);
    }else{
#ifdef USE_RGB_LED
      updateLed();
#endif
    }
    break;
  case CONFIGURE_MODE:
    if(isModeButtonPressed()){
      int value = getPatchSelectionValue();
      if(abs(patchselect - value) > 1){
	patchselect = value;
	value = max(1, min((int)registry.getNumberOfPatches()-1, value/4 + 1));
	if(program.getProgramIndex() != value){
	  program.loadProgram(value);
	  program.resetProgram(false);
	  setLed(0, value & 0x01 ? BLUE_COLOUR : GREEN_COLOUR);
	}
      }
      value = getGainSelectionValue();
      if(abs(gainselect - value) > 2){
	gainselect = value;
	value = max(0, min(127, value/4));
	codec.setOutputGain(value);    
	setLed(0, value & 0x01 ? YELLOW_COLOUR : CYAN_COLOUR);
      }
    }else{
      owl.setOperationMode(RUN_MODE);
    }
    break;
  case ERROR_MODE:
    setLed(0, RED_COLOUR);
    if(isModeButtonPressed())
      program.resetProgram(false); // runAudioTask() changes to RUN_MODE
    break;
  }
}
#endif /* USE_MODE_BUTTON */

__weak void setup(){
#ifdef OWL_BIOSIGNALS
  ble_init();
  setLed(1, NO_COLOUR);
#endif

#ifdef USE_ENCODERS
  __HAL_TIM_SET_COUNTER(&ENCODER_TIM1, INT16_MAX/2);
  __HAL_TIM_SET_COUNTER(&ENCODER_TIM2, INT16_MAX/2);
  HAL_TIM_Encoder_Start_IT(&ENCODER_TIM1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&ENCODER_TIM2, TIM_CHANNEL_ALL);
#endif /* OWL_PLAYERF7 */

#ifdef OWL_WAVETABLE
  extern ADC_HandleTypeDef hadc1;
  extern ADC_HandleTypeDef hadc3;
  HAL_StatusTypeDef ret = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4);
  if(ret != HAL_OK)
    error(CONFIG_ERROR, "ADC1 Start failed");
  ret = HAL_ADC_Start_DMA(&hadc3, (uint32_t*)(adc_values+4), 4);
  if(ret != HAL_OK)
    error(CONFIG_ERROR, "ADC3 Start failed");
#endif
  initLed();
  setLed(0, NO_COLOUR);
  owl.setup();
}

__weak void loop(void){
#ifdef USE_MODE_BUTTON
  owl_mode_button();
#endif /* USE_MODE_BUTTON */

#ifdef FASCINATION_MACHINE
  static int output_gain = 0;
  int gain = adc_values[ADC_D]*255/4095;
  if(abs(gain - output_gain) > 1){
    output_gain = gain;
    codec.setOutputGain(output_gain/2);    
  }
  static int patch_index = 0;
  int patch = adc_values[ADC_E]*10/4095;
  if(abs(patch - patch_index) > 1){
    patch_index = patch;
    patch = patch/2 + 1;
    if(program.getProgramIndex() != patch){
      program.loadProgram(patch);
      program.resetProgram(false);
    }
  }
#endif

#ifdef USE_USB_HOST
#if defined USB_HOST_PWR_FAULT_Pin && defined USB_HOST_PWR_EN_Pin
  if(HAL_GPIO_ReadPin(USB_HOST_PWR_FAULT_GPIO_Port, USB_HOST_PWR_FAULT_Pin) == GPIO_PIN_RESET){
    if(HAL_GPIO_ReadPin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin) == GPIO_PIN_SET){
      HAL_GPIO_WritePin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin, GPIO_PIN_RESET);
      error(USB_ERROR, "USBH PWR Fault");
    }
  }else{
    MX_USB_HOST_Process();
  }
#else
  MX_USB_HOST_Process();
#endif
#endif

  owl.loop();

#ifdef OWL_PRISM
  int16_t encoders[NOF_ENCODERS] = {(int16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIM1),
				    (int16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIM2) };
  graphics.params.updateEncoders(encoders, 2);
#ifndef OWL_RACK
  for(int i=0; i<NOF_ENCODERS; ++i)
    graphics.params.updateValue(i, getAnalogValue(i)-2048); // update two bipolar cv inputs
  for(int i=2; i<NOF_PARAMETERS; ++i)
    graphics.params.updateValue(i, 0);
#endif
#endif /* OWL_PRISM */
}

__weak void onChangeMode(OperationMode new_mode, OperationMode old_mode){
  setLed(0, new_mode == RUN_MODE ? GREEN_COLOUR : YELLOW_COLOUR);
}

__weak void setAnalogValue(uint8_t ch, int16_t value){
#ifdef USE_DAC
  switch(ch){
  case PARAMETER_F:
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, __USAT(value, 12));
    break;
  case PARAMETER_G:
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, __USAT(value, 12));
    break;
  }
#endif
}

__weak void setGateValue(uint8_t ch, int16_t value){
}

__weak void setLed(uint8_t led, uint32_t rgb){
  // rgb should be a 3x 10 bit value
#if defined OWL_TESSERACT
  TIM2->CCR1 = 1023 - ((rgb>>20)&0x3ff);
  TIM3->CCR4 = 1023 - ((rgb>>10)&0x3ff);
  TIM2->CCR2 = 1023 - ((rgb>>00)&0x3ff);
#elif defined OWL_BIOSIGNALS
  if(led == 0){
#ifdef USE_LED_PWM
    rgb &= COLOUR_LEVEL5; // turn down intensity
    TIM1->CCR1 = 1023 - ((rgb>>20)&0x3ff); // red
    TIM1->CCR3 = 1023 - ((rgb>>10)&0x3ff); // green
    TIM1->CCR2 = 1023 - ((rgb>>00)&0x3ff); // blue
#else
    switch(rgb){ // sinking current
    case RED_COLOUR:
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      break;
    case GREEN_COLOUR:
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      break;
    case YELLOW_COLOUR:
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      break;
    case NO_COLOUR:
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      break;
    }
#endif      
  }else if(led == 1){
    if(rgb == NO_COLOUR)
      HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
  }
#endif // OWL_BIOSIGNALS
}


__weak void onChangePin(uint16_t pin){
  switch(pin){
#ifdef OWL_BIOSIGNALS
  case ADC_DRDY_Pin: {
    ads_drdy();
    break;
  }
#ifdef USE_KX122
  case ACC_INT1_Pin: {
    kx122_drdy();
    break;
  }
#endif
#endif
#ifdef PUSHBUTTON_Pin
  case PUSHBUTTON_Pin: {
    bool isSet = !(PUSHBUTTON_GPIO_Port->IDR & PUSHBUTTON_Pin);
    setButtonValue(PUSHBUTTON, isSet);
    midi_tx.sendCc(PUSHBUTTON, isSet ? 127 : 0);
    break;
  }
#endif
#ifdef OWL_TESSERACT
  case TOGGLE_A1_Pin:
    break;
  case TOGGLE_A2_Pin:
    break;
  case TOGGLE_B1_Pin:
    break;
  case TOGGLE_B2_Pin:
    break;
#endif
#ifdef OWL_PLAYERF7
  // sw1() pg14
  // sw2() pb4
  // tr1() pc11
  // tr2() pc10
  case ENC1_SW_Pin: // GPIO_PIN_14:
    setButtonValue(BUTTON_A, !(ENC1_SW_GPIO_Port->IDR & ENC1_SW_Pin));
    setButtonValue(PUSHBUTTON, !(ENC1_SW_GPIO_Port->IDR & ENC1_SW_Pin));
    break;
  case ENC2_SW_Pin: // GPIO_PIN_4:
    setButtonValue(BUTTON_B, !(ENC2_SW_GPIO_Port->IDR & ENC2_SW_Pin));
    break;
  case TR_IN_A_Pin: // GPIO_PIN_11:
    setButtonValue(BUTTON_C, !(TR_IN_A_GPIO_Port->IDR & TR_IN_A_Pin));
    break;
  case TR_IN_B_Pin: // GPIO_PIN_10:
    setButtonValue(BUTTON_D, !(TR_IN_B_GPIO_Port->IDR & TR_IN_B_Pin));
    break;
#endif
#ifdef OWL_PRISM
  case ENC1_SW_Pin:
    setButtonValue(BUTTON_A, !(ENC1_SW_GPIO_Port->IDR & ENC1_SW_Pin));
    setButtonValue(PUSHBUTTON, !(ENC1_SW_GPIO_Port->IDR & ENC1_SW_Pin));
    break;
  case ENC2_SW_Pin:
    setButtonValue(BUTTON_B, !(ENC2_SW_GPIO_Port->IDR & ENC2_SW_Pin));
    break;
#endif    
  }
}

__weak void setProgress(uint16_t value, const char* msg){
  debugMessage(msg, (int)(100*value/4095));
  setParameterValue(LOAD_INDICATOR_PARAMETER, value);
}

// Called on init, resource operation, storage erase
__weak void onResourceUpdate(void){
}
