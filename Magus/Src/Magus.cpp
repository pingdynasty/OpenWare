#include "Owl.h"
#include "device.h"
#include "errorhandlers.h"
#include "Graphics.h"
#include "purple-blue-cyan.h"
#include "orange-red-pink.h"
#include "HAL_TLC5946.h"
#include "HAL_MAX11300.h"
#include "HAL_Encoders.h"
#include "Pin.h"
#include "ApplicationSettings.h"

// 63, 19, 60 // TODO: balance levels

const uint32_t* dyn_rainbowinputs = rainbowinputs;
const uint32_t* dyn_rainbowoutputs = rainbowoutputs;
Graphics graphics;

extern "C" void onResourceUpdate(void);

static bool updateMAX11300 = false;
static uint8_t portMode[20];
void setPortMode(uint8_t index, uint8_t mode){
  // todo: select range automatically based on output value
  if(index < 20){
    if(portMode[index] != mode){
      portMode[index] = mode;
      updateMAX11300 = true;
      // MAX11300_setDACValue(index+1, 0);
    }
  }
}

uint8_t getPortMode(uint8_t index){
  if(index < 20)
    return portMode[index];
  return PORT_UNI_INPUT;
}

void setLed(uint8_t led, uint32_t rgb){
#ifdef USE_TLC5946
  TLC5946_setRGB(led+1, ((rgb>>20)&0x3ff)<<2, ((rgb>>10)&0x3ff)<<2, ((rgb>>00)&0x3ff)<<2);
#endif
}

void onResourceUpdate(void){
  extern const uint32_t rainbowinputs[];
  extern const uint32_t rainbowoutputs[];
  extern const uint32_t* dyn_rainbowinputs;
  extern const uint32_t* dyn_rainbowoutputs;
  Resource* res = storage.getResourceByName("Rainbow.in");
  if(res && res->isMemoryMapped()){
    dyn_rainbowinputs = (uint32_t*)res->getData();
  }else{
    dyn_rainbowinputs = rainbowinputs;
  }
  res = storage.getResourceByName("Rainbow.out");
  if(res && res->isMemoryMapped()){
    dyn_rainbowoutputs = (uint32_t*)res->getData();
  }else{
    dyn_rainbowoutputs = rainbowoutputs;
  }
}

void setup(){
  HAL_GPIO_WritePin(TLC_BLANK_GPIO_Port, TLC_BLANK_Pin, GPIO_PIN_SET); // LEDs off
  Pin enc_nrst(ENC_NRST_GPIO_Port, ENC_NRST_Pin);
  enc_nrst.outputMode();
  enc_nrst.low();

#ifdef USE_TLC5946
  {
    extern SPI_HandleTypeDef TLC5946_SPI;

    // LEDs
    TLC5946_init(&TLC5946_SPI);
    TLC5946_setAll_DC(0); // Start with 0 brightness here, update from settings later
    TLC5946_setAll(0x10, 0x10, 0x10);

    HAL_GPIO_WritePin(TLC_BLANK_GPIO_Port, TLC_BLANK_Pin, GPIO_PIN_RESET);

    TLC5946_Refresh_DC();
    TLC5946_Refresh_GS();
    HAL_Delay(100);

    // Start BLANK PWM
    extern TIM_HandleTypeDef htim2;
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // Start LED Driver PWM : GSCLK
    extern TIM_HandleTypeDef htim3;
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  }
#endif
  {
    // Encoders
    extern SPI_HandleTypeDef ENCODERS_SPI;
    Encoders_init(&ENCODERS_SPI);
  }
  {
    // Pixi
    extern SPI_HandleTypeDef MAX11300_SPI;
    MAX11300_init(&MAX11300_SPI);
    MAX11300_setDeviceControl(DCR_RESET);
  }

  HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET); // OLED off
  extern SPI_HandleTypeDef OLED_SPI;
  graphics.begin(&OLED_SPI);

#ifdef USE_USB_HOST
  // enable USB Host power
  HAL_GPIO_WritePin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin, GPIO_PIN_SET);
#endif

  owl.setup();

  // Update LEDs brighness from settings
  TLC5946_setAll_DC(settings.leds_brightness);
  TLC5946_Refresh_DC();

  // enable pull-up resistors to un-reset encoder
  // allows us to program chip with SWD
  enc_nrst.inputMode();
  enc_nrst.setPull(PIN_PULL_UP);
  Encoders_readAll();
}

void loop(void){

#ifdef USE_USB_HOST
  if(HAL_GPIO_ReadPin(USB_HOST_PWR_FAULT_GPIO_Port, USB_HOST_PWR_FAULT_Pin) == GPIO_PIN_RESET){
    if(HAL_GPIO_ReadPin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin) == GPIO_PIN_SET){
      HAL_GPIO_WritePin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin, GPIO_PIN_RESET);
      error(USB_ERROR, "USBH PWR Fault");
    }
  }else{
    MX_USB_HOST_Process();
  }
#endif

#ifdef USE_SCREEN
  graphics.draw();
  graphics.display();
#endif /* USE_SCREEN */

  owl.loop();

  if(updateMAX11300){
    MAX11300_setDeviceControl(DCR_DACCTL_ImmUpdate|DCR_DACREF_Int|DCR_ADCCTL_ContSweep /* |DCR_ADCCONV_200ksps|DCR_BRST_Contextual*/);
    for(int i=0; i<20; ++i){
      uint16_t mode;
      switch(portMode[i]){
      case PORT_UNI_OUTPUT:
	mode = PCR_Range_DAC_0_P10|PCR_Mode_DAC;
	break;
      case PORT_UNI_INPUT:
      default:
	mode = PCR_Range_ADC_0_P10|PCR_Mode_ADC_SgEn_PosIn|PCR_ADCSamples_16|PCR_ADCref_INT;
	break;
      }
      MAX11300_setPortMode(i+1, mode);
    }
    updateMAX11300 = false;
  }
#ifdef USE_TLC5946
  TLC5946_Refresh_GS();
#endif
  Encoders_readAll();
  graphics.params.updateEncoders(Encoders_get(), 7);
  MAX11300_bulkreadADC();
  for(int i=0; i<16; ++i){
    if(getPortMode(i) == PORT_UNI_INPUT){
      graphics.params.updateValue(i, MAX11300_getADCValue(i+1));
      uint16_t val = graphics.params.parameters[i]>>2;
      setLed(i, dyn_rainbowinputs[val&0x3ff]);
    }else{
      // DACs
    // TODO: store values set from patch somewhere and multiply with user[] value for outputs
    // graphics.params.updateOutput(i, getOutputValue(i));
      // MAX11300_setDACValue(i+1, graphics.params.parameters[i]);
      graphics.params.updateValue(i, 0);
      uint16_t val = graphics.params.parameters[i]>>2;
      setLed(i, dyn_rainbowoutputs[val&0x3ff]);
      MAX11300_setDAC(i+1, graphics.params.parameters[i]);
    }
  }
  for(int i=16; i<20; ++i){
    if(getPortMode(i) == PORT_UNI_INPUT){
      graphics.params.updateValue(i, MAX11300_getADCValue(i+1));
    }else{
      graphics.params.updateValue(i, 0);
      MAX11300_setDAC(i+1, graphics.params.parameters[i]);
    }
  }
  // MAX11300_bulkwriteDAC();
}
