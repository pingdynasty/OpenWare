#include <string.h>
#include <stdlib.h>
#include "device.h"
#include "Owl.h"
#ifdef USE_CODEC
#include "Codec.h"
#endif
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
#include "Storage.h"
#include "PatchRegistry.h"

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

#ifdef USE_DIGITALBUS
#include "bus.h"
#endif /* USE_DIGITALBUS */

Owl owl;
uint32_t ledstatus;
MidiController midi_tx;
MidiReceiver midi_rx;
ApplicationSettings settings;
#ifdef USE_CODEC
Codec codec;
#endif
#ifdef USE_ADC
uint16_t adc_values[NOF_ADC_VALUES] DMA_RAM;
#endif
#ifdef USE_DAC
extern DAC_HandleTypeDef hdac;
#endif

int16_t getAnalogValue(uint8_t ch){
#ifdef USE_ADC
  if(ch < NOF_ADC_VALUES)
    return adc_values[ch];
#endif
  return 0;
}

void midiSetInputChannel(int8_t channel){
  midi_rx.setInputChannel(channel);
}

void midiSetOutputChannel(int8_t channel){
  settings.midi_output_channel = channel;
  midi_tx.setOutputChannel(channel);
}

__weak void initLed(){
  // Initialise RGB LED PWM timers
#if defined OWL_TESSERACT
  extern TIM_HandleTypeDef htim2;
  extern TIM_HandleTypeDef htim3;
  // Red
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  // Green
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  // Blue
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
#elif defined OWL_BIOSIGNALS
#ifdef USE_LED_PWM
  extern TIM_HandleTypeDef htim1;
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
#else
  /*Configure GPIO pin : LED_GREEN_Pin, LED_RED_Pin */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_GREEN_Pin | LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);
#endif
#endif
}


extern "C" {

void HAL_GPIO_EXTI_Callback(uint16_t pin){
  onChangePin(pin);
}
}

static TickType_t xLastWakeTime;
static TickType_t xFrequency;

void Owl::setup(void){
#ifdef USE_IWDG
#ifdef STM32H7xx
  IWDG1->KR = 0xCCCC; // Enable IWDG and turn on LSI
  IWDG1->KR = 0x5555; // ensure watchdog register write is allowed
  IWDG1->PR = 0x05;   // prescaler 128
  IWDG1->RLR = 0x753; // reload 8 seconds
#else
  IWDG->KR = 0xCCCC; // Enable IWDG and turn on LSI
  IWDG->KR = 0x5555; // ensure watchdog register write is allowed
  IWDG->PR = 0x05;   // prescaler 128
  IWDG->RLR = 0x753; // reload 8 seconds
#endif
#endif
#ifdef USE_BKPSRAM
  HAL_PWR_EnableBkUpAccess();
#endif
  ledstatus = 0;
  storage.init();
  registry.init();
  settings.init(); // settings need the registry to be initialised first
  onResourceUpdate();
#ifdef USE_CODEC
  codec.init();
  codec.set(0);
  codec.bypass(false);
  codec.setInputGain(settings.audio_input_gain);
  codec.setOutputGain(settings.audio_output_gain);
#endif /* USE_CODEC */

  program.startManager();

#ifdef USE_DAC
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  setAnalogValue(PARAMETER_F, 0);
  setAnalogValue(PARAMETER_G, 0);
#endif

#if defined USE_ADC && !defined OWL_WAVETABLE
  extern ADC_HandleTypeDef ADC_PERIPH;
  HAL_StatusTypeDef ret = HAL_ADC_Start_DMA(&ADC_PERIPH, (uint32_t*)adc_values, NOF_ADC_VALUES);
  if(ret != HAL_OK)
    error(CONFIG_ERROR, "ADC Start failed");
#endif /* USE_ADC */

  midi_rx.init();
  midiSetInputChannel(settings.midi_input_channel);
  midiSetOutputChannel(settings.midi_output_channel);

  xLastWakeTime = xTaskGetTickCount();
  xFrequency = MAIN_LOOP_SLEEP_MS / portTICK_PERIOD_MS; // 20mS = 50Hz refresh rate

#ifdef USE_DIGITALBUS
  bus_setup();
  bus_set_input_channel(settings.midi_input_channel);
#endif /* USE_DIGITALBUS */
}

#ifdef USE_DIGITALBUS
int busstatus;
#endif

#ifdef USE_RGB_LED
void updateLed(){
  uint32_t colour =
    (adc_values[ADC_A]>>3)+
    (adc_values[ADC_B]>>3)+
    (adc_values[ADC_C]>>3)+
    (adc_values[ADC_D]>>3);
#ifdef ADC_E
  colour += (adc_values[ADC_E]>>3);
#endif
#ifdef FASCINATION_MACHINE
  extern float audio_envelope;
  colour = colour*(1+audio_envelope);
#endif
  colour &= 0x3ff;
  setLed(0, ledstatus | rainbow[colour]);
  // setLed(4095-adc_values[0], 4095-adc_values[1], 4095-adc_values[2]);
}
#endif /*USE_RGB_LED */

OperationMode Owl::getOperationMode(){
  return operationMode;
}

void Owl::setOperationMode(OperationMode mode){
  onChangeMode(mode, operationMode);
  operationMode = mode;
}

void Owl::loop(){
#ifdef USE_DIGITALBUS
  busstatus = bus_status();
#endif
#ifdef OLED_DMA
  // When using OLED_DMA this must delay for a minimum amount to allow screen to update
  vTaskDelay(xFrequency);
#else
  vTaskDelayUntil(&xLastWakeTime, xFrequency);
#endif
  midi_rx.receive();
  midi_tx.transmit();
#ifdef USE_IWDG
#ifdef STM32H7xx
  IWDG1->KR = 0xaaaa; // reset the watchdog timer (if enabled)
#else
  IWDG->KR = 0xaaaa; // reset the watchdog timer (if enabled)
#endif
#endif
  if(backgroundTask != NULL)
    backgroundTask->loop();
}

void Owl::setBackgroundTask(BackgroundTask* bt){
  if(backgroundTask != NULL)
    backgroundTask->end();
  backgroundTask = bt;
  if(backgroundTask != NULL)
    backgroundTask->begin();
}

void jump_to_bootloader(void){
#ifdef USE_USB_HOST
#if defined USB_HOST_PWR_EN_GPIO_Port && defined USB_HOST_PWR_EN_Pin
  HAL_GPIO_WritePin(USB_HOST_PWR_EN_GPIO_Port, USB_HOST_PWR_EN_Pin, GPIO_PIN_RESET);
#endif
#endif
// #ifdef USE_USBD_MIDI
//   extern USBD_HandleTypeDef USBD_HANDLE;
//   USBD_DeInit(&USBD_HANDLE);
// #endif
  *OWLBOOT_MAGIC_ADDRESS = OWLBOOT_MAGIC_NUMBER;
#ifdef USE_BKPSRAM
  extern RTC_HandleTypeDef hrtc;
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0);
#endif
  /* Disable all interrupts */
#ifdef STM32H7xx
  RCC->CIER = 0x00000000;
#else
  RCC->CIR = 0x00000000;
#endif
  NVIC_SystemReset();
  /* Shouldn't get here */
  while(1);
}

void device_reset(){
  *OWLBOOT_MAGIC_ADDRESS = 0;
#ifdef USE_BKPSRAM
  extern RTC_HandleTypeDef hrtc;
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0);
#endif
  /* Disable all interrupts */
#ifdef STM32H7xx
  RCC->CIER = 0x00000000;
#else
  RCC->CIR = 0x00000000;
#endif
  NVIC_SystemReset();
  /* Shouldn't get here */
  while(1);
}

// called from patch program: Patch::sendMidi(MidiMessage)
void midi_send(uint8_t port, uint8_t status, uint8_t d1, uint8_t d2){
  midi_tx.send(MidiMessage(port, status, d1, d2));
}

const char* getFirmwareVersion(){ 
  return (const char*)(HARDWARE_VERSION " " FIRMWARE_VERSION) ;
}

extern char _BOOTLOADER, _ISR_VECTOR_SIZE;
VersionToken* bootloader_token = reinterpret_cast<VersionToken*>(
  (uint32_t)&_BOOTLOADER + (uint32_t)&_ISR_VECTOR_SIZE);

const char* getBootloaderVersion(){
  if (bootloader_token->magic == BOOTLOADER_MAGIC){
    return bootloader_token->version;
  }
  else {
    return "N/A";
  }
}

const char* getDeviceName(){
  static char name[22];
  static char* ptr = 0;
  if(ptr == 0){
    uint32_t* id = (uint32_t*)UID_BASE; // get a pointer to the 96-bit unique device id
    unsigned int hash = id[0]^id[1]^id[2]; // hash into 32-bit value
    hash = (hash>>16)^(hash&0xffff); // hash into 16-bit value
    char* p = stpcpy(name, "OWL-" HARDWARE_VERSION "-");
    stpcpy(p, msg_itoa(hash, 16));
    ptr = name;
  }
  return ptr;
}
