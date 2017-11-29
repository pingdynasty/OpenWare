#include "main.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_flash.h"
#include "stm32f7xx_hal_flash_ex.h"

#define OWL_PLAYERF7
/* #define OWL_TESSERACT */
/* #define OWL_QUADFM */
/* #define OWL_PLAYER */
/* #define OWL_MODULAR */
/* #define OWL_PEDAL */

#ifdef OWL_PLAYERF7
#define OWL_ARCH_F7
#define USE_ADC
#define ADC_PERIPH hadc3
#define ADC_A 0
#define ADC_B 1
#define ADC_C 2
#define ADC_D 3
/* #define PUSHBUTTON_Pin GP7_Pin */
/* #define PUSHBUTTON_GPIO_Port GP7_GPIO_Port */
#define USE_USBD_HS
#define USE_USB_HOST
#define OWL2
#define USE_USBHOST
#define USE_SCREEN
#define SSD1309
#define OLED_DMA
#define OLED_SOFT_CS
#define OLED_SPI hspi2
/* #define OLED_IT */
/* #define OLED_BITBANG */
#define USE_ENCODERS
#define ENCODER_TIM1 htim2
#define ENCODER_TIM2 htim3
#define USE_CODEC
#define USE_CS4271
#define CODEC_SPI hspi4
#endif

#define EEPROM_PAGE_BEGIN            ((uint32_t)0x08060000)
#define EEPROM_PAGE_SIZE             (128*1024)
#define EEPROM_PAGE_END              ((uint32_t)0x08100000)
#define STORAGE_MAX_BLOCKS           64

#define DEBUG_DWT
#define DEBUG_STACK
#define DEBUG_STORAGE

#ifdef SSD1331
#define OLED_WIDTH			96
#define OLED_HEIGHT			64
#define OLED_BUFFER_SIZE        (OLED_WIDTH*OLED_HEIGHT/8)
#elif defined SEPS114A
#define OLED_WIDTH			96
#define OLED_HEIGHT			96
#define OLED_BUFFER_SIZE        (OLED_WIDTH*OLED_HEIGHT*sizeof(uint16_t))
#elif defined SSD1309
#define OLED_WIDTH			128
#define OLED_HEIGHT			64
#define OLED_BUFFER_SIZE        (OLED_WIDTH*OLED_HEIGHT/8)
#endif

/* #define CS_CS_Pin CS_nCS_Pin */
/* #define CS_CS_GPIO_Port CS_nCS_GPIO_Port */
/* #define CS_RST_Pin CS_nRST_Pin */
/* #define CS_RST_GPIO_Port CS_nRST_GPIO_Port */

#define CODEC_BLOCKSIZE 512
#define CODEC_BUFFER_SIZE (4*CODEC_BLOCKSIZE)
#define AUDIO_MAX_BLOCK_SIZE (CODEC_BUFFER_SIZE/4)

/* #define OLED_RST_Pin GPIO_PIN_2 */
/* #define OLED_RST_GPIO_Port GPIOC */
/* #define OLED_MOSI_Pin GPIO_PIN_1 */
/* #define OLED_MOSI_GPIO_Port GPIOC */
/* #define OLED_CS_Pin GPIO_PIN_12 */
/* #define OLED_CS_GPIO_Port GPIOB */
/* #define OLED_SPI SPI2 */
/* #define OLED_DC_Pin GPIO_PIN_3 */
/* #define OLED_DC_GPIO_Port GPIOC */
/* #define OLED_SCK_Pin GPIO_PIN_10 */
/* #define OLED_SCK_GPIO_Port GPIOB */

#define NOF_ADC_VALUES               4
#define NOF_PARAMETERS               16
#define NOF_BUTTONS                  5
#define MAX_SYSEX_FIRMWARE_SIZE      ((16+16+64+128+128)*1024) // FLASH sectors 2-6
#define MAX_SYSEX_PROGRAM_SIZE       (128*1024) // 128k, one flash sector
#define MAX_FACTORY_PATCHES          36
#define MAX_USER_PATCHES             4

#define MAX_NUMBER_OF_PATCHES        40
#define MAX_NUMBER_OF_RESOURCES      12

#define PROGRAM_TASK_STACK_SIZE          (4*1024/sizeof(portSTACK_TYPE))
#define MANAGER_TASK_STACK_SIZE          (512/sizeof(portSTACK_TYPE))
#define FLASH_TASK_STACK_SIZE            (512/sizeof(portSTACK_TYPE))
#define PC_TASK_STACK_SIZE               (512/sizeof(portSTACK_TYPE))
#define IDLE_TASK_STACK_SIZE         (128/sizeof(portSTACK_TYPE))
#define UTILITY_TASK_STACK_SIZE      (512/sizeof(portSTACK_TYPE))
#define ARM_CYCLES_PER_SAMPLE        3500 /* 168MHz / 48kHz */

#define MIDI_OMNI_CHANNEL            (-1)
#define MIDI_OUTPUT_CHANNEL          0

/* +0db in and out */
#define AUDIO_INPUT_GAIN_LEFT        0x017
#define AUDIO_INPUT_GAIN_RIGHT       0x017
#define AUDIO_OUTPUT_GAIN_LEFT       0x079
#define AUDIO_OUTPUT_GAIN_RIGHT      0x079 
#define AUDIO_INPUT_OFFSET           0xffffefaa /* -0.06382 * 65535 */
#define AUDIO_INPUT_SCALAR           0xfffbb5c7 /* -4.290 * 65535 */
#define AUDIO_OUTPUT_OFFSET          0x00001eec /* 0.1208 * 65535 */
#define AUDIO_OUTPUT_SCALAR          0xfffb5bab /* -4.642 * 65535 */
#define DEFAULT_PROGRAM              1
#define BUTTON_PROGRAM_CHANGE
#define AUDIO_BIGEND
/* #define AUDIO_SATURATE_SAMPLES // SATURATE adds almost 500 cycles to 24-bit mode */
#define AUDIO_PROTOCOL               I2S_PROTOCOL_PHILIPS
#define AUDIO_BITDEPTH               24    /* bits per sample */
#define AUDIO_DATAFORMAT             24
#define AUDIO_CODEC_MASTER           true
#define AUDIO_CHANNELS               2
#define AUDIO_SAMPLINGRATE           48000
#define AUDIO_BLOCK_SIZE             128   /* size in samples of a single channel audio block */
