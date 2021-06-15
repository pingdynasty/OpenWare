#include "main.h"
#include "stm32f4xx_hal.h"

#define OWL_MAGUS
#define HARDWARE_ID                  MAGUS_HARDWARE
#define HARDWARE_VERSION             "Magus"

#define USE_SPI_FLASH

#define USE_SCREEN
#define SSD1309
/* #define OLED_DMA */
#define OLED_SOFT_CS
#define OLED_SPI hspi6
/* #define OLED_IT */
/* #define OLED_BITBANG */
#define USE_CODEC
#define USE_CS4271
#define CODEC_SPI hspi4

/* USB audio settings */
#define AUDIO_BITS_PER_SAMPLE       16
#define AUDIO_BYTES_PER_SAMPLE      (AUDIO_BITS_PER_SAMPLE/8)
#define AUDIO_CHANNELS              2
#define AUDIO_INT32_TO_SAMPLE(x)    ((x)>>8)
#define AUDIO_SAMPLE_TO_INT32(x)    ((int32_t)(x)<<8)

#define MAIN_LOOP_SLEEP_MS          20

#define USE_USBD_AUDIO
#define USE_USBD_AUDIO_TX           // microphone
/* #define USE_USBD_AUDIO_RX           // speaker */
#define USE_USBD_FS
#define USBD_HANDLE                 hUsbDeviceFS
#define USBH_HANDLE                 hUsbHostHS
#define USE_USB_HOST
#define USE_USBH_MIDI
#define USB_HOST_RX_BUFF_SIZE       256

// Serial MIDI TX only
#define USE_UART_MIDI_TX
#define UART_MIDI_HANDLE huart2
#define UART_MIDI_RX_BUFFER_SIZE    256

#define USE_TLC5946
#define LEDS_BRIGTHNESS             20
#define TLC5946_SPI hspi6
#define MAX11300_SPI hspi5
#define ENCODERS_SPI hspi5

#undef USE_ADC
#define NOF_ADC_VALUES              0
#define NOF_PARAMETERS              20
#define NOF_BUTTONS                 0
#undef USE_DAC

#define AUDIO_INPUT_OFFSET          ((uint32_t)(0.00007896*65535))
#define AUDIO_INPUT_SCALAR          ((uint32_t)(-6.43010423*65535))
#define AUDIO_OUTPUT_OFFSET         ((uint32_t)(0.00039729*65535))
#define AUDIO_OUTPUT_SCALAR         ((uint32_t)(5.03400000*65535))
