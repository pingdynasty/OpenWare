#include <inttypes.h>

#include "main.h"

#define FIRMWARE_VERSION             "002"
#define HARDWARE_VERSION             "Expander Rev01"

#define MIDI_MAX_MESSAGE_SIZE        256

// #define USE_MAX
#define USE_MAX_DMA
#define USE_TLC
#define TLC_CONTINUOUS
// #define MAX_CONTINUOUS

#define MAX11300_SPI hspi1
#define TLC5946_SPI hspi2

/**
 * Digital Bus on USART1
 */
/* #define USART_BAUDRATE               115200 */
/* #define USART_PERIPH                 USART1 */

#ifdef  USE_FULL_ASSERT
#ifdef __cplusplus
 extern "C" {
#endif
   void assert_failed(uint8_t* file, uint32_t line);
#ifdef __cplusplus
}
#endif
//#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
#endif
