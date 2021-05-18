#include "serial.h"
#include "device.h"

extern UART_HandleTypeDef huart1;

#define ASSERT(cond, msg) assert_param(cond)

/* void serial_read(uint8_t* data, uint16_t size){ */
/*   if(HAL_UART_Receive_IT(&huart1, data, size) != HAL_OK) */
/*     ASSERT(0, "UART rx failed"); */
/* } */

/* void serial_write(uint8_t* data, uint16_t size){ */
/*     bus_tx_buf.push(data, size); */
/*     USART_ITConfig(USART_PERIPH, USART_IT_TXE, ENABLE); */

  /* static const int timeout = 100; */
  /* /\* if(HAL_UART_Transmit_IT(&huart1, data, size) != HAL_OK)  *\/ */
  /* if(HAL_UART_Transmit(&huart1, data, size, timeout) != HAL_OK) */
  /*   ASSERT(0, "UART tx failed"); */
/* } */
