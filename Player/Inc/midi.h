#ifndef __MIDI_H
#define __MIDI_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

   void midi_rx_usb_buffer(uint8_t *buffer, uint32_t length);
   void midi_tx_usb_buffer(uint8_t* buffer, uint32_t length);
   uint8_t midi_device_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* __MIDI_H */
