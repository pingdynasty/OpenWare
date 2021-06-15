#ifndef __SYSEX_H
#define __SYSEX_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
 extern "C" {
#endif

   size_t data_to_sysex(uint8_t *data, uint8_t *sysex, size_t len);
   size_t sysex_to_data(uint8_t *sysex, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SYSEX_H */
