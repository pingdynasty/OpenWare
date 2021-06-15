#ifndef __RESOURCE_HEADER_H
#define __RESOURCE_HEADER_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

   struct ResourceHeader {
     uint32_t magic; // 0xDADADEED for resources, 0xDADAC0DE for patches
     uint32_t size;
     char name[20];
     uint32_t flags;
   };

#ifdef __cplusplus
}
#endif

#endif /* __RESOURCE_HEADER_H */
