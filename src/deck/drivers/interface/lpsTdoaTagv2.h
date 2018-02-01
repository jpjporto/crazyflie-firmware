#ifndef __LPS_TDOA_TAGV2_H__
#define __LPS_TDOA_TAGV2_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

extern uwbAlgorithm_t uwbTdoaTagAlgorithm;

typedef struct rangePacket_s {
  uint8_t type;
  uint8_t Idx;
  uint32_t timestamps[8];
  uint16_t distances[8];
} __attribute__((packed)) rangePacket_t;

#endif // __LPS_TDOA_TAGV2_H__
