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

#ifdef DEC_DECA
typedef struct {
  uint8_t type;
  uint8_t source;
  union {
    uint8_t data[48];
    struct {
      float x;
      float y;
      float z;
      float vx;
      float vy;
      float vz;
      float roll;
      float pitch;
      float yaw;
      float vroll;
      float vpitch;
      float vyaw;
    } cfstate_s;
  };
} __attribute__((packed)) lpsCFStatePacket_t;

void setCFState(const state_t *state);
#endif

#define LPP_STATE_PACKET 0xAA

#endif // __LPS_TDOA_TAGV2_H__
