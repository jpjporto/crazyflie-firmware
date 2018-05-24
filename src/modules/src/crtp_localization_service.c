/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "crtp_localization_service.h"
#include "log.h"
#include "param.h"

#include "stabilizer_types.h"
#include "stabilizer.h"

#include "locodeck.h"

#include "estimator_kalman.h"

#define NBR_OF_RANGES_IN_PACKET   5
#define DEFAULT_EMERGENCY_STOP_TIMEOUT (1 * RATE_MAIN_LOOP)

#define VICON_RATE   100

typedef enum
{
  EXT_POSITION  = 0,
  GENERIC_TYPE  = 1,
  BROADCAST_POS = 2,
} locsrvChannels_t;

typedef struct
{
  uint8_t type;
  struct
  {
    uint8_t id;
    float range;
  } __attribute__((packed)) ranges[NBR_OF_RANGES_IN_PACKET];
} __attribute__((packed)) rangePacket;

/**
 * Position data cache
 */
typedef struct
{
  struct CrtpExtPosition targetVal[2];
  bool activeSide;
  uint32_t timestamp; // FreeRTOS ticks
} ExtPositionCache;

typedef struct
{
  struct CrtpBroadExtPosition targetVal;
  //bool activeSide;
  uint32_t timestamp; // FreeRTOS ticks
} ExtBroadPosCache;

// Struct for logging position information
static positionMeasurement_t ext_pos;
static ExtPositionCache crtpExtPosCache;
static CRTPPacket pkRange;
static uint8_t rangeIndex;
static bool enableRangeStreamFloat = false;
static bool isInit = false;

static void locSrvCrtpCB(CRTPPacket* pk);
static void extPositionHandler(CRTPPacket* pk);
static void genericLocHandle(CRTPPacket* pk);

static void broadcastPosHandler(CRTPPacket* pk);
static ExtBroadPosCache crtpBroadExtPosCache1, crtpBroadExtPosCache2;
static uint8_t prevSeqNum[CFNUM] = {0xFF};

#if CFNUM >= 2
static point_t prevPoscf1;
#if CFNUM >= 3
static point_t prevPoscf2;
#if CFNUM >= 4
static point_t prevPoscf3;
#if CFNUM == 5
static point_t prevPoscf4;
#endif
#endif
#endif
#endif

void locSrvInit()
{
  if (isInit) {
    return;
  }

  crtpRegisterPortCB(CRTP_PORT_LOCALIZATION, locSrvCrtpCB);
  isInit = true;
}

static void locSrvCrtpCB(CRTPPacket* pk)
{
  switch (pk->channel)
  {
    case EXT_POSITION:
      extPositionHandler(pk);
      break;
    case GENERIC_TYPE:
      genericLocHandle(pk);
      break;
    case BROADCAST_POS:
      broadcastPosHandler(pk);
      break;
    default:
      break;
  }
}

static void extPositionHandler(CRTPPacket* pk)
{
  crtpExtPosCache.targetVal[!crtpExtPosCache.activeSide] = *((struct CrtpExtPosition*)pk->data);
  crtpExtPosCache.activeSide = !crtpExtPosCache.activeSide;
  crtpExtPosCache.timestamp = xTaskGetTickCount();
}

static void genericLocHandle(CRTPPacket* pk)
{
  uint8_t type = pk->data[0];
  if (pk->size < 1) return;

  if (type == LPS_SHORT_LPP_PACKET && pk->size >= 2) {
    bool success = lpsSendLppShort(pk->data[1], &pk->data[2], pk->size-2);

    pk->port = CRTP_PORT_LOCALIZATION;
    pk->channel = GENERIC_TYPE;
    pk->size = 3;
    pk->data[2] = success?1:0;
    crtpSendPacket(pk);
  } else if (type == EMERGENCY_STOP) {
    stabilizerSetEmergencyStop();
  } else if (type == EMERGENCY_STOP_WATCHDOG) {
    stabilizerSetEmergencyStopTimeout(DEFAULT_EMERGENCY_STOP_TIMEOUT);
  }
}

static void broadcastPosHandler(CRTPPacket* pk)
{
  if (pk->data[0] == 1)
  {
    crtpBroadExtPosCache1.targetVal = *((struct CrtpBroadExtPosition*)&pk->data[1]);
    //crtpBroadExtPosCache.activeSide = !crtpBroadExtPosCache.activeSide;
    crtpBroadExtPosCache1.timestamp = xTaskGetTickCount();
  }
  else if(pk->data[0] == 2)
  {
    crtpBroadExtPosCache2.targetVal = *((struct CrtpBroadExtPosition*)&pk->data[1]);
    //crtpBroadExtPosCache.activeSide = !crtpBroadExtPosCache.activeSide;
    crtpBroadExtPosCache2.timestamp = xTaskGetTickCount();
  }
}

bool getExtPosition(state_t *state)
{
  // Only use position information if it's valid and recent
  if ((xTaskGetTickCount() - crtpExtPosCache.timestamp) < M2T(5)) {
    // Get the updated position from the mocap
    ext_pos.x = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].x;
    ext_pos.y = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].y;
    ext_pos.z = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].z;
    ext_pos.stdDev = 0.01f;
    estimatorKalmanEnqueuePosition(&ext_pos);

    return true;
  }
  
  if ((xTaskGetTickCount() - crtpBroadExtPosCache1.timestamp) < M2T(5)) {
#if CFNUM == 1
    uint8_t seqDiff0 = crtpBroadExtPosCache1.targetVal.seq0 - prevSeqNum[0];
    if(seqDiff0 != 0) {
      prevSeqNum[0] = crtpBroadExtPosCache1.targetVal.seq0;
      ext_pos.x = (float)crtpBroadExtPosCache1.targetVal.x0 * 0.000125f; // Scale factor, same as dividing by 8000
      ext_pos.y = (float)crtpBroadExtPosCache1.targetVal.y0 * 0.000125f;
      ext_pos.z = (float)crtpBroadExtPosCache1.targetVal.z0 * 0.000125f;
      ext_pos.stdDev = 0.01f;
      estimatorKalmanEnqueuePosition(&ext_pos);
      return true;
    }
#elif CFNUM == 2
    uint8_t seqDiff0 = crtpBroadExtPosCache1.targetVal.seq0 - prevSeqNum[0];
    if(seqDiff0 != 0) {
      prevSeqNum[0] = crtpBroadExtPosCache1.targetVal.seq0;
      state->poscf1.x = (float)crtpBroadExtPosCache1.targetVal.x0 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf1.y = (float)crtpBroadExtPosCache1.targetVal.y0 * 0.000125f;
      state->poscf1.z = (float)crtpBroadExtPosCache1.targetVal.z0 * 0.000125f;
      //Decentralized controller, estimate velocity of leader
      state->velcf1.x = (state->poscf1.x - prevPoscf1.x) * VICON_RATE * seqDiff0; //Vicon updates every 100hz
      state->velcf1.y = (state->poscf1.y - prevPoscf1.y) * VICON_RATE * seqDiff0;
      state->velcf1.z = (state->poscf1.z - prevPoscf1.z) * VICON_RATE * seqDiff0;
      prevPoscf1.x = state->poscf1.x;
      prevPoscf1.y = state->poscf1.y;
      prevPoscf1.z = state->poscf1.z;
    }
    
    uint8_t seqDiff1 = crtpBroadExtPosCache1.targetVal.seq1 - prevSeqNum[1];
    if(seqDiff1 != 0) {
      prevSeqNum[1] = crtpBroadExtPosCache1.targetVal.seq1;
      ext_pos.x = (float)crtpBroadExtPosCache1.targetVal.x1 * 0.000125f; // Scale factor, same as dividing by 8000
      ext_pos.y = (float)crtpBroadExtPosCache1.targetVal.y1 * 0.000125f;
      ext_pos.z = (float)crtpBroadExtPosCache1.targetVal.z1 * 0.000125f;
      ext_pos.stdDev = 0.01f;
      estimatorKalmanEnqueuePosition(&ext_pos);
      return true;
    }
#elif CFNUM == 3
    uint8_t seqDiff0 = crtpBroadExtPosCache1.targetVal.seq0 - prevSeqNum[0];
    if(seqDiff0 != 0) {
      prevSeqNum[0] = crtpBroadExtPosCache1.targetVal.seq0;
      state->poscf1.x = (float)crtpBroadExtPosCache1.targetVal.x0 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf1.y = (float)crtpBroadExtPosCache1.targetVal.y0 * 0.000125f;
      state->poscf1.z = (float)crtpBroadExtPosCache1.targetVal.z0 * 0.000125f;
      //Decentralized controller, estimate velocity of leader
      state->velcf1.x = (state->poscf1.x - prevPoscf1.x) * VICON_RATE * seqDiff0; //Vicon updates every 100hz
      state->velcf1.y = (state->poscf1.y - prevPoscf1.y) * VICON_RATE * seqDiff0;
      state->velcf1.z = (state->poscf1.z - prevPoscf1.z) * VICON_RATE * seqDiff0;
      prevPoscf1.x = state->poscf1.x;
      prevPoscf1.y = state->poscf1.y;
      prevPoscf1.z = state->poscf1.z;
    }
    
    uint8_t seqDiff1 = crtpBroadExtPosCache1.targetVal.seq1 - prevSeqNum[1];
    if(seqDiff1 != 0) {
      prevSeqNum[1] = crtpBroadExtPosCache1.targetVal.seq1;
      state->poscf2.x = (float)crtpBroadExtPosCache1.targetVal.x1 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf2.y = (float)crtpBroadExtPosCache1.targetVal.y1 * 0.000125f;
      state->poscf2.z = (float)crtpBroadExtPosCache1.targetVal.z1 * 0.000125f;
      //Decentralized controller, estimate velocity of robot in front
      state->velcf2.x = (state->poscf2.x - prevPoscf2.x) * VICON_RATE * seqDiff1; //Vicon updates every 100hz
      state->velcf2.y = (state->poscf2.y - prevPoscf2.y) * VICON_RATE * seqDiff1;
      state->velcf2.z = (state->poscf2.z - prevPoscf2.z) * VICON_RATE * seqDiff1;
      prevPoscf2.x = state->poscf2.x;
      prevPoscf2.y = state->poscf2.y;
      prevPoscf2.z = state->poscf2.z;
    }
    
    uint8_t seqDiff2 = crtpBroadExtPosCache1.targetVal.seq2 - prevSeqNum[2];
    if(seqDiff2 != 0) {
      prevSeqNum[2] = crtpBroadExtPosCache1.targetVal.seq2;
      ext_pos.x = (float)crtpBroadExtPosCache1.targetVal.x2 * 0.000125f; // Scale factor, same as dividing by 8000
      ext_pos.y = (float)crtpBroadExtPosCache1.targetVal.y2 * 0.000125f;
      ext_pos.z = (float)crtpBroadExtPosCache1.targetVal.z2 * 0.000125f;
      ext_pos.stdDev = 0.01f;
      estimatorKalmanEnqueuePosition(&ext_pos);
      return true;
    }
#elif CFNUM == 4
    uint8_t seqDiff0 = crtpBroadExtPosCache1.targetVal.seq0 - prevSeqNum[0];
    if(seqDiff0 != 0) {
      prevSeqNum[0] = crtpBroadExtPosCache1.targetVal.seq0;
      state->poscf1.x = (float)crtpBroadExtPosCache1.targetVal.x0 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf1.y = (float)crtpBroadExtPosCache1.targetVal.y0 * 0.000125f;
      state->poscf1.z = (float)crtpBroadExtPosCache1.targetVal.z0 * 0.000125f;
      //Decentralized controller, estimate velocity of leader
      state->velcf1.x = (state->poscf1.x - prevPoscf1.x) * VICON_RATE * seqDiff0; //Vicon updates every 100hz
      state->velcf1.y = (state->poscf1.y - prevPoscf1.y) * VICON_RATE * seqDiff0;
      state->velcf1.z = (state->poscf1.z - prevPoscf1.z) * VICON_RATE * seqDiff0;
      prevPoscf1.x = state->poscf1.x;
      prevPoscf1.y = state->poscf1.y;
      prevPoscf1.z = state->poscf1.z;
    }
    
    uint8_t seqDiff1 = crtpBroadExtPosCache1.targetVal.seq1 - prevSeqNum[1];
    if(seqDiff1 != 0) {
      prevSeqNum[1] = crtpBroadExtPosCache1.targetVal.seq1;
      state->poscf2.x = (float)crtpBroadExtPosCache1.targetVal.x1 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf2.y = (float)crtpBroadExtPosCache1.targetVal.y1 * 0.000125f;
      state->poscf2.z = (float)crtpBroadExtPosCache1.targetVal.z1 * 0.000125f;
      //Decentralized controller, estimate velocity of robot in front
      state->velcf2.x = (state->poscf2.x - prevPoscf2.x) * VICON_RATE * seqDiff1; //Vicon updates every 100hz
      state->velcf2.y = (state->poscf2.y - prevPoscf2.y) * VICON_RATE * seqDiff1;
      state->velcf2.z = (state->poscf2.z - prevPoscf2.z) * VICON_RATE * seqDiff1;
      prevPoscf2.x = state->poscf2.x;
      prevPoscf2.y = state->poscf2.y;
      prevPoscf2.z = state->poscf2.z;
    }
    
    uint8_t seqDiff2 = crtpBroadExtPosCache1.targetVal.seq2 - prevSeqNum[2];
    if(seqDiff2 != 0) {
      prevSeqNum[2] = crtpBroadExtPosCache1.targetVal.seq2;
      state->poscf3.x = (float)crtpBroadExtPosCache1.targetVal.x2 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf3.y = (float)crtpBroadExtPosCache1.targetVal.y2 * 0.000125f;
      state->poscf3.z = (float)crtpBroadExtPosCache1.targetVal.z2 * 0.000125f;
      //Decentralized controller, estimate velocity of robot in front
      state->velcf3.x = (state->poscf3.x - prevPoscf3.x) * VICON_RATE * seqDiff2; //Vicon updates every 100hz
      state->velcf3.y = (state->poscf3.y - prevPoscf3.y) * VICON_RATE * seqDiff2;
      state->velcf3.z = (state->poscf3.z - prevPoscf3.z) * VICON_RATE * seqDiff2;
      prevPoscf3.x = state->poscf3.x;
      prevPoscf3.y = state->poscf3.y;
      prevPoscf3.z = state->poscf3.z;
    }
    
    uint8_t seqDiff3 = crtpBroadExtPosCache1.targetVal.seq3 - prevSeqNum[3];
    if(seqDiff3 != 0) {
      prevSeqNum[3] = crtpBroadExtPosCache1.targetVal.seq3;
      ext_pos.x = (float)crtpBroadExtPosCache1.targetVal.x3 * 0.000125f; // Scale factor, same as dividing by 8000
      ext_pos.y = (float)crtpBroadExtPosCache1.targetVal.y3 * 0.000125f;
      ext_pos.z = (float)crtpBroadExtPosCache1.targetVal.z3 * 0.000125f;
      ext_pos.stdDev = 0.01f;
      estimatorKalmanEnqueuePosition(&ext_pos);
      return true;
    }
#elif CFNUM == 5
uint8_t seqDiff0 = crtpBroadExtPosCache1.targetVal.seq0 - prevSeqNum[0];
    if(seqDiff0 != 0) {
      prevSeqNum[0] = crtpBroadExtPosCache1.targetVal.seq0;
      state->poscf1.x = (float)crtpBroadExtPosCache1.targetVal.x0 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf1.y = (float)crtpBroadExtPosCache1.targetVal.y0 * 0.000125f;
      state->poscf1.z = (float)crtpBroadExtPosCache1.targetVal.z0 * 0.000125f;
      //Decentralized controller, estimate velocity of leader
      state->velcf1.x = (state->poscf1.x - prevPoscf1.x) * VICON_RATE * seqDiff0; //Vicon updates every 100hz
      state->velcf1.y = (state->poscf1.y - prevPoscf1.y) * VICON_RATE * seqDiff0;
      state->velcf1.z = (state->poscf1.z - prevPoscf1.z) * VICON_RATE * seqDiff0;
      prevPoscf1.x = state->poscf1.x;
      prevPoscf1.y = state->poscf1.y;
      prevPoscf1.z = state->poscf1.z;
    }
    
    uint8_t seqDiff1 = crtpBroadExtPosCache1.targetVal.seq1 - prevSeqNum[1];
    if(seqDiff1 != 0) {
      prevSeqNum[1] = crtpBroadExtPosCache1.targetVal.seq1;
      state->poscf2.x = (float)crtpBroadExtPosCache1.targetVal.x1 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf2.y = (float)crtpBroadExtPosCache1.targetVal.y1 * 0.000125f;
      state->poscf2.z = (float)crtpBroadExtPosCache1.targetVal.z1 * 0.000125f;
      //Decentralized controller, estimate velocity of robot in front
      state->velcf2.x = (state->poscf2.x - prevPoscf2.x) * VICON_RATE * seqDiff1; //Vicon updates every 100hz
      state->velcf2.y = (state->poscf2.y - prevPoscf2.y) * VICON_RATE * seqDiff1;
      state->velcf2.z = (state->poscf2.z - prevPoscf2.z) * VICON_RATE * seqDiff1;
      prevPoscf2.x = state->poscf2.x;
      prevPoscf2.y = state->poscf2.y;
      prevPoscf2.z = state->poscf2.z;
    }
    
    uint8_t seqDiff2 = crtpBroadExtPosCache1.targetVal.seq2 - prevSeqNum[2];
    if(seqDiff2 != 0) {
      prevSeqNum[2] = crtpBroadExtPosCache1.targetVal.seq2;
      state->poscf3.x = (float)crtpBroadExtPosCache1.targetVal.x2 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf3.y = (float)crtpBroadExtPosCache1.targetVal.y2 * 0.000125f;
      state->poscf3.z = (float)crtpBroadExtPosCache1.targetVal.z2 * 0.000125f;
      //Decentralized controller, estimate velocity of robot in front
      state->velcf3.x = (state->poscf3.x - prevPoscf3.x) * VICON_RATE * seqDiff2; //Vicon updates every 100hz
      state->velcf3.y = (state->poscf3.y - prevPoscf3.y) * VICON_RATE * seqDiff2;
      state->velcf3.z = (state->poscf3.z - prevPoscf3.z) * VICON_RATE * seqDiff2;
      prevPoscf3.x = state->poscf3.x;
      prevPoscf3.y = state->poscf3.y;
      prevPoscf3.z = state->poscf3.z;
    }
    
    uint8_t seqDiff3 = crtpBroadExtPosCache1.targetVal.seq3 - prevSeqNum[3];
    if(seqDiff3 != 0) {
      prevSeqNum[3] = crtpBroadExtPosCache1.targetVal.seq3;
      state->poscf4.x = (float)crtpBroadExtPosCache1.targetVal.x3 * 0.000125f; // Scale factor, same as dividing by 8000
      state->poscf4.y = (float)crtpBroadExtPosCache1.targetVal.y3 * 0.000125f;
      state->poscf4.z = (float)crtpBroadExtPosCache1.targetVal.z3 * 0.000125f;
      //Decentralized controller, estimate velocity of robot in front
      state->velcf4.x = (state->poscf4.x - prevPoscf4.x) * VICON_RATE * seqDiff3; //Vicon updates every 100hz
      state->velcf4.y = (state->poscf4.y - prevPoscf4.y) * VICON_RATE * seqDiff3;
      state->velcf4.z = (state->poscf4.z - prevPoscf4.z) * VICON_RATE * seqDiff3;
      prevPoscf4.x = state->poscf4.x;
      prevPoscf4.y = state->poscf4.y;
      prevPoscf4.z = state->poscf4.z;
    }
#endif

#if CFNUM == 5
  if ((xTaskGetTickCount() - crtpBroadExtPosCache2.timestamp) < M2T(5)) {
    uint8_t seqDiff4 = crtpBroadExtPosCache2.targetVal.seq0 - prevSeqNum[4];
    if(seqDiff4 != 0) {
      prevSeqNum[4] = crtpBroadExtPosCache2.targetVal.seq0;
      ext_pos.x = (float)crtpBroadExtPosCache2.targetVal.x0 * 0.000125f; // Scale factor, same as dividing by 8000
      ext_pos.y = (float)crtpBroadExtPosCache2.targetVal.y0 * 0.000125f;
      ext_pos.z = (float)crtpBroadExtPosCache2.targetVal.z0 * 0.000125f;
      ext_pos.stdDev = 0.01f;
      estimatorKalmanEnqueuePosition(&ext_pos);
      return true;
    }
  }
#endif
  }
  return false;
}

void locSrvSendPacket(locsrv_t type, uint8_t *data, uint8_t length)
{
  CRTPPacket pk;

  ASSERT(length < CRTP_MAX_DATA_SIZE);

  pk.port = CRTP_PORT_LOCALIZATION;
  pk.channel = GENERIC_TYPE;
  memcpy(pk.data, data, length);
  crtpSendPacket(&pk);
}

void locSrvSendRangeFloat(uint8_t id, float range)
{
  rangePacket *rp = (rangePacket *)pkRange.data;

  ASSERT(rangeIndex <= NBR_OF_RANGES_IN_PACKET);

  if (enableRangeStreamFloat)
  {
    rp->ranges[rangeIndex].id = id;
    rp->ranges[rangeIndex].range = range;
    rangeIndex++;

    if (rangeIndex >= 5)
    {
      rp->type = RANGE_STREAM_FLOAT;
      pkRange.port = CRTP_PORT_LOCALIZATION;
      pkRange.channel = GENERIC_TYPE;
      pkRange.size = sizeof(rangePacket);
      crtpSendPacket(&pkRange);
      rangeIndex = 0;
    }
  }
}

/*
LOG_GROUP_START(ext_pos)
  LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
  LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
  LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)*/

PARAM_GROUP_START(locSrv)
PARAM_ADD(PARAM_UINT8, enRangeStreamFP32, &enableRangeStreamFloat)
PARAM_GROUP_STOP(locSrv)
