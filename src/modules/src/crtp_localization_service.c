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
  struct CrtpBroadExtPosition targetVal[2];
  bool activeSide;
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
static ExtBroadPosCache crtpBroadExtPosCache;
static uint8_t prevSeqNum[3];

#if CFNUM >= 2
static point_t prevPoscf1;
  #if CFNUM == 3
static point_t prevPoscf2;
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
  crtpBroadExtPosCache.targetVal[!crtpBroadExtPosCache.activeSide] = *((struct CrtpBroadExtPosition*)pk->data);
  crtpBroadExtPosCache.activeSide = !crtpBroadExtPosCache.activeSide;
  crtpBroadExtPosCache.timestamp = xTaskGetTickCount();
}

bool getExtPosition(state_t *state)
{
  // Only use position information if it's valid and recent
  if ((xTaskGetTickCount() - crtpExtPosCache.timestamp) < M2T(5)) {
    // Get the updated position from the mocap
    ext_pos.x = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].x;
    ext_pos.y = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].y;
    ext_pos.z = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].z;
    ext_pos.stdDev = 0.01;
    estimatorKalmanEnqueuePosition(&ext_pos);

    return true;
  }
  
  if ((xTaskGetTickCount() - crtpBroadExtPosCache.timestamp) < M2T(5)) {
    uint8_t seqDiff0 = crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].seq0 - prevSeqNum[0];
    if(seqDiff0 != 0) {
      prevSeqNum[0] = crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].seq0;
#if defined(CONTROLLER_TYPE_hinfdec) && CFNUM >= 2
      state->poscf1.x = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].x0 / 8000.0f;
      state->poscf1.y = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].y0 / 8000.0f;
      state->poscf1.z = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].z0 / 8000.0f;
      //Decentralized controller, estimate velocity of leader
      state->velcf1.x = (state->poscf1.x - prevPoscf1.x) * 100 * seqDiff0; //Vicon updates every 100hz
      state->velcf1.y = (state->poscf1.y - prevPoscf1.y) * 100 * seqDiff0;
      state->velcf1.z = (state->poscf1.z - prevPoscf1.z) * 100 * seqDiff0;
      prevPoscf1.x = state->poscf1.x;
      prevPoscf1.y = state->poscf1.y;
      prevPoscf1.z = state->poscf1.z;
#else
      ext_pos.x = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].x0 / 8000.0f;
      ext_pos.y = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].y0 / 8000.0f;
      ext_pos.z = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].z0 / 8000.0f;
      ext_pos.stdDev = 0.01;
      estimatorKalmanEnqueuePosition(&ext_pos);
      return true;
#endif
    }
#if CFNUM >= 2
    uint8_t seqDiff1 = crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].seq1 - prevSeqNum[1];
    if(seqDiff1 != 0) {
      prevSeqNum[1] = crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].seq1;
  #if CFNUM == 3
      state->poscf2.x = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].x1 / 8000.0f;
      state->poscf2.y = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].y1 / 8000.0f;
      state->poscf2.z = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].z1 / 8000.0f;
      //Decentralized controller, estimate velocity of robot in front
      state->velcf2.x = (state->poscf2.x - prevPoscf2.x) * 100 * seqDiff1; //Vicon updates every 100hz
      state->velcf2.y = (state->poscf2.y - prevPoscf2.y) * 100 * seqDiff1;
      state->velcf2.z = (state->poscf2.z - prevPoscf2.z) * 100 * seqDiff1;
      prevPoscf2.x = state->poscf2.x;
      prevPoscf2.y = state->poscf2.y;
      prevPoscf2.z = state->poscf2.z;
  #else
      ext_pos.x = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].x1 / 8000.0f;
      ext_pos.y = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].y1 / 8000.0f;
      ext_pos.z = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].z1 / 8000.0f;
      ext_pos.stdDev = 0.01;
      estimatorKalmanEnqueuePosition(&ext_pos);
      return true;
  #endif
    }
  #if CFNUM == 3
    uint8_t seqDiff2 = crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].seq2 - prevSeqNum[2];
    if(seqDiff2 != 0) {
      prevSeqNum[2] = crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].seq2;
      ext_pos.x = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].x2 / 8000.0f;
      ext_pos.y = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].y2 / 8000.0f;
      ext_pos.z = (float)crtpBroadExtPosCache.targetVal[crtpBroadExtPosCache.activeSide].z2 / 8000.0f;
      ext_pos.stdDev = 0.01;
      estimatorKalmanEnqueuePosition(&ext_pos);
      return true;
    }
  #endif
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

LOG_GROUP_START(ext_pos)
  LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
  LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
  LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)

PARAM_GROUP_START(locSrv)
PARAM_ADD(PARAM_UINT8, enRangeStreamFP32, &enableRangeStreamFloat)
PARAM_GROUP_STOP(locSrv)
