/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * lpsTdoaTag.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoaTag.c.  If not, see <http://www.gnu.org/licenses/>.
 */

 
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "num.h"
#include "lpsTdoaTagv2.h"

#include "stabilizer_types.h"
#include "cfassert.h"

#include "estimator.h"
#include "estimator_kalman.h"

#define ANCHOR_OK_TIMEOUT 1500
#define TDOA_RECEIVE_TIMEOUT 10000

#define DECA_DEBUG

static lpsAlgoOptions_t* options;

#ifdef DECA_DEBUG
static uint16_t anchorDistanceLog[LOCODECK_NR_OF_ANCHORS];
static float clockCorrectionLog[LOCODECK_NR_OF_ANCHORS];

static uint32_t statsReceivedPackets = 0;
static uint32_t statsRejectedSeq = 0;
static uint32_t statsAcceptedPackets = 0;
static uint16_t statsRecv[LOCODECK_NR_OF_ANCHORS];
static uint16_t statsDecRecv = 0;
#endif

static uint8_t previousAnchor;
static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_ANCHORS];
static dwTime_t arrivals[LOCODECK_NR_OF_ANCHORS];
static uint8_t sequenceNrs[LOCODECK_NR_OF_ANCHORS];

static float uwbTdoaDistDiff[LOCODECK_NR_OF_ANCHORS];
static double clockCorrection_T_To_A[LOCODECK_NR_OF_ANCHORS];

#define MEASUREMENT_NOISE_STD 0.15f

static bool rangingOk;
static uint32_t anchorStatusTimeout[LOCODECK_NR_OF_ANCHORS];

#ifdef DEC_DECA
static dwTime_t last_rx = {.full = 0};
static packet_t txPacket;
void setCFState(const state_t *state)
{
  lpsCFStatePacket_t *statePacket = (lpsCFStatePacket_t *)txPacket.payload;
  
  statePacket->cfstate_s.x = state->position.x;
  statePacket->cfstate_s.y = state->position.y;
  statePacket->cfstate_s.z = state->position.z;
  statePacket->cfstate_s.vx = state->velocity.x;
  statePacket->cfstate_s.vy = state->velocity.y;
  statePacket->cfstate_s.vz = state->velocity.z;
  statePacket->cfstate_s.roll = state->attitude.roll;
  statePacket->cfstate_s.pitch = state->attitude.pitch;
  statePacket->cfstate_s.yaw = state->attitude.yaw;
  statePacket->cfstate_s.vroll = state->attitudeRate.roll;
  statePacket->cfstate_s.vpitch = state->attitudeRate.pitch;
  statePacket->cfstate_s.vyaw = state->attitudeRate.yaw;
}

#if CFNUM >= 2
float stemp[12 * (CFNUM-1)];
uint8_t new_state;
#endif

void getDecState(state_t *state)
{
    #if CFNUM >= 2
    if(new_state)
    {
        memcpy(state->s_dec, stemp, sizeof(stemp));
        new_state = 0;
    }
    #endif
}

static void sendCFState(dwDevice_t *dev)
{
  static uint8_t firstEntry = 1;
  dwIdle(dev);

  if(firstEntry)
  {
    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
    txPacket.pan = 0xbccf;
    txPacket.sourceAddress = 0xbccf00000000cf00 | CFNUM;
    txPacket.destAddress = 0xbccf0000000000FF;
    
    txPacket.payload[0] = LPP_STATE_PACKET;
    txPacket.payload[1] = CFNUM;
    firstEntry = 0;
  }
  
  dwTime_t txTime = last_rx;
  txTime.full += STATE_TX_DLY;
  
  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+sizeof(lpsCFStatePacket_t));
  dwSetTxRxTime(dev, txTime);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}
#endif

//static uint64_t truncateToLocalTimeStamp(uint64_t fullTimeStamp) {
//  return fullTimeStamp & 0x00FFFFFFFFul;
//}

static inline uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static void enqueueTDOA(uint8_t anchor1, uint8_t anchor2, double distanceDiff) {
  tdoaMeasurement_t tdoa = {
    .stdDev = MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff,

    .anchorPosition[0] = options->anchorPosition[anchor1],
    .anchorPosition[1] = options->anchorPosition[anchor2]
  };

  estimatorKalmanEnqueueTDOA(&tdoa);
}

// The default receive time in the anchors for messages from other anchors is 0
// and is overwritten with the actual receive time when a packet arrives.
// That is, if no message was received the rx time will be 0.
//static bool isValidTimeStamp(const int64_t anchorRxTime) {
//  return anchorRxTime != 0;
//}

static bool isSeqNrConsecutive(uint8_t prevSeqNr, uint8_t currentSeqNr) {
  return (currentSeqNr == ((prevSeqNr + 1) & 0xFF));
}

static bool isSameFrame(const uint8_t Ar, const uint8_t An, const uint8_t packetIdx) {
    if (Ar < An) {
      return rxPacketBuffer[Ar].Idx == packetIdx;
    }
    else
    {
      return rxPacketBuffer[Ar].Idx == (packetIdx - 1);
    }
}

#pragma GCC push_options
#pragma GCC optimize ("O3")
static bool calcClockCorrection(double* clockCorrection, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {
  if (! isSeqNrConsecutive(rxPacketBuffer[anchor].Idx, packet->Idx)) {
    #ifdef DECA_DEBUG
    statsRejectedSeq++;
    #endif
    return false;
  }

/*  const int64_t previous_txAn_in_cl_An = rxPacketBuffer[anchor].timestamps[anchor];
  const int64_t rxAn_by_T_in_cl_T = arrival->full;
  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t previous_rxAn_by_T_in_cl_T = arrivals[anchor].full;
  const double frameTime_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - previous_txAn_in_cl_An);
  const double frameTime_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - previous_rxAn_by_T_in_cl_T);

  *clockCorrection = frameTime_in_cl_An / frameTime_in_T;*/
  
  const uint32_t previous_txAn_in_cl_An = rxPacketBuffer[anchor].timestamps[anchor];
  const uint32_t rxAn_by_T_in_cl_T = arrival->full;
  const uint32_t txAn_in_cl_An = packet->timestamps[anchor];
  const uint32_t previous_rxAn_by_T_in_cl_T = arrivals[anchor].full;
  uint32_t frameTime_in_cl_An = (txAn_in_cl_An - previous_txAn_in_cl_An);
  uint32_t frameTime_in_T = (rxAn_by_T_in_cl_T - previous_rxAn_by_T_in_cl_T);

  *clockCorrection = (double)frameTime_in_cl_An / (double)frameTime_in_T;
  return true;
}

static bool calcDistanceDiff(float* tdoaDistDiff, const uint8_t previousAnchor, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {
//  if (! isSeqNrConsecutive(sequenceNrs[anchor], packet->Idx)) {
//    return false;
//  }
  if (! isSameFrame(previousAnchor, anchor, packet->Idx)) {
    return false;
  }
  
  //Can probably change most of these to uint32_t
  const uint32_t rxAr_by_An_in_cl_An = packet->timestamps[previousAnchor];
  const uint32_t tof_Ar_to_An_in_cl_An = packet->distances[previousAnchor];
  const double clockCorrection = clockCorrection_T_To_A[anchor];

  const bool isAnchorDistanceOk = (tof_Ar_to_An_in_cl_An != 0);
  const bool isRxTimeInTagOk = (rxAr_by_An_in_cl_An != 0); // Same as isValidTimeStamp() fcn
  const bool isClockCorrectionOk = (clockCorrection != 0.0);

  if (! (isAnchorDistanceOk && isRxTimeInTagOk && isClockCorrectionOk)) {
    return false;
  }

  const uint32_t rxAn_by_T_in_cl_T  = arrival->full;
  const uint32_t txAn_in_cl_An = packet->timestamps[anchor];
  const uint32_t rxAr_by_T_in_cl_T = arrivals[previousAnchor].full;

  int32_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + (txAn_in_cl_An - rxAr_by_An_in_cl_An));
  uint32_t tmp1 = rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T;
  int32_t tmp2 = (double)tmp1 * clockCorrection;
  int32_t timeDiffOfArrival_in_cl_An =  tmp2 - delta_txAr_to_txAn_in_cl_An;

  //*tdoaDistDiff = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_An / LOCODECK_TS_FREQ;
  *tdoaDistDiff = (float)timeDiffOfArrival_in_cl_An * 0.004691763978616f;

  return true;
}
#pragma GCC pop_options

#ifdef DECA_DEBUG
static void addToLog(const uint8_t anchor, const float tdoaDistDiff, const rangePacket_t* packet) {
  // Only store diffs for anchors with succeeding numbers. In case of packet
  // loss we can get ranging between any anchors and that messes up the graphs.
  if (((previousAnchor + 1) & 0x07) == anchor) {
    uwbTdoaDistDiff[anchor] = tdoaDistDiff;
    anchorDistanceLog[anchor] = packet->distances[previousAnchor];
  }
}
#endif

// A note on variable names. They might seem a bit verbose but express quite a lot of information
// We have three actors: Reference anchor (Ar), Anchor n (An) and the deck on the CF called Tag (T)
// rxAr_by_An_in_cl_An should be interpreted as "The time when packet was received from the Reference Anchor by Anchor N expressed in the clock of Anchor N"
#ifdef DEC_DECA
static bool rxcallback(dwDevice_t *dev)
#else
static void rxcallback(dwDevice_t *dev)
#endif
{
  #ifdef DECA_DEBUG
  statsReceivedPackets++;
  #endif

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

#ifdef DEC_DECA
  if (rxPacket.payload[0] == PACKET_TYPE_RANGE)
  {
#endif
    dwTime_t arrival = {.full = 0};
    dwGetReceiveTimestamp(dev, &arrival);
    const uint8_t anchor = rxPacket.sourceAddress & 0xFF;
    
    const rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;
    #ifdef DECA_DEBUG
    statsRecv[anchor]++;
    #endif
    
    calcClockCorrection(&clockCorrection_T_To_A[anchor], anchor, packet, &arrival);
    #ifdef DECA_DEBUG
    clockCorrectionLog[anchor] = clockCorrection_T_To_A[anchor];
    #endif

    if (anchor != previousAnchor) 
    {
      float tdoaDistDiff = 0.0f;
      if (calcDistanceDiff(&tdoaDistDiff, previousAnchor, anchor, packet, &arrival)) 
      {
        enqueueTDOA(previousAnchor, anchor, tdoaDistDiff);
        #ifdef DECA_DEBUG
        addToLog(anchor, tdoaDistDiff, packet);

        statsAcceptedPackets++;
        #endif
      }
    }

    arrivals[anchor].full = arrival.full;
    memcpy(&rxPacketBuffer[anchor], rxPacket.payload, sizeof(rangePacket_t));
    sequenceNrs[anchor] = packet->Idx;
    
    anchorStatusTimeout[anchor] = xTaskGetTickCount() + ANCHOR_OK_TIMEOUT;

    previousAnchor = anchor;
#ifdef DEC_DECA
    // Broadcast position between anchor messages
    if ((anchor == CFNUM-1) || (anchor == CFNUM+3)) //This allows up to 4 cfs to send pos
    {
      dwGetReceiveTimestamp(dev, &last_rx);
      return true;
    }
  }
  else if (rxPacket.payload[0] == LPP_STATE_PACKET)
  {
    #if CFNUM >= 2
    if (rxPacket.payload[1] < CFNUM)
    {
      // Got a valid package, save state info
      const lpsCFStatePacket_t* packet = (lpsCFStatePacket_t*)rxPacket.payload;
    
      memcpy(&stemp[(packet->source-1)*12], packet->data, 12*sizeof(float));
      new_state = 1;
      #ifdef DECA_DEBUG
      statsDecRecv++;
      #endif
    }
    #endif
  }
  return false;
#endif
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
#ifdef DEC_DECA
      if(rxcallback(dev))
      {
        sendCFState(dev);
      }
      else
      {
        setRadioInReceiveMode(dev);
      }
#else
      rxcallback(dev);
      setRadioInReceiveMode(dev);
#endif
      break;
    case eventTimeout:
      setRadioInReceiveMode(dev);
      break;
    case eventReceiveTimeout:
      setRadioInReceiveMode(dev);
      break;
#ifdef DEC_DECA
    case eventPacketSent:
      setRadioInReceiveMode(dev);
      break;
#endif
    default:
      ASSERT_FAILED();
  }
  uint32_t now = xTaskGetTickCount();
  
  options->rangingState = 0;
  for (int anchor = 0; anchor < LOCODECK_NR_OF_ANCHORS; anchor++) {
    if (now < anchorStatusTimeout[anchor]) {
      options->rangingState |= (1 << anchor);
    }
  }

  return MAX_TIMEOUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
  options = algoOptions;

  // Reset module state. Needed by unit tests
  memset(rxPacketBuffer, 0, sizeof(rxPacketBuffer));
  memset(arrivals, 0, sizeof(arrivals));

  memset(clockCorrection_T_To_A, 0, sizeof(clockCorrection_T_To_A));

  previousAnchor = 0;

  memset(uwbTdoaDistDiff, 0, sizeof(uwbTdoaDistDiff));

  #ifdef DECA_DEBUG
  statsReceivedPackets = 0;
  statsAcceptedPackets = 0;
  statsRejectedSeq = 0;
  memset(statsRecv, 0, sizeof(statsRecv));
  #endif
  
  dwSetReceiveWaitTimeout(dev, TDOA_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);
  
  rangingOk = false;
  
  #ifdef DEC_DECA
  #if CFNUM >= 2
  memset(stemp, 0, sizeof(stemp));
  new_state = 0;
  #endif
  #endif
}
#pragma GCC diagnostic pop

static bool isRangingOk()
{
  return rangingOk;
}

uwbAlgorithm_t uwbTdoaTagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
  .isRangingOk = isRangingOk,
};

#ifdef DECA_DEBUG
LOG_GROUP_START(tdoa)
LOG_ADD(LOG_FLOAT, d7-0, &uwbTdoaDistDiff[0])
LOG_ADD(LOG_FLOAT, d0-1, &uwbTdoaDistDiff[1])
LOG_ADD(LOG_FLOAT, d1-2, &uwbTdoaDistDiff[2])
LOG_ADD(LOG_FLOAT, d2-3, &uwbTdoaDistDiff[3])
LOG_ADD(LOG_FLOAT, d3-4, &uwbTdoaDistDiff[4])
LOG_ADD(LOG_FLOAT, d4-5, &uwbTdoaDistDiff[5])
LOG_ADD(LOG_FLOAT, d5-6, &uwbTdoaDistDiff[6])
LOG_ADD(LOG_FLOAT, d6-7, &uwbTdoaDistDiff[7])

LOG_ADD(LOG_FLOAT, cc0, &clockCorrectionLog[0])
LOG_ADD(LOG_FLOAT, cc1, &clockCorrectionLog[1])
LOG_ADD(LOG_FLOAT, cc2, &clockCorrectionLog[2])
LOG_ADD(LOG_FLOAT, cc3, &clockCorrectionLog[3])
LOG_ADD(LOG_FLOAT, cc4, &clockCorrectionLog[4])
LOG_ADD(LOG_FLOAT, cc5, &clockCorrectionLog[5])
LOG_ADD(LOG_FLOAT, cc6, &clockCorrectionLog[6])
LOG_ADD(LOG_FLOAT, cc7, &clockCorrectionLog[7])

LOG_ADD(LOG_UINT16, dist7-0, &anchorDistanceLog[0])
LOG_ADD(LOG_UINT16, dist0-1, &anchorDistanceLog[1])
LOG_ADD(LOG_UINT16, dist1-2, &anchorDistanceLog[2])
LOG_ADD(LOG_UINT16, dist2-3, &anchorDistanceLog[3])
LOG_ADD(LOG_UINT16, dist3-4, &anchorDistanceLog[4])
LOG_ADD(LOG_UINT16, dist4-5, &anchorDistanceLog[5])
LOG_ADD(LOG_UINT16, dist5-6, &anchorDistanceLog[6])
LOG_ADD(LOG_UINT16, dist6-7, &anchorDistanceLog[7])

LOG_ADD(LOG_UINT32, rxCnt, &statsReceivedPackets)
LOG_ADD(LOG_UINT32, okCnt, &statsAcceptedPackets)
LOG_ADD(LOG_UINT32, seqCnt, &statsRejectedSeq)
LOG_ADD(LOG_UINT16, recv0, &statsRecv[0])
LOG_ADD(LOG_UINT16, recv1, &statsRecv[1])
LOG_ADD(LOG_UINT16, recv2, &statsRecv[2])
LOG_ADD(LOG_UINT16, recv3, &statsRecv[3])
LOG_ADD(LOG_UINT16, recv4, &statsRecv[4])
LOG_ADD(LOG_UINT16, recv5, &statsRecv[5])
LOG_ADD(LOG_UINT16, recv6, &statsRecv[6])
LOG_ADD(LOG_UINT16, recv7, &statsRecv[7])

LOG_ADD(LOG_UINT16, recvDec, &statsDecRecv)
LOG_GROUP_STOP(tdoa)
#endif
