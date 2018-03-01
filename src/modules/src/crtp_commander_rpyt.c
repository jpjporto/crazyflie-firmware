/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
#include <math.h>
#include <stdbool.h>

#include "crtp_commander.h"

#include "commander.h"
#include "crtp.h"
#include "param.h"
#include "FreeRTOS.h"
#include "num.h"
#include "task.h"

#define MIN_THRUST  1000
#define MAX_THRUST  60000

/**
 * CRTP commander rpyt packet format
 */
struct CommanderCrtpLegacyValues
{
  float roll;       // deg
  float pitch;      // deg
  float yaw;        // deg
  uint16_t thrust;
} __attribute__((packed));

struct CommanderCrtpBroadcast
{
  uint8_t seq;
  uint8_t mode;
  int16_t xs0;
  int16_t ys0;
  int16_t zs0;
  int16_t xs1;
  int16_t ys1;
  int16_t zs1;
  int16_t xs2;
  int16_t ys2;
  int16_t zs2;
} __attribute__((packed));

/**
 * Stabilization modes for Roll, Pitch, Yaw.
 */
typedef enum
{
  RATE    = 0,
  ANGLE   = 1,
} RPYType;

/**
 * Yaw flight Modes
 */
typedef enum
{
  CAREFREE  = 0, // Yaw is locked to world coordinates thus heading stays the same when yaw rotates
  PLUSMODE  = 1, // Plus-mode. Motor M1 is defined as front
  XMODE     = 2, // X-mode. M1 & M4 are defined as front
} YawModeType;

static RPYType stabilizationModeRoll  = ANGLE; // Current stabilization type of roll (rate or angle)
static RPYType stabilizationModePitch = ANGLE; // Current stabilization type of pitch (rate or angle)
static RPYType stabilizationModeYaw   = RATE;  // Current stabilization type of yaw (rate or angle)

static YawModeType yawMode = DEFAULT_YAW_MODE; // Yaw mode configuration
static bool carefreeResetFront;             // Reset what is front in carefree mode

static bool thrustLocked = true;
static bool altHoldMode = false;
static bool posHoldMode = false;
static bool posSetMode = false;

/**
 * Rotate Yaw so that the Crazyflie will change what is considered front.
 *
 * @param yawRad Amount of radians to rotate yaw.
 */
static void rotateYaw(setpoint_t *setpoint, float yawRad)
{
  float cosy = cosf(yawRad);
  float siny = sinf(yawRad);
  float originalRoll = setpoint->attitude.roll;
  float originalPitch = setpoint->attitude.pitch;

  setpoint->attitude.roll = originalRoll * cosy - originalPitch * siny;
  setpoint->attitude.pitch = originalPitch * cosy + originalRoll * siny;
}

/**
 * Update Yaw according to current setting
 */
static void yawModeUpdate(setpoint_t *setpoint)
{
  switch (yawMode)
  {
    case CAREFREE:
      // TODO: Add frame of reference to setpoint
      ASSERT(false);
      break;
    case PLUSMODE:
      rotateYaw(setpoint, 45 * M_PI / 180);
      break;
    case XMODE: // Fall through
    default:
      // Default in x-mode. Do nothing
      break;
  }
}

void crtpCommanderRpytDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
{
  struct CommanderCrtpLegacyValues *values = (struct CommanderCrtpLegacyValues*)pk->data;

  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
  }
  if (values->thrust == 0) {
    thrustLocked = false;
  }

  // Thrust
  uint16_t rawThrust = values->thrust;

  if (thrustLocked || (rawThrust < MIN_THRUST)) {
    setpoint->thrust = 0;
  } else {
    setpoint->thrust = min(rawThrust, MAX_THRUST);
  }

  if (altHoldMode) {
    setpoint->thrust = 0;
    setpoint->mode.z = modeVelocity;

    setpoint->velocity.z = ((float) rawThrust - 32767.f) / 32767.f;
  } else {
    setpoint->mode.z = modeDisable;
  }

  // roll/pitch
  if (posHoldMode) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;

    setpoint->velocity.x = values->pitch/30.0f;
    setpoint->velocity.y = values->roll/30.0f;
    setpoint->attitude.roll  = 0;
    setpoint->attitude.pitch = 0;
  } else if (posSetMode && values->thrust != 0) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;

    setpoint->position.x = -values->pitch;
    setpoint->position.y = values->roll;
    setpoint->position.z = values->thrust/1000.0f;

    setpoint->attitude.roll  = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0;
  } else {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;

    if (stabilizationModeRoll == RATE) {
      setpoint->mode.roll = modeVelocity;
      setpoint->attitudeRate.roll = values->roll;
      setpoint->attitude.roll = 0;
    } else {
      setpoint->mode.roll = modeAbs;
      setpoint->attitudeRate.roll = 0;
      setpoint->attitude.roll = values->roll;
    }

    if (stabilizationModePitch == RATE) {
      setpoint->mode.pitch = modeVelocity;
      setpoint->attitudeRate.pitch = values->pitch;
      setpoint->attitude.pitch = 0;
    } else {
      setpoint->mode.pitch = modeAbs;
      setpoint->attitudeRate.pitch = 0;
      setpoint->attitude.pitch = values->pitch;
    }

    setpoint->velocity.x = 0;
    setpoint->velocity.y = 0;
  }

  // Yaw
  if (!posSetMode) {
    setpoint->attitudeRate.yaw  = values->yaw;
    yawModeUpdate(setpoint);

    setpoint->mode.yaw = modeVelocity;
  }
}

#ifdef SETPOINT_VEL
static point_t prevSetpoint1;
#if CFNUM >=2
static point_t prevSetpoint2;
#endif
#endif
static uint8_t prevSeqNum;

typedef struct
{
  struct CommanderCrtpBroadcast targetVal[2];
  bool activeSide;
  uint32_t timestamp; // FreeRTOS ticks
} BroadSetCache;
static BroadSetCache crtpBroadSetpointCache;

void broadcastSetpointHandler(CRTPPacket* pk)
{
  crtpBroadSetpointCache.targetVal[!crtpBroadSetpointCache.activeSide] = *((struct CommanderCrtpBroadcast*)pk->data);
  crtpBroadSetpointCache.activeSide = !crtpBroadSetpointCache.activeSide;
  crtpBroadSetpointCache.timestamp = xTaskGetTickCount();
}

//void crtpDecodeBroadcastSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
void getSetpoint(setpoint_t *setpoint)
{
  if ((xTaskGetTickCount() - crtpBroadSetpointCache.timestamp) < M2T(5)) {
    struct CommanderCrtpBroadcast *values = &crtpBroadSetpointCache.targetVal[!crtpBroadSetpointCache.activeSide];
    uint8_t seqDiff = values->seq - prevSeqNum;
    if(seqDiff != 0) {
      prevSeqNum = values->seq;
      setpoint->sys_mode = values->mode;
      setpoint->poscf1.x = (float)values->xs0 / 8000.0f;
      setpoint->poscf1.y = (float)values->ys0 / 8000.0f;
      setpoint->poscf1.z = (float)values->zs0 / 8000.0f;
  #if CFNUM >= 2
      setpoint->poscf2.x = (float)values->xs1 / 8000.0f;
      setpoint->poscf2.y = (float)values->ys1 / 8000.0f;
      setpoint->poscf2.z = (float)values->zs1 / 8000.0f;
    #if CFNUM == 3
      setpoint->poscf3.x = (float)values->xs2 / 8000.0f;
      setpoint->poscf3.y = (float)values->ys2 / 8000.0f;
      setpoint->poscf3.z = (float)values->zs2 / 8000.0f;
    #endif
  #endif

#ifdef SETPOINT_VEL
      setpoint->velcf1.x = (setpoint->poscf1.x - prevSetpoint1.x) * 500 * seqDiff; // This line assumes we are getting a packet every 2ms.
      setpoint->velcf1.y = (setpoint->poscf1.y - prevSetpoint1.y) * 500 * seqDiff; // This assumption gives a more conservative approximation of the derivative,
      setpoint->velcf1.z = (setpoint->poscf1.z - prevSetpoint1.z) * 500 * seqDiff; // but it might be the best option given that xTaskGetTickCount() only has 1ms resolution.
      prevSetpoint1.x = setpoint->poscf1.x;
      prevSetpoint1.y = setpoint->poscf1.y;
      prevSetpoint1.z = setpoint->poscf1.z;
  #if CFNUM >=2
      setpoint->velcf2.x = (setpoint->poscf2.x - prevSetpoint2.x) * 500 * seqDiff; // This line assumes we are getting a packet every 2ms.
      setpoint->velcf2.y = (setpoint->poscf2.y - prevSetpoint2.y) * 500 * seqDiff; // This assumption gives a more conservative approximation of the derivative,
      setpoint->velcf2.z = (setpoint->poscf2.z - prevSetpoint2.z) * 500 * seqDiff; // but it might be the best option given that xTaskGetTickCount() only has 1ms resolution.
      prevSetpoint2.x = setpoint->poscf2.x;
      prevSetpoint2.y = setpoint->poscf2.y;
      prevSetpoint2.z = setpoint->poscf2.z;
  #endif
#endif
    }
  }
}

// Params for flight modes
PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
PARAM_ADD(PARAM_UINT8, poshold, &posHoldMode)
PARAM_ADD(PARAM_UINT8, posSet, &posSetMode)
PARAM_ADD(PARAM_UINT8, yawMode, &yawMode)
PARAM_ADD(PARAM_UINT8, yawRst, &carefreeResetFront)
PARAM_ADD(PARAM_UINT8, stabModeRoll, &stabilizationModeRoll)
PARAM_ADD(PARAM_UINT8, stabModePitch, &stabilizationModePitch)
PARAM_ADD(PARAM_UINT8, stabModeYaw, &stabilizationModeYaw)
PARAM_GROUP_STOP(flightmode)
