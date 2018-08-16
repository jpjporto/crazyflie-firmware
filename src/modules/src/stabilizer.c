/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "crtp_commander.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator_kalman.h"
#include "estimator.h"

#ifdef DEC_DECA
#include "locodeck.h"
#include "lpsTdoaTagv2.h"
#endif

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static void stabilizerTask(void* param);

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
#if !defined(CONTROLLER_TYPE_pid) && !defined(CONTROLLER_TYPE_mellinger) && !defined(CONTROLLER_TYPE_lqr)
  hinfControllerInit();
#elif defined(CONTROLLER_TYPE_lqr)
  lqrControllerInit();
#else
  stateControllerInit();
#endif
  powerDistributionInit();
  
#if defined(CONTROLLER_TYPE_pid) || defined(CONTROLLER_TYPE_mellinger)
  if (estimator == kalmanEstimator)
  {
    sitAwInit();
  }
#endif

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
#if defined(CONTROLLER_TYPE_pid) || defined(CONTROLLER_TYPE_mellinger)
  pass &= stateControllerTest();
#endif
  pass &= powerDistributionTest();

  return pass;
}

#if defined(CONTROLLER_TYPE_pid) || defined(CONTROLLER_TYPE_mellinger)
static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}
#endif

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    getExtPosition(&state);
    stateEstimator(&state, &sensorData, &control, tick);
    
    #ifdef DEC_DECA
    setCFState(&state);
    getDecState(&state);
    #endif

#if !defined(CONTROLLER_TYPE_pid) && !defined(CONTROLLER_TYPE_mellinger)
    getSetpoint(&setpoint);
#else
    commanderGetSetpoint(&setpoint, &state);
#endif

#if defined(CONTROLLER_TYPE_pid) || defined(CONTROLLER_TYPE_mellinger)
    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
#endif

#if !defined(CONTROLLER_TYPE_pid) && !defined(CONTROLLER_TYPE_mellinger) && !defined(CONTROLLER_TYPE_lqr)
    hinfController(&control, &setpoint, &state, tick);
#elif defined(CONTROLLER_TYPE_lqr)
    lqrController(&control, &setpoint, &state, tick);
#else
    stateController(&control, &setpoint, &sensorData, &state, tick);
#endif

    if (RATE_DO_EXECUTE(HINF_RATE, tick))
    {
#if defined(CONTROLLER_TYPE_pid) || defined(CONTROLLER_TYPE_mellinger)    
      checkEmergencyStopTimeout();

      if (emergencyStop) {
        powerStop();
      } else {
        powerDistribution(&control);
      }
#else
      powerDistribution(&control);
#endif
    }

    tick++;
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/*
LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(setpoint)
LOG_ADD(LOG_FLOAT, x, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, y, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, z, &setpoint.position.z)
LOG_GROUP_STOP(setpoint)*/

/*
LOG_GROUP_START(cfsetpoint)
LOG_ADD(LOG_FLOAT, x, &setpoint.poscf1.x)
LOG_ADD(LOG_FLOAT, y, &setpoint.poscf1.y)
LOG_ADD(LOG_FLOAT, z, &setpoint.poscf1.z)
LOG_GROUP_STOP(cfsetpoint)*/

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
#if !defined(CONTROLLER_TYPE_pid) && !defined(CONTROLLER_TYPE_mellinger)
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)
#else
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
#endif
LOG_GROUP_STOP(stabilizer)

#if defined(CONTROLLER_TYPE_pid) || defined(CONTROLLER_TYPE_mellinger)
LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)
#endif

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

/*
LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)*/

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

/*
LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)*/

LOG_GROUP_START(controller)
#if !defined(CONTROLLER_TYPE_pid) && !defined(CONTROLLER_TYPE_mellinger)
LOG_ADD(LOG_FLOAT, ctr_thrust, &control.thrust)
LOG_ADD(LOG_FLOAT, ctr_roll, &control.roll)
LOG_ADD(LOG_FLOAT, ctr_pitch, &control.pitch)
LOG_ADD(LOG_FLOAT, ctr_yaw, &control.yaw)
LOG_ADD(LOG_UINT8, enabled, &control.enabled)
#else
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
#endif
LOG_GROUP_STOP(controller)

LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)
LOG_GROUP_STOP(stateEstimate)

#ifdef DEC_DECA
#if CFNUM >= 2
LOG_GROUP_START(DecState)
LOG_ADD(LOG_FLOAT, x, &state.s_dec[0])
LOG_ADD(LOG_FLOAT, y, &state.s_dec[1])
LOG_ADD(LOG_FLOAT, z, &state.s_dec[2])
LOG_GROUP_STOP(DecState)
#endif
#endif
