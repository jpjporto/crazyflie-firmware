/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
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
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#include "power_distribution.h"

#include "log.h"
#include "param.h"
#include "num.h"

#include "motors.h"

#include "math.h"
#include "arm_math.h"

#define CRAZYFLIE_ARM_LENGTH 0.046f
#define THRUST_TO_TORQUE_m   0.005964552f
#define PWM_TO_THRUST_a      0.091492681f
#define PWM_TO_THRUST_b      0.067673604f

#define hover_omega 15465.36f
#define d2we        3.2330317e-05f
#define d2we4Ct     2.5592359e+04f
#define d2we4lCt    5.5635563e+05f
#define d2we4Cd     1.0182264e+06f
#define SQRT2       1.4142135f

#define MAX_TORQUE 0.008f

static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_sqrt_f32(in, &pOut); return pOut; }

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void)
{
  motorsInit(motorMapDefaultBrushed);
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)
{
  int32_t Power[4];
  if (control->enabled)
  {
    float F[4];
    F[0] = (control->thrust + 0.0755370f);
    F[1] = (control->roll + 0.0755370f);
    F[2] = (control->pitch + 0.0755370f);
    F[3] = (control->yaw + 0.0755370f);
    
    for (int ii = 0; ii < 4; ii++)
    {
      float motor_pwm = (-PWM_TO_THRUST_b + arm_sqrt(PWM_TO_THRUST_b*PWM_TO_THRUST_b + 4.0f * PWM_TO_THRUST_a * F[ii])) / (2.0f * PWM_TO_THRUST_a);
      Power[ii] = (int32_t)(65535.0f*motor_pwm);
      if (Power[ii] < 7000) Power[ii] = 7000;
    }
    
  }
  else
  {
    Power[0] = 0;
    Power[1] = 0;
    Power[2] = 0;
    Power[3] = 0;
  }
  motorPower.m1 = limitThrust(Power[0]);
  motorPower.m2 = limitThrust(Power[1]);
  motorPower.m3 =  limitThrust(Power[2]);
  motorPower.m4 =  limitThrust(Power[3]);


  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_GROUP_STOP(motor)
