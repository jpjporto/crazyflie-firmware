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
#define invCF_ARM_LENGTH     30.7437725f
#define invTTT               1.6765718e+02f
#define inv2PWM2TA           5.4649181f
#define PWM_TO_THRUST_bSQRD  0.004579716678349f
#define PWM_TO_THRUST_a4     0.365970724f

#define hover_omega 15465.36f
#define d2we        3.2330317e-05f
#define d2we4Ct     2.5592359e+04f
#define d2we4lCt    5.5635563e+05f
#define d2we4Cd     1.0182264e+06f
#define SQRT2       1.4142135f

#define MAX_TORQUE 0.008f

static inline float arm_sqrt(float32_t in)
{ float pOut; arm_sqrt_f32(in, &pOut); return pOut; }

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
  float omega[4];
  int32_t Power[4];
  if (control->enabled)
  {

    float motor_pwm[4];
    float Tpart = constrain(control->thrust, 0.0f, 0.6f);  
    float tauXpart = constrain(control->roll, -MAX_TORQUE, MAX_TORQUE) * invCF_ARM_LENGTH; // / (CRAZYFLIE_ARM_LENGTH * .707106781f);
    float tauYpart = constrain(control->pitch, -MAX_TORQUE, MAX_TORQUE) * invCF_ARM_LENGTH; // / (CRAZYFLIE_ARM_LENGTH * .707106781f);
    float tauZpart = control->yaw * invTTT; // / THRUST_TO_TORQUE_m;

    omega[0] = (Tpart - tauXpart - tauYpart - tauZpart)/4.0f;
    omega[1] = (Tpart - tauXpart + tauYpart + tauZpart)/4.0f;
    omega[2] = (Tpart + tauXpart + tauYpart - tauZpart)/4.0f;
    omega[3] = (Tpart + tauXpart - tauYpart + tauZpart)/4.0f;

    for (int i = 0; i < 4; i++)
    {
      if (omega[i] < 0.0f) 
      {
        //omega[i] = 0.0f;
        motor_pwm[i] = 0.0f;
      } 
      else 
      {
        motor_pwm[i] = (-PWM_TO_THRUST_b + arm_sqrt(PWM_TO_THRUST_b*PWM_TO_THRUST_b + 4.0f * PWM_TO_THRUST_a * omega[i])) * inv2PWM2TA; // / (2.0f * PWM_TO_THRUST_a);
      }
      motor_pwm[i] = constrain(motor_pwm[i], 0.1f, 1.0f);
      Power[i] = (int32_t)(65535*motor_pwm[i]);
    }

  
/*
    float Fz = control->thrust*d2we4Ct;
    float Mx = control->roll*SQRT2*d2we4lCt;
    float My = control->pitch*SQRT2*d2we4lCt;
    float Mz = control->yaw*d2we4Cd;

    omega[0] = (Fz - Mx - My - Mz) + hover_omega; // [RPM]
    omega[1] = (Fz - Mx + My + Mz) + hover_omega;
    omega[2] = (Fz + Mx + My - Mz) + hover_omega;
    omega[3] = (Fz + Mx - My + Mz) + hover_omega;

    for (int ii = 0; ii < 4; ii++)
    {
      if(omega[ii] < hover_omega)
      {
        Power[ii] = 0;
      }
      else
      {
        Power[ii] = (int32_t)((omega[ii] - 4070.3f)*3.7243948f);
      }
    }
    */

/*    float Fz = control->thrust*d2we4Ct;
    float Mx = control->roll*SQRT2*d2we4lCt;
    float My = control->pitch*SQRT2*d2we4lCt;
    float Mz = control->yaw*d2we4Cd;

    omega[0] = (Fz - Mx - My - Mz) + hover_omega; // [RPM]
    omega[1] = (Fz - Mx + My + Mz) + hover_omega;
    omega[2] = (Fz + Mx + My - Mz) + hover_omega;
    omega[3] = (Fz + Mx - My + Mz) + hover_omega;
    
    for (int ii = 0; ii < 4; ii++)
    {
      Power[ii] = (int32_t)(-2.7232165E-09f*omega[ii]*omega[ii]*omega[ii] + 1.7707789E-04f*omega[ii]*omega[ii] - 1.0094497E-01f*omega[ii] + 2.9031373E+02f);
    }*/
    
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
