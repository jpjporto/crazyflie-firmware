
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "controller.h"

#include "log.h"
#include "param.h"
#include "math.h"

#include <stdlib.h>
#include <string.h>

static bool isInit = false;

static float e[12];

#ifdef CTRL_DEBUG
static uint8_t ctrTest = 0;
#endif

void lqrControllerInit(void)
{
  if(isInit)
  {
    return;
  }
  
  memset(e, 0.0f, sizeof(e));
  
  isInit= true;
}

void lqrController(control_t *control, setpoint_t *setpoint,
                                         const state_t *state,
                                         const uint32_t tick)
{
  float u[4];
  
  if(setpoint->position.z > 0.0f)
  {
    control->enabled = 1;
  }
  else
  {
    control->enabled = 0;
  }
  
  if ( fabsf(state->attitude.pitch) > 1.1f || fabsf(state->attitude.roll) > 1.1f)
  {
    control->enabled = 0;
  }
  else if (state->position.z >= 1.5f)
  {
    control->enabled = 0;
  }
  
#ifdef CTRL_DEBUG
  if(ctrTest != 0)
  {
    control->enabled = 1;
  }
  else
  {
    control->enabled = 0;
  }
#endif
  
  if (RATE_DO_EXECUTE(HINF_RATE, tick) && (control->enabled)) 
  {
    e[0] = (setpoint->position.x - state->position.x);
    e[1] = (setpoint->position.y - state->position.y);
    e[2] = (setpoint->position.z - state->position.z);
    e[3] = (0.0f - state->velocity.x);
    e[4] = (0.0f - state->velocity.y);
    e[5] = (0.0f - state->velocity.z);
    e[6] = (0.0f - state->attitude.roll);
    e[7] = (0.0f - state->attitude.pitch);
    e[8] = (setpoint->attitude.yaw - state->attitude.yaw); 
    e[9] = (0.0f - state->attitudeRate.roll);
    e[10] = (0.0f - state->attitudeRate.pitch);
    e[11] = (0.0f - state->attitudeRate.yaw);
    
    u[0] = +3.1479776e-01f*e[2]+1.3927911e-01f*e[5];
    u[1] = -3.7231175e-03f*e[1]-5.2886191e-03f*e[4]+3.5034373e-02f*e[6]+1.1844500e-02f*e[9];
    u[2] = +3.7231175e-03f*e[0]+5.2886191e-03f*e[3]+3.5034373e-02f*e[7]+1.1844500e-02f*e[10];
    u[3] = +3.1087829e-03f*e[8]+5.4567843e-04f*e[11];


/*    e[0] = e[0] + 0.002f*(setpoint->position.x - state->position.x);
    e[1] = e[1] + 0.002f*(setpoint->position.y - state->position.y);
    e[2] = e[2] + 0.002f*(setpoint->position.z - state->position.z);
    e[3] = (setpoint->position.x - state->position.x);
    e[4] = (setpoint->position.y - state->position.y);
    e[5] = (setpoint->position.z - state->position.z);
    e[6] = (0.0f - state->velocity.x);
    e[7] = (0.0f - state->velocity.y);
    e[8] = (0.0f - state->velocity.z);
    e[9] = e[9] + 0.002f*(0.0f - state->attitude.roll);
    e[10] = e[10] + 0.002f*(0.0f - state->attitude.pitch);
    e[11] = e[11] + 0.002f*(setpoint->attitude.yaw - state->attitude.yaw);
    e[12] = (0.0f - state->attitude.roll);
    e[13] = (0.0f - state->attitude.pitch);
    e[14] = (setpoint->attitude.yaw - state->attitude.yaw); 
    e[15] = (0.0f - state->attitudeRate.roll);
    e[16] = (0.0f - state->attitudeRate.pitch);
    e[17] = (0.0f - state->attitudeRate.yaw);
    
    u[0] = +1.6949817e-10f*e[0]+3.1479462e-03f*e[2]+2.1823539e-09f*e[3]+9.6918973e-10f*e[4]+3.1618750e-01f*e[5]+5.3797839e-06f*e[6]+3.0526687e-06f*e[7]+1.3958539e-01f*e[8]+2.9946910e-05f*e[9]-5.2771047e-05f*e[10]+1.3613549e-10f*e[12]+2.8142288e-09f*e[13];
    u[1] = -3.7201948e-03f*e[1]-8.7627508e-03f*e[4]+4.9001116e-08f*e[6]-8.4590660e-03f*e[7]-4.8871007e-06f*e[9]-4.8053647e-07f*e[10]+4.4308931e-02f*e[12]+1.1854014e-02f*e[15];
    u[2] = +3.7201948e-03f*e[0]+8.7627508e-03f*e[3]+1.2869066e-10f*e[4]+1.3162252e-10f*e[5]+8.4585445e-03f*e[6]+4.2787710e-07f*e[7]+1.0214420e-10f*e[8]+4.1976077e-06f*e[9]+2.3006376e-07f*e[10]+4.4308931e-02f*e[13]+1.1854014e-02f*e[16];
    u[3] = +1.1121946e-10f*e[4]-1.0423726e-08f*e[6]+3.4175372e-07f*e[7]+3.3526012e-06f*e[9]+1.0226533e-07f*e[10]+3.1084739e-04f*e[11]+3.1628818e-03f*e[14]+5.4884021e-04f*e[17]; */

/*    e[0] = 0;
    e[1] = e[10] + 0.002f*(0.0f - state->attitude.pitch);
    e[2] = 0;
    e[3] = 0;
    e[4] = (0.0f - state->attitude.pitch);
    e[5] = 0; 
    e[6] = 0;
    e[7] = (0.0f - state->attitudeRate.pitch);
    e[8] = 0;

    u[0] = 0.0f;
    u[1] = +1.7809579e-03f*e[0]+1.8369358e-02f*e[3]+5.6857648e-03f*e[6];
    u[2] = +1.7809579e-03f*e[1]+1.8369358e-02f*e[4]+5.6857648e-03f*e[7];
    u[3] = +2.2547771e-03f*e[2]+2.3259068e-02f*e[5]+7.2251684e-03f*e[8];*/
    
    control->thrust = u[0] + 0.29f;
    control->roll = u[1];
    control->pitch = u[2];
    control->yaw = u[3];
    control->enabled = 1;
  }

}

#ifdef CTRL_DEBUG
PARAM_GROUP_START(lqr)
  PARAM_ADD(PARAM_UINT8, ctrTest, &ctrTest)
PARAM_GROUP_STOP(lqr)
#endif