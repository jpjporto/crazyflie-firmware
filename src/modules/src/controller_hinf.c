
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "controller.h"

#include "log.h"
#include "param.h"
#include "math.h"

#include <stdlib.h>
#include <string.h>

static bool isInit = false;

#define HINF_STATES 16
static float xHinf[HINF_STATES];
static float e[12];

#ifdef CTRL_DEBUG
static uint8_t ctrTest = 0;
#endif

#define DEG_TO_RAD 0.0174533f

void hinfControllerInit(void)
{
  if(isInit)
  {
    return;
  }
  
  memset(xHinf, 0.0f, sizeof(xHinf));
  memset(e, 0.0f, sizeof(e));
  
  isInit= true;
}

void hinfController(control_t *control, setpoint_t *setpoint,
                                         const state_t *state,
                                         const uint32_t tick)
{
  float u[4];
  float xHinfNext[HINF_STATES];
  
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
/*    e[0] = 0;
    e[1] = (0.0f - state->attitude.pitch);
    e[2] = 0; 
    e[3] = 0;
    e[4] = (0.0f - state->attitudeRate.pitch);
    e[5] = 0; */

    u[0] = +7.5768394e-04f*xHinf[6]+2.1943735e-02f*xHinf[7]+1.9211540e-01f*xHinf[15]+5.3596216e-01f*e[2]+1.2740450e-01f*e[5];
    u[1] = +2.2578548e-07f*xHinf[1]-1.1558200e-03f*xHinf[2]+1.4502642e-04f*xHinf[4]-5.1852481e-07f*xHinf[5]+2.6618673e-07f*xHinf[9]-1.7929920e-03f*xHinf[10]-7.8582651e-09f*xHinf[11]-2.4210294e-03f*xHinf[12]+4.4969652e-06f*xHinf[13]-3.0783778e-03f*xHinf[14]-1.1304592e-04f*e[1]-4.3651275e-03f*e[4]+6.5390490e-02f*e[6]+1.3359322e-06f*e[9];
    u[2] = +1.1558200e-03f*xHinf[1]+2.2578548e-07f*xHinf[2]+5.1852481e-07f*xHinf[4]+1.4502642e-04f*xHinf[5]-1.7929920e-03f*xHinf[9]-2.6618673e-07f*xHinf[10]+2.4210294e-03f*xHinf[11]-7.8582651e-09f*xHinf[12]-3.0783778e-03f*xHinf[13]-4.4969652e-06f*xHinf[14]+1.1304592e-04f*e[0]+4.3651275e-03f*e[3]+6.5390490e-02f*e[7]+1.3359322e-06f*e[10];
    u[3] = +1.3459299e-04f*xHinf[0]-1.5343287e-03f*xHinf[3]+2.3703255e-02f*xHinf[8]+1.0906060e-03f*e[8]+2.3653166e-04f*e[11];


    xHinfNext[0] = +8.8100797e-01f*xHinf[0]+1.9945113e-01f*xHinf[3]-2.9148951e+00f*xHinf[8]-1.0962050e-01f*e[8]+1.7683990e-01f*e[11];
    xHinfNext[1] = +8.2671744e-01f*xHinf[1]-8.9959591e-05f*xHinf[4]-2.3857336e-02f*xHinf[5]+2.0941813e-01f*xHinf[9]+7.1999231e-05f*xHinf[10]-2.0690498e-01f*xHinf[11]+4.1089763e-05f*xHinf[12]+5.2499056e-01f*xHinf[13]+8.6947368e-04f*xHinf[14]-1.8891701e-02f*e[0]+3.6904294e-06f*e[1]-7.2700572e-01f*e[3]+1.4201809e-04f*e[4]-1.7717138e-03f*e[6]-9.0695915e+00f*e[7]+3.9063838e-07f*e[9]+1.9997195e-03f*e[10];
    xHinfNext[2] = +8.2671744e-01f*xHinf[2]+2.3857336e-02f*xHinf[4]-8.9959591e-05f*xHinf[5]+7.1999231e-05f*xHinf[9]-2.0941813e-01f*xHinf[10]-4.1089763e-05f*xHinf[11]-2.0690498e-01f*xHinf[12]+8.6947368e-04f*xHinf[13]-5.2499056e-01f*xHinf[14]-3.6904294e-06f*e[0]-1.8891701e-02f*e[1]-1.4201809e-04f*e[3]-7.2700572e-01f*e[4]+9.0695915e+00f*e[6]-1.7717138e-03f*e[7]-1.9997195e-03f*e[9]+3.9063838e-07f*e[10];
    xHinfNext[3] = +1.2656644e-02f*xHinf[0]+9.7765619e-01f*xHinf[3]+3.1236729e-01f*xHinf[8]+2.5608671e-01f*e[8]-1.9899443e-02f*e[11];
    xHinfNext[4] = +4.5443453e-08f*xHinf[1]-1.2051631e-05f*xHinf[2]+9.9879009e-01f*xHinf[4]-1.8872766e-05f*xHinf[9]-5.5072093e-03f*xHinf[10]-1.7011498e-06f*xHinf[11]+4.7622769e-04f*xHinf[12]-6.9500783e-07f*xHinf[13]-3.2867922e-04f*xHinf[14]-4.4593602e-04f*e[0]+1.2472404e-01f*e[1]-9.0123802e-05f*e[3]+2.5206765e-02f*e[4]-2.1552604e-03f*e[6]-7.7058776e-06f*e[7]-1.0232359e-06f*e[9]-3.6584586e-09f*e[10];
    xHinfNext[5] = +1.2051631e-05f*xHinf[1]+4.5443453e-08f*xHinf[2]+9.9879009e-01f*xHinf[5]-5.5072093e-03f*xHinf[9]+1.8872766e-05f*xHinf[10]-4.7622769e-04f*xHinf[11]-1.7011498e-06f*xHinf[12]-3.2867922e-04f*xHinf[13]+6.9500783e-07f*xHinf[14]-1.2472404e-01f*e[0]-4.4593602e-04f*e[1]-2.5206765e-02f*e[3]-9.0123802e-05f*e[4]+7.7058776e-06f*e[6]-2.1552604e-03f*e[7]+3.6584586e-09f*e[9]-1.0232359e-06f*e[10];
    xHinfNext[6] = +9.9605364e-01f*xHinf[6]-6.9153304e-03f*xHinf[7]-4.0315110e-03f*xHinf[15]+2.3713642e-01f*e[2]+2.1684339e-02f*e[5];
    xHinfNext[7] = -3.8690541e-02f*xHinf[6]+8.6185217e-01f*xHinf[7]-4.4324655e-02f*xHinf[15]-1.9926026e-01f*e[2]+4.6293929e-01f*e[5];
    xHinfNext[8] = -5.5323914e-03f*xHinf[0]+8.9751584e-03f*xHinf[3]+8.6460704e-01f*xHinf[8]+1.0127493e-01f*e[8]+1.5285577e-02f*e[11];
    xHinfNext[9] = +1.9590059e-02f*xHinf[1]+6.7351825e-06f*xHinf[2]-8.8208835e-05f*xHinf[4]-2.5739973e-02f*xHinf[5]+7.3596209e-01f*xHinf[9]+1.7953065e-01f*xHinf[11]+2.6070307e-05f*xHinf[12]+8.6098900e-03f*xHinf[13]+1.1299302e-05f*xHinf[14]+2.0450175e-02f*e[0]+3.0360231e-06f*e[1]-8.2288516e-01f*e[3]-1.2216513e-04f*e[4]-2.4455428e-04f*e[6]+1.6472791e+00f*e[7]-5.2547789e-06f*e[10];
    xHinfNext[10] = +6.7351825e-06f*xHinf[1]-1.9590059e-02f*xHinf[2]-2.5739973e-02f*xHinf[4]+8.8208835e-05f*xHinf[5]+7.3596209e-01f*xHinf[10]+2.6070307e-05f*xHinf[11]-1.7953065e-01f*xHinf[12]-1.1299302e-05f*xHinf[13]+8.6098900e-03f*xHinf[14]+3.0360231e-06f*e[0]-2.0450175e-02f*e[1]-1.2216513e-04f*e[3]+8.2288516e-01f*e[4]+1.6472791e+00f*e[6]+2.4455428e-04f*e[7]-5.2547789e-06f*e[9];
    xHinfNext[11] = -1.8429762e-01f*xHinf[1]-3.6600111e-05f*xHinf[2]-2.9478275e-05f*xHinf[4]-8.2522836e-03f*xHinf[5]+7.6157272e-01f*xHinf[9]+1.1059078e-04f*xHinf[10]-8.1133002e-01f*xHinf[11]-1.5746173e-01f*xHinf[13]-2.2951262e-04f*xHinf[14]+1.6029345e-02f*e[0]+5.2028629e-08f*e[1]-1.7284629e-01f*e[3]-5.6103079e-07f*e[4]+5.2695934e-05f*e[6]-1.6234932e+01f*e[7]-2.1316746e-04f*e[10];
    xHinfNext[12] = +3.6600111e-05f*xHinf[1]-1.8429762e-01f*xHinf[2]+8.2522836e-03f*xHinf[4]-2.9478275e-05f*xHinf[5]+1.1059078e-04f*xHinf[9]-7.6157272e-01f*xHinf[10]-8.1133002e-01f*xHinf[12]-2.2951262e-04f*xHinf[13]+1.5746173e-01f*xHinf[14]-5.2028629e-08f*e[0]+1.6029345e-02f*e[1]+5.6103079e-07f*e[3]-1.7284629e-01f*e[4]+1.6234932e+01f*e[6]+5.2695934e-05f*e[7]+2.1316746e-04f*e[9];
    xHinfNext[13] = -1.5000764e-01f*xHinf[1]-2.4843815e-04f*xHinf[2]-1.4914261e-04f*xHinf[4]-7.0531689e-02f*xHinf[5]+1.8172401e-01f*xHinf[9]-2.3848789e-04f*xHinf[10]-1.4970435e+00f*xHinf[11]-2.1820564e-03f*xHinf[12]+8.5601032e-01f*xHinf[13]-7.8765184e-02f*e[0]-1.1506199e-04f*e[1]-2.2182922e+00f*e[3]-3.2405322e-03f*e[4]+1.9526232e-02f*e[6]-1.3366597e+01f*e[7]+2.7652388e-07f*e[9]-1.8929322e-04f*e[10];
    xHinfNext[14] = -2.4843815e-04f*xHinf[1]+1.5000764e-01f*xHinf[2]-7.0531689e-02f*xHinf[4]+1.4914261e-04f*xHinf[5]+2.3848789e-04f*xHinf[9]+1.8172401e-01f*xHinf[10]-2.1820564e-03f*xHinf[11]+1.4970435e+00f*xHinf[12]+8.5601032e-01f*xHinf[14]-1.1506199e-04f*e[0]+7.8765184e-02f*e[1]-3.2405322e-03f*e[3]+2.2182922e+00f*e[4]-1.3366597e+01f*e[6]-1.9526232e-02f*e[7]-1.8929322e-04f*e[9]-2.7652388e-07f*e[10];
    xHinfNext[15] = +1.5221736e-03f*xHinf[6]-6.8158493e-03f*xHinf[7]-6.3492340e-01f*xHinf[15]+1.0203387e+00f*e[2]+2.2930031e-01f*e[5];

    
    memcpy(xHinf, xHinfNext, sizeof(xHinf));
/*    for(int i = 0; i<HINF_STATES; i++)
    {
      xHinf[i] = xHinfNext[i];
    }*/
    
    control->thrust = u[0] + 0.29f;
    control->roll = u[1];
    control->pitch = u[2];
    control->yaw = u[3];
    control->enabled = 1;
  }

}

#ifdef CTRL_DEBUG
PARAM_GROUP_START(hinf)
  PARAM_ADD(PARAM_UINT8, ctrTest, &ctrTest)
PARAM_GROUP_STOP(hinf)
#endif