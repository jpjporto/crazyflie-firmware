
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
float u[4];
float xHinfNext[HINF_STATES];

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

void hinfController(control_t *control, setpoint_t *setpoint, const state_t *state, const uint32_t tick)
{

  if(setpoint->poscf1.z > 0.0f) {
    control->enabled = 1;
  } else {
    control->enabled = 0;
  }
  
  if ( fabsf(state->attitude.pitch) > 1.1f || fabsf(state->attitude.roll) > 1.1f) {
    control->enabled = 0;
  } else if (state->position.z >= 2.5f) {
    control->enabled = 0;
  }
  
  
  if (RATE_DO_EXECUTE(HINF_RATE, tick) && (control->enabled)) 
  {
    e[0] = (setpoint->poscf1.x - state->position.x);
    e[1] = (setpoint->poscf1.y - state->position.y);
    e[2] = (setpoint->poscf1.z - state->position.z);
    e[3] = (setpoint->velcf1.x - state->velocity.x);
    e[4] = (setpoint->velcf1.y - state->velocity.y);
    e[5] = (setpoint->velcf1.z - state->velocity.z);
    e[6] = (0.0f - state->attitude.roll);
    e[7] = (0.0f - state->attitude.pitch);
    e[8] = (setpoint->attitude.yaw - state->attitude.yaw); 
    e[9] = (0.0f - state->attitudeRate.roll);
    e[10] = (0.0f - state->attitudeRate.pitch);
    e[11] = (0.0f - state->attitudeRate.yaw);

    u[0] = -2.0636132e-01f*xHinf[6]-1.2786106e+00f*xHinf[10]-1.3030629e-01f*xHinf[15]+2.4666642e-01f*e[2]+4.0848694e+00f*e[5];
    u[1] = -2.3245355e-04f*xHinf[2]-8.8148443e-07f*xHinf[4]-1.0615231e-03f*xHinf[5]+8.3435381e-08f*xHinf[8]-1.8976266e-03f*xHinf[9]+1.5828530e-08f*xHinf[11]+1.7795403e-03f*xHinf[12]-3.2220782e-08f*xHinf[13]+1.3405423e-03f*xHinf[14]-1.0121283e-05f*e[1]-1.1210024e-02f*e[4]+5.5704650e-02f*e[6]+4.7441574e-07f*e[9];
    u[2] = +2.3245355e-04f*xHinf[1]+1.0615231e-03f*xHinf[4]-8.8148443e-07f*xHinf[5]+1.8976266e-03f*xHinf[8]+8.3435381e-08f*xHinf[9]+1.7795403e-03f*xHinf[11]-1.5828530e-08f*xHinf[12]+1.3405423e-03f*xHinf[13]+3.2220782e-08f*xHinf[14]+1.0121283e-05f*e[0]+1.1210024e-02f*e[3]+5.5704650e-02f*e[7]+4.7441574e-07f*e[10];
    u[3] = -2.1231767e-04f*xHinf[0]+3.2204343e-04f*xHinf[3]+1.2907463e-02f*xHinf[7]+1.0400592e-04f*e[8]+1.1335908e-04f*e[11];


    xHinfNext[0] = +9.9727291e-01f*xHinf[0]-4.5061052e-02f*xHinf[3]-5.2535754e-01f*xHinf[7]+5.2606765e-02f*e[8]+6.2332112e-02f*e[11];
    xHinfNext[1] = +9.9790025e-01f*xHinf[1]-6.5532775e-05f*xHinf[4]+5.4418148e-08f*xHinf[5]-2.8205153e-02f*xHinf[8]-1.2401322e-06f*xHinf[9]+3.2145679e-03f*xHinf[11]-2.8592719e-08f*xHinf[12]+3.4210933e-03f*xHinf[13]+8.2228141e-08f*xHinf[14]-3.1089820e-02f*e[0]-9.9875703e-02f*e[3]-4.3449793e-03f*e[7]+1.6373743e-07f*e[10];
    xHinfNext[2] = +9.9790025e-01f*xHinf[2]-5.4418130e-08f*xHinf[4]-6.5532775e-05f*xHinf[5]+1.2401322e-06f*xHinf[8]-2.8205153e-02f*xHinf[9]-2.8592710e-08f*xHinf[11]-3.2145679e-03f*xHinf[12]+8.2228169e-08f*xHinf[13]-3.4210933e-03f*xHinf[14]-3.1089820e-02f*e[1]-9.9875703e-02f*e[4]+4.3449793e-03f*e[6]-1.6373743e-07f*e[9];
    xHinfNext[3] = -7.2277435e-03f*xHinf[0]+8.7030053e-01f*xHinf[3]-1.5097607e+00f*xHinf[7]-3.2547925e-02f*e[8]+1.8213929e-01f*e[11];
    xHinfNext[4] = -3.8553476e-02f*xHinf[1]-3.2014646e-05f*xHinf[2]+8.4587151e-01f*xHinf[4]-3.6343312e-01f*xHinf[8]-3.1777291e-04f*xHinf[9]-8.5311316e-02f*xHinf[11]+7.1600989e-05f*xHinf[12]-3.1263280e-01f*xHinf[13]+2.5209467e-04f*xHinf[14]-1.6772976e-03f*e[0]-1.3928211e-06f*e[1]-1.8596734e+00f*e[3]-1.5442651e-03f*e[4]+6.0052583e-03f*e[6]-7.2318015e+00f*e[7]-6.2234494e-07f*e[9]+7.4945571e-04f*e[10];
    xHinfNext[5] = +3.2014646e-05f*xHinf[1]-3.8553476e-02f*xHinf[2]+8.4587151e-01f*xHinf[5]+3.1777291e-04f*xHinf[8]-3.6343312e-01f*xHinf[9]+7.1600989e-05f*xHinf[11]+8.5311316e-02f*xHinf[12]+2.5209467e-04f*xHinf[13]+3.1263280e-01f*xHinf[14]+1.3928211e-06f*e[0]-1.6772976e-03f*e[1]+1.5442651e-03f*e[3]-1.8596734e+00f*e[4]+7.2318015e+00f*e[6]+6.0052583e-03f*e[7]-7.4945571e-04f*e[9]-6.2234494e-07f*e[10];
    xHinfNext[6] = +1.0064979e+00f*xHinf[6]+4.0603984e-02f*xHinf[10]+1.6273124e-03f*xHinf[15]+1.2084093e-01f*e[2]-1.2731768e-01f*e[5];
    xHinfNext[7] = -2.9331536e-04f*xHinf[0]-5.9018736e-03f*xHinf[3]+9.3100417e-01f*xHinf[7]+1.7005915e-02f*e[8]+1.2499838e-02f*e[11];
    xHinfNext[8] = -6.2199108e-02f*xHinf[1]+2.7347880e-06f*xHinf[2]+4.5410702e-03f*xHinf[4]-3.9705487e-06f*xHinf[5]+1.4465265e-01f*xHinf[8]+1.4126639e-01f*xHinf[11]+4.9547139e-06f*xHinf[12]+8.7578952e-02f*xHinf[13]+5.9557115e-06f*xHinf[14]+3.9264825e-03f*e[0]-1.7264070e-07f*e[1]-2.9947639e+00f*e[3]+1.3167462e-04f*e[4]+1.5981404e-05f*e[6]+3.6347574e-01f*e[7]-1.5631053e-06f*e[10];
    xHinfNext[9] = -2.7347880e-06f*xHinf[1]-6.2199108e-02f*xHinf[2]+3.9705487e-06f*xHinf[4]+4.5410702e-03f*xHinf[5]+1.4465265e-01f*xHinf[9]+4.9547139e-06f*xHinf[11]-1.4126639e-01f*xHinf[12]+5.9557115e-06f*xHinf[13]-8.7578952e-02f*xHinf[14]+1.7264070e-07f*e[0]+3.9264825e-03f*e[1]-1.3167462e-04f*e[3]-2.9947639e+00f*e[4]-3.6347574e-01f*e[6]+1.5981404e-05f*e[7]+1.5631053e-06f*e[9];
    xHinfNext[10] = -1.4354648e-01f*xHinf[6]+1.0879292e-01f*xHinf[10]+2.3701459e-02f*xHinf[15]-6.5868877e-02f*e[2]+2.7222288e+00f*e[5];
    xHinfNext[11] = +5.4374713e-02f*xHinf[1]-4.8364848e-07f*xHinf[2]-1.4204738e-01f*xHinf[4]+1.1921903e-04f*xHinf[5]+1.0405655e+00f*xHinf[8]+3.6496327e-05f*xHinf[9]-4.0442935e-01f*xHinf[11]+4.9031076e-01f*xHinf[13]+1.6146110e-05f*xHinf[14]+4.7285468e-03f*e[0]-4.2059145e-08f*e[1]+2.6317675e+00f*e[3]-2.3408860e-05f*e[4]-1.0998864e-04f*e[6]-1.2365596e+01f*e[7]-6.3316867e-05f*e[10];
    xHinfNext[12] = -4.8364848e-07f*xHinf[1]-5.4374713e-02f*xHinf[2]+1.1921903e-04f*xHinf[4]+1.4204738e-01f*xHinf[5]+3.6496327e-05f*xHinf[8]-1.0405655e+00f*xHinf[9]-4.0442935e-01f*xHinf[12]-1.6146110e-05f*xHinf[13]+4.9031076e-01f*xHinf[14]-4.2059145e-08f*e[0]-4.7285468e-03f*e[1]-2.3408860e-05f*e[3]-2.6317675e+00f*e[4]-1.2365596e+01f*e[6]+1.0998864e-04f*e[7]-6.3316867e-05f*e[9];
    xHinfNext[13] = +1.5517156e-01f*xHinf[1]+3.7296461e-06f*xHinf[2]+1.6833226e-01f*xHinf[4]-1.3573644e-04f*xHinf[5]+1.7532723e+00f*xHinf[8]+1.1922938e-04f*xHinf[9]+1.3098993e+00f*xHinf[11]-4.3135460e-05f*xHinf[12]+4.7370419e-02f*xHinf[13]+8.6614592e-03f*e[0]+2.0818366e-07f*e[1]+7.4928608e+00f*e[3]+1.8009565e-04f*e[4]-3.5449033e-04f*e[6]+1.4748533e+01f*e[7]-1.9905768e-09f*e[9]+8.2817736e-05f*e[10];
    xHinfNext[14] = +3.7296461e-06f*xHinf[1]-1.5517156e-01f*xHinf[2]-1.3573644e-04f*xHinf[4]-1.6833226e-01f*xHinf[5]+1.1922938e-04f*xHinf[8]-1.7532723e+00f*xHinf[9]+4.3135460e-05f*xHinf[11]+1.3098993e+00f*xHinf[12]+4.7370419e-02f*xHinf[14]+2.0818366e-07f*e[0]-8.6614592e-03f*e[1]+1.8009565e-04f*e[3]-7.4928608e+00f*e[4]+1.4748533e+01f*e[6]+3.5449033e-04f*e[7]+8.2817736e-05f*e[9]+1.9905768e-09f*e[10];
    xHinfNext[15] = +2.3376054e-01f*xHinf[6]+1.4703484e+00f*xHinf[10]-8.5115319e-01f*xHinf[15]-2.8199422e-01f*e[2]-4.6272826e+00f*e[5];

    
    memcpy(xHinf, xHinfNext, sizeof(xHinf));
    
    control->thrust = u[0] + 0.31f;
    control->roll = u[1];
    control->pitch = u[2];
    control->yaw = u[3];
    control->enabled = 1;
  }

}
