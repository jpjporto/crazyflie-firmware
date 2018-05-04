
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "controller.h"
#include "controllerMatrices.h"

#include "log.h"
#include "param.h"
#include "math.h"

#include "FreeRTOS.h"
#include "arm_math.h"

#include <stdlib.h>
#include <string.h>

static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_add(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_add_f32(pSrcA, pSrcB, pDst)); }

static bool isInit = false;

#define HINF_STATES 48
static float xHinf[HINF_STATES * CFNUM];
static float e[12 * CFNUM];
//static float xHinfNext[HINF_STATES * CFNUM];
static float u[4];

//static arm_matrix_instance_f32 xHinfm = {HINF_STATES * CFNUM, 1, (float *)xHinf};
//static arm_matrix_instance_f32 em = {12 * CFNUM, 1, (float *)e};
//static arm_matrix_instance_f32 um = {4, 1, (float *)u};
//temp matrices
//static float tmpCx[4];
//static arm_matrix_instance_f32 tmpCxm = {4, 1, tmpCx};
//static float tmpDe[4];
//static arm_matrix_instance_f32 tmpDem = {4, 1, tmpDe};
//static float tmpAx[HINF_STATES * CFNUM];
//static arm_matrix_instance_f32 tmpAxm = {HINF_STATES * CFNUM, 1, tmpAx};
//static float tmpBe[HINF_STATES * CFNUM];
//static arm_matrix_instance_f32 tmpBem = {HINF_STATES * CFNUM, 1, tmpBe};

#if CFNUM >= 1
static arm_matrix_instance_f32 xHinf1m = {HINF_STATES, 1, (float *)xHinf};
static arm_matrix_instance_f32 e1m = {12 * CFNUM, 1, (float *)e};
static float tmpCx1[4];
static arm_matrix_instance_f32 tmpCx1m = {4, 1, tmpCx1};
static float tmpDe1[4];
static arm_matrix_instance_f32 tmpDe1m = {4, 1, tmpDe1};
static float tmpAx11[HINF_STATES];
static arm_matrix_instance_f32 tmpAx11m = {HINF_STATES, 1, tmpAx11};
static float tmpBe11[HINF_STATES];
static arm_matrix_instance_f32 tmpBe11m = {HINF_STATES, 1, tmpBe11};
#endif
#if CFNUM >= 2
static arm_matrix_instance_f32 xHinf2m = {HINF_STATES, 1, (float *)&xHinf[HINF_STATES]};
static arm_matrix_instance_f32 e2m = {12, 1, (float *)&e[HINF_STATES]};
static float tmpCx2[4];
static arm_matrix_instance_f32 tmpCx2m = {4, 1, tmpCx2};
static float tmpDe2[4];
static arm_matrix_instance_f32 tmpDe2m = {4, 1, tmpDe2};
static float tmpAx21[HINF_STATES];
static arm_matrix_instance_f32 tmpAx21m = {HINF_STATES, 1, tmpAx21};
static float tmpBe21[HINF_STATES];
static arm_matrix_instance_f32 tmpBe21m = {HINF_STATES, 1, tmpBe21};
static float tmpAx22[HINF_STATES];
static arm_matrix_instance_f32 tmpAx22m = {HINF_STATES, 1, tmpAx22};
static float tmpBe22[HINF_STATES];
static arm_matrix_instance_f32 tmpBe22m = {HINF_STATES, 1, tmpBe22};
//static float tmpCD1[4];
//static arm_matrix_instance_f32 tmpCD1m = {4, 1, tmpCD1};
//static float tmpCD2[4];
//static arm_matrix_instance_f32 tmpCD2m = {4, 1, tmpCD2};
#endif
#if CFNUM >= 3
static arm_matrix_instance_f32 xHinf3m = {HINF_STATES, 1, (float *)&xHinf[HINF_STATES*2]};
static arm_matrix_instance_f32 e3m = {12, 1, (float *)&e[HINF_STATES*2]};
static float tmpCx3[4];
static arm_matrix_instance_f32 tmpCx3m = {4, 1, tmpCx3};
static float tmpDe3[4];
static arm_matrix_instance_f32 tmpDe3m = {4, 1, tmpDe3};
static float tmpAx31[HINF_STATES];
static arm_matrix_instance_f32 tmpAx31m = {HINF_STATES, 1, tmpAx31};
static float tmpBe31[HINF_STATES];
static arm_matrix_instance_f32 tmpBe31m = {HINF_STATES, 1, tmpBe31};
static float tmpAx32[HINF_STATES];
static arm_matrix_instance_f32 tmpAx32m = {HINF_STATES, 1, tmpAx32};
static float tmpBe32[HINF_STATES];
static arm_matrix_instance_f32 tmpBe32m = {HINF_STATES, 1, tmpBe32};
static float tmpAx33[HINF_STATES];
static arm_matrix_instance_f32 tmpAx33m = {HINF_STATES, 1, tmpAx33};
static float tmpBe33[HINF_STATES];
static arm_matrix_instance_f32 tmpBe33m = {HINF_STATES, 1, tmpBe33};
//static float tmpCD3[4];
//static arm_matrix_instance_f32 tmpCD3m = {4, 1, tmpCD3};
//static float tmpCD4[4];
//static arm_matrix_instance_f32 tmpCD4m = {4, 1, tmpCD4};
#endif
//static arm_matrix_instance_f32 um = {4, 1, (float *)u};



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
  } else if (state->position.z >= 5.0f) {
    control->enabled = 0;
  }
  
  
  if (RATE_DO_EXECUTE(HINF_RATE, tick))// && (control->enabled)) 
  {
#if CFNUM==1
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
#elif CFNUM==2
    e[0] = (setpoint->poscf1.x - state->poscf1.x);
    e[1] = (setpoint->poscf1.y - state->poscf1.y);
    e[2] = (setpoint->poscf1.z - state->poscf1.z);
    e[3] = (setpoint->velcf1.x - state->velcf1.x);
    e[4] = (setpoint->velcf1.y - state->velcf1.y);
    e[5] = (setpoint->velcf1.z - state->velcf1.z);
    e[6] = 0.0f;
    e[7] = 0.0f;
    e[8] = 0.0f; 
    e[9] = 0.0f;
    e[10] = 0.0f;
    e[11] = 0.0f;
    e[12] = (setpoint->poscf2.x + state->poscf1.x - state->position.x); //ref + eps - pos
    e[13] = (setpoint->poscf2.y + state->poscf1.y - state->position.y);
    e[14] = (setpoint->poscf2.z + state->poscf1.z - state->position.z);
    e[15] = (setpoint->velcf2.x + state->velcf1.x - state->velocity.x);
    e[16] = (setpoint->velcf2.y + state->velcf1.y - state->velocity.y);
    e[17] = (setpoint->velcf2.z + state->velcf1.z - state->velocity.z);
    e[18] = (0.0f - state->attitude.roll); // for angles, ref and eps will always equal 0
    e[19] = (0.0f - state->attitude.pitch);
    e[20] = (setpoint->attitude.yaw - state->attitude.yaw); 
    e[21] = (0.0f - state->attitudeRate.roll);
    e[22] = (0.0f - state->attitudeRate.pitch);
    e[23] = (0.0f - state->attitudeRate.yaw);
#elif CFNUM==3
    e[0] = (setpoint->poscf1.x - state->poscf1.x);
    e[1] = (setpoint->poscf1.y - state->poscf1.y);
    e[2] = (setpoint->poscf1.z - state->poscf1.z);
    e[3] = (setpoint->velcf1.x - state->velcf1.x);
    e[4] = (setpoint->velcf1.y - state->velcf1.y);
    e[5] = (setpoint->velcf1.z - state->velcf1.z);
    e[6] = 0.0f;
    e[7] = 0.0f;
    e[8] = 0.0f; 
    e[9] = 0.0f;
    e[10] = 0.0f;
    e[11] = 0.0f;
    e[12] = (setpoint->poscf2.x + state->poscf1.x - state->poscf2.x); //ref + eps - pos
    e[13] = (setpoint->poscf2.y + state->poscf1.y - state->poscf2.y);
    e[14] = (setpoint->poscf2.z + state->poscf1.z - state->poscf2.z);
    e[15] = (setpoint->velcf2.x + state->velcf1.x - state->velcf2.x);
    e[16] = (setpoint->velcf2.y + state->velcf1.y - state->velcf2.y);
    e[17] = (setpoint->velcf2.z + state->velcf1.z - state->velcf2.z);
    e[18] = 0.0f; // for angles, ref and eps will always equal 0
    e[19] = 0.0f;
    e[20] = 0.0f; 
    e[21] = 0.0f;
    e[22] = 0.0f;
    e[23] = 0.0f;
    e[24] = (setpoint->poscf3.x + state->poscf2.x - state->position.x); //ref + eps - pos
    e[25] = (setpoint->poscf3.y + state->poscf2.y - state->position.y);
    e[26] = (setpoint->poscf3.z + state->poscf2.z - state->position.z);
    e[27] = (setpoint->velcf3.x + state->velcf2.x - state->velocity.x);
    e[28] = (setpoint->velcf3.y + state->velcf2.y - state->velocity.y);
    e[29] = (setpoint->velcf3.z + state->velcf2.z - state->velocity.z);
    e[30] = (0.0f - state->attitude.roll); // for angles, ref and eps will always equal 0
    e[31] = (0.0f - state->attitude.pitch);
    e[32] = (setpoint->attitude.yaw - state->attitude.yaw); 
    e[33] = (0.0f - state->attitudeRate.roll);
    e[34] = (0.0f - state->attitudeRate.pitch);
    e[35] = (0.0f - state->attitudeRate.yaw);
#elif CFNUM==4
    e[0] = (setpoint->poscf1.x - state->poscf1.x);
    e[1] = (setpoint->poscf1.y - state->poscf1.y);
    e[2] = (setpoint->poscf1.z - state->poscf1.z);
    e[3] = (setpoint->velcf1.x - state->velcf1.x);
    e[4] = (setpoint->velcf1.y - state->velcf1.y);
    e[5] = (setpoint->velcf1.z - state->velcf1.z);
    e[6] = 0.0f;
    e[7] = 0.0f;
    e[8] = 0.0f; 
    e[9] = 0.0f;
    e[10] = 0.0f;
    e[11] = 0.0f;
    e[12] = (setpoint->poscf2.x + state->poscf1.x - state->poscf2.x); //ref + eps - pos
    e[13] = (setpoint->poscf2.y + state->poscf1.y - state->poscf2.y);
    e[14] = (setpoint->poscf2.z + state->poscf1.z - state->poscf2.z);
    e[15] = (setpoint->velcf2.x + state->velcf1.x - state->velcf2.x);
    e[16] = (setpoint->velcf2.y + state->velcf1.y - state->velcf2.y);
    e[17] = (setpoint->velcf2.z + state->velcf1.z - state->velcf2.z);
    e[18] = 0.0f; // for angles, ref and eps will always equal 0
    e[19] = 0.0f;
    e[20] = 0.0f; 
    e[21] = 0.0f;
    e[22] = 0.0f;
    e[23] = 0.0f;
    e[24] = (setpoint->poscf3.x + state->poscf2.x - state->poscf3.x); //ref + eps - pos
    e[25] = (setpoint->poscf3.y + state->poscf2.y - state->poscf3.y);
    e[26] = (setpoint->poscf3.z + state->poscf2.z - state->poscf3.z);
    e[27] = (setpoint->velcf3.x + state->velcf2.x - state->velcf3.x);
    e[28] = (setpoint->velcf3.y + state->velcf2.y - state->velcf3.y);
    e[29] = (setpoint->velcf3.z + state->velcf2.z - state->velcf3.z);
    e[30] = 0.0f; // for angles, ref and eps will always equal 0
    e[31] = 0.0f;
    e[32] = 0.0f; 
    e[33] = 0.0f;
    e[34] = 0.0f;
    e[35] = 0.0f;
    e[36] = (setpoint->poscf4.x + state->poscf3.x - state->position.x); //ref + eps - pos
    e[37] = (setpoint->poscf4.y + state->poscf3.y - state->position.y);
    e[38] = (setpoint->poscf4.z + state->poscf3.z - state->position.z);
    e[39] = (setpoint->velcf4.x + state->velcf3.x - state->velocity.x);
    e[40] = (setpoint->velcf4.y + state->velcf3.y - state->velocity.y);
    e[41] = (setpoint->velcf4.z + state->velcf3.z - state->velocity.z);
    e[42] = (0.0f - state->attitude.roll); // for angles, ref and eps will always equal 0
    e[43] = (0.0f - state->attitude.pitch);
    e[44] = (setpoint->attitude.yaw - state->attitude.yaw); 
    e[45] = (0.0f - state->attitudeRate.roll);
    e[46] = (0.0f - state->attitudeRate.pitch);
    e[47] = (0.0f - state->attitudeRate.yaw);
#endif

#if CFNUM == 1
    static arm_matrix_instance_f32 CK11m = {4, HINF_STATES, (float *)CK11};
    static arm_matrix_instance_f32 DK11m = {4, 12, (float *)DK11};

    mat_mult(&CK11m, &xHinf1m, &tmpCx1m); // C*xHinf
    mat_mult(&DK11m, &e1m, &tmpDe1m); // D*e
    arm_add_f32(tmpCx1, tmpDe1, u, 4);
//    mat_add(&tmpCx1m, &tmpDe1m, &um);
#elif CFNUM == 2
    static arm_matrix_instance_f32 CK21m = {4, HINF_STATES, (float *)CK21};
    static arm_matrix_instance_f32 DK21m = {4, 12, (float *)DK21};
    static arm_matrix_instance_f32 CK22m = {4, HINF_STATES, (float *)CK22};
    static arm_matrix_instance_f32 DK22m = {4, 12, (float *)DK22};

    mat_mult(&CK21m, &xHinf1m, &tmpCx1m); // C*xHinf
    mat_mult(&CK22m, &xHinf2m, &tmpCx2m);
    mat_mult(&DK21m, &e1m, &tmpDe1m); // D*e
    mat_mult(&DK22m, &e2m, &tmpDe2m);
    arm_add_f32(tmpCx1, tmpDe1, u, 4);
    arm_add_f32(tmpCx2, u, u, 4);
    arm_add_f32(tmpDe2, u, u, 4);
//    mat_add(&tmpCx1m, &tmpDe1m, &tmpCD1m);
//    mat_add(&tmpCx2m, &tmpDe2m, &tmpCD2m);
//    mat_add(&tmpCD1m, &tmpCD2m, &um);
#elif CFNUM == 3
    static arm_matrix_instance_f32 CK31m = {4, HINF_STATES, (float *)CK31};
    static arm_matrix_instance_f32 DK31m = {4, 12, (float *)DK31};
    static arm_matrix_instance_f32 CK32m = {4, HINF_STATES, (float *)CK32};
    static arm_matrix_instance_f32 DK32m = {4, 12, (float *)DK32};
    static arm_matrix_instance_f32 CK33m = {4, HINF_STATES, (float *)CK33};
    static arm_matrix_instance_f32 DK33m = {4, 12, (float *)DK33};

    mat_mult(&CK31m, &xHinf1m, &tmpCx1m); // C*xHinf
    mat_mult(&CK32m, &xHinf2m, &tmpCx2m);
    mat_mult(&CK33m, &xHinf3m, &tmpCx3m);
    mat_mult(&DK31m, &e1m, &tmpDe1m); // D*e
    mat_mult(&DK32m, &e2m, &tmpDe2m);
    mat_mult(&DK33m, &e3m, &tmpDe3m);
    arm_add_f32(tmpCx1, tmpDe1, u, 4);
    arm_add_f32(tmpCx2, u, u, 4);
    arm_add_f32(tmpCx3, u, u, 4);
    arm_add_f32(tmpDe2, u, u, 4);
    arm_add_f32(tmpDe3, u, u, 4);
//    mat_add(&tmpCx1m, &tmpDe1m, &tmpCD1m);
//    mat_add(&tmpCx2m, &tmpDe2m, &tmpCD2m);
//    mat_add(&tmpCx3m, &tmpDe3m, &tmpCD3m);
//    mat_add(&tmpCD1m, &tmpCD2m, &tmpCD4m);
//    mat_add(&tmpCD3m, &tmpCD4m, &um);
#endif

#if CFNUM >= 1
    static arm_matrix_instance_f32 AK11m = {HINF_STATES, HINF_STATES, (float *)AK11};
    static arm_matrix_instance_f32 BK11m = {HINF_STATES, 12, (float *)BK11};
    
    mat_mult(&AK11m, &xHinf1m, &tmpAx11m); // A*xHinf
    mat_mult(&BK11m, &e1m, &tmpBe11m); // B*e
#endif
#if CFNUM >= 2
    static arm_matrix_instance_f32 AK21m = {HINF_STATES, HINF_STATES, (float *)AK21};
    static arm_matrix_instance_f32 BK21m = {HINF_STATES, 12, (float *)BK21};
    static arm_matrix_instance_f32 AK22m = {HINF_STATES, HINF_STATES, (float *)AK22};
    static arm_matrix_instance_f32 BK22m = {HINF_STATES, 12, (float *)BK22};
    
    mat_mult(&AK21m, &xHinf1m, &tmpAx21m); // A*xHinf
    mat_mult(&AK22m, &xHinf2m, &tmpAx22m);
    mat_mult(&BK21m, &e1m, &tmpBe21m); // B*e
    mat_mult(&BK22m, &e2m, &tmpBe22m);
#endif
#if CFNUM >= 3
    static arm_matrix_instance_f32 AK31m = {HINF_STATES, HINF_STATES, (float *)AK31};
    static arm_matrix_instance_f32 BK31m = {HINF_STATES, 12, (float *)BK31};
    static arm_matrix_instance_f32 AK32m = {HINF_STATES, HINF_STATES, (float *)AK32};
    static arm_matrix_instance_f32 BK32m = {HINF_STATES, 12, (float *)BK32};
    static arm_matrix_instance_f32 AK33m = {HINF_STATES, HINF_STATES, (float *)AK33};
    static arm_matrix_instance_f32 BK33m = {HINF_STATES, 12, (float *)BK33};
    
    mat_mult(&AK31m, &xHinf1m, &tmpAx31m); // A*xHinf
    mat_mult(&AK32m, &xHinf2m, &tmpAx32m);
    mat_mult(&AK33m, &xHinf3m, &tmpAx33m);
    mat_mult(&BK31m, &e1m, &tmpBe31m); // B*e
    mat_mult(&BK32m, &e2m, &tmpBe32m); 
    mat_mult(&BK33m, &e3m, &tmpBe33m);
#endif

#if CFNUM>=1
    arm_add_f32(tmpAx11, tmpBe11, xHinf, HINF_STATES);
//    mat_add(&tmpAx11m, &tmpBe11m, &xHinf1m);
#endif
#if CFNUM>=2
    arm_add_f32(tmpAx21, tmpAx22, &xHinf[HINF_STATES], HINF_STATES);
    arm_add_f32(tmpBe21, &xHinf[HINF_STATES], &xHinf[HINF_STATES], HINF_STATES);
    arm_add_f32(tmpBe22, &xHinf[HINF_STATES], &xHinf[HINF_STATES], HINF_STATES);
//    mat_add(&tmpAx21m, &tmpAx22m, &xHinf2m);
//    mat_add(&tmpBe21m, &xHinf2m, &xHinf2m);
//    mat_add(&tmpBe22m, &xHinf2m, &xHinf2m);
#endif
#if CFNUM>=3
    arm_add_f32(tmpAx31, tmpAx32, &xHinf[HINF_STATES*2], HINF_STATES);
    arm_add_f32(tmpAx33, &xHinf[HINF_STATES*2], &xHinf[HINF_STATES*2], HINF_STATES);
    arm_add_f32(tmpBe31, &xHinf[HINF_STATES*2], &xHinf[HINF_STATES*2], HINF_STATES);
    arm_add_f32(tmpBe32, &xHinf[HINF_STATES*2], &xHinf[HINF_STATES*2], HINF_STATES);
    arm_add_f32(tmpBe33, &xHinf[HINF_STATES*2], &xHinf[HINF_STATES*2], HINF_STATES);
//    mat_add(&tmpAx31m, &tmpAx32m, &xHinf3m);
//    mat_add(&tmpAx33m, &xHinf3m, &xHinf3m);
//    mat_add(&tmpBe31m, &xHinf3m, &xHinf3m);
//    mat_add(&tmpBe32m, &xHinf3m, &xHinf3m);
//    mat_add(&tmpBe33m, &xHinf3m, &xHinf3m);
#endif

    
    //memcpy(xHinf, xHinfNext, sizeof(xHinf));
    
    #if CFNUM==1
    control->thrust = u[0] + 0.33f;
    #elif CFNUM==2
    control->thrust = u[0] + 0.3345f;
    #elif CFNUM==3
    control->thrust = u[0] + 0.3316f;
    #elif CFNUM==4
    control->thrust = u[0] + 0.3316f;
    #endif
    control->roll = u[1];
    control->pitch = u[2];
    control->yaw = u[3];
    control->enabled = 0;
  }

}
