
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "controller.h"
#include "controllerMatricesSparseH1.h"

#include "log.h"
#include "param.h"
#include "math.h"

#include "FreeRTOS.h"
#include "arm_math.h"

#include <stdlib.h>
#include <string.h>

static bool isInit = false;

#define HINF_STATES 64
static float xHinf[HINF_STATES * CFNUM];
static float e[12 * CFNUM];
static float u[4];

float tmpCx[4], tmpDe[4], tmpAx[HINF_STATES * CFNUM], tmpBe[HINF_STATES * CFNUM];

const float *AK, *BK, *CK, *DK;
const uint16_t * AKrows, *BKrows, *CKrows, *DKrows, *AKcols, *BKcols, *CKcols, *DKcols;

static uint8_t seq_num, prev_mode, current_mode;


#define DEG_TO_RAD 0.0174533f

#pragma GCC push_options
#pragma GCC optimize ("O3")
void sparse_mult(const float * S, const uint16_t * Srows, const uint16_t * Scols, const uint32_t vec_size, float * x, float * y)
{
    // Here vec_size is the size of the output vector
    // Srows = num of elements on each row (num of nonzero cols)
    // We are computing y = Sx, assuming x is a vector with same size as Srows
    // Use ideas from arm_mat_mult algorithm
    float in1, in2, in3, in4;
    float sum;
    uint16_t colCnt, numColsA;
    uint16_t row = vec_size;
    do
    {
        sum = 0.0f;  // Set sum accumulator to zero
        numColsA = *Srows++;  // Number of nonzero variables on current row
        colCnt = numColsA >> 2U; // Loop unrolling to compute 4 Multiple-Accumulate operations (MACs) simultaneously
        
        while(colCnt > 0u)
        {
            in3 = x[*Scols++];
            in1 = *S++;
            in2 = *S++;
            sum += in1 * in3;
            in4 = x[*Scols++];
            sum += in2 * in4;
            
            in3 = x[*Scols++];
            in1 = *S++;
            in2 = *S++;
            sum += in1 * in3;
            in4 = x[*Scols++];
            sum += in2 * in4;
            
            colCnt--; // Decrement the loop count
        }
        
        colCnt = numColsA % 0x4U;
        
        /*while(colCnt > 0U)
        {
            sum += *S++ * x[*Scols++];
            colCnt--;
        }*/
        if(colCnt == 3)
        {
            in3 = x[*Scols++];
            in1 = *S++;
            in2 = *S++;
            sum += in1 * in3;
            in4 = x[*Scols++];
            sum += in2 * in4;
            
            in3 = x[*Scols++];
            in1 = *S++;
            sum += in1 * in3;
        }
        else if (colCnt == 2)
        {
            in3 = x[*Scols++];
            in1 = *S++;
            in2 = *S++;
            sum += in1 * in3;
            in4 = x[*Scols++];
            sum += in2 * in4;
        }
        else if (colCnt == 1)
        {
            sum += *S++ * x[*Scols++];
        }

        *y++ = sum;
        
        row--; //decrement row loop counter
    } while(row > 0u);
}
#pragma GCC pop_options

uint8_t seq_calc(setpoint_t *setpoint)
{
    // This function assumes we are sending the switching seq via the computer.
    if(setpoint->sys_mode != 0) current_mode = setpoint->sys_mode;
    
    uint8_t temp = current_mode*10 + prev_mode;
    
    uint8_t i = 0;
    while((temp != Psi[i]) && (i<psi_length))
    {
        i++;
    }
    
    prev_mode = current_mode;
    
    return i;
}

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
  } else if (state->position.z >= 4.0f) {
    control->enabled = 0;
  }
  
  
  if (RATE_DO_EXECUTE(HINF_RATE, tick) && (control->enabled)) 
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
    e[8] = (0.0f - state->attitude.yaw); 
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
    e[14] = (state->poscf1.z - state->position.z);
    e[15] = (setpoint->velcf2.x + state->velcf1.x - state->velocity.x);
    e[16] = (setpoint->velcf2.y + state->velcf1.y - state->velocity.y);
    e[17] = (state->velcf1.z - state->velocity.z);
    e[18] = (0.0f - state->attitude.roll); // for angles, ref and eps will always equal 0
    e[19] = (0.0f - state->attitude.pitch);
    e[20] = (0.0f - state->attitude.yaw); 
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
    e[14] = (state->poscf1.z - state->poscf2.z);
    e[15] = (setpoint->velcf2.x + state->velcf1.x - state->velcf2.x);
    e[16] = (setpoint->velcf2.y + state->velcf1.y - state->velcf2.y);
    e[17] = (state->velcf1.z - state->velcf2.z);
    e[18] = 0.0f; // for angles, ref and eps will always equal 0
    e[19] = 0.0f;
    e[20] = 0.0f; 
    e[21] = 0.0f;
    e[22] = 0.0f;
    e[23] = 0.0f;
    e[24] = (setpoint->poscf3.x + state->poscf2.x - state->position.x); //ref + eps - pos
    e[25] = (setpoint->poscf3.y + state->poscf2.y - state->position.y);
    e[26] = (state->poscf2.z - state->position.z); // setpoint->poscf3.z = 0 for all t
    e[27] = (setpoint->velcf3.x + state->velcf2.x - state->velocity.x);
    e[28] = (setpoint->velcf3.y + state->velcf2.y - state->velocity.y);
    e[29] = (state->velcf2.z - state->velocity.z);
    e[30] = (0.0f - state->attitude.roll); // for angles, ref and eps will always equal 0
    e[31] = (0.0f - state->attitude.pitch);
    e[32] = (0.0f - state->attitude.yaw); 
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
    e[14] = (state->poscf1.z - state->poscf2.z);
    e[15] = (setpoint->velcf2.x + state->velcf1.x - state->velcf2.x);
    e[16] = (setpoint->velcf2.y + state->velcf1.y - state->velcf2.y);
    e[17] = (state->velcf1.z - state->velcf2.z);
    e[18] = 0.0f; // for angles, ref and eps will always equal 0
    e[19] = 0.0f;
    e[20] = 0.0f; 
    e[21] = 0.0f;
    e[22] = 0.0f;
    e[23] = 0.0f;
    e[24] = (setpoint->poscf3.x + state->poscf2.x - state->poscf3.x); //ref + eps - pos
    e[25] = (setpoint->poscf3.y + state->poscf2.y - state->poscf3.y);
    e[26] = (state->poscf2.z - state->poscf3.z);
    e[27] = (setpoint->velcf3.x + state->velcf2.x - state->velcf3.x);
    e[28] = (setpoint->velcf3.y + state->velcf2.y - state->velcf3.y);
    e[29] = (state->velcf2.z - state->velcf3.z);
    e[30] = 0.0f; // for angles, ref and eps will always equal 0
    e[31] = 0.0f;
    e[32] = 0.0f; 
    e[33] = 0.0f;
    e[34] = 0.0f;
    e[35] = 0.0f;
    e[36] = (setpoint->poscf4.x + state->poscf3.x - state->position.x); //ref + eps - pos
    e[37] = (setpoint->poscf4.y + state->poscf3.y - state->position.y);
    e[38] = (state->poscf3.z - state->position.z);
    e[39] = (setpoint->velcf4.x + state->velcf3.x - state->velocity.x);
    e[40] = (setpoint->velcf4.y + state->velcf3.y - state->velocity.y);
    e[41] = (state->velcf3.z - state->velocity.z);
    e[42] = (0.0f - state->attitude.roll); // for angles, ref and eps will always equal 0
    e[43] = (0.0f - state->attitude.pitch);
    e[44] = (0.0f - state->attitude.yaw); 
    e[45] = (0.0f - state->attitudeRate.roll);
    e[46] = (0.0f - state->attitudeRate.pitch);
    e[47] = (0.0f - state->attitudeRate.yaw);
#elif CFNUM==5
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
    e[14] = (state->poscf1.z - state->poscf2.z);
    e[15] = (setpoint->velcf2.x + state->velcf1.x - state->velcf2.x);
    e[16] = (setpoint->velcf2.y + state->velcf1.y - state->velcf2.y);
    e[17] = (state->velcf1.z - state->velcf2.z);
    e[18] = 0.0f; // for angles, ref and eps will always equal 0
    e[19] = 0.0f;
    e[20] = 0.0f; 
    e[21] = 0.0f;
    e[22] = 0.0f;
    e[23] = 0.0f;
    e[24] = (setpoint->poscf3.x + state->poscf2.x - state->poscf3.x); //ref + eps - pos
    e[25] = (setpoint->poscf3.y + state->poscf2.y - state->poscf3.y);
    e[26] = (state->poscf2.z - state->poscf3.z);
    e[27] = (setpoint->velcf3.x + state->velcf2.x - state->velcf3.x);
    e[28] = (setpoint->velcf3.y + state->velcf2.y - state->velcf3.y);
    e[29] = (state->velcf2.z - state->velcf3.z);
    e[30] = 0.0f; // for angles, ref and eps will always equal 0
    e[31] = 0.0f;
    e[32] = 0.0f; 
    e[33] = 0.0f;
    e[34] = 0.0f;
    e[35] = 0.0f;
    e[36] = (setpoint->poscf4.x + state->poscf3.x - state->poscf4.x); //ref(i) + eps(i-1) - pos(i)
    e[37] = (setpoint->poscf4.y + state->poscf3.y - state->poscf4.y);
    e[38] = (state->poscf3.z - state->poscf4.z);
    e[39] = (setpoint->velcf4.x + state->velcf3.x - state->velcf4.x);
    e[40] = (setpoint->velcf4.y + state->velcf3.y - state->velcf4.y);
    e[41] = (state->velcf3.z - state->velcf4.z);
    e[42] = 0.0f; // for angles, ref and eps will always equal 0
    e[43] = 0.0f;
    e[44] = 0.0f; 
    e[45] = 0.0f;
    e[46] = 0.0f;
    e[47] = 0.0f;
    e[48] = (setpoint->poscf5.x + state->poscf4.x - state->position.x); //ref + eps - pos
    e[49] = (setpoint->poscf5.y + state->poscf4.y - state->position.y);
    e[50] = (state->poscf4.z - state->position.z);
    e[51] = (setpoint->velcf5.x + state->velcf4.x - state->velocity.x);
    e[52] = (setpoint->velcf5.y + state->velcf4.y - state->velocity.y);
    e[53] = (state->velcf4.z - state->velocity.z);
    e[54] = (0.0f - state->attitude.roll); // for angles, ref and eps will always equal 0
    e[55] = (0.0f - state->attitude.pitch);
    e[56] = (0.0f - state->attitude.yaw); 
    e[57] = (0.0f - state->attitudeRate.roll);
    e[58] = (0.0f - state->attitudeRate.pitch);
    e[59] = (0.0f - state->attitudeRate.yaw);
#endif
    
    seq_num = seq_calc(setpoint);
    switch(seq_num)
    {
        case 0:
            AK = AK1;
            AKrows = AK1rows;
            AKcols = AK1cols;
            BK = BK1;
            BKrows = BK1rows;
            BKcols = BK1cols;
            CK = CK1;
            CKrows = CK1rows;
            CKcols = CK1cols;
            DK = DK1;
            DKrows = DK1rows;
            DKcols = DK1cols;
            break;
        case 1:
            AK = AK2;
            AKrows = AK2rows;
            AKcols = AK2cols;
            BK = BK2;
            BKrows = BK2rows;
            BKcols = BK2cols;
            CK = CK2;
            CKrows = CK2rows;
            CKcols = CK2cols;
            DK = DK2;
            DKrows = DK2rows;
            DKcols = DK2cols;
            break;
        case 2:
            AK = AK3;
            AKrows = AK3rows;
            AKcols = AK3cols;
            BK = BK3;
            BKrows = BK3rows;
            BKcols = BK3cols;
            CK = CK3;
            CKrows = CK3rows;
            CKcols = CK3cols;
            DK = DK3;
            DKrows = DK3rows;
            DKcols = DK3cols;
            break;
        case 3:
            AK = AK4;
            AKrows = AK4rows;
            AKcols = AK4cols;
            BK = BK4;
            BKrows = BK4rows;
            BKcols = BK4cols;
            CK = CK4;
            CKrows = CK4rows;
            CKcols = CK4cols;
            DK = DK4;
            DKrows = DK4rows;
            DKcols = DK4cols;
            break;
        case 4:
            AK = AK5;
            AKrows = AK5rows;
            AKcols = AK5cols;
            BK = BK5;
            BKrows = BK5rows;
            BKcols = BK5cols;
            CK = CK5;
            CKrows = CK5rows;
            CKcols = CK5cols;
            DK = DK5;
            DKrows = DK5rows;
            DKcols = DK5cols;
            break;
        case 5:
            AK = AK6;
            AKrows = AK6rows;
            AKcols = AK6cols;
            BK = BK6;
            BKrows = BK6rows;
            BKcols = BK6cols;
            CK = CK6;
            CKrows = CK6rows;
            CKcols = CK6cols;
            DK = DK6;
            DKrows = DK6rows;
            DKcols = DK6cols;
            break;
        case 6:
        default:
            AK = AK7;
            AKrows = AK7rows;
            AKcols = AK7cols;
            BK = BK7;
            BKrows = BK7rows;
            BKcols = BK7cols;
            CK = CK7;
            CKrows = CK7rows;
            CKcols = CK7cols;
            DK = DK7;
            DKrows = DK7rows;
            DKcols = DK7cols;
            break;
    }
    

    sparse_mult(CK, CKrows, CKcols, 4, xHinf, tmpCx); // C*xHinf
    sparse_mult(DK, DKrows, DKcols, 4, e, tmpDe); // D*e
    arm_add_f32(tmpCx, tmpDe, u, 4);
    
    sparse_mult(AK, AKrows, AKcols, HINF_STATES * CFNUM, xHinf, tmpAx); // A*xHinf
    sparse_mult(BK, BKrows, BKcols, HINF_STATES * CFNUM, e, tmpBe); // B*e
    arm_add_f32(tmpAx, tmpBe, xHinf, HINF_STATES * CFNUM);

    
    //memcpy(xHinf, xHinfNext, sizeof(xHinf));
    
    #if CFNUM==1
    control->thrust = u[0] + 0.33f;
    #elif CFNUM==2
    control->thrust = u[0] + 0.3345f;
    #elif CFNUM==3
    control->thrust = u[0] + 0.3316f;
    #elif CFNUM==4
    control->thrust = u[0] + 0.3316f;
    #elif CFNUM==5
    control->thrust = u[0] + 0.33f;
    #endif
    control->roll = u[1];
    control->pitch = u[2];
    control->yaw = u[3];
    control->enabled = 1;
  }

}
