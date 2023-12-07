#include "../include/nn.h"
#include "../include/mlp.h"
#include "../include/neighbor_encoder.h"
#include "../include/self_encoder.h"
#include "../include/self_observation.h"
#include "../include/neighbor_observation.h"
#include "../include/vec.h"
#include "log.h"
#include "param.h"

#include <math.h>


 
 static nnInputOutput* nnIO;
 static self_obs* selfObservation;

 static float selfObservationArr[3];
 static neighb_obs kNearestObservations[6];
 static mlpInput* mlpInp;
 static uint8_t isInitialized = 0;

static void nnInit();

static void nnInit()
{
  // for(int k = 0; k < 18; k++)
  // {
  //   selfObservation->self_obs_arr[k] = 0.0;
  // }
  
  // selfObservation->obsLen = 18;
  mlpInp->len = 24;

  for(int k = 0; k < 6; k++)
  {
    kNearestObservations[k].obsLen = 6;
  }
}


void feedForwardNN(float* thrusts)
{//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXx
    if(!isInitialized)
    {
      nnInit();
      isInitialized = 1;
    }

    updateSelfObservation(selfObservation);

    float inparr[18];

    selfObservationArr[0] = selfObservation->px;
    selfObservationArr[1] = selfObservation->py;
    selfObservationArr[2] = selfObservation->pz;

    inparr[0] = selfObservation->px;
    inparr[1] = selfObservation->py;
    inparr[2] = selfObservation->pz;
    inparr[15] = selfObservation->wx;
    inparr[16] = selfObservation->wy;
    inparr[17] = selfObservation->wz;

    inparr[3] = selfObservation->vx;
    inparr[4] = selfObservation->vy;
    inparr[5] = selfObservation->vz;

    inparr[6] = selfObservation->r1;
    inparr[7] = selfObservation->r2;
    inparr[8] = selfObservation->r3;
    inparr[9] = selfObservation->r4;
    inparr[10] = selfObservation->r5;
    inparr[11] = selfObservation->r6;
    inparr[12] = selfObservation->r7;
    inparr[13] = selfObservation->r8;
    inparr[14] = selfObservation->r9;

    calcSelfEncoderOutput(inparr, nnIO->outputSelf);
    updateNeighbObservation(kNearestObservations);
    calcNeighborEncoderOutput(kNearestObservations, nnIO->outputNeighbor);

    for(int k = 0; k < 16; k++)
    {
        mlpInp->inputVec[k] = nnIO->outputSelf[k];
    }

    for(int k = 0; k < 8; k++)
    {
        mlpInp->inputVec[k+16] = nnIO->outputNeighbor[k];
    }

    calcMlpOutput(mlpInp, nnIO->output);

    for(int k = 0; k < 4; k++)
    {
        nnIO->output[k] = 0.5f*(clipVal(nnIO->output[k], -1.0f, 1.0f)+1.0f);
        thrusts[k] = nnIO->output[k];
    }



}

LOG_GROUP_START(selfObsLog)
LOG_ADD(LOG_FLOAT, observedPx, &(selfObservationArr[0]))
LOG_ADD(LOG_FLOAT, observedPy, &(selfObservationArr[1]))
LOG_ADD(LOG_FLOAT, observedPz, &(selfObservationArr[2]))
LOG_GROUP_STOP(selfObsLog)
