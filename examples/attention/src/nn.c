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



static float selfObservation[18];
static float thrusts_out[4];
static float outputSelf[16];
static float outputNeighbor[8];
 
static float mlpInput[24];
static neighb_obs kNearestObservations[K_NEIGHBORS];


void feedForwardNN(float* thrusts)
{

    updateSelfObservation(selfObservation);
    calcSelfEncoderOutput(selfObservation, outputSelf);
    updateNeighbObservation(kNearestObservations);
    calcNeighborEncoderOutput(kNearestObservations, outputNeighbor);

    for(int k = 0; k < 16; k++)
    {
        mlpInput[k] = outputSelf[k];
    }

    for(int k = 0; k < 8; k++)
    {
        mlpInput[k+16] = outputNeighbor[k];
    }

    calcMlpOutput(mlpInput, thrusts);

    for(int k = 0; k < 4; k++)
    {
      thrusts[k] = 0.5f*(clipVal(thrusts[k], -1.0f, 1.0f)+1.0f);
      thrusts_out[k] = thrusts[k];
    }

}

LOG_GROUP_START(selfObsLog)
LOG_ADD(LOG_FLOAT, observedPx, &(selfObservation[0]))
LOG_ADD(LOG_FLOAT, observedPy, &(selfObservation[1]))
LOG_ADD(LOG_FLOAT, observedPz, &(selfObservation[2]))
LOG_ADD(LOG_FLOAT, observedVx, &(selfObservation[3]))
LOG_ADD(LOG_FLOAT, observedVy, &(selfObservation[4]))
LOG_ADD(LOG_FLOAT, observedVz, &(selfObservation[5]))
LOG_ADD(LOG_FLOAT, observedR1, &(selfObservation[6]))
LOG_ADD(LOG_FLOAT, observedR2, &(selfObservation[7]))
LOG_ADD(LOG_FLOAT, observedR3, &(selfObservation[8]))
LOG_ADD(LOG_FLOAT, observedR4, &(selfObservation[9]))
LOG_ADD(LOG_FLOAT, observedR5, &(selfObservation[10]))
LOG_ADD(LOG_FLOAT, observedR6, &(selfObservation[11]))
LOG_ADD(LOG_FLOAT, observedR7, &(selfObservation[12]))
LOG_ADD(LOG_FLOAT, observedR8, &(selfObservation[13]))
LOG_ADD(LOG_FLOAT, observedR9, &(selfObservation[14]))
LOG_ADD(LOG_FLOAT, observedWx, &(selfObservation[15]))
LOG_ADD(LOG_FLOAT, observedWy, &(selfObservation[16]))
LOG_ADD(LOG_FLOAT, observedWz, &(selfObservation[17]))

LOG_ADD(LOG_FLOAT, t0, &(thrusts_out[0]))
LOG_ADD(LOG_FLOAT, t1, &(thrusts_out[1]))
LOG_ADD(LOG_FLOAT, t2, &(thrusts_out[2]))
LOG_ADD(LOG_FLOAT, t3, &(thrusts_out[3]))

LOG_ADD(LOG_FLOAT, os1, &(outputSelf[0]))
LOG_ADD(LOG_FLOAT, os2, &(outputSelf[1]))
LOG_ADD(LOG_FLOAT, os3, &(outputSelf[2]))
LOG_ADD(LOG_FLOAT, os4, &(outputSelf[3]))
LOG_ADD(LOG_FLOAT, os5, &(outputSelf[4]))
LOG_ADD(LOG_FLOAT, os6, &(outputSelf[5]))
LOG_ADD(LOG_FLOAT, os7, &(outputSelf[6]))
LOG_ADD(LOG_FLOAT, os8, &(outputSelf[7]))
LOG_ADD(LOG_FLOAT, os9, &(outputSelf[8]))
LOG_ADD(LOG_FLOAT, os10, &(outputSelf[9]))
LOG_ADD(LOG_FLOAT, os11, &(outputSelf[10]))
LOG_ADD(LOG_FLOAT, os12, &(outputSelf[11]))
LOG_ADD(LOG_FLOAT, os13, &(outputSelf[12]))
LOG_ADD(LOG_FLOAT, os14, &(outputSelf[13]))
LOG_ADD(LOG_FLOAT, os15, &(outputSelf[14]))
LOG_ADD(LOG_FLOAT, os16, &(outputSelf[15]))
LOG_GROUP_STOP(selfObsLog)


LOG_GROUP_START(nebOb)
LOG_ADD(LOG_FLOAT, on1, &(outputNeighbor[0]))
LOG_ADD(LOG_FLOAT, on2, &(outputNeighbor[1]))
LOG_ADD(LOG_FLOAT, on3, &(outputNeighbor[2]))
LOG_ADD(LOG_FLOAT, on4, &(outputNeighbor[3]))
LOG_ADD(LOG_FLOAT, on5, &(outputNeighbor[4]))
LOG_ADD(LOG_FLOAT, on6, &(outputNeighbor[5]))
LOG_ADD(LOG_FLOAT, on7, &(outputNeighbor[6]))
LOG_ADD(LOG_FLOAT, on8, &(outputNeighbor[7]))
LOG_GROUP_STOP(nebOb)


LOG_GROUP_START(mlpNet)
LOG_ADD(LOG_FLOAT, mlp1, &(mlpInput[0]))
LOG_ADD(LOG_FLOAT, mlp2, &(mlpInput[1]))
LOG_ADD(LOG_FLOAT, mlp3, &(mlpInput[2]))
LOG_ADD(LOG_FLOAT, mlp4, &(mlpInput[3]))
LOG_ADD(LOG_FLOAT, mlp5, &(mlpInput[4]))
LOG_ADD(LOG_FLOAT, mlp6, &(mlpInput[5]))
LOG_ADD(LOG_FLOAT, mlp7, &(mlpInput[6]))
LOG_ADD(LOG_FLOAT, mlp8, &(mlpInput[7]))
LOG_GROUP_STOP(mlpNet)

