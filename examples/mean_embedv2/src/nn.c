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
 
static float      mlpInput[24];
static neighb_obs kNearestObservations[6];

void feedForwardNN(float* thrusts)
{
    updateSelfObservation(selfObservation);

    // float selfO[18] =   {0.5488135,  0.71518937, 0.60276338, 0.54488318, 0.4236548,  0.64589411,
    //                 0.43758721, 0.891773   ,0.96366276 ,0.38344152 ,0.79172504,  0.52889492,
    //                 0.56804456, 0.92559664 ,0.07103606 ,0.0871293  ,0.0202184 ,  0.83261985};
    // for(int k = 0; k < 18; k++)
    // {
    //     selfObservation[k] = selfO[k];
    // }
    // selfObservation[0] = selfObservation[0]/10.0f;
    // selfObservation[1] = selfObservation[1]/10.0f;
    // selfObservation[2] = selfObservation[2]/10.0f;

    // selfObservation[3] = selfObservation[3]/3.0f;
    // selfObservation[4] = selfObservation[4]/3.0f;
    // selfObservation[5] = selfObservation[5]/3.0f;

    // selfObservation[15] = selfObservation[15]/40.0f;
    // selfObservation[16] = selfObservation[16]/40.0f;
    // selfObservation[17] = selfObservation[17]/40.0f;

    calcSelfEncoderOutput(selfObservation, outputSelf);
    updateNeighbObservation(kNearestObservations);

    //  float neighbO[6][6] = {{0.77815675, 0.87001215, 0.97861834, 0.79915856, 0.46147936, 0.78052918,},
    //                         {0.11827443, 0.63992102, 0.14335329, 0.94466892, 0.52184832, 0.41466194,},
    //                         {0.26455561, 0.77423369, 0.45615033, 0.56843395, 0.0187898 , 0.6176355 ,},
    //                         {0.61209572, 0.616934  , 0.94374808, 0.6818203 , 0.3595079 , 0.43703195}};
    
    //  for(int k = 0; k < 6; k++)
    // {
    //     kNearestObservations[k].relPos.x = neighbO[k][0];
    //     kNearestObservations[k].relPos.y = neighbO[k][1];
    //     kNearestObservations[k].relPos.z = neighbO[k][2];

    //     kNearestObservations[k].relVel.x = neighbO[k][3];
    //     kNearestObservations[k].relVel.y = neighbO[k][4];
    //     kNearestObservations[k].relVel.z = neighbO[k][5];
    // }
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
LOG_ADD(LOG_FLOAT, Px,         &(selfObservation[0]))
LOG_ADD(LOG_FLOAT, Py,         &(selfObservation[1]))
LOG_ADD(LOG_FLOAT, Pz,         &(selfObservation[2]))
LOG_ADD(LOG_FLOAT, Vx,         &(selfObservation[3]))
LOG_ADD(LOG_FLOAT, Vy,         &(selfObservation[4]))
LOG_ADD(LOG_FLOAT, Vz,         &(selfObservation[5]))
LOG_ADD(LOG_FLOAT, R1,         &(selfObservation[6]))
LOG_ADD(LOG_FLOAT, R2,         &(selfObservation[7]))
LOG_ADD(LOG_FLOAT, R3,         &(selfObservation[8]))
LOG_ADD(LOG_FLOAT, R4,         &(selfObservation[9]))
LOG_ADD(LOG_FLOAT, R5,         &(selfObservation[10]))
LOG_ADD(LOG_FLOAT, R6,         &(selfObservation[11]))
LOG_ADD(LOG_FLOAT, R7,         &(selfObservation[12]))
LOG_ADD(LOG_FLOAT, R8,         &(selfObservation[13]))
LOG_ADD(LOG_FLOAT, R9,         &(selfObservation[14]))
LOG_ADD(LOG_FLOAT, Wx,         &(selfObservation[15]))
LOG_ADD(LOG_FLOAT, Wy,         &(selfObservation[16]))
LOG_ADD(LOG_FLOAT, Wz,         &(selfObservation[17]))

LOG_ADD(LOG_FLOAT, t0, &(thrusts_out[0]))
LOG_ADD(LOG_FLOAT, t1, &(thrusts_out[1]))
LOG_ADD(LOG_FLOAT, t2, &(thrusts_out[2]))
LOG_ADD(LOG_FLOAT, t3, &(thrusts_out[3]))

LOG_ADD(LOG_FLOAT, os1,  &(outputSelf[0]))
LOG_ADD(LOG_FLOAT, os2,  &(outputSelf[1]))
LOG_ADD(LOG_FLOAT, os3,  &(outputSelf[2]))
LOG_ADD(LOG_FLOAT, os4,  &(outputSelf[3]))
LOG_ADD(LOG_FLOAT, os5,  &(outputSelf[4]))
LOG_ADD(LOG_FLOAT, os6,  &(outputSelf[5]))
LOG_ADD(LOG_FLOAT, os7,  &(outputSelf[6]))
LOG_ADD(LOG_FLOAT, os8,  &(outputSelf[7]))
LOG_ADD(LOG_FLOAT, os9,  &(outputSelf[8]))
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


/*
{{relPos = {x = 0.43612361, y = 0.411226243, z = 0.564345956}, relVel = {x = 0.0732458681, y = 0.635959864, z = 0.774004281}}, {relPos = {x = 0.976625443, y = 0.632692397, z = 0.55660677}, relVel = {x = 0.633000076, y = 0.963036358, z = 0.367442966}}, {relPos = {x = 0.0508353412, y = 0.670255601, z = 0.918097079}, relVel = {x = 0.083669588, y = 0.0781115964, z = 0.949854314}}, {relPos = {x = 0.56072408, y = 0.212242752, z = 0.951712608}, relVel = {x = 0.400482684, y = 0.554931879, z = 0.00556677999}}, {relPos = {x = 0.919300139, y = 0.166269556, z = 0.10357473}, relVel = {x = 0.545766056, y = 0.341175854, z = 0.98846215}}, {relPos = {x = 0.586700082, y = 0.165764853, z = 0.183052376}, relVel = {x = 0.220226064, y = 0.363634437, z = 0.244552955}}}

*/