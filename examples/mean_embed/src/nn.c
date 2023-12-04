#include "../include/nn.h"
#include "../include/mlp.h"
#include "../include/neighbor_encoder.h"
#include "../include/self_encoder.h"
#include "../include/self_observation.h"
#include "../include/neighbor_observation.h"
#include "../include/vec.h"
#include <math.h>


 
static nnInputOutput* nnIO;
static self_obs* selfObservation;
static neighb_obs kNearestObservations[6];
static mlpInput* mlpInp;
static int myId = 0;


int getMyId()
{
    return myId;
}


void feedForwardNN(float* thrusts)
{
    updateSelfObservation(selfObservation);
    calcSelfEncoderOutput(selfObservation->self_obs_arr, nnIO->outputSelf);
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
        nnIO->output[k] = 0.5f*(clipVal(nnIO->output[k], -1.0f, 1.0f)+1);
        thrusts[k] = nnIO->output[k];
    }



}

