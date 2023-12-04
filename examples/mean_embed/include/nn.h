#ifndef __NN_H__
#define __NN_H__

#define N_DRONES 8

typedef struct _NN_INPUT_OUTPUT_
{
    float outputSelf[16];
    float outputNeighbor[8];
    float output[4];
}nnInputOutput;


void feedForwardNN();
int getMyId();


#endif