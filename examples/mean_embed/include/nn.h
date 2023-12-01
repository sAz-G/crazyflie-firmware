#ifndef __NN_H__
#define __NN_H__

#define N_DRONES 8

typedef struct _NN_INPUT_OUTPUT_
{
    float* inputSelf;
    float* inputNeighbor;
    float* output;

}nnInputOutput;


void feedForwardNN();


#endif