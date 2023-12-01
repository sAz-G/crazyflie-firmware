#include "../include/mlp.h"

static const float phi_a_w0[OUTPUT0][PHI_A_H];
static const float phi_a_w1[PHI_A_V][OUTPUT0];

static const float phi_a_b0[OUTPUT0];
static const float phi_a_b1[PHI_A_V];

static inpOutVec* networkVecs;

static void          feedForwardMLP();
static void          feedForwardPhiA0(int);
static void          feedForwardPhiA1(int);
static void          updateMlpInput();

static  inpOutVec*   getInputOutputMlpVecs();
static  float*       getInputMlpVec();
static  float*       getOutputMlpVec0();
static  float*       getOutputMlpVec1();
static  int          getInputMlpSize();
static  int          getOutputMlpSize();
static  int          getOutput0MlpSize();

void calcMlpOutput()
{
    feedForwardMLP();
}

static void feedForwardMLP()
{
    updateMlpInput();
    int sz0 = getOutput0MlpSize();
    int sz1 = getOutputMlpSize();

    for(int k = 0; k < sz0; k++)
    {
        feedForwardPhiA0(k);
    }

    for(int k = 0; k < sz1; k++)
    {
        feedForwardPhiA1(k);
    }

    
}

static void updateMlpInput()
{
    return;
}

static int getOutput0MlpSize()
{
    return (int)OUTPUT0;
}

static int getOutputMlpSize()
{
    return (int)PHI_A_V;
}

static void feedForwardPhiA0(int raw)
{
    float* inpVec0  = getInputMlpVec();
    int sz          = getInputMlpSize();
    float tmp       = 0;

    for(int k = 0; k < sz; k++)
    {
        tmp += inpVec0[k]*phi_a_w0[raw][k];
    }

    float* outVec0  = getOutputMlpVec0();
    outVec0[raw]    = tmp + phi_a_b0[raw];

}

static float* getInputMlpVec()
{
    return networkVecs->inputVec;
}

static float* getOutputMlpVec0()
{
    return getInputOutputMlpVecs()->out0;
}

static float* getOutputMlpVec1()
{
    return getInputOutputMlpVecs()->phi_a;
}

static inpOutVec* getInputOutputMlpVecs()
{
    return networkVecs;
}


static int getInputMlpSize()
{
    return (int)PHI_A_H;
}

static void feedForwardPhiA1(int raw)
{
   float* inpVec1 = getOutputMlpVec0();
   int sz         = getInputMlpSize();
   float tmp = 0;

   for(int k = 0; k < sz; k++)
   {
        tmp += inpVec1[k]*phi_a_w1[raw][k];
   }

   float* outVec1 = getOutputMlpVec1();
   outVec1[raw]   = tmp +  phi_a_b1[raw];
}




