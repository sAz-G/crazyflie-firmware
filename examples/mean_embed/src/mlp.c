#include "../include/mlp.h"

static const float phi_a_w0[OUTPUT0][PHI_A_H];
static const float phi_a_w1[PHI_A_V][OUTPUT0];

static const float phi_a_b0[OUTPUT0];
static const float phi_a_b1[PHI_A_V];

static inpOutVec* networkVecs;

static void  feedForwardMLP(float*);
static void  feedForwardPhiA0(float*, int);
static void  feedForwardPhiA1(float*, int);

static  int  getInputMlpSize();
static  int  getOutputMlpSize();
static  int  getOutput0MlpSize();

void calcMlpOutput(mlpInput* input, float* out)
{

    feedForwardMLP(input->inputVec);
    for(int k = 0; k < 4; k++)
    {
        out[k] = networkVecs->phi_a[k];    
    }
}

static void feedForwardMLP(float* inp)
{
    int sz0 = getOutput0MlpSize();
    int sz1 = getOutputMlpSize();

    for(int k = 0; k < sz0; k++)
    {
        feedForwardPhiA0(inp, k);
    }

    for(int k = 0; k < sz1; k++)
    {
        feedForwardPhiA1(networkVecs->out0, k);
    }
    
}

static void feedForwardPhiA0(float* inp, int raw)
{
    int sz          = getInputMlpSize();
    float tmp       = 0;

    for(int k = 0; k < sz; k++)
    {
        tmp += inp[k]*phi_a_w0[raw][k];
    }

    networkVecs->out0[raw]    = tmp + phi_a_b0[raw];

}

static void feedForwardPhiA1(float* inp, int raw)
{
   int sz         = getInputMlpSize();
   float tmp      = 0;

   for(int k = 0; k < sz; k++)
   {
        tmp += inp[k]*phi_a_w1[raw][k];
   }

   networkVecs->phi_a[raw]   = tmp +  phi_a_b1[raw];
}

static int getInputMlpSize()
{
    return (int)PHI_A_H;
}

static int getOutput0MlpSize()
{
    return (int)OUTPUT0;
}

static int getOutputMlpSize()
{
    return (int)PHI_A_V;
}


