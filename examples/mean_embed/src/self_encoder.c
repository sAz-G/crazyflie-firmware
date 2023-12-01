#include "../include/self_encoder.h"

static const float psi_s_w0[PSI_S_H][PSI_S_V];
static const float psi_s_w1[PSI_S_V][PSI_S_V];

static const float psi_s_b0[PSI_S_V];
static const float psi_s_b1[PSI_S_V];


//static selfVecs*  inputOutputVectors;       
//static float      encoderOutput[SELF_NETWORK_OUT]; // e_s in the paper

/*
functions
*/

// getters
//static selfVecs* getInputOutputVectors();
//static float*    getSelfEncoderOutput();
static int       getSelfEncoderOutputSize();
static int       getSelfEncoderOutput0Size();
static int       getEncoderInputSize();
//static float*    getSelfObservationVector();
// static float*    getOut0();
//static void      updateSelfInfo();
//static void      setSelfInput(float*);



static void    feedForwardSelfEncoder(float* inp, float* dst);
static void    feedForwardPsiS(float* inp, float* dst);
static void    feedForwardPsiS0(float* inp, float* dst,int k);
static void    feedForwardPsiS1(float* inp, float* dst,int k);


//static float out0[PSI_S_V];



void calcSelfEncoderOutput(float* inp, float* dst)
{
    //setSelfInput(inp);
    feedForwardSelfEncoder(inp, dst);
}

static void feedForwardSelfEncoder(float* inp, float* dst)
{
    //updateSelfInfo();
    feedForwardPsiS(inp, dst);
}

/*static void updateSelfInfo()
{
    return;
}*/

static void  feedForwardPsiS(float* inp, float* dst)
{
    float* out0[getSelfEncoderOutput0Size()];
    for(int k = 0; k < getSelfEncoderOutputSize(); k++)
    {
        feedForwardPsiS0(inp, out0, k);
    }   

    float* out1[getSelfEncoderOutput0Size()];
    for(int k = 0; k < getSelfEncoderOutputSize(); k++)
    {
        feedForwardPsiS1(out0, out1, k);
    }   
}


static void feedForwardPsiS0(float* inp, float* out, int raw)
{
    int sz        = getEncoderInputSize();
    float temp    = 0.0;    

    for(int k = 0; k < sz; k++)
    {
        temp += psi_s_w0[raw][k]*inp[k];
    }

    out[raw] = temp + psi_s_b0[raw];
}

static void feedForwardPsiS1(float* inp, float* out, int raw)
{
    int sz        = getSelfEncoderOutputSize();
    float temp    = 0.0;    

    for(int k = 0; k < sz; k++)
    {
        temp += psi_s_w1[raw][k]*inp[k];
    }

    out[raw] = temp + psi_s_b1[raw];
}

// static selfVecs* getInputOutputVectors()
// {
//     return inputOutputVectors;
// }


// static float* getSelfObservationVector()
// {
//     return getInputOutputVectors()->obsStruct->self_obs_arr;
// }

// static float* getSelfEncoderOutput()
// {
//     return getInputOutputVectors()->e_s;
// }

// static float* getOut0()
// {
//     return out0;
// }

static int getEncoderInputSize()
{
    return (int)PSI_S_H;
}

static int getSelfEncoderOutputSize()
{
    return (int)PSI_S_V;
}

static int getSelfEncoderOutput0Size()
{
    return (int)PSI_S_V;
}

// float*  getSelfOutput()
// {
//     return inputOutputVectors->e_s;
// }

// void  setSelfInput(float* arr)
// {
//     for(int k = 0; k < getEncoderInputSize(); k++)
//     {
//         inputOutputVectors->obsStruct->self_obs_arr[k] = arr[k];
//     }
    
// }
