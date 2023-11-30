#include "../include/self_encoder.h"

static const float psi_s_w0[PSI_S_H][PSI_S_V];
static const float psi_s_w1[PSI_S_V][PSI_S_V];

static const float psi_s_b0[PSI_S_V];
static const float psi_s_b1[PSI_S_V];


static selfVecs*  inputOutputVectors;       
//static float      encoderOutput[SELF_NETWORK_OUT]; // e_s in the paper

/*
functions
*/

// getters
static selfVecs* getInputOutputVectors();
//static float*    getSelfEncoderOutput();
static int       getSelfEncoderOutputSize();
static int       getEncoderInputSize();
static float*    getSelfObservationVector();
static float*    getOut0();
//static void      updateSelfInfo();


static void    feedForwardSelfEncoder();
static void    feedForwardPsiS();
static void    feedForwardPsiS0(int k);
static void    feedForwardPsiS1(int k);


static float out0[PSI_S_V];



void calcSelfEncoderOutput()
{
    feedForwardSelfEncoder();
}

static void feedForwardSelfEncoder()
{
    //updateSelfInfo();
    feedForwardPsiS();
}

/*static void updateSelfInfo()
{
    return;
}*/

static void  feedForwardPsiS()
{
    for(int k = 0; k < getSelfEncoderOutputSize(); k++)
    {
        feedForwardPsiS0(k);
    }   

    for(int k = 0; k < getSelfEncoderOutputSize(); k++)
    {
        feedForwardPsiS1(k);
    }   
}


static void feedForwardPsiS0(int raw)
{
    int sz        = getEncoderInputSize();
    float* inp    = getSelfObservationVector();
    float* out    = getOut0();
    float temp    = 0.0;    

    for(int k = 0; k < sz; k++)
    {
        temp += psi_s_w0[raw][k]*inp[k];
    }

    out[raw] = temp + psi_s_b0[raw];
}

static void feedForwardPsiS1(int raw)
{
    int sz        = getSelfEncoderOutputSize();
    float* inp    = getOut0();
    //float*  out;    
   // out = getSelfEncoderOutput();
    float temp    = 0.0;    

    for(int k = 0; k < sz; k++)
    {
        temp += psi_s_w1[raw][k]*inp[k];
    }

    inputOutputVectors->e_s[raw] = temp + psi_s_b1[raw];
}

static selfVecs* getInputOutputVectors()
{
    return inputOutputVectors;
}


static float* getSelfObservationVector()
{
    return getInputOutputVectors()->obsStruct->self_obs_arr;
}

// static float* getSelfEncoderOutput()
// {
//     selfVecs* IOVecs = getInputOutputVectors();
//     return IOVecs->e_s;
// }

static float* getOut0()
{
    return out0;
}

static int getEncoderInputSize()
{
    return (int)PSI_S_H;
}

static int getSelfEncoderOutputSize()
{
    return (int)PSI_S_V;
}