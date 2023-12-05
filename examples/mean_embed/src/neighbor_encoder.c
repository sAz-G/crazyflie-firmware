#include "../include/neighbor_encoder.h"
#include "../include/vec.h"
#include "../include/neighbor_observation.h"


// static variables 
//////////////////////////////////////// WEIGHTS /////////////////////////////////////////////////
typedef struct ENCODER_VECTORS
{
    float inp[6];
    float out0[8];
    float out1[8];
}encoderVectors;

static const float psi_eta_w0[PSI_ETA_V][PSI_ETA_H] = {{ 0.5417, -0.6794, -0.6421,  0.0920, -0.2375, -0.1385},
                                                       { 0.5207, -0.7076, -0.4285, -0.1278,  0.3393, -0.1915},
                                                       { 0.4607, -0.6019, -0.3728, -0.3768,  0.4533,  0.2590},
                                                       { 0.5416, -0.5736,  0.0822, -0.4494,  0.4473,  0.0059},
                                                       {-0.0306, -0.1329,  0.1382,  0.1680, -0.0375, -0.3448},
                                                       {-0.4118,  0.3257,  0.2667, -0.4095, -0.0998, -0.2194},
                                                       { 0.8233, -0.1962,  0.1271, -0.1577, -0.2611,  0.2663},
                                                       { 0.3432, -0.3766, -0.7890,  0.0856,  0.2210,  0.2198}};



static const float psi_eta_w1[PSI_ETA_V][PSI_ETA_V] =  {{-3.2516e-02, -3.8135e-01,  2.4380e-01, -3.0701e-01, -1.7432e-01, 3.5791e-01,  2.9315e-01, -1.3260e-01},
                                                        { 1.2666e-03,  2.2348e-01,  3.3893e-01, -6.0730e-01,  3.4299e-01, -6.8567e-01, -1.1741e+00, -3.3565e-01},
                                                        {-8.4525e-01, -6.0548e-01, -5.9543e-01,  4.0336e-02, -2.1229e-01, -2.7084e-01, -2.0249e-01,  2.6816e-01},
                                                        { 3.0907e-01,  1.4779e-01, -5.6850e-01,  3.8580e-01, -1.3173e-01, 5.4307e-01, -2.3703e-01, -5.3131e-01},
                                                        {-5.9161e-01,  3.2005e-02,  4.0227e-01,  4.1339e-01, -2.2896e-02, -1.7285e-02, -2.4415e-01, -4.6611e-01},
                                                        {-7.8942e-02,  3.7274e-01,  4.5881e-01, -2.9711e-01,  5.6955e-01, -2.3767e-01, -1.8596e-01, -8.4011e-01},
                                                        {-7.7198e-01, -5.1864e-01,  5.4855e-01, -1.8149e-01,  4.5347e-01, -7.9741e-02, -7.5352e-02, -1.6073e-01},
                                                        {-8.1760e-01, -2.0768e-01,  3.6510e-01,  1.9212e-01, -7.1986e-04, 4.5544e-02, -1.7139e-01, -4.4614e-01}};







static const float psi_eta_b0[PSI_ETA_V] = { 0.0905,  0.1520, -0.1166,  0.0220,  0.2115,  0.0347,  0.1080,  0.1474};








static const float psi_eta_b1[PSI_ETA_V] = { 0.2088,  0.2443,  0.2780,  0.1496, -0.0144,  0.1181,  0.1876,  0.2691};






static encoderVectors encoderVecs;


static const int   KNeighbors = (int)K_NEIGHBORS;

// static functions
static void         calcMean(float*);
static int          getAmountKNeighbors();
static int          getAmountOutputs();
static void         feedForwardNeighborEncoder(neighb_obs* inp, float* outp);
static void         feedForwardPsiEta(neighb_obs obsK);
static void         feedForwardPsiEta0(float* inp, int raw);
static void         feedForwardPsiEta1(float* inp, int raw);
static void         addVectors(float* arr1, float* arr2, int len);


//feed forward functions for psi_eta, adapted for the FPGA
//feed forward variables for psi_eta, adapted for the FPGA

void calcNeighborEncoderOutput(neighb_obs* inp, float* outp)
{
    feedForwardNeighborEncoder(inp, outp);
}

static void feedForwardNeighborEncoder(neighb_obs* inp, float* outpEncoder)
{
    int sz = getAmountKNeighbors();

    for(int k = 0; k < sz; k++)
    {
        feedForwardPsiEta(inp[k]);    
        addVectors(encoderVecs.out1,outpEncoder,8);
    }
    
    calcMean(outpEncoder);
}

static void feedForwardPsiEta(neighb_obs obsK)
{
    int sz       = (int)PSI_ETA_V;

    encoderVecs.inp[0] = obsK.relPos.x;
    encoderVecs.inp[1] = obsK.relPos.y;
    encoderVecs.inp[2] = obsK.relPos.z;
    encoderVecs.inp[3] = obsK.relVel.x;
    encoderVecs.inp[4] = obsK.relVel.y;
    encoderVecs.inp[5] = obsK.relVel.z;

    for(int k = 0; k < sz; k++)
    {
        feedForwardPsiEta0(encoderVecs.inp, k);
    }

    for(int k = 0; k < sz; k++)
    {
        feedForwardPsiEta1(encoderVecs.out0, k);
    }

}

static void feedForwardPsiEta0(float* inp, int raw)
{
    int   sz     = (int)PSI_ETA_H;
    float temp   = 0.0F;    

    for(int k = 0; k < sz; k++)
    {
        temp += psi_eta_w0[raw][k]*inp[k];
    }

    
    temp += psi_eta_b0[raw];

    encoderVecs.out0[raw] = temp >= 0 ? temp : 0;
}

static void feedForwardPsiEta1(float* inp, int raw)
{
    int sz        = (int)PSI_ETA_V;
    float temp    = 0.0F;

    for(int k = 0; k < sz; k++)
    {
        temp += psi_eta_w1[raw][k]*inp[k];
    }

    temp += psi_eta_b1[raw];

    encoderVecs.out1[raw] = temp >= 0 ? temp : 0;
}


static void calcMean(float* networkOut)
{
    float scalar;

    if(getAmountKNeighbors() == 0.0)
    {
        scalar  = 1.0;
    }
    else
    {
        scalar = getAmountKNeighbors();
    }

    scaleVec(networkOut, (1.0f)/scalar, getAmountOutputs());
}

static int getAmountKNeighbors()
{
    return (int)KNeighbors;
}

static int  getAmountOutputs()
{
    return (int)NEIGHBOR_NETWORK_OUT;
}

static void addVectors(float* arr1, float* arr2, int len)
{

    /*perform addition of encoder outputs*/
    int vecLen         = len;

    for(int k = 0; k < vecLen; k++)
    {
        arr2[k] = arr1[k] + arr2[k];
    }
}

