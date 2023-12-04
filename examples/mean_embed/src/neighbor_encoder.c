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

static const float psi_eta_w0[PSI_ETA_H][PSI_ETA_V];
static const float psi_eta_w1[PSI_ETA_V][PSI_ETA_V];

static const float psi_eta_b0[PSI_ETA_V];
static const float psi_eta_b1[PSI_ETA_V];

//static neighbor_obs  neighbObsArr[K_NEIGHBORS];          // the neighbor observations 
                                                      // contains the observation array and output array of each neighbor 
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

    for(int k = 0; k < sz; k++)
    {
        temp += psi_eta_b0[k];
    }

    encoderVecs.out0[raw] = temp;
}

static void feedForwardPsiEta1(float* inp, int raw)
{
    int sz        = (int)PSI_ETA_V;
    float temp    = 0.0F;

    for(int k = 0; k < sz; k++)
    {
        temp += psi_eta_w1[raw][k]*inp[k];
    }

    for(int k = 0; k < sz; k++)
    {
        temp += psi_eta_b1[k];
    }

    encoderVecs.out1[raw] = temp;
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

