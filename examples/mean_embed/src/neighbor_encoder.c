#include "../include/neighbor_encoder.h"
#include "../include/vec.h"

// static variables 
//////////////////////////////////////// WEIGHTS /////////////////////////////////////////////////

static const float psi_eta_w0[PSI_ETA_H][PSI_ETA_V];
static const float psi_eta_w1[PSI_ETA_V][PSI_ETA_V];

static const float psi_eta_b0[PSI_ETA_V];
static const float psi_eta_b1[PSI_ETA_V];

//static neighbor_obs  neighbObsArr[K_NEIGHBORS];          // the neighbor observations 
static float         networkOutput[NEIGHBOR_NETWORK_OUT]; // e_j in the paper
static float         encoderOutput[NEIGHBOR_NETWORK_OUT]; // e_m in the paper
static neighborVecs  neighborVecsArr[K_NEIGHBORS];       // an array of structs, where each struct 
                                                          // contains the observation array and output array of each neighbor 

static const int   KNeighbors = (int)K_NEIGHBORS;

// static functions
static void    calcMean(float*);
static void    addVectors(neighborVecs*, int len);
static int     getAmountKNeighbors();
static int     getAmountOutputs();
static float*  getNetworkOutput();
static float*  getEncoderOutput();
static float*  getEjVector(int k);
static void    feedForwardNeighborEncoder();
static void    feedForwardPsiEta(int k);
static void    feedForwardPsiEta0(int k, int j);
static void    feedForwardPsiEta1(int k, int j);
static float*  getObservationVector(int j);

//feed forward functions for psi_eta, adapted for the FPGA
//feed forward variables for psi_eta, adapted for the FPGA
static float out0[K_NEIGHBORS][PSI_ETA_V];

void calcNeighborEncoderOutput()
{
    feedForwardNeighborEncoder();
}


static void feedForwardNeighborEncoder()
{
    int sz = getAmountKNeighbors();
    for(int k = 0; k < sz; k++)
    {
        feedForwardPsiEta(k);    
    }
    
    addVectors(neighborVecsArr,getAmountKNeighbors());
    calcMean(getNetworkOutput());
}

static void feedForwardPsiEta(int j)
{
    //float* obs_j = getObservationVector(j);
    int sz       = (int)PSI_ETA_V;
    
    for(int k = 0; k < sz; k++)
    {
        feedForwardPsiEta0(k, j);
    }

    for(int k = 0; k < sz; k++)
    {
        feedForwardPsiEta1(k, j);
    }
}

static void feedForwardPsiEta0(int raw, int j)
{
    int sz        = (int)PSI_ETA_H;
    float* obs_j  = getObservationVector(j);
    float temp = 0.0;    
    for(int k = 0; k < sz; k++)
    {
        temp += psi_eta_w0[raw][k]*obs_j[k];
    }

    for(int k = 0; k < sz; k++)
    {
        temp += psi_eta_b0[k];
    }

    out0[j][raw] = temp;
}

static void feedForwardPsiEta1(int raw, int j)
{
    int sz        = (int)PSI_ETA_V;
    float temp    = 0;

    for(int k = 0; k < sz; k++)
    {
        temp += psi_eta_w1[raw][k]*out0[j][k];
    }

    for(int k = 0; k < sz; k++)
    {
        temp += psi_eta_b1[k];
    }

    float* out1_j = getEjVector(j);
    out1_j[raw] = temp;
}

static float*  getObservationVector(int j)
{
    return neighborVecsArr[j].obsStruct->neighb_obs_arr;
}

static float*  getEjVector(int j)
{
    return neighborVecsArr[j].e_j;
}

static void calcMean(float* networkOut)
{
    float scalar;
    float* meanVec = getEncoderOutput();

    if(getAmountKNeighbors() == 0.0)
    {
        scalar  = 1.0;
    }
    else
    {
        scalar = getAmountKNeighbors();
    }

    scaleVec(meanVec, (1.0f)/scalar, getAmountOutputs());
}

static float* getEncoderOutput()
{
    return encoderOutput;
}

static float* getNetworkOutput()
{
    return networkOutput;
}

static int getAmountKNeighbors()
{
    return (int)KNeighbors;
}

static int  getAmountOutputs()
{
    return (int)NEIGHBOR_NETWORK_OUT;
}

static void addVectors(neighborVecs* arr, int len)
{

    /*perform addition of encoder outputs*/
    float* additionvec = getNetworkOutput();
    int KAdditions     = getAmountKNeighbors();
    int vecLen         = len;

    for(int k = 0; k < vecLen; k++)
    {
        additionvec[k] = 0.0;
    }

    for(int k = 0; k < KAdditions; k++)
    {
        addVecf(additionvec, (arr + k)->e_j, vecLen);
    }
}


