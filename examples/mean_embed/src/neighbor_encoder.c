#include "../include/neighbor_encoder.h"
#include "../include/vec.h"

// static variables 
static neighbor_obs  neighbObsArr[K_NEIGHBOURS];          // the neighbor observations 
static float         networkOutput[NEIGHBOR_NETWORK_OUT]; //e_j in the paper
static float         encoderOutput[NEIGHBOR_NETWORK_OUT]; //e_m in the paper
static neighborVecs  neighborVecsArr[K_NEIGHBOURS];       // an array of structs, where each struct 
                                                          // contains the observation array and output array of each neighbor 

static const int   KNeighbors = (int)K_NEIGHBOURS;

// static functions
static void    calcMean(neighbor_obs*);
static void    addVectors(neighbor_obs*);
static float*  getNetworkOutput();
static float*  getEncoderOutput();
static float*  getEjVector(int k);
static void    feedForwardNeighborEncoder();

void calcNeighborEncoderOutput()
{
    int sz = getAmountKNeighbors();
    for(int k = 0; k < sz; k++)
    {
        
    }
}


static void feedForwardNeighborEncoder()
{

    addVectors(obserVec);
    calcMean(getNetworkOutput());
}

static void calcMean(neighbor_obs* obserVec)
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

    scaleVec(meanVec, 1.0/scalar);
}

static float* getNetworkOutput()
{
    return networkOutput;
}

static getAmountKNeighbors()
{
    return KNeighbors;
}

static void addVectors(neighbor_obs* obserVec)
{

    /*perform addition of encoder outputs*/
    float* additionvec = getNetworkOutput();
    int KAdditions     = getAmountKNeighbors();

    for(int k = 0; k < KAdditions; k++)
    {
        addVecf(additionvec, (obserVec + k)->neighb_obs_arr, sizeof(additionvec)/sizeof(additionvec[0]));
    }
}

