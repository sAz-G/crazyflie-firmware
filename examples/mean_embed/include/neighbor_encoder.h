#ifndef __NEIGHBOR_ENCOODER_H__
#define __NEIGHBOR_ENCOODER_H__


#include "vec.h"

// structure constants
#define NEIGHBOR_NETWORK_OUT                    8
#define PSI_ETA_H                               6
#define PSI_ETA_V                               8

#define K_NEIGHBOURS                            6
#define N_DRONES                                8
#define NEIGHB_ENCODER_W                        {{PSI_ETA_V, PSI_ETA_H}, {PSI_ETA_V,PSI_ETA_V}};
#define NEIGHB_ENCODER_B                        {PSI_ETA_V, PSI_ETA_V};


typedef struct _NEIGHB_OBS
{
    union 
    {
        struct 
        {
            float px;
            float py;
            float pz;
            float vx;
            float vy;
            float vz;
        };
        float neighb_obs_arr[6*sizeof(float)];
    };
    
}neighbor_obs;

typedef struct _NEIGHBOR_VECS
{
    neighbor_obs* obsStruct;
    float* e_j;
    
}neighborVecs;


void calcNeighborEncoderOutput();

#endif