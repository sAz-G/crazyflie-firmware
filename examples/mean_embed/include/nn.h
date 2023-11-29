#ifndef __NN_H__
#define __NN_H__

#include "vec.h"

// structure constants
#define N_OUT         4
#define PHI_S_H       18
#define PHI_S_V       16
#define PSI_ETA_H     6
#define PSI_ETA_V     8
#define FF_H_0        24
#define FF_V_0        32
#define FF_H_1        32
#define FF_V_1        4

#define K_NEIGHBOURS                            6
#define N_DRONES                                8
#define SELF_ENCODER_W                          {{PHI_S_V, PHI_S_H}, {PHI_S_V,PHI_S_V}};
#define SELF_ENCODER_B                          {PHI_S_V, PHI_S_V};
#define NEIGHB_ENCODER_W                        {{PSI_ETA_V, PSI_ETA_H}, {PSI_ETA_V,PSI_ETA_V}};
#define NEIGHB_ENCODER_B                        {PSI_ETA_V, PSI_ETA_V};
#define MLP_W                                   {{FF_V_0, FF_H_0}, {N_OUT,FF_H_1}};
#define MLP_B                                   {FF_H_1, N_OUT};


/* can be used for attention mechanism
typedef struct weights
{
    union 
    {
        struct 
        {
            float alpha1;
            float alpha2;
            float alpha3;
            float alpha4;
            float alpha5;
            float alpha6;
        };
        float softmaxWeights[6*sizeof(float)];
    };
    
}softmax_w;*/

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

typedef struct _SELF_OBS
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

            float r1;
            float r2;
            float r3;
            float r4;
            float r5;
            float r6;
            float r7;
            float r8;
            float r9;

            float wx;
            float wy;
            float wz;
        };
        float self_obs_arr[18*sizeof(float)];
    };
    
}self_obs;


void calcMeanEmbed();
void getMeanEmbed();




#endif