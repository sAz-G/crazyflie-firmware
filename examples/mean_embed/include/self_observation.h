#ifndef __SELF_OBSERVATION_H__
#define __SELF_OBSERVATION_H__
#include "../include/self_encoder.h"

#define SELFOBSERVATIONSIZE PSI_S_H

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
        float self_obs_arr[PSI_S_H];
    };

    int obsLen;
    
}self_obs;

// typedef struct _SELF_VECS
// {
//     self_obs* obsStruct;
//     float e_s[PSI_S_V];
// }selfVecs;

void updateSelfObservation(self_obs*);

#endif