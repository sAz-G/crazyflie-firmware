#ifndef __NEIGHBOR_OBSERVATION_H__
#define __NEIGHBOR_OBSERVATION_H__

#include "../include/vec.h"

#define NEIGHBOROBSSIZE 6
#define KNEARESTAMOUNT 6
#define MAXNEIGhBVELX 3
#define MAXNEIGhBVELY 3
#define MAXNEIGhBVELZ 3

#define MAXNEIGHBPOSX 10 
#define MAXNEIGHBPOSY 10
#define MAXNEIGHBPOSZ 10

#define MINNEIGHBVELX -MAXNEIGHBVELX
#define MINNEIGHBVELY -MAXNEIGHBVELY
#define MINNEIGHBVELZ -MAXNEIGHBVELZ

#define MINNEIGHBPOSX -MAXNEIGHBPOSX
#define MINNEIGHBPOSY -MAXNEIGHBPOSY
#define MINNEIGHBPOSZ -MAXNEIGHBPOSZ


typedef struct _NEIGHB_OBS
{
    union 
    {
        struct 
        {
            Vector3 relPos;
            Vector3 relVel;
        };
        Vector3 neighb_obs_arr[NEIGHBOROBSSIZE];
    };

    int obsLen;
    
}neighb_obs;

void updateNeighbObservation(neighb_obs* kNearest);


#endif