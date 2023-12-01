#ifndef __NEIGHBOR_ENCOODER_H__
#define __NEIGHBOR_ENCOODER_H__

// structure constants
#define NEIGHBOR_NETWORK_OUT                    8
#define PSI_ETA_H                               6
#define PSI_ETA_V                               8

#define K_NEIGHBORS                             6
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
        float neighb_obs_arr[PSI_ETA_H];
    };
    
}neighbor_obs;

typedef struct _NEIGHBOR_VECS
{
    neighbor_obs* obsStruct;
    float* e_j;
}neighborVecs;


void calcNeighborEncoderOutput(float*);

#endif