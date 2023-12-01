#ifndef __SELF_ENCOODER_H__
#define __SELF_ENCOODER_H__


// structure constant
#define SELF_NETWORK_OUT                      16
#define PSI_S_H                               18
#define PSI_S_V                               16

#define SELF_ENCODER_W                        {{PSI_S_V, PSI_S_H}, {PSI_S_V,PSI_S_V}};
#define SELF_ENCODER_B                        {PSI_S_V, PSI_S_V};

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
    
}self_obs;

typedef struct _SELF_VECS
{
    self_obs* obsStruct;
    float e_s[PSI_S_V];
}selfVecs;


void    calcSelfEncoderOutput(float*);
float*  getSelfOutput();

#endif