#ifndef __NEIGHBOR_ENCOODER_H__
#define __NEIGHBOR_ENCOODER_H__

#include "../include/neighbor_observation.h"
// structure constants
#define NEIGHBOR_NETWORK_OUT                    8
#define PSI_ETA_H                               6
#define PSI_ETA_V                               8

#define K_NEIGHBORS                             6
#define NEIGHB_ENCODER_W                        {{PSI_ETA_V, PSI_ETA_H}, {PSI_ETA_V,PSI_ETA_V}};
#define NEIGHB_ENCODER_B                        {PSI_ETA_V, PSI_ETA_V};



void calcNeighborEncoderOutput(neighb_obs* inp, float* outp);

#endif