#ifndef __NEIGHBOR_ENCOODER_H__
#define __NEIGHBOR_ENCOODER_H__

#include "../include/neighbor_observation.h"
// structure constants
#define NEIGHBOR_NETWORK_OUT  8

#define PSI_E_INP             6
#define PSI_E_OUT             8

#define PSI_H_INP             8
#define PSI_H_OUT             8

#define PSI_ALPHA_INP         2*PSI_E_OUT
#define PSI_ALPHA_HID         PSI_E_OUT
#define PSI_ALPHA_OUT         1

#define K_NEIGHBORS           6


void calcNeighborEncoderOutput(neighb_obs* inp, float* outp);

#endif