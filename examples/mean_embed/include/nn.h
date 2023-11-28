#ifndef __NN_H__
#define __NN_H__

#define K_NEIGHBOURS                            6
#define N_DRONES                                8
#define SELF_ENCODER_W                          {{16, 18}, {16,16}}
#define SELF_ENCODER_B                          {16, 16}
#define NEIGHB_ENCODER_W                        {{8, 6}, {8,8}}
#define NEIGHB_ENCODER_B                        {8, 8}
#define MLP_W                                   {{32, 24}, {32,32}}
#define MLP_B                                   {4, 32}


#define N_IN        OBSERVATION_DIM
#define N_OUT       OUTPUT_DIM
#define PHI_S_H     SELF_ENCODER_HORIZONTAL_DIM
#define PHI_S_V     SELF_ENCODER_VERTICAL_DIM
#define PSI_ETA_H   NEIGHBOUR_ENCODER_HORIZONTAL_DIM
#define PSI_ETA_V   NEIGHBOUR_ENCODER_VERTICAL_DIM
#define FF_V        FEED_FORWARD_VERTICAL_DIM
#define FF_H        FEED_FORWARD_HORIZONTAL_DIM



#endif