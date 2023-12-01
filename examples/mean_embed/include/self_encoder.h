#ifndef __SELF_ENCOODER_H__
#define __SELF_ENCOODER_H__


// structure constant
#define SELF_NETWORK_OUT                      16
#define PSI_S_H                               18
#define PSI_S_V                               16

void    calcSelfEncoderOutput(float*, float*);
float*  getSelfOutput();

#endif