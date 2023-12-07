#ifndef __MLP_H__
#define __MLP_H__

#define PHI_A_H 24
#define OUTPUT0 32
#define PHI_A_V 4

typedef struct _INPUTOUTPUT
{
    float out0[OUTPUT0];
    float phi_a[PHI_A_V];
}inpOutVec;

typedef struct MLPINPUT
{
    int len;
    float inputVec[PHI_A_H];

}mlpInput;

void calcMlpOutput(mlpInput* inp , float* out);

#endif