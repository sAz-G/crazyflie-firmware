#include "../include/self_encoder.h"

static const float psi_s_w0[PSI_S_H][PSI_S_V];
static const float psi_s_w1[PSI_S_V][PSI_S_V];

static const float psi_s_b0[PSI_S_V];
static const float psi_s_b1[PSI_S_V];


static selfVecs*  inputOutputVectors;       
//static float      encoderOutput[SELF_NETWORK_OUT]; // e_s in the paper

/*
functions
*/

// getters
static selfVecs* getInputOutputVectors();
static float*    getEncoderOutput();
static float*    getEncoderInput();
static float*    getSelfObservationVector();
static void      updateSelfInfo();


static void    feedForwardSelfEncoder();
static void    feedForwardPsiS();
static void    feedForwardPsiS0(int k);
static void    feedForwardPsiS1(int k);
static void    layer0();
static void    layer1();

static float out0[PSI_S_V];


static void    feedForwardSelfEncoder()
{
    updateSelfInfo();
    feedForwardPsiS();

}

static void updateSelfInfo()
{
    return;
}

static void  feedForwardPsiS()
{
    layer0();

}

static void layer0()
{

}
