#include "../include/neighbor_encoder.h"
#include "../include/vec.h"
#include "../include/neighbor_observation.h"


// static variables 
//////////////////////////////////////// WEIGHTS /////////////////////////////////////////////////
static const float psi_e_w0[PSI_E_OUT][PSI_E_INP];
static const float psi_e_b0[PSI_E_OUT];
static const float psi_e_w1[PSI_E_OUT][PSI_E_OUT];
static const float psi_e_b1[PSI_E_OUT];


static const float psi_h_w0[PSI_H_OUT][PSI_ALPHA_INP];
static const float psi_h_b0[PSI_H_OUT];
static const float psi_h_w1[PSI_H_OUT][PSI_H_OUT];
static const float psi_h_b1[PSI_H_OUT];


static const float psi_alpha_w0[PSI_ALPHA_HID][PSI_ALPHA_INP];
static const float psi_alpha_b0[PSI_ALPHA_HID];
static const float psi_alpha_w1[PSI_ALPHA_HID][PSI_ALPHA_HID];
static const float psi_alpha_b1[PSI_ALPHA_HID];
static const float psi_alpha_w2[PSI_ALPHA_HID];
static const float psi_alpha_b2;


static float inp[PSI_E_INP];
static float outE0[PSI_E_OUT];
static float outE1[K_NEIGHBORS][PSI_E_OUT];

static float outH0[PSI_H_OUT];
static float outH1[PSI_H_OUT];

static float outAlpha0[PSI_ALPHA_HID];
static float outAlpha1[PSI_ALPHA_HID];
static float outAlphaVec[K_NEIGHBORS];

static float outMean[PSI_E_OUT];


static const int   KNeighbors = K_NEIGHBORS;

// static functions
static void         calcMean();
static void         feedForwardNeighborEncoder(neighb_obs* inp, float* outp);

static void         feedForwardPsiE(neighb_obs obsK, int neighb);
static void         feedForwardPsiE0(float* inp, int raw);
static void         feedForwardPsiE1(float* inp, int raw, int neighb);

static void         feedForwardPsiH(float*);
static void         feedForwardPsiH0(float* inp, int raw);
static void         feedForwardPsiH1(float* inp, int raw);

static void         feedForwardPsiAlpha(float*, int);
static void         feedForwardPsiAlpha0(float* inp, int raw);
static void         feedForwardPsiAlpha1(float* inp, int raw);
static void         feedForwardPsiAlpha2(float* inp, int raw);

static void         addVectors(float* arr1, float* arr2, int len);


//feed forward functions for psi_eta, adapted for the FPGA
//feed forward variables for psi_eta, adapted for the FPGA

void calcNeighborEncoderOutput(neighb_obs* inp, float* outp)
{
    for(int k = 0; k < PSI_E_OUT; k++)
    {
        outE0[k] = 0.0f;

        outE1[0][k] = 0.0f;
        outE1[1][k] = 0.0f;
        outE1[2][k] = 0.0f;
        outE1[3][k] = 0.0f;
        outE1[4][k] = 0.0f;
        outE1[5][k] = 0.0f;

        outH0[k] = 0.0f;
        outH1[k] = 0.0f;

        outAlpha0[k] = 0.0f;
        outAlpha1[k] = 0.0f;

        outp[k] = 0.0f;
    }

    for(int u = 0; u < K_NEIGHBORS; u++)
    {
        outAlphaVec[u] = 0.0f;
    }

    feedForwardNeighborEncoder(inp, outp);
}

static void feedForwardNeighborEncoder(neighb_obs* inp, float* outpEncoder)
{
    for(int k = 0; k < K_NEIGHBORS; k++)
    {    // calc e output 
        feedForwardPsiE(inp[k], k);    
    }   

    calcMean(outpEncoder); // calc mean

    for(int k = 0; k < K_NEIGHBORS; k++)
    { // calc alpha output
        feedForwardPsiAlpha(outE1[k], k);    
    }   

    for(int k = 0; k < K_NEIGHBORS; k++)
    { // calc h output and final output
        feedForwardPsiH(outE1[k]);  

        for(int u = 0; u < PSI_H_OUT; u++)
        {
            outpEncoder[u] = outpEncoder[u] + outAlphaVec[k]*outH1[u];
        }  
    }   
}

static void feedForwardPsiE(neighb_obs obsK, int neighb)
{

    inp[0] = obsK.relPos.x;
    inp[1] = obsK.relPos.y;
    inp[2] = obsK.relPos.z;
    inp[3] = obsK.relVel.x;
    inp[4] = obsK.relVel.y;
    inp[5] = obsK.relVel.z;

    for(int k = 0; k < PSI_E_OUT; k++)
    {
        feedForwardPsiE0(inp, k);
    }

    for(int k = 0; k < PSI_E_OUT; k++)
    {
        feedForwardPsiE1(outE0, k, neighb);
    }

}

static void feedForwardPsiE0(float* inp, int raw)
{
    float temp   = 0.0F;    
    for(int k = 0; k < PSI_E_OUT; k++)
    {
        temp += psi_e_w0[raw][k]*inp[k];
    }

    
    temp += psi_e_b0[raw];

    outE0[raw] = temp >= 0 ? temp : 0;
}

static void feedForwardPsiE1(float* inp, int raw, int neighb)
{
    float temp    = 0.0F;

    for(int k = 0; k < PSI_E_OUT; k++)
    {
        temp += psi_e_w1[raw][k]*inp[k];
    }

    temp += psi_e_b1[raw];
    outE1[neighb][raw] = temp >= 0 ? temp : 0;
}

static void feedForwardPsiAlpha(float* inp, int kNeighb)
{
    float inpAlpha[PSI_ALPHA_INP];

    inpAlpha[0] = inp[0];
    inpAlpha[1] = inp[1];
    inpAlpha[2] = inp[2];
    inpAlpha[3] = inp[3];
    inpAlpha[4] = inp[4];
    inpAlpha[5] = inp[5];
    inpAlpha[6] = inp[6];
    inpAlpha[7] = inp[7];

    inpAlpha[8] = outMean[0];
    inpAlpha[9] = outMean[1];
    inpAlpha[10] = outMean[2];
    inpAlpha[11] = outMean[3];
    inpAlpha[12] = outMean[4];
    inpAlpha[13] = outMean[5];
    inpAlpha[14] = outMean[6];
    inpAlpha[15] = outMean[7];

    for(int k = 0; k < PSI_ALPHA_HID; k++)
    {
        feedForwardPsiAlpha0(inpAlpha, k);
    }

    for(int u = 0; u < PSI_ALPHA_HID; u++)
    {
        feedForwardPsiAlpha1(outAlpha0, u);
    }

    feedForwardPsiAlpha2(outAlpha1, kNeighb);
}

static void feedForwardPsiAlpha0(float* inp, int raw)
{
    float tmp = 0.0f;

    for(int k = 0; k < PSI_ALPHA_INP; k++)
    {
        tmp += psi_alpha_w0[raw][k]*inp[k];
    }
    tmp += psi_alpha_b0[raw];
    outAlpha0[raw] = tmp >= 0 ? tmp : 0;
}

static void feedForwardPsiAlpha1(float* inp, int raw)
{
    float tmp = 0.0f;

    for(int k = 0; k < PSI_ALPHA_INP; k++)
    {
        tmp += psi_alpha_w1[raw][k]*inp[k];
    }
    tmp += psi_alpha_b1[raw];
    outAlpha1[raw] = tmp >= 0 ? tmp : 0;
}

static void feedForwardPsiAlpha2(float* inp, int raw)
{
    float tmp = 0.0f;

    for(int k = 0; k < PSI_ALPHA_INP; k++)
    {
        tmp += psi_alpha_w2[k]*inp[k];
    }
    tmp += psi_alpha_b2;
    outAlphaVec[raw] = tmp >= 0 ? tmp : 0;
}

static void feedForwardPsiH(float* inp)
{
    for(int k = 0; k < PSI_H_OUT; k++)
    {
        feedForwardPsiH0(inp, k);
    }

    for(int k = 0; k < PSI_H_OUT; k++)
    {
        feedForwardPsiH1(outH0, k);
    }
}

static void feedForwardPsiH0(float* inp, int raw)
{
    float tmp = 0.0f;

    for(int k = 0; k < PSI_H_INP; k++)
    {
        tmp += psi_h_w0[raw][k]*inp[k];
    }

    tmp += psi_h_b0[raw];
    outH0[raw] = tmp >= 0 ? tmp : 0;
}

static void feedForwardPsiH1(float* inp, int raw)
{
    float tmp = 0.0f;

    for(int k = 0; k < PSI_H_OUT; k++)
    {
        tmp += psi_h_w1[raw][k]*inp[k];
    }

    tmp += psi_h_b1[raw];
    outH1[raw] = tmp >= 0 ? tmp : 0;
}

static void calcMean()
{
    for(int k = 0; k < K_NEIGHBORS; k++)
    {
        addVectors(outE1[k], outMean, PSI_E_OUT);
    }

    for(int k = 0; k < K_NEIGHBORS; k++)
    {
        outMean[k] = outMean[k]/((float)K_NEIGHBORS);
    }
}


static void addVectors(float* arr1, float* arr2, int len)
{

    /*perform addition of encoder outputs*/
    int vecLen         = len;

    for(int k = 0; k < vecLen; k++)
    {
        arr2[k] = arr1[k] + arr2[k];
    }
}

