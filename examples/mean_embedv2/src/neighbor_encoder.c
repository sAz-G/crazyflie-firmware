#include "../include/neighbor_encoder.h"
#include "../include/vec.h"
#include "../include/neighbor_observation.h"


// static variables 
//////////////////////////////////////// WEIGHTS /////////////////////////////////////////////////


static const float psi_eta_w0[PSI_ETA_V][PSI_ETA_H] = 
{
{
0.5416633486747742 ,
-0.6793858408927917 ,
-0.6420764327049255 ,
0.09204913675785065 ,
-0.23749549686908722 ,
-0.13850753009319305 ,
},
{
0.5207025408744812 ,
-0.7075570225715637 ,
-0.4285285174846649 ,
-0.1277514547109604 ,
0.33927252888679504 ,
-0.1915159970521927 ,
},
{
0.4606701135635376 ,
-0.6019311547279358 ,
-0.37280139327049255 ,
-0.3767601549625397 ,
0.45328933000564575 ,
0.25898477435112 ,
},
{
0.541625440120697 ,
-0.5736227035522461 ,
0.08217182755470276 ,
-0.44940185546875 ,
0.4473400115966797 ,
0.005928817205131054 ,
},
{
-0.030595339834690094 ,
-0.13287974894046783 ,
0.13821372389793396 ,
0.16801367700099945 ,
-0.03746205195784569 ,
-0.34477144479751587 ,
},
{
-0.41178369522094727 ,
0.32565295696258545 ,
0.26669827103614807 ,
-0.4095054864883423 ,
-0.09976258128881454 ,
-0.21939535439014435 ,
},
{
0.8232560157775879 ,
-0.19622144103050232 ,
0.1271458864212036 ,
-0.15770114958286285 ,
-0.26105400919914246 ,
0.266299307346344 ,
},
{
0.3432365655899048 ,
-0.3765560984611511 ,
-0.7889770269393921 ,
0.08556654304265976 ,
0.2209974229335785 ,
0.2198306918144226 ,
},
};



static const float psi_eta_w1[PSI_ETA_V][PSI_ETA_V] = {
{
-0.03251625597476959 ,
-0.38134801387786865 ,
0.24380429089069366 ,
-0.3070105314254761 ,
-0.17432427406311035 ,
0.3579081594944 ,
0.29315465688705444 ,
-0.13260476291179657 ,
},
{
0.0012665789108723402 ,
0.22348414361476898 ,
0.3389277458190918 ,
-0.6072970628738403 ,
0.3429899215698242 ,
-0.6856699585914612 ,
-1.1740769147872925 ,
-0.33564692735671997 ,
},
{
-0.8452470302581787 ,
-0.6054815053939819 ,
-0.5954283475875854 ,
0.04033582657575607 ,
-0.2122931331396103 ,
-0.270844042301178 ,
-0.20248886942863464 ,
0.2681557536125183 ,
},
{
0.3090740144252777 ,
0.14779238402843475 ,
-0.5685044527053833 ,
0.38580095767974854 ,
-0.13173352181911469 ,
0.543065071105957 ,
-0.2370309680700302 ,
-0.5313069820404053 ,
},
{
-0.5916076898574829 ,
0.03200463578104973 ,
0.4022670090198517 ,
0.41339126229286194 ,
-0.022895516827702522 ,
-0.01728454977273941 ,
-0.24415399134159088 ,
-0.4661141037940979 ,
},
{
-0.07894216477870941 ,
0.37273523211479187 ,
0.4588146507740021 ,
-0.29711103439331055 ,
0.5695515871047974 ,
-0.23766610026359558 ,
-0.18596145510673523 ,
-0.8401055932044983 ,
},
{
-0.7719787359237671 ,
-0.5186449885368347 ,
0.5485462546348572 ,
-0.18148839473724365 ,
0.4534667134284973 ,
-0.07974095642566681 ,
-0.07535210251808167 ,
-0.16072696447372437 ,
},
{
-0.8175962567329407 ,
-0.2076832801103592 ,
0.365100234746933 ,
0.1921238899230957 ,
-0.0007198587991297245 ,
0.04554393142461777 ,
-0.17138516902923584 ,
-0.44614166021347046 ,
},
};






static const float psi_eta_b0[PSI_ETA_V] ={
0.09054089337587357 ,
0.15199235081672668 ,
-0.11657754331827164 ,
0.021963154897093773 ,
0.21147382259368896 ,
0.034715186804533005 ,
0.10801771283149719 ,
0.14744867384433746 ,
};








static const float psi_eta_b1[PSI_ETA_V] = {
0.20881527662277222 ,
0.24426570534706116 ,
0.27795931696891785 ,
0.14956460893154144 ,
-0.014420254155993462 ,
0.11806763708591461 ,
0.18756170570850372 ,
0.2690894603729248 ,
};






static float inp[6];
static float out0[(int)NEIGHBOR_NETWORK_OUT];
static float out1[(int)NEIGHBOR_NETWORK_OUT];

static const int   KNeighbors = (int)K_NEIGHBORS;

// static functions
static void         calcMean(float*);
static void         feedForwardNeighborEncoder(neighb_obs* inp, float* outp);
static void         feedForwardPsiEta(neighb_obs obsK);
static void         feedForwardPsiEta0(float* inp, int raw);
static void         feedForwardPsiEta1(float* inp, int raw);
static void         addVectors(float* arr1, float* arr2, int len);


//feed forward functions for psi_eta, adapted for the FPGA
//feed forward variables for psi_eta, adapted for the FPGA

void calcNeighborEncoderOutput(neighb_obs* inp, float* outp)
{
    for(int k = 0; k < NEIGHBOR_NETWORK_OUT; k++)
    {
        out1[k] = 0.0f;
        out0[k] = 0.0f;
        outp[k] = 0.0f;
    }

    feedForwardNeighborEncoder(inp, outp);
}

static void feedForwardNeighborEncoder(neighb_obs* inp, float* outpEncoder)
{
    
    for(int k = 0; k < K_NEIGHBORS; k++)
    {
        feedForwardPsiEta(inp[k]);    
        addVectors(out1,outpEncoder,(int)NEIGHBOR_NETWORK_OUT);
    }
    
    calcMean(outpEncoder);
}

static void feedForwardPsiEta(neighb_obs obsK)
{
    inp[0] = obsK.relPos.x;
    inp[1] = obsK.relPos.y;
    inp[2] = obsK.relPos.z;
    inp[3] = obsK.relVel.x;
    inp[4] = obsK.relVel.y;
    inp[5] = obsK.relVel.z;

    for(int k = 0; k < PSI_ETA_V; k++)
    {
        feedForwardPsiEta0(inp, k);
    }

    for(int k = 0; k < PSI_ETA_V; k++)
    {
        feedForwardPsiEta1(out0, k);
    }

}

static void feedForwardPsiEta0(float* inp, int raw)
{
    float temp   = 0.0F;    

    for(int k = 0; k < PSI_ETA_H; k++)
    {
        temp += psi_eta_w0[raw][k]*inp[k];
    }

    
    temp += psi_eta_b0[raw];

    out0[raw] = temp >= 0 ? temp : 0;
}

static void feedForwardPsiEta1(float* inp, int raw)
{
    float temp    = 0.0F;

    for(int k = 0; k < PSI_ETA_V; k++)
    {
        temp += psi_eta_w1[raw][k]*inp[k];
    }

    temp += psi_eta_b1[raw];
    out1[raw] = temp >= 0 ? temp : 0;
}


static void calcMean(float* networkOut)
{
    scaleVec(networkOut, 1.0f/6.0f, NEIGHBOR_NETWORK_OUT);
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

