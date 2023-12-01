#include "../include/nn.h"
#include "../include/mlp.h"
#include "../include/neighbor_encoder.h"
#include "../include/self_encoder.h"


static void   updateSelfInput();
static void   updateNeighborInput();
static float* getNnSelfInput();
static float* getNnNeighborInput();
static float* getNnOutput();
static void   setNnInput(float*);
static void   setNnOutput(float*);

static nnInputOutput* nnIO;

void feedForwardNN()
{
    updateSelfInput();
    calcSelfEncoderOutput(getNnSelfInput());
    updateNeighborInput();
    calcNeighborEncoderOutput(getNnNeighborInput());
}
