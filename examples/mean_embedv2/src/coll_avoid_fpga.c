
#define maxVelocity 409
#define maxVelocityZNegative 204
#define maxVelocityZ 819
#define maxYaw 348160
#define SHIFT 12

//using 12 fractional bits

#include <stdio.h>
#include <math.h>
#include "../include/iic/iic_functionality.h"



int relu(int num) 
{
	if (num > 0) {
		return num;
	} else {
		return 0;
	}
}

int clip(int num, int min, int max)
{
	if (num < min){num = min;}
	else if (num > max){num = max;}
	return num;
}

int tanh_approx(int in){
	//Tanh-Approximation
	int output;
	if(in < -(15 << (SHIFT-3))) // in kleiner -1,875
		output = -(1 << SHIFT); // output = -1
	else if (in > (15 << (SHIFT-3))) // in größer 1,875
		output = 1 << SHIFT; // output = 1
  	else if (in < -(9 << (SHIFT-3))) // in kleiner -1,125
  	      output = in/4  - (17 << (SHIFT-5));  // output = x/4 - 17/32
   	else if (in > (9 << (SHIFT-3))) // in größer 1,125
       	output = in/4 + (17 << (SHIFT-5)); // output = x/4 + 17/32
	else if(in < -(1 << (SHIFT-1))) // in kleiner -0,5
		output = (in >> 1) - (1 << (SHIFT-2));  // output = in/2 - 0,25
	else if(in > (1 << (SHIFT-1))) // in größer 0,5
		output = (in >> 1) + (1 << (SHIFT-2));  // output = in/2 + 0,25
	else output = in; // In der Mitte: ouput = in
	return output;
}
















void networkEvaluate(const float* state_array);
int main_nn(float *outdatav);

#define NGHBRSDIVISOR 512
#define NGHBRS 6
#define NGHBRSDIM 6 
#define NDRNS 8

static const int structure [6][2] = {{6, 8},{8, 8},{18, 16},{16, 16},{24, 32},{32, 4}};
static const int NEIGHBORS  = NGHBRS;
static const int NBR_DIM    = NGHBRSDIM; 
static const int NUMQUADS   = NDRNS; 

static int dronesInfo[NDRNS][NGHBRSDIM];
static int kNearestArr[NGHBRS][NGHBRSDIM];

static int output_0[8];
static int output_1[8];
static int output_2[16];
static int output_3[16];
static int inputff[24];
static int output_4[32];
static int output_5[4];

static const int actor_encoder_neighbor_encoder_embedding_mlp_0_weight[6][8];
static const int actor_encoder_neighbor_encoder_embedding_mlp_2_weight[8][8];
static const int actor_encoder_self_encoder_0_weight[18][16];                
static const int actor_encoder_self_encoder_2_weight[16][16];                
static const int actor_encoder_feed_forward_0_weight[24][32];                
static const int action_parameterization_distribution_linear_weight[32][4];  
static const int actor_encoder_neighbor_encoder_embedding_mlp_0_bias[8];     
static const int actor_encoder_neighbor_encoder_embedding_mlp_2_bias[8];     
static const int actor_encoder_self_encoder_0_bias[16];                      
static const int actor_encoder_self_encoder_2_bias[16];                      
static const int actor_encoder_feed_forward_0_bias[32];                      
static const int action_parameterization_distribution_linear_bias[4];        

void networkEvaluate(const float *state_array) {
      ///////////////////////////////// NEIGHBOR OUTPUT CALCULATION ///////////////////////////////////////////
      int meanNeighbArr[8];
      meanNeighbArr[0] = 0;
      meanNeighbArr[1] = 0;
      meanNeighbArr[2] = 0;
      meanNeighbArr[3] = 0;
      meanNeighbArr[4] = 0;
      meanNeighbArr[5] = 0;
      meanNeighbArr[6] = 0;
      meanNeighbArr[7] = 0;

        for(int k = 0; k < NEIGHBORS; k++)
        {
          for (int i = 0; i < structure[0][1]; i++) {
              output_0[i] = 0;
              for (int j = 0; j < structure[0][0]; j++) {
                output_0[i] += (kNearestArr[k][j] * actor_encoder_neighbor_encoder_embedding_mlp_0_weight[j][i])>> SHIFT;
              }
              output_0[i] += actor_encoder_neighbor_encoder_embedding_mlp_0_bias[i];
              output_0[i] = tanh_approx(output_0[i]);
          }
      
          for (int i = 0; i < structure[1][1]; i++) {
              output_1[i] = 0;
              for (int j = 0; j < structure[1][0]; j++) {
                  output_1[i] += (output_0[j] * actor_encoder_neighbor_encoder_embedding_mlp_2_weight[j][i]) >> SHIFT;
              }
              output_1[i] += actor_encoder_neighbor_encoder_embedding_mlp_2_bias[i];
              output_1[i] = tanh_approx(output_1[i]);
          }


          meanNeighbArr[0] = meanNeighbArr[0]+output_1[0];
          meanNeighbArr[1] = meanNeighbArr[1]+output_1[1];
          meanNeighbArr[2] = meanNeighbArr[2]+output_1[2];
          meanNeighbArr[3] = meanNeighbArr[3]+output_1[3];
          meanNeighbArr[4] = meanNeighbArr[4]+output_1[4];
          meanNeighbArr[5] = meanNeighbArr[5]+output_1[5];
          meanNeighbArr[6] = meanNeighbArr[6]+output_1[6];
          meanNeighbArr[7] = meanNeighbArr[7]+output_1[7];
        } 

        meanNeighbArr[0] = (meanNeighbArr[0]*NGHBRSDIVISOR) >> SHIFT;
        meanNeighbArr[1] = (meanNeighbArr[1]*NGHBRSDIVISOR) >> SHIFT;
        meanNeighbArr[2] = (meanNeighbArr[2]*NGHBRSDIVISOR) >> SHIFT;
        meanNeighbArr[3] = (meanNeighbArr[3]*NGHBRSDIVISOR) >> SHIFT;
        meanNeighbArr[4] = (meanNeighbArr[4]*NGHBRSDIVISOR) >> SHIFT;
        meanNeighbArr[5] = (meanNeighbArr[5]*NGHBRSDIVISOR) >> SHIFT;
        meanNeighbArr[6] = (meanNeighbArr[6]*NGHBRSDIVISOR) >> SHIFT;
        meanNeighbArr[7] = (meanNeighbArr[7]*NGHBRSDIVISOR) >> SHIFT;

          /////////////////// SELF_OUTPUT ///////////////////////////////////////////////////////////////////
          for (int i = 0; i < structure[2][1]; i++) {
              output_2[i] = 0;
              for (int j = 0; j < structure[2][0]; j++) {
                  output_2[i] += (state_array[j] * actor_encoder_self_encoder_0_weight[j][i])>> SHIFT;
              }
              output_2[i] += actor_encoder_self_encoder_0_bias[i];
              output_2[i] = tanh_approx(output_2[i]);

          }
          
          for (int i = 0; i < structure[3][1]; i++) {
              output_3[i] = 0;
              for (int j = 0; j < structure[3][0]; j++) {
                  output_3[i] += (output_2[j] * actor_encoder_self_encoder_2_weight[j][i]) >> SHIFT;
              }
              output_3[i] += actor_encoder_self_encoder_2_bias[i];
              output_3[i] = tanh_approx(output_3[i]);

          }
          

          ////////////////////////////////////////////////////// FEED FORWARD /////////////////////////////////////////////

          for(int k = 0; k < structure[3][1]; k++ )
          {
            inputff[k] = output_3[k];
          }

          for(int k = 0; k < structure[1][1]; k++ )
          {
            inputff[k+16] = meanNeighbArr[k];
          }

          for (int i = 0; i < structure[4][1]; i++) {
              output_4[i] = 0;
              for (int j = 0; j < structure[4][0]; j++) {
                  output_4[i] += (inputff[j] * actor_encoder_feed_forward_0_weight[j][i]) >> SHIFT;
              }
              output_4[i] += actor_encoder_feed_forward_0_bias[i];
              output_4[i] = tanh_approx(output_4[i]);
          }
          
          for (int i = 0; i < structure[5][1]; i++) {
              output_5[i] = 0;
              for (int j = 0; j < structure[5][0]; j++) {
                  output_5[i] += (output_4[j] * action_parameterization_distribution_linear_weight[j][i]) >> SHIFT;
              }
              output_5[i] += action_parameterization_distribution_linear_bias[i];
          }
    }













































int
main ()
{
	// Initialize IIC
	int Status;
	int BytesToReceive;
	Status = IicInit();
	if (Status != XST_SUCCESS) {
		//xil_printf("IIC initialization Failed\r\n");
		return XST_FAILURE;
	}
	
	while (1){ // Loop network calculation
		BytesToReceive = 20;
		Status = SlaveReadData(ReadBuffer, BytesToReceive); // wait for and receive input data from master 
		if (Status != XST_SUCCESS){
			//xil_printf("Receive Failed");
			return XST_FAILURE;
		}
		disable_acknowledge();

		for (int i = 0; i < 10; i++){
			// only the lower two bytes contain information due to the fact that 12 Bits fix point arthmethic is used
			// and the input values are between -1 and 1. The upper two bytes have to be set or not depending on the sign
			state_array[i] = 0x00000000;
			state_array[i] = (0x00000000 | ReadBuffer[2*i]) | ((0x00000000 | ReadBuffer[2*i+1]) << 8);
			if (ReadBuffer[2*i+1] >= 128)
				state_array[i] |= 0xFFFF0000;
			
		}
		networkEvaluate();

		for (int j = 0; j < 3; j++){
			// only the lower two/three bytes contain information due to the fact that 12 Bits fix point arthmethic is used
			// and the largest output is at most 85. The first two values are smaller and only need 2 Bytes.
			WriteBuffer[2*j] = (u8) (output_2[j] & 0xFF);
			WriteBuffer[2*j+1] = (u8) ((output_2[j]>>8) & 0xFF);
			if (j == 2) // third value may need 3 bytes
				WriteBuffer[2*j+2] = (u8) ((output_2[j]>>16) & 0xFF);
		}
		enable_acknowledge();
		Status = SlaveWriteData(7); // wait for master to ask for output data and send them
		if (Status != XST_SUCCESS) {
			//xil_printf("Send Failed\r\n");
			return XST_FAILURE;
		}
	
	}

	return 0;
}
