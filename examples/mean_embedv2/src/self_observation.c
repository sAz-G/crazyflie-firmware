#include "../include/self_observation.h"
#include "../include/vec.h"
#include "estimator_kalman.h"
#include "stabilizer_types.h"
#include "stabilizer.h"
#include "../include/vec.h"
#include "param.h"
#include "log.h"
#include <math.h>

/* min max values
 EnvInfo(obs_space=Dict('obs': Box([-10. -10. -10.  -3.  -3.  -3.  -1.  -1.  -1.  -1.  -1.  -1.  -1.  -1.
  -1. -40. -40. -40. -10. -10. -10.  -6.  -6.  -6. -10. -10. -10.  -6.
  -6.  -6. -10. -10. -10.  -6.  -6.  -6. -10. -10. -10.  -6.  -6.  -6.
 -10. -10. -10.  -6.  -6.  -6. -10. -10. -10.  -6.  -6.  -6.], [10. 10. 10.  3.  3.  3.  1.  1.  1.  1.  1.  1.  1.  1.  1. 40. 40. 40.
 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6.
 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6.]

*/
//static const int selfObservationSize = (int)SELFOBSERVATIONSIZE;


// constants variables
struct selfObservationLimit
{
    float maxVelx;
    float maxVely;
    float maxVelz;

    float maxPosx;
    float maxPosy;
    float maxPosz;

    float maxR1;
    float maxR2;
    float maxR3;
    float maxR4;
    float maxR5;
    float maxR6;
    float maxR7;
    float maxR8;
    float maxR9;

    float maxW1;
    float maxW2;
    float maxW3;


    float minVelx;
    float minVely;
    float minVelz;

    float minPosx;
    float minPosy;
    float minPosz;

    float minR1;
    float minR2;
    float minR3;
    float minR4;
    float minR5;
    float minR6;
    float minR7;
    float minR8;
    float minR9;

    float minW1;
    float minW2;
    float minW3;
}obsLimit 
= 
{ (float) MAXVELX,
  (float) MAXVELY,
  (float) MAXVELZ,

  (float) MAXPOSX,
  (float) MAXPOSY,
  (float) MAXPOSZ,

  1.0f,
  1.0f,
  1.0f,
  1.0f,
  1.0f,
  1.0f,
  1.0f,
  1.0f,
  1.0f,

  (float)MAXW1,
  (float)MAXW2,
  (float)MAXW3,
  
  (float)MINVELX,
  (float)MINVELY,
  (float)MINVELZ,

  (float)MINPOSX,
  (float)MINPOSY,
  (float)MINPOSZ,

  -1.0f,
  -1.0f,
  -1.0f,
  -1.0f,
  -1.0f,
  -1.0f,
  -1.0f,
  -1.0f,
  -1.0f,

  (float)MINW1,
  (float)MINW2,
  (float)MINW3
  };

//static self_obs  selfObservation;
static Vector3 targetPos = {.x = 0.f, .y = 0.f , .z= 1.0f};
static void clipOrientation(float*);
static void clipAngularVel(Vector3*);


void updateSelfObservation(float* slfObs)
{// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYy
    // get position  
    Vector3 ownPos  = getPosition();
    targetPos.x     = ownPos.x;
    targetPos.y     = ownPos.y;

    clipPosition(&ownPos);

    Vector3 ownVel = getVelocity();
    clipVelocity(&ownVel);

    Vector3 ownAngVel = getAngularVelocity();
    clipAngularVel(&ownAngVel);

    /*
    [[1.0 - 2 * qy ** 2 - 2 * qz ** 2,     2 * qx * qy - 2 * qz * qw,             2 * qx * qz + 2 * qy * qw],
    [2 * qx * qy + 2 * qz * qw,            1.0 - 2 * qx ** 2 - 2 * qz ** 2,       2 * qy * qz - 2 * qx * qw],
    [2 * qx * qz - 2 * qy * qw,            2 * qy * qz + 2 * qx * qw,             1.0 - 2 * qx ** 2 - 2 * qy ** 2]]
    */

    float rotationMat[9];

    float qx = getqx();
    float qy = getqy();
    float qz = getqz();
    float qw = getqw();

    rotationMat[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;
    rotationMat[1] = 2.0f * qx * qy - 2.0f * qz * qw;
    rotationMat[2] = 2.0f * qx * qz + 2.0f * qy * qw;

    rotationMat[3] = 2.0f * qx * qy + 2.0f * qz * qw;
    rotationMat[4] = 1.0f - 2.0f * qx*qx - 2.0f * qz*qz;
    rotationMat[5] = 2.0f * qy * qz - 2.0f * qx * qw;

    rotationMat[6] = 2.0f * qx * qz - 2.0f * qy * qw;
    rotationMat[7] = 2.0f * qy * qz + 2.0f * qx * qw;
    rotationMat[8] = 1.0f - 2.0f * qx*qx - 2.0f * qy *qy;

/*
    float alpha = getRoll(); // alpha
    float beta = -getPitch(); // beta, bitcraze representation is different than the standard
    float gamma = getYaw(); // gamma

    rotationMat[0] =  cosf(beta)*cosf(gamma);
    rotationMat[1] =  cosf(beta)*sinf(gamma);
    rotationMat[2] = -sinf(beta);

    rotationMat[3] = sinf(alpha)*sinf(beta)*cosf(gamma) - cosf(alpha)*sinf(gamma);
    rotationMat[4] = sinf(alpha)*sinf(beta)*sinf(gamma) + cosf(alpha)*cosf(gamma);
    rotationMat[5] = sinf(alpha)*cosf(beta);

    rotationMat[6] = cosf(alpha)*sinf(beta)*cosf(gamma) + sinf(alpha)*sinf(gamma);
    rotationMat[7] = cosf(alpha)*sinf(beta)*sinf(gamma) - sinf(alpha)*cosf(gamma);
    rotationMat[8] = cosf(alpha)*cosf(beta);
    */
    //estimatorKalmanGetEstimatedRot(rotationMat);
    clipOrientation(rotationMat);

    slfObs[0] = ownPos.x - targetPos.x;
    slfObs[1] = ownPos.y - targetPos.y;
    slfObs[2] = ownPos.z - targetPos.z;

    slfObs[3] = ownVel.x;
    slfObs[4] = ownVel.y;
    slfObs[5] = ownVel.z;

    slfObs[6]  = rotationMat[0];
    slfObs[7]  = rotationMat[3];
    slfObs[8]  = rotationMat[6];
    slfObs[9]  = rotationMat[1];
    slfObs[10] = rotationMat[4];
    slfObs[11] = rotationMat[7];
    slfObs[12] = rotationMat[2];
    slfObs[13] = rotationMat[5];
    slfObs[14] = rotationMat[8];

    slfObs[15] = ownAngVel.z;
    slfObs[16] = ownAngVel.y; 
    slfObs[17] = ownAngVel.x;

}

void clipPosition(Vector3* pos)
{
    pos->x = clipVal(pos->x, -10.0f, 10.0f);
    pos->y = clipVal(pos->y, -10.0f, 10.0f);
    pos->z = clipVal(pos->z, -10.0f, 10.0f);
}

void clipVelocity(Vector3* vel)
{
    vel->x = clipVal(vel->x, -3.0f, 3.0f);
    vel->y = clipVal(vel->y, -3.0f, 3.0f);
    vel->z = clipVal(vel->z, -3.0f, 3.0f);
}

void clipVelocityRel(Vector3* vel)
{
    vel->x = clipVal(vel->x, -3.0f*2.0f, 3.0f*2.0f);
    vel->y = clipVal(vel->y, -3.0f*2.0f, 3.0f*2.0f);
    vel->z = clipVal(vel->z, -3.0f*2.0f, 3.0f*2.0f);
}

static void clipOrientation(float* orientArr)
{
    for(int k = 0; k < 9;k ++)
    {
        orientArr[k] = clipVal(orientArr[k], -1, 1);
    }
}


static void clipAngularVel(Vector3* angVel)
{
    angVel->x = clipVal(angVel->x, -40, 40);
    angVel->y = clipVal(angVel->y, -40, 40);
    angVel->z = clipVal(angVel->z, -40, 40);
}


Vector3 getPosition(void)
{
    point_t pt;
    estimatorKalmanGetEstimatedPos(&pt);
    return (Vector3){.x = pt.x, .y=pt.y, .z=pt.z};
} 

Vector3 getVeloc(void)
{
    return getVelocity();
}



PARAM_GROUP_START(selfobs)
PARAM_ADD(PARAM_FLOAT, xt, &targetPos.x)
PARAM_ADD(PARAM_FLOAT, yt, &targetPos.y)
PARAM_ADD(PARAM_FLOAT, zt, &targetPos.z)
PARAM_GROUP_STOP(selfobs)



LOG_GROUP_START(selfObs)
LOG_ADD(LOG_FLOAT, xt, &targetPos.x)
LOG_ADD(LOG_FLOAT, yt, &targetPos.y)
LOG_ADD(LOG_FLOAT, zt, &targetPos.z)
LOG_GROUP_STOP(selfObs)











