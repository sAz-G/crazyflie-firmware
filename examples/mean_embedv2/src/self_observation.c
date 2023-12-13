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
static Vector3 targetPos = {.x = 0.f, .y = 0.f , .z= 1.036f};
static void clipOrientation(float*);
static void clipAngularVel(Vector3*);


void updateSelfObservation(float* slfObs)
{// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYy
    // get position  
    Vector3 ownPos = getPosition();
    targetPos.x = ownPos.x;
    targetPos.y = ownPos.y;

    clipPosition(&ownPos);

    Vector3 ownVel = getVelocity();
    clipVelocity(&ownVel);

    Vector3 ownAngVel = getAngularVelocity();
    clipAngularVel(&ownAngVel);

    float rotationMat[9];

    float alpha = getRoll(); // alpha
    float beta = getPitch(); // beta
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
    
    //estimatorKalmanGetEstimatedRot(rotationMat);
    clipOrientation(rotationMat);

    slfObs[0] = ownPos.x - targetPos.x;
    slfObs[1] = ownPos.y - targetPos.y;
    slfObs[2] = ownPos.z - targetPos.z;

    slfObs[3] = ownVel.x;
    slfObs[4] = ownVel.y;
    slfObs[5] = ownVel.z;

    slfObs[15] = ownAngVel.x;
    slfObs[16] = -ownAngVel.y; // bitcraze representation is different than the standard
    slfObs[17] = ownAngVel.z;

    slfObs[6]  = rotationMat[0];
    slfObs[7]  = rotationMat[1];
    slfObs[8]  = rotationMat[2];
    slfObs[9]  = rotationMat[3];
    slfObs[10] = rotationMat[4];
    slfObs[11] = rotationMat[5];
    slfObs[12] = rotationMat[6];
    slfObs[13] = rotationMat[7];
    slfObs[14] = rotationMat[8];

}

void clipPosition(Vector3* pos)
{
    pos->x = clipVal(pos->x, obsLimit.minPosx, obsLimit.maxPosx);
    pos->y = clipVal(pos->y, obsLimit.minPosy, obsLimit.maxPosy);
    pos->z = clipVal(pos->z, obsLimit.minPosz, obsLimit.maxPosz);
}

void clipVelocity(Vector3* vel)
{
    vel->x = clipVal(vel->x, obsLimit.minVelx, obsLimit.maxVelx);
    vel->y = clipVal(vel->y, obsLimit.minVely, obsLimit.maxVely);
    vel->z = clipVal(vel->z, obsLimit.minVelz, obsLimit.maxVelz);
}

void clipVelocityRel(Vector3* vel)
{
    vel->x = clipVal(vel->x, obsLimit.minVelx*2.0f, obsLimit.maxVelx*2.0f);
    vel->y = clipVal(vel->y, obsLimit.minVely*2.0f, obsLimit.maxVely*2.0f);
    vel->z = clipVal(vel->z, obsLimit.minVelz*2.0f, obsLimit.maxVelz*2.0f);
}

static void clipOrientation(float* orientArr)
{
    orientArr[0] = clipVal(orientArr[0], obsLimit.minR1, obsLimit.maxR1);
    orientArr[1] = clipVal(orientArr[1], obsLimit.minR2, obsLimit.maxR2);
    orientArr[2] = clipVal(orientArr[2], obsLimit.minR3, obsLimit.maxR3);
    orientArr[3] = clipVal(orientArr[3], obsLimit.minR4, obsLimit.maxR4);
    orientArr[4] = clipVal(orientArr[4], obsLimit.minR5, obsLimit.maxR5);
    orientArr[5] = clipVal(orientArr[5], obsLimit.minR6, obsLimit.maxR6);
    orientArr[6] = clipVal(orientArr[6], obsLimit.minR7, obsLimit.maxR7);
    orientArr[7] = clipVal(orientArr[7], obsLimit.minR8, obsLimit.maxR8);
    orientArr[8] = clipVal(orientArr[8], obsLimit.minR9, obsLimit.maxR9);
}


static void clipAngularVel(Vector3* angVel)
{
    angVel->x = clipVal(angVel->x, obsLimit.minW1, obsLimit.maxW1);
    angVel->y = clipVal(angVel->y, obsLimit.minW2, obsLimit.maxW2);
    angVel->z = clipVal(angVel->z, obsLimit.minW3, obsLimit.maxW3);
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











