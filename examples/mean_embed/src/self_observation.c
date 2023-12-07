#include "../include/self_observation.h"
#include "../include/vec.h"
#include "estimator_kalman.h"
#include "stabilizer_types.h"
#include "stabilizer.h"
#include "../include/vec.h"
#include "param.h"
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
static Vector3 targetPos = {.x = 0.f, .y = 0.f , .z= .4f};
static void clipOrientation(float*);
static void clipAngularVel(Vector3*);


void updateSelfObservation(self_obs* slfObs)
{// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYy
    // get position  
    Vector3 ownPos = getPosition();
    clipPosition(&ownPos);

    Vector3 ownVel = getVelocity();
    clipVelocity(&ownVel);

    Vector3 ownAngVel = getAngularVelocity();
    clipAngularVel(&ownAngVel);

    float rotationMat[9];
    estimatorKalmanGetEstimatedRot(rotationMat);
    clipOrientation(rotationMat);

    slfObs->px = targetPos.x - ownPos.x;
    slfObs->py = targetPos.y - ownPos.y;
    slfObs->pz = targetPos.z - ownPos.z;

    slfObs->vx = ownVel.x;
    slfObs->vy = ownVel.y;
    slfObs->vz = ownVel.z;

    slfObs->wx = ownAngVel.x;
    slfObs->wy = ownAngVel.y;
    slfObs->wz = ownAngVel.z;

    slfObs->r1 = rotationMat[0];
    slfObs->r2 = rotationMat[1];
    slfObs->r3 = rotationMat[2];
    slfObs->r4 = rotationMat[3];
    slfObs->r5 = rotationMat[4];
    slfObs->r6 = rotationMat[5];
    slfObs->r7 = rotationMat[6];
    slfObs->r8 = rotationMat[7];
    slfObs->r9 = rotationMat[8];

}

void clipPosition(Vector3* pos)
{
    pos->x = clipVal(pos->x, obsLimit.minPosx, obsLimit.maxPosx);
    pos->y = clipVal(pos->y, obsLimit.minPosy, obsLimit.maxPosy);
    pos->x = clipVal(pos->x, obsLimit.minPosx, obsLimit.maxPosx);
}

void clipVelocity(Vector3* vel)
{
    vel->x = clipVal(vel->x, obsLimit.minVelx, obsLimit.maxVelx);
    vel->y = clipVal(vel->y, obsLimit.minVely, obsLimit.maxVely);
    vel->x = clipVal(vel->x, obsLimit.minVelx, obsLimit.maxVelx);
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


/*
LOG_GROUP_START(selfObsLog)
LOG_ADD(LOG_FLOAT, thrst1, &thrusts[0])
LOG_ADD(LOG_FLOAT, thrst2, &thrusts[2])
LOG_ADD(LOG_FLOAT, thrst3, &thrusts[3])
LOG_ADD(LOG_FLOAT, thrst4, &thrusts[4])
LOG_GROUP_STOP(selfObsLog)*/











