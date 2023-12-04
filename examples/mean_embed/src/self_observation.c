#include "../include/self_observation.h"
#include "../include/vec.h"
#include "estimator_kalman.h"
#include "stabilizer_types.h"
#include "stabilizer.h"
#include "../include/vec.h"

/* min max values
 EnvInfo(obs_space=Dict('obs': Box([-10. -10. -10.  -3.  -3.  -3.  -1.  -1.  -1.  -1.  -1.  -1.  -1.  -1.
  -1. -40. -40. -40. -10. -10. -10.  -6.  -6.  -6. -10. -10. -10.  -6.
  -6.  -6. -10. -10. -10.  -6.  -6.  -6. -10. -10. -10.  -6.  -6.  -6.
 -10. -10. -10.  -6.  -6.  -6. -10. -10. -10.  -6.  -6.  -6.], [10. 10. 10.  3.  3.  3.  1.  1.  1.  1.  1.  1.  1.  1.  1. 40. 40. 40.
 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6.
 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6.]

*/
//static const int selfObservationSize = (int)SELFOBSERVATIONSIZE;
static self_obs  selfObservation;

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

  (float) MAXR1,
  (float) MAXR2,
  (float) MAXR3,
  (float) MAXR4,
  (float) MAXR5,
  (float) MAXR6,
  (float) MAXR7,
  (float) MAXR8,
  (float) MAXR9,

  (float)MAXW1,
  (float)MAXW2,
  (float)MAXW3,
  
  (float)MINVELX,
  (float)MINVELY,
  (float)MINVELZ,
  (float)MINPOSX,
  (float)MINPOSY,
  (float)MINPOSZ,
  (float)MINR1,
  (float)MINR2,
  (float)MINR3,
  (float)MINR4,
  (float)MINR5,
  (float)MINR6,
  (float)MINR7,
  (float)MINR8,
  (float)MINR9,
  (float)MINW1,
  (float)MINW2,
  (float)MINW3
  };


static void clipOrientation(float*);
static void clipAngularVel(Vector3*);


void updateSelfObservation(self_obs* slfObs)
{
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

    selfObservation.px = ownPos.x;
    selfObservation.py = ownPos.y;
    selfObservation.pz = ownPos.z;

    selfObservation.vx = ownVel.x;
    selfObservation.vy = ownVel.y;
    selfObservation.vz = ownVel.z;

    selfObservation.wx = ownAngVel.x;
    selfObservation.wy = ownAngVel.y;
    selfObservation.wz = ownAngVel.z;

    selfObservation.r1 = ownAngVel.x;
    selfObservation.r2 = ownAngVel.y;
    selfObservation.r3 = ownAngVel.z;
    selfObservation.r4 = ownAngVel.x;
    selfObservation.r5 = ownAngVel.y;
    selfObservation.r6 = ownAngVel.z;
    selfObservation.r7 = ownAngVel.x;
    selfObservation.r8 = ownAngVel.y;
    selfObservation.r9 = ownAngVel.z;
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



















