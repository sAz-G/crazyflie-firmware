#ifndef __SELF_OBSERVATION_H__
#define __SELF_OBSERVATION_H__
#include "../include/vec.h"

#define SELFOBSERVATIONSIZE 18
#define N_DRONES 8

#define MAXVELX 3
#define MAXVELY 3
#define MAXVELZ 3

#define MAXPOSX 10 
#define MAXPOSY 10
#define MAXPOSZ 10

#define MAXR1  1   
#define MAXR2  1
#define MAXR3  1
#define MAXR4  1
#define MAXR5  1
#define MAXR6  1
#define MAXR7  1
#define MAXR8  1
#define MAXR9  1

#define MAXW1 40 
#define MAXW2 40
#define MAXW3 40

#define MINVELX -MAXVELX
#define MINVELY -MAXVELY
#define MINVELZ -MAXVELZ

#define MINPOSX -MAXPOSX
#define MINPOSY -MAXPOSY
#define MINPOSZ -MAXPOSZ

#define  MINR1  -MAXR1  
#define  MINR2  -MAXR2 
#define  MINR3  -MAXR3 
#define  MINR4  -MAXR4 
#define  MINR5  -MAXR5 
#define  MINR6  -MAXR6 
#define  MINR7  -MAXR7 
#define  MINR8  -MAXR8 
#define  MINR9  -MAXR9 

#define  MINW1  -MAXW1
#define  MINW2  -MAXW2
#define  MINW3  -MAXW3


typedef struct _SELF_OBS
{
    
            float px;
            float py;
            float pz;

            float vx;
            float vy;
            float vz;

            float r1;
            float r2;
            float r3;
            float r4;
            float r5;
            float r6;
            float r7;
            float r8;
            float r9;

            float wx;
            float wy;
            float wz;
       
}self_obs;


void updateSelfObservation(self_obs*);
Vector3 getPosition(void);
Vector3 getVeloc(void);
void clipPosition(Vector3*);
void clipVelocity(Vector3*);

#endif