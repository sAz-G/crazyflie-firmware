#include "../include/self_observation.h"
#include "../include/vec.h"

/* min max values
 EnvInfo(obs_space=Dict('obs': Box([-10. -10. -10.  -3.  -3.  -3.  -1.  -1.  -1.  -1.  -1.  -1.  -1.  -1.
  -1. -40. -40. -40. -10. -10. -10.  -6.  -6.  -6. -10. -10. -10.  -6.
  -6.  -6. -10. -10. -10.  -6.  -6.  -6. -10. -10. -10.  -6.  -6.  -6.
 -10. -10. -10.  -6.  -6.  -6. -10. -10. -10.  -6.  -6.  -6.], [10. 10. 10.  3.  3.  3.  1.  1.  1.  1.  1.  1.  1.  1.  1. 40. 40. 40.
 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6.
 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6. 10. 10. 10.  6.  6.  6.]

*/
static const int selfObservationSize = (int)SELFOBSERVATIONSIZE;
static self_obs* selfObservation[(int)SELFOBSERVATIONSIZE];

// constants variables 
static float maxVelx;
static float maxVely;
static float maxVelz;

static float maxPosx;
static float maxPosy;
static float maxPosz;

static float maxR1;
static float maxR2;
static float maxR3;
static float maxR4;
static float maxR5;
static float maxR6;
static float maxR7;
static float maxR8;
static float maxR9;

static float maxW1;
static float maxW2;
static float maxW3;


static float minVelx;
static float minVely;
static float minVelz;

static float minPosx;
static float minPosy;
static float minPosz;

static float minR1;
static float minR2;
static float minR3;
static float minR4;
static float minR5;
static float minR6;
static float minR7;
static float minR8;
static float minR9;

static float minW1;
static float minW2;
static float minW3;

static void clipObservation(self_obs*);

void updateSelfObservation(self_obs* self_obs)
{
    return;
}



