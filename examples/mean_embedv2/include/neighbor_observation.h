#ifndef __NEIGHBOR_OBSERVATION_H__
#define __NEIGHBOR_OBSERVATION_H__

#include "../include/vec.h"
#include <stdint.h>
#define NEIGHBOROBSSIZE 6
#define KNEARESTAMOUNT 6

typedef struct _NEIGHB_OBS
{
    Vector3 relPos;
    Vector3 relVel;
}neighb_obs;

typedef struct _PacketData
{
  uint8_t id;
  Vector3 pos;
  Vector3 vel;
} PacketData;  // size: ? bytes + 12 bytes + 12 bytes  // max: 60 bytes

void updateNeighbObservation(neighb_obs* kNearest);

#endif