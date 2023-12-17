#ifndef __COLL_AVOID_MAIN_H__
#define __COLL_AVOID_MAIN_H__

#include <stdint.h>

typedef struct control_t_n {
	float thrust_0;
	float thrust_1;
	float thrust_2;
	float thrust_3;
} control_t_n;


typedef struct _Vec3
{
    float x;
    float y;
    float z;
}Vector3;

typedef struct _PacketData
{
  uint8_t id;
  Vector3 pos;
  Vector3 vel;
} PacketData;  // size: ? bytes + 12 bytes + 12 bytes  // max: 60 bytes


void    getThrusts(uint16_t*);
uint8_t isHighLevel();
#endif