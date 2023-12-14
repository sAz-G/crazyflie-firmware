#include "../include/neighbor_observation.h"
#include "../include/vec.h"
#include "radiolink.h"
#include "../include/self_observation.h"
#include <string.h>

#include "log.h"


static PacketData   neighborInfo[N_DRONES];

static bool isInit = false;
static int lastAdded        = 0;
static int farthestNeighbor = 0;

void p2pcallbackHandler(P2PPacket *p)
{
    PacketData received;
    memcpy(&received, p->data, sizeof(PacketData));
    lastAdded = received.id;
    neighborInfo[received.id] = received;
}

void updateNeighbObservation(neighb_obs* kNearestArr)
{   
    Vector3 ownPos = getPosition();
    Vector3 ownVel = getVeloc();

    if(!isInit)
    {
        Vector3 defaultPos = {.x = 9.5f, .y = 9.5f, .z = 0.0f};
        Vector3 defaultVel = {.x = 0.0f, .y = 0.0f, .z = 0.0f};

        for(int k = 0; k < KNEARESTAMOUNT; k++)
        {
            kNearestArr[k].relPos = substractVec(ownPos, defaultPos);
            kNearestArr[k].relVel = substractVec(ownVel, defaultVel);
            clipPosition(&(kNearestArr[k].relPos));
            clipVelocityRel(&(kNearestArr[k].relVel));
            defaultPos = (Vector3){.x = 9.5f, .y = ((float)k)*1.5f, .z = 9.5f};
        }

        for(int k = 0; k < N_DRONES; k++)
        {
            neighborInfo[k].pos = defaultPos;
            neighborInfo[k].vel = defaultVel;
            defaultPos = (Vector3){.x = 9.5f, .y = ((float)k)*1.5f, .z = 9.5f};
        }

        p2pRegisterCB(p2pcallbackHandler);
        isInit = true;
    }

    Vector3 newPos = neighborInfo[lastAdded].pos;
    Vector3 newVel = neighborInfo[lastAdded].vel;

    for(int u = 0; u < KNEARESTAMOUNT; u++)
    {
        if(calcDistf(ownPos,newPos) < calcNormf(kNearestArr[farthestNeighbor].relPos) )
        {

            kNearestArr[farthestNeighbor].relPos = substractVec(ownPos, newPos);
            kNearestArr[farthestNeighbor].relVel = substractVec(ownVel, newVel);
            clipPosition(&(kNearestArr[farthestNeighbor].relPos));
            clipVelocityRel(&(kNearestArr[farthestNeighbor].relVel));

            Vector3 checkPos = kNearestArr[0].relPos;
            farthestNeighbor = 0;
            for(int k = 1; k < 6; k++)
            {
                if(calcNormf(checkPos) < calcNormf(kNearestArr[k].relPos) )
                {
                    farthestNeighbor = k;
                    checkPos = kNearestArr[k].relPos;
                }
            }
        }
    }
}

LOG_GROUP_START(neigStt)
LOG_ADD(LOG_FLOAT, n0px, &(neighborInfo[0].pos.x))
LOG_ADD(LOG_FLOAT, n0py, &(neighborInfo[0].pos.y))
LOG_ADD(LOG_FLOAT, n0pz, &(neighborInfo[0].pos.z))
LOG_ADD(LOG_FLOAT, n0vx, &(neighborInfo[0].vel.x))
LOG_ADD(LOG_FLOAT, n0vy, &(neighborInfo[0].vel.y))
LOG_ADD(LOG_FLOAT, n0vz, &(neighborInfo[0].vel.z))

LOG_ADD(LOG_FLOAT, n1px, &(neighborInfo[1].pos.x))
LOG_ADD(LOG_FLOAT, n1py, &(neighborInfo[1].pos.y))
LOG_ADD(LOG_FLOAT, n1pz, &(neighborInfo[1].pos.z))
LOG_ADD(LOG_FLOAT, n1vx, &(neighborInfo[1].vel.x))
LOG_ADD(LOG_FLOAT, n1vy, &(neighborInfo[1].vel.y))
LOG_ADD(LOG_FLOAT, n1vz, &(neighborInfo[1].vel.z))

LOG_ADD(LOG_FLOAT, n2px, &(neighborInfo[2].pos.x))
LOG_ADD(LOG_FLOAT, n2py, &(neighborInfo[2].pos.y))
LOG_ADD(LOG_FLOAT, n2pz, &(neighborInfo[2].pos.z))
LOG_ADD(LOG_FLOAT, n2vx, &(neighborInfo[2].vel.x))
LOG_ADD(LOG_FLOAT, n2vy, &(neighborInfo[2].vel.y))
LOG_ADD(LOG_FLOAT, n2vz, &(neighborInfo[2].vel.z))

LOG_ADD(LOG_FLOAT, n3px, &(neighborInfo[3].pos.x))
LOG_ADD(LOG_FLOAT, n3py, &(neighborInfo[3].pos.y))
LOG_ADD(LOG_FLOAT, n3pz, &(neighborInfo[3].pos.z))
LOG_ADD(LOG_FLOAT, n3vx, &(neighborInfo[3].vel.x))
LOG_ADD(LOG_FLOAT, n3vy, &(neighborInfo[3].vel.y))
LOG_ADD(LOG_FLOAT, n3vz, &(neighborInfo[3].vel.z))

LOG_ADD(LOG_FLOAT, n4px, &(neighborInfo[4].pos.x))
LOG_ADD(LOG_FLOAT, n4py, &(neighborInfo[4].pos.y))
LOG_ADD(LOG_FLOAT, n4pz, &(neighborInfo[4].pos.z))
LOG_ADD(LOG_FLOAT, n4vx, &(neighborInfo[4].vel.x))
LOG_ADD(LOG_FLOAT, n4vy, &(neighborInfo[4].vel.y))
LOG_ADD(LOG_FLOAT, n4vz, &(neighborInfo[4].vel.z))

LOG_ADD(LOG_FLOAT, n5px, &(neighborInfo[5].pos.x))
LOG_ADD(LOG_FLOAT, n5py, &(neighborInfo[5].pos.y))
LOG_ADD(LOG_FLOAT, n5pz, &(neighborInfo[5].pos.z))
LOG_ADD(LOG_FLOAT, n5vx, &(neighborInfo[5].vel.x))
LOG_ADD(LOG_FLOAT, n5vy, &(neighborInfo[5].vel.y))
LOG_ADD(LOG_FLOAT, n5vz, &(neighborInfo[5].vel.z))

LOG_ADD(LOG_FLOAT, n6px, &(neighborInfo[6].pos.x))
LOG_ADD(LOG_FLOAT, n6py, &(neighborInfo[6].pos.y))
LOG_ADD(LOG_FLOAT, n6pz, &(neighborInfo[6].pos.z))
LOG_ADD(LOG_FLOAT, n6vx, &(neighborInfo[6].vel.x))
LOG_ADD(LOG_FLOAT, n6vy, &(neighborInfo[6].vel.y))
LOG_ADD(LOG_FLOAT, n6vz, &(neighborInfo[6].vel.z))

LOG_ADD(LOG_FLOAT, n7px, &(neighborInfo[7].pos.x))
LOG_ADD(LOG_FLOAT, n7py, &(neighborInfo[7].pos.y))
LOG_ADD(LOG_FLOAT, n7pz, &(neighborInfo[7].pos.z))
LOG_ADD(LOG_FLOAT, n7vx, &(neighborInfo[7].vel.x))
LOG_ADD(LOG_FLOAT, n7vy, &(neighborInfo[7].vel.y))
LOG_ADD(LOG_FLOAT, n7vz, &(neighborInfo[7].vel.z))

LOG_GROUP_STOP(neigStt)
