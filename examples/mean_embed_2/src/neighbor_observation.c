#include "../include/neighbor_observation.h"
#include "../include/vec.h"
#include "radiolink.h"
#include "../include/self_observation.h"
#include <string.h>


static PacketData   neighborInfo[N_DRONES];

static bool isInit = false;
static int lastAdded            = 0;
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
        Vector3 defaultPos = {.x = 9.5f, .y = 9.5f, .z = 9.5f};
        Vector3 defaultVel = {.x = 0.0f, .y = 0.0f, .z = 0.0f};

        for(int k = 0; k < KNEARESTAMOUNT; k++)
        {
            kNearestArr[k].relPos = substractVec(ownPos, defaultPos);
            kNearestArr[k].relVel = substractVec(ownVel, defaultVel);
            clipPosition(&(kNearestArr[k].relPos));
            clipVelocity(&(kNearestArr[k].relVel));
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
            clipVelocity(&(kNearestArr[farthestNeighbor].relVel));

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