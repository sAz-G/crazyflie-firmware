#include "../include/neighbor_observation.h"
#include "../include/vec.h"
#include "radiolink.h"
#include "../include/self_observation.h"
#include <string.h>




//static void checkAmountAdded();

static PacketData   neighborInfo[N_DRONES];

static bool isInit = false;
//static int collectedK           = 0;
//static int farthestNeighbor     = 0;
static int lastAdded            = 0;
//static int collectedList[6]     = {-1,-1,-1,-1,-1,-1};
//static uint8_t isprinted        = 0;
//static uint8_t amountDrones     = N_DRONES;
//static uint8_t amountneighbors  = 1;

void p2pcallbackHandler(P2PPacket *p)
{
    PacketData received;
    memcpy(&received, p->data, sizeof(PacketData));
    lastAdded = received.id;
    neighborInfo[received.id] = received;
}

void updateNeighbObservation(neighb_obs* kNearestArr)
{   
    if(!isInit)
    {
        p2pRegisterCB(p2pcallbackHandler);
        isInit = true;
    }

    Vector3 ownPos = getPosition();
    Vector3 ownVel = getVeloc();


    kNearestArr[0].relPos = negateVec(ownPos);
    kNearestArr[0].relVel = negateVec(ownVel);

    clipPosition(&(kNearestArr[0].relPos));
    clipVelocity(&(kNearestArr[0].relVel));

    /*
    checkAmountAdded();
    if(collectedK == 0)
    {
        if(!isprinted)
        {
            consolePrintf("collected 0 neighbor \n");
            isprinted = 1;
        }
        return;
    }
    else if(collectedK == 1)
    {
        if(!isprinted)
        {
            consolePrintf("collected 1 neighbor \n");
            isprinted = 1;
        }
        kNearestArr[0].relPos = substractVec(ownPos, newPos);
        kNearestArr[0].relVel = substractVec(ownVel, newVel);

        clipPosition(&(kNearestArr[0].relPos));
        clipVelocity(&(kNearestArr[0].relVel));
        
        farthestNeighbor              = 0;
    }
    else if((collectedK < 6) && (collectedK > 1)) 
    {
        if(!isprinted)
        {
            consolePrintf("collected 1 neighbor \n");
            isprinted = 1;
        }   
        kNearestArr[collectedK].relPos = substractVec(ownPos, newPos);
        kNearestArr[collectedK].relVel = substractVec(ownVel, newVel);

        clipPosition(&(kNearestArr[collectedK].relPos));
        clipVelocity(&(kNearestArr[collectedK].relVel));

        Vector3 newRelPos = kNearestArr[collectedK].relPos;

        for(int k = 0; k < collectedK; k++)
        {
            if(calcNormf(newRelPos)  < calcNormf(kNearestArr[k].relPos) )
            {
                farthestNeighbor = k;
                newRelPos = kNearestArr[k].relPos;
            }
        }

    }
    else
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

    }*/
}
/*
static void checkAmountAdded()
{
    
   int amountAdded  = 0;
   int lastIsInList = 0;
   for(int k = 0; k < 6; k++)
   {
        if(collectedList[k] != -1)
        {
            amountAdded++;
            if(collectedList[k] == lastAdded)
            {
                lastIsInList = 1;
            }
        }
        else 
        {
            if(!lastIsInList)
            {
                collectedList[k] = lastAdded;
                amountAdded++;
            }
            break;
        }
   }

   collectedK = amountAdded;
}*/