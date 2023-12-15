/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "../include/nn.h"
#include "../include/neighbor_observation.h"
#include "../include/self_observation.h"
#include "radiolink.h"
#include "param.h"
#include "log.h"
#include "configblock.h"
#include "stabilizer_types.h"
#include "commander.h"
#include "../include/coll_avoid_main.h"


/*
static uint16_t testThrust[80][4] = 
{
{37616,36336,38187,35198},
{35902,36773,37768,37873},
{37278,40410,42259,31571},
{40172,42530,43273,35539},
{42298,42702,45966,39821},
{45996,40596,43844,40974},
{43313,41029,43788,38591},
{43540,38034,43223,41196},
{43332,40235,44185,39351},
{43608,39365,44500,41007},
{44011,40856,44889,40060},
{44372,40357,44711,40546},
{44245,40568,44780,40236},
{43958,40241,45060,40461},
{43884,41290,45101,39892},
{42738,39728,43862,40670},
{42088,39857,43345,39058},
{41178,38421,42998,40306},
{41575,39679,42954,38500},
{41069,38522,43021,40529},
{41996,39683,43516,39010},
{42030,39368,43521,40019},
{42433,38760,43760,41224},
{42509,39838,44453,39884},
{42801,39977,44198,40193},
{42858,39120,44239,41111},
{42457,39814,44979,40384},
{42774,40141,44486,40072},
{42888,39180,44322,41241},
{43374,39468,43837,40536},
{43611,39706,43251,40422},
{43461,38818,43286,41421},
{43277,39353,43581,40859},
{43152,39666,43310,40439},
{43022,38619,43291,41816},
{43261,39611,43228,40818},
{43188,40555,43253,40316},
{42423,38896,42935,42592},
{42638,40296,43473,40894},
{43490,39808,43093,40956},
{43039,39976,43210,40663},
{42033,39316,43193,42488},
{43060,39825,43124,41747},
{43309,39630,43797,41163},
{38656,42002,47339,39140},
{38168,42235,47535,39015},
{38776,41060,46979,40522},
{38159,41326,47648,40413},
{38864,41762,47372,39717},
{39014,42098,47135,39299},
{38183,41064,47268,40732},
{38792,41672,47060,39989},
{38753,41684,47146,39734},
{38425,42159,47100,39583},
{38575,41305,47116,40629},
{38571,41901,47316,39816},
{38546,42748,47280,39148},
{37448,41263,47170,41154},
{37895,41325,46810,39358},
{37098,39797,45576,38270},
{33862,39251,44343,36417},
{32364,39915,42858,33897},
{33781,38261,42946,34834},
{38200,39930,44469,36633},
{40051,40347,46215,37672},
{43213,39262,44173,40621},
{43426,39671,45076,39280},
};*/


static uint8_t    state = 0;
static float      thrusts[4];
static uint16_t   thrustsToMotor[4];
static uint64_t   timer = 0;
static PacketData ownPacket;
static uint8_t    isHighLevelController = 0;
static uint8_t    isMeanEmbed = 0;
static float      mototrKValue = 1.0f;


void communicate();
 
void communicate()
{
  if (timer == ownPacket.id)
  {
    ownPacket.pos = getPosition();
    ownPacket.vel = getVeloc();
    P2PPacket packet;
    packet.port = 0;
    packet.size = sizeof(PacketData);
    memcpy(packet.data, &ownPacket, sizeof(PacketData));
    radiolinkSendP2PPacketBroadcast(&packet);
  }
}

void getThrusts(uint16_t* thrst)
{
  thrst[0] = thrustsToMotor[0];
  thrst[1] = thrustsToMotor[1];
  thrst[2] = thrustsToMotor[2];
  thrst[3] = thrustsToMotor[3];
}

uint8_t isHighLevel()
{
  return isHighLevelController;
}

void appMain() { 
  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id    = (uint8_t)((address) & 0x00000000ff);
  ownPacket.id     = my_id;
  static setpoint_t setpoint;
  static Vector3 lastPosition; 
  vTaskDelay(M2T(10000));


  while(1) {
   // vTaskDelay(M2T(3));

    vTaskDelay(M2T(1));
    lastPosition = getPosition();
    //communicate();

    if(timer >= N_DRONES)
    {
      timer = 0;
    }

    switch (state)
    {
    case 0: // do nothing
      break;
    case 1: // start 
      if(isHighLevel())
        {
          setpoint.mode.x = modeAbs;
          setpoint.mode.y = modeAbs;
          setpoint.mode.z = modeAbs; 

          setpoint.position.x = lastPosition.x;
          setpoint.position.y = lastPosition.y;
          setpoint.position.z = .9f;


          commanderSetSetpoint(&setpoint, 3);
        }
        else{

          if(!isMeanEmbed)
          {
            //consolePrintf("entered app main mean embed \n");
            isMeanEmbed = 1;
          }
          for(int k = 0; k < 4; k++)
          {
            thrustsToMotor[k] = (uint16_t)(UINT16_MAX*thrusts[k]*(mototrKValue));
          }
        }
      break;
    case 2: // land

        if(isHighLevel())
        {
          setpoint.mode.x       = modeAbs;
          setpoint.mode.y       = modeAbs;
          setpoint.mode.z       = modeAbs; 

          setpoint.position.x   = lastPosition.x;
          setpoint.position.y   = lastPosition.y;
          setpoint.position.z   = .05f;

          if(getPosition().z < 0.15f)
          {
            setpoint.mode.x = modeDisable;
            setpoint.mode.y = modeDisable;
            setpoint.mode.z = modeDisable;

            state = 0;
          }
          commanderSetSetpoint(&setpoint, 3);
        }
        else{
          

          if(getPosition().z < 0.15f)
          {
            for(int k = 0; k < 4; k++)
            {
               thrustsToMotor[k] = (uint16_t)(UINT16_MAX*(0.0f));
            }
            state = 0;
          }
        }
        
      break;
    default:
      break;
    }
    feedForwardNN(thrusts);
  }
}



PARAM_GROUP_START(ctrlnn)
PARAM_ADD(PARAM_FLOAT, thrst1, &thrusts[0])
PARAM_ADD(PARAM_FLOAT, thrst2, &thrusts[1])
PARAM_ADD(PARAM_FLOAT, thrst3, &thrusts[2])
PARAM_ADD(PARAM_FLOAT, thrst4, &thrusts[3])
PARAM_ADD(PARAM_UINT8, stt,    &state)
PARAM_ADD(PARAM_UINT8, id,     &(ownPacket.id))
PARAM_ADD(PARAM_UINT8, controller,  &isHighLevelController)
PARAM_ADD(PARAM_FLOAT, kValMotor, &mototrKValue)
//PARAM_ADD(PARAM_INT32, int, &dbgint)
PARAM_GROUP_STOP(ctrlnn)

LOG_GROUP_START(logNN)
LOG_ADD(LOG_FLOAT, thrst1, &thrusts[0])
LOG_ADD(LOG_FLOAT, thrst2, &thrusts[1])
LOG_ADD(LOG_FLOAT, thrst3, &thrusts[2])
LOG_ADD(LOG_FLOAT, thrst4, &thrusts[3])
LOG_ADD(LOG_UINT16, thrstint1, &thrustsToMotor[0])
LOG_ADD(LOG_UINT16, thrstint2, &thrustsToMotor[1])
LOG_ADD(LOG_UINT16, thrstint3, &thrustsToMotor[2])
LOG_ADD(LOG_UINT16, thrstint4, &thrustsToMotor[3])
LOG_ADD(LOG_FLOAT, kValMotor, &mototrKValue)
LOG_GROUP_STOP(logNN)