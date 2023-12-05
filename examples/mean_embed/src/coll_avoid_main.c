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



uint8_t state = 0;
static float      thrusts[4];
static int32_t    thrustsToMotor[4];
static uint64_t   timer = 0;
static PacketData ownPacket;
static uint8_t isHighLevelController = 1;

static void communicate();

static void communicate()
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

void getThrusts(int32_t* thrst)
{
  thrst[0] = thrustsToMotor[0];
  thrst[1] = thrustsToMotor[1];
  thrst[2] = thrustsToMotor[2];
  thrst[3] = thrustsToMotor[3];
}

int8_t isHighLevel()
{
  return isHighLevelController;
}

void appMain() {

  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
  ownPacket.id = my_id;
  static setpoint_t setpoint;
  static Vector3 lastPosition; 
 



  while(1) {
    vTaskDelay(M2T(10));
    lastPosition = getPosition();
    communicate();
    timer++;

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
          setpoint.position.z = .4f;


          commanderSetSetpoint(&setpoint, 3);
        }
        else{
          feedForwardNN(thrusts);
          for(int k = 0; k < 4; k++)
          {
            thrustsToMotor[k] = (int32_t)((1<<31)*thrusts[k]);
          }
        }
      break;
    case 2: // land
        isHighLevelController = 1;
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
        }
        commanderSetSetpoint(&setpoint, 3);
      break;
    default:
      break;
    }
  }
}

PARAM_GROUP_START(ctrlnn)
//PARAM_ADD(PARAM_FLOAT, flt, &dbgflt)
PARAM_ADD(PARAM_UINT8, stt, &state)
PARAM_ADD(PARAM_UINT8, id,  &(ownPacket.id))
PARAM_ADD(PARAM_UINT8, controller,  &isHighLevelController)
//PARAM_ADD(PARAM_INT32, int, &dbgint)
PARAM_GROUP_STOP(ctrlnn)

