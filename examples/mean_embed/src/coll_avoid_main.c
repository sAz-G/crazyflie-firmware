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


uint8_t state = 0;
static float      thrusts[4];
static int16_t    thrustsToMotor[4];
static uint64_t   timer = 0;
static PacketData ownPacket;

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

void appMain() {

  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
  ownPacket.id = my_id;

  while(1) {
    vTaskDelay(M2T(10));
    communicate();
    timer++;

    if(timer >= N_DRONES)
    {
      timer = 0;
    }

    switch (state)
    {
    case 0:
      break;
    case 1:
      break;
    case 2:
      feedForwardNN(thrusts);
      for(int k = 0; k < 4; k++)
      {
        thrustsToMotor[k] = (int16_t)((2<<16)*thrusts[k]);
      }
    default:
      break;
    }
  }
}

PARAM_GROUP_START(ctrlnn)
//PARAM_ADD(PARAM_FLOAT, flt, &dbgflt)
PARAM_ADD(PARAM_UINT8, stt, &state)
//PARAM_ADD(PARAM_INT32, int, &dbgint)
PARAM_GROUP_STOP(ctrlnn)

