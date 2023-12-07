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
#include "radiolink.h"
#include "param.h"
#include "log.h"
#include "configblock.h"
#include "stabilizer_types.h"
#include "commander.h"
#include "estimator_kalman.h"

static uint8_t state                 = 0;
static uint8_t isHighLevelController = 1;
static int16_t thrustsToMotor[4];
static float   thrusts[4];

uint8_t isHighLevel()
{
  return isHighLevelController;
}

void getThrusts(int16_t* thrst)
{
//  consolePrintf("entered getThrusts\n");
  thrst[0] = thrustsToMotor[0];
  thrst[1] = thrustsToMotor[1];
  thrst[2] = thrustsToMotor[2];
  thrst[3] = thrustsToMotor[3];
}

void appMain() 
{
  /*
  uint64_t address  = configblockGetRadioAddress();
  uint8_t  my_id    = (uint8_t)((address) & 0x00000000ff);
 */
  setpoint_t setpoint;
  point_t pt;

  thrusts[0] = 1.0f; 
  thrusts[1] = 1.0f;
  thrusts[2] = 1.0f;
  thrusts[3] = 1.0f;
  uint8_t isprinted = 0;

  while(1) 
  {
    estimatorKalmanGetEstimatedPos(&pt);

    vTaskDelay(M2T(10));

    switch (state)
    {
    case 0: // do nothing
      break;
    case 1: // start 
      if(isHighLevelController)
        {
          setpoint.mode.x     = modeAbs;
          setpoint.mode.y     = modeAbs;
          setpoint.mode.z     = modeAbs; 

          setpoint.position.x = pt.x;
          setpoint.position.y = pt.y;
          setpoint.position.z = .9f;
          if(!isprinted)
          {
            DEBUG_PRINT("entered high level\n");
            isprinted = 1;
          }

          commanderSetSetpoint(&setpoint, 3);
        }
        else{
          for(int k = 0; k < 4; k++)
          {
            thrustsToMotor[k] = (int16_t)((INT16_MAX)*thrusts[k]);
          }
        }
      break;
    case 2: // land
        isHighLevelController = 1;
        setpoint.mode.x       = modeAbs;
        setpoint.mode.y       = modeAbs;
        setpoint.mode.z       = modeAbs; 

        setpoint.position.x   = pt.x;
        setpoint.position.y   = pt.y;
        setpoint.position.z   = .05f;

        if(pt.z < 0.15f)
        {
          setpoint.mode.x = modeDisable;
          setpoint.mode.y = modeDisable;
          setpoint.mode.z = modeDisable;

          state = 0;
        }
        commanderSetSetpoint(&setpoint, 3);
      break;
    default:
      break;
    }
  }
}

PARAM_GROUP_START(ctrlnn)
PARAM_ADD(PARAM_FLOAT, thrst1, &thrusts[0])
PARAM_ADD(PARAM_FLOAT, thrst2, &thrusts[1])
PARAM_ADD(PARAM_FLOAT, thrst3, &thrusts[2])
PARAM_ADD(PARAM_FLOAT, thrst4, &thrusts[3])
//PARAM_ADD(PARAM_UINT8, id,  &(ownPacket.id))
PARAM_ADD(PARAM_UINT8, controller,  &isHighLevelController)
PARAM_ADD(PARAM_UINT8, stt,  &state)
PARAM_GROUP_STOP(ctrlnn)

