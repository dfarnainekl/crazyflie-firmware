/*
 * takeoff.c
 *
 *  Created on: 15.08.2015
 *      Author: Franky333
 */

#define DEBUG_MODULE "TAKEOFF"

#include "takeoff.h"

#include <stdlib.h>
#include "imu.h"
#include "debug.h"
#include "param.h"
#include "commander.h"
#include "positionControl.h"


bool takeOffActive = false;			// Currently in takeoff mode
bool setTakeOffActive = false;		// takeoff mode has just been activated

static uint16_t takeoffThrust = TAKEOFF_THRUST;
static float takeoffYawRate = TAKEOFF_YAWRATE;
static float takeoffPitch = TAKEOFF_PITCH;
static float takeoffRoll = TAKEOFF_ROLL;
static float takeoff_timeout = TAKEOFF_TIMEOUT;
static uint32_t takeoff_counter = 0;

//desired RPYT values, get read and applied by the stabilizer
static float rollDesired = 0;
static float pitchDesired = 0;
static float yawRateDesired = 0;
static uint16_t thrustDesired = 0;

void takeoff_update() //gets called with IMU_UPDATE_FREQ Hz
{
	commanderGetTakeoff(&takeOffActive,&setTakeOffActive);

	if(setTakeOffActive)
	{
		takeoff_counter = 0;
		DEBUG_PRINT("takeoff on\n");
	}
	if(takeOffActive)
	{
		pitchDesired = takeoffPitch;
		rollDesired = takeoffRoll;
		yawRateDesired = takeoffYawRate;
		thrustDesired = takeoffThrust;
		if(positionControl_getWmcStatus() == WMC_STATUS_OK) //pattern recognized
		{
			DEBUG_PRINT("pattern recognized, takeoff off, posCtrl on\n");
			commanderSetTakeoff(false);
			commanderSetPositionControl(true);
		}
		//takeoff timeout
		takeoff_counter++;
		if(((float)takeoff_counter/(float)IMU_UPDATE_FREQ > takeoff_timeout) && (takeoff_timeout != 0.0))
		{
			DEBUG_PRINT("takeoff off (timeout), landing on\n");
			commanderSetTakeoff(false);
			commanderSetLanding(true);
		}
	}
}

void takeoff_getRPYT(setpoint_t *setpoint)
{
	setpoint->attitude.roll = rollDesired;
	setpoint->attitude.pitch = pitchDesired;
	setpoint->attitudeRate.yaw = yawRateDesired;
	setpoint->thrust = thrustDesired;
}


PARAM_GROUP_START(takeoff)
PARAM_ADD(PARAM_UINT16, thrust, &takeoffThrust)
PARAM_ADD(PARAM_FLOAT, yawRate, &takeoffYawRate)
PARAM_ADD(PARAM_FLOAT, pitch, &takeoffPitch)
PARAM_ADD(PARAM_FLOAT, roll, &takeoffRoll)
PARAM_ADD(PARAM_FLOAT, timeout, &takeoff_timeout)
PARAM_GROUP_STOP(takeoff)
