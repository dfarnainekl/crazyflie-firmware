/*
 * landing.c
 *
 *  Created on: 15.08.2015
 *      Author: Franky333
 */

#define DEBUG_MODULE "LANDING"

#include "landing.h"

#include <stdlib.h>
#include "imu.h"
#include "debug.h"
#include "param.h"
#include "commander.h"
#include "positionControl.h"


bool landingActive = false;			// Currently in landing mode
bool setLandingActive = false;		// landing mode has just been activated

static float sinkrate = SINKRATE;
static float landing_duration = LANDING_DURATION;
static uint16_t landingThrust = LANDING_THRUST;
static float landingYawRate = LANDING_YAWRATE;
static float landingPitch = LANDING_PITCH;
static float landingRoll = LANDING_ROLL;

//desired RPYT values, get read and applied by the stabilizer
static float rollDesired = 0;
static float pitchDesired = 0;
static float yawRateDesired = 0;
static uint16_t thrustDesired = 0;


void landing_update()
{
	commanderGetLanding(&landingActive,&setLandingActive);

	static float positionCtrlDesiredAltBackup;
	static bool patternVisible;
	static uint32_t landing_counter;

	if(setLandingActive)
	{
		positionCtrlDesiredAltBackup = *positionControl_getDesiredAltitudePtr();
		DEBUG_PRINT("landing on\n");
		commanderSetTakeoff(false);
		patternVisible = true;
	}
	if(landingActive)
	{
		landing_counter++;
		if(patternVisible) //controlled descent
		{
			positionControl_getRPYT(&rollDesired, &pitchDesired, &yawRateDesired, &thrustDesired);
			*positionControl_getDesiredAltitudePtr() -= sinkrate;

			if(positionControl_getWmcStatus() != WMC_STATUS_OK)
			{
				patternVisible = false;
				landing_counter = 0;
				commanderSetPositionControl(false);
				*positionControl_getDesiredAltitudePtr() = positionCtrlDesiredAltBackup;
			}
		}
		else if((float)landing_counter/(float)IMU_UPDATE_FREQ < landing_duration) //free descent
		{
			pitchDesired = landingPitch;
			rollDesired = landingRoll;
			yawRateDesired = landingYawRate;
			thrustDesired = landingThrust;
		}
		else //landing finished
		{
			pitchDesired = 0.0;
			rollDesired = 0.0;
			yawRateDesired = 0.0;
			thrustDesired = 0;
			DEBUG_PRINT("landing finished\n");
			commanderSetLanding(false);
		}

	}
}

void landing_getRPYT(float *roll, float *pitch, float *yawRate, uint16_t *thrust)
{
	*roll = rollDesired;
	*pitch = pitchDesired;
	*yawRate = yawRateDesired;
	*thrust = thrustDesired;
}

PARAM_GROUP_START(landing)
PARAM_ADD(PARAM_FLOAT, sinkrate, &sinkrate)
PARAM_ADD(PARAM_UINT16, thrust, &landingThrust)
PARAM_ADD(PARAM_FLOAT, yawRate, &landingYawRate)
PARAM_ADD(PARAM_FLOAT, pitch, &landingPitch)
PARAM_ADD(PARAM_FLOAT, roll, &landingRoll)
PARAM_GROUP_STOP(landing)
