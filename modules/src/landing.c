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


bool landingActive = false;			// Currently in landing mode
bool setLandingActive = false;		// landing mode has just been activated

static uint16_t landingThrust = LANDING_THRUST;
static float landingYawRate = LANDING_YAWRATE;
static float landingPitch = LANDING_PITCH;
static float landingRoll = LANDING_ROLL;
static float landing_duration = LANDING_DURATION;
static float landing_delay = LANDING_DELAY;
static uint32_t landing_counter = 0;

//desired RPYT values, get read and applied by the stabilizer
static float rollDesired = 0;
static float pitchDesired = 0;
static float yawRateDesired = 0;
static uint16_t thrustDesired = 0;


void landing_update()
{
	commanderGetLanding(&landingActive,&setLandingActive);

	if(setLandingActive)
	{
		landing_counter = 0;
		DEBUG_PRINT("landing on\n");
		commanderSetTakeoff(false);
		commanderSetPositionControl(false);
	}
	if(landingActive)
	{
		landing_counter++;
		if((float)landing_counter/(float)IMU_UPDATE_FREQ < landing_duration) //slow landing
		{
			pitchDesired = landingPitch;
			rollDesired = landingRoll;
			yawRateDesired = landingYawRate;
			thrustDesired = landingThrust;
		}
		else if((float)landing_counter/(float)IMU_UPDATE_FREQ > (landing_duration + landing_delay)) //landing done
		{
			DEBUG_PRINT("landing finished\n");
			commanderSetLanding(false);
		}
		else //fast landing (falling)
		{
			pitchDesired = 0.0;
			rollDesired = 0.0;
			yawRateDesired = 0.0;
			thrustDesired = 0;
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
