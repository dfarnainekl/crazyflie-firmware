/*
 * wmcPosition.c
 *
 *  Created on: 01.05.2015
 *      Author: daniel
 */

#define DEBUG_MODULE "POSITIONCONTROL"

#include "positionControl.h"

#include <math.h>
#include "debug.h"
#include "eprintf.h"
#include "log.h"
#include "imu.h"
#include "sensfusion6.h"
#include "wiiMoteCam.h"
#include "gp2y0a60sz0f.h"


//positionControl_update() gets called with IMU_UPDATE_FREQ Hz, gets divided down
uint32_t posCtrlCounter = 0;
#define POSCTRL_UPDATE_RATE_DIVIDER 5 //100Hz
#define POSCTRL_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / POSCTRL_UPDATE_RATE_DIVIDER))

//actual RPY values
float rollActual = 0;
float pitchActual = 0;
float yawActual = 0;

//desired RPYT values, get read and applied by the stabilizer
float rollDesired = 0;
float pitchDesired = 0;
float yawDesired = 0;
uint16_t thrustDesired = 0;

// Infrared Althold stuff
float gp2y0a60sz0f_value_sum = 0;
float irAlt_raw = 0; //raw (i.e. not corrected for tilt) altitude above ground
float tilt = 0;
float irAlt = 0; // altitude above ground

// wmcTracking stuff
struct WmcBlob wmcBlobs[4]={{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}};
uint8_t wmcBlobCount = 0;
uint8_t wmcPattern_F = 0, wmcPattern_L = 1, wmcPattern_M = 2, wmcPattern_R = 3;
#define PATTERN_F wmcBlobs[wmcPattern_F]
#define PATTERN_L wmcBlobs[wmcPattern_L]
#define PATTERN_M wmcBlobs[wmcPattern_M]
#define PATTERN_R wmcBlobs[wmcPattern_R]
float wmcYaw = 0;
float wmcAlt = 0;
float wmcX = 0;
float wmcY = 0;


static float pointToLineSegmentDistance(float x, float y, float x1, float y1, float x2, float y2);
static void findWmcPatternBlobMap(struct WmcBlob WMCBlob[4]);


uint8_t positionControl_init()
{
	wmc_init(); //wmc_init_basic(); //FIXME: if wmc not connected, bus gets blocked (scl low) --> no eeprom comm
	gp2y0a60sz0f_init();
	return 0;
}


uint8_t positionControl_update()
{
	sensfusion6GetEulerRPY(&rollActual, &pitchActual, &yawActual);
	gp2y0a60sz0f_value_sum += gp2y0a60sz0f_getValue();
	// 100Hz
	if (++posCtrlCounter >= POSCTRL_UPDATE_RATE_DIVIDER)
	{
		wmc_readBlobs(&wmcBlobs);

		uint8_t i;
		for(i=0;i<4;i++)
		{
		  if(wmcBlobs[i].isVisible) wmcBlobCount++;
		}

		findWmcPatternBlobMap(wmcBlobs);

		wmcYaw = atan2(wmcBlobs[wmcPattern_M].x - wmcBlobs[wmcPattern_F].x, wmcBlobs[wmcPattern_M].y - wmcBlobs[wmcPattern_F].y);
		wmcAlt = 90 / tan(sqrt(pow(wmcBlobs[wmcPattern_L].x - wmcBlobs[wmcPattern_R].x, 2) + pow(wmcBlobs[wmcPattern_L].y - wmcBlobs[wmcPattern_R].y, 2)));
		wmcX = 0;
		wmcY = 0;


		irAlt_raw = gp2y0a60sz0f_valueToDistance(gp2y0a60sz0f_value_sum / POSCTRL_UPDATE_RATE_DIVIDER);
		gp2y0a60sz0f_value_sum = 0;
		tilt = atan( hypotf( tan( rollActual * M_PI / 180 ), tan( pitchActual * M_PI / 180 ) ));
		irAlt = irAlt_raw * cos(tilt);

		posCtrlCounter = 0;
	}
	return 0;
}


uint8_t positionControl_getRPYT(float *roll, float *pitch, float *yaw, uint16_t *thrust)
{
	*roll = rollDesired;
	*pitch = pitchDesired;
	*yaw = yawDesired;
	*thrust = thrustDesired;
	return 0;
}



//from http://stackoverflow.com/a/11172574/3658125
static float pointToLineSegmentDistance(float x, float y, float x1, float y1, float x2, float y2)
{
	float A = x - x1;
	float B = y - y1;
	float C = x2 - x1;
	float D = y2 - y1;

	float dot = A * C + B * D;
	float len_sq = C * C + D * D;
	float param = dot / len_sq;

	float xx, yy;

	if (param < 0 || (x1 == x2 && y1 == y2))
	{
		xx = x1;
		yy = y1;
	}
	else if (param > 1)
	{
		xx = x2;
		yy = y2;
	}
	else
	{
		xx = x1 + param * C;
		yy = y1 + param * D;
	}

	float dx = x - xx;
	float dy = y - yy;

	return hypotf(dx,dy);
}

static void findWmcPatternBlobMap(struct WmcBlob WMCBlobs[4])
{
	float distance;
	//2 0 1
	float shortestDistance = pointToLineSegmentDistance(WMCBlobs[2].x, WMCBlobs[2].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[1].x, WMCBlobs[1].y);
	uint8_t pattern_m = 2; uint8_t pattern_f = 3; uint8_t pattern_a = 0;
	//3 0 1
	distance = pointToLineSegmentDistance(WMCBlobs[3].x, WMCBlobs[3].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[1].x, WMCBlobs[1].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 3; pattern_f = 2; pattern_a = 0;}
	//1 0 2
	distance = pointToLineSegmentDistance(WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[2].x, WMCBlobs[2].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 1; pattern_f = 3; pattern_a = 0;}
	//3 0 2
	distance = pointToLineSegmentDistance(WMCBlobs[3].x, WMCBlobs[3].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[2].x, WMCBlobs[2].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 3; pattern_f = 1; pattern_a = 0;}
	//1 0 3
	distance = pointToLineSegmentDistance(WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 1; pattern_f = 2; pattern_a = 0;}
	//2 0 3
	distance = pointToLineSegmentDistance(WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 2; pattern_f = 1; pattern_a = 0;}
	//0 1 2
	distance = pointToLineSegmentDistance(WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[2].x, WMCBlobs[2].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 0; pattern_f = 3; pattern_a = 1;}
	//3 1 2
	distance = pointToLineSegmentDistance(WMCBlobs[3].x, WMCBlobs[3].y, WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[2].x, WMCBlobs[2].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 3; pattern_f = 0; pattern_a = 1;}
	//0 1 3
	distance = pointToLineSegmentDistance(WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 0; pattern_f = 2; pattern_a = 1;}
	//2 1 3
	distance = pointToLineSegmentDistance(WMCBlobs[2].x, WMCBlobs[2].y, WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 2; pattern_f = 0; pattern_a = 1;}
	//0 2 3
	distance = pointToLineSegmentDistance(WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[2].x, WMCBlobs[2].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 0; pattern_f = 1; pattern_a = 2;}
	//1 2 3
	distance = pointToLineSegmentDistance(WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[2].x, WMCBlobs[2].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 1; pattern_f = 0; pattern_a = 2;}

	uint8_t pattern_l;
	uint8_t pattern_r;

	if(((WMCBlobs[pattern_a].x - WMCBlobs[pattern_m].x) * (WMCBlobs[pattern_f].y - WMCBlobs[pattern_m].y) - (WMCBlobs[pattern_a].y - WMCBlobs[pattern_m].y) * (WMCBlobs[pattern_f].x - WMCBlobs[pattern_m].x)) < 0)
	{
		pattern_l = pattern_a;
		pattern_r = 6 - pattern_l - pattern_m - pattern_f;
	}
	else
	{
		pattern_r = pattern_a;
		pattern_l = 6 - pattern_r - pattern_m - pattern_f;
	}

	wmcPattern_F = pattern_f;
	wmcPattern_L = pattern_l;
	wmcPattern_M = pattern_m;
	wmcPattern_R = pattern_r;
}



LOG_GROUP_START(irAlt)
LOG_ADD(LOG_FLOAT, alt_raw, &irAlt_raw)
LOG_ADD(LOG_FLOAT, tilt, &tilt)
LOG_ADD(LOG_FLOAT, alt, &irAlt)
LOG_GROUP_STOP(irAlt)

LOG_GROUP_START(wmc)
LOG_ADD(LOG_UINT8, blob_0_size, &wmcBlobs[0].s)
LOG_ADD(LOG_UINT16, blob_0_x, &wmcBlobs[0].x)
LOG_ADD(LOG_UINT16, blob_0_y, &wmcBlobs[0].y)
LOG_ADD(LOG_UINT8, blob_1_size, &wmcBlobs[1].s)
LOG_ADD(LOG_UINT16, blob_1_x, &wmcBlobs[1].x)
LOG_ADD(LOG_UINT16, blob_1_y, &wmcBlobs[1].y)
LOG_ADD(LOG_UINT8, blob_2_size, &wmcBlobs[2].s)
LOG_ADD(LOG_UINT16, blob_2_x, &wmcBlobs[2].x)
LOG_ADD(LOG_UINT16, blob_2_y, &wmcBlobs[2].y)
LOG_ADD(LOG_UINT8, blob_3_size, &wmcBlobs[3].s)
LOG_ADD(LOG_UINT16, blob_3_x, &wmcBlobs[3].x)
LOG_ADD(LOG_UINT16, blob_3_y, &wmcBlobs[3].y)
LOG_ADD(LOG_UINT8, pattern_f, &wmcPattern_F)
LOG_ADD(LOG_UINT8, pattern_l, &wmcPattern_L)
LOG_ADD(LOG_UINT8, pattern_m, &wmcPattern_M)
LOG_ADD(LOG_UINT8, pattern_r, &wmcPattern_R)
LOG_ADD(LOG_FLOAT, wmcYaw, &wmcYaw)
LOG_ADD(LOG_FLOAT, wmcAlt, &wmcAlt)
LOG_GROUP_STOP(wmc)
