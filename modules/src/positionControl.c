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
float gp2y0a60sz0f_value_sum = 0; //for averaging ir distance sensor readings
float irAlt_raw = 0; //raw (i.e. not corrected for tilt) altitude above ground ir distance sensor
float tilt = 0; //tilt in rad used for correcting altitude reading
float irAlt = 0; // altitude above ground from ir distance sensor

// wmcTracking stuff
struct WmcBlob wmcBlobs[4]={{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}; //all four wiiMote cam blobs
uint8_t wmcBlobCount = 0; //number of recognized blobs
uint8_t wmcPattern_F = 0; //blob id of front led in pattern
uint8_t wmcPattern_L = 1; //blob id of left led in pattern
uint8_t wmcPattern_M = 2; //blob id of middle led in pattern
uint8_t wmcPattern_R = 3; //blob id of right led in pattern
float wmcYaw = 0; //yaw angle calculated from wmc + pattern, in degree
float wmcAlt1 = 0; //altitude calculated from wmc + pattern, from L-R distance
float wmcAlt2 = 0; //altitude calculated from wmc + pattern, from L-F distance
float wmcAlt3 = 0; //altitude calculated from wmc + pattern, from R-F distance
float wmcAlt4 = 0; //altitude calculated from wmc + pattern, from M-F distance
float wmcAlt = 0; //altitude calculated from wmc + pattern, in mm
float wmcX = 0; //x position calculated from wmc + pattern/point, in mm
float wmcY = 0; //y position calculated from wmc + pattern/point, in mm

//random stuff
int i; //for for-loops


//static function prototypes
static float wmcBlobToBlobAngle(struct WmcBlob wmcBlob1, struct WmcBlob wmcBlob2);
static float pointToLineSegmentDistance2D(float x, float y, float x1, float y1, float x2, float y2);
static void findWmcPatternBlobMapping(struct WmcBlob WMCBlob[4]);


//initializes positionControl
uint8_t positionControl_init()
{
	wmc_init(); //settings in wiiMoteCam.h, alternatively: wmc_init_basic();
	gp2y0a60sz0f_init();
	return 0;
}


//updates positionControl, has to get called at IMU_UPDATE_FREQ Hz
uint8_t positionControl_update()
{
	sensfusion6GetEulerRPY(&rollActual, &pitchActual, &yawActual); //get actual roll, pitch and yaw angles
	gp2y0a60sz0f_value_sum += gp2y0a60sz0f_getValue(); //add ir distance sensor reading to sum (to be divided later to get mean value)

	if (++posCtrlCounter >= POSCTRL_UPDATE_RATE_DIVIDER) //100Hz
	{
		wmc_readBlobs(&wmcBlobs); //get wiiMoteCam blobs
		for(i=0;i<4;i++) if(wmcBlobs[i].isVisible) wmcBlobCount++; //find number of recognized blobs

		//calculate position & yaw angle relative to T-pattern (from http://www.cogsys.cs.uni-tuebingen.de/publikationen/2010/Wenzel2010imav.pdf)
		findWmcPatternBlobMapping(wmcBlobs); //find blob id for each point in pattern
		wmcYaw = atan2f(wmcBlobs[wmcPattern_M].x - wmcBlobs[wmcPattern_F].x, wmcBlobs[wmcPattern_M].y - wmcBlobs[wmcPattern_F].y) * 180 / M_PI ;
		wmcAlt1 = PATTERN_DISTANCE_L_R / tanf(wmcBlobToBlobAngle(wmcBlobs[wmcPattern_L], wmcBlobs[wmcPattern_R]));
		wmcAlt2 = PATTERN_DISTANCE_L_F / tanf(wmcBlobToBlobAngle(wmcBlobs[wmcPattern_L], wmcBlobs[wmcPattern_F]));
		wmcAlt3 = PATTERN_DISTANCE_R_F / tanf(wmcBlobToBlobAngle(wmcBlobs[wmcPattern_R], wmcBlobs[wmcPattern_F]));
		wmcAlt4 = PATTERN_DISTANCE_M_F / tanf(wmcBlobToBlobAngle(wmcBlobs[wmcPattern_M], wmcBlobs[wmcPattern_F]));
		//TODO: verify correct pattern allocation (from deviations in wmcAlt1 to wmcAlt4)
		wmcAlt = (wmcAlt1 + wmcAlt2 + wmcAlt3 + wmcAlt4) / 4;
		wmcX = wmcAlt * tanf((wmcBlobs[wmcPattern_M].x_angle + wmcBlobs[wmcPattern_F].x_angle)/2 + pitchActual*M_PI/180 + WMC_CAL_X);
		wmcY = wmcAlt * tanf((wmcBlobs[wmcPattern_M].y_angle + wmcBlobs[wmcPattern_F].y_angle)/2 + pitchActual*M_PI/180 + WMC_CAL_X);

		//calculate mean sensor reading, convert to altitude and correct for tilt
		irAlt_raw = gp2y0a60sz0f_valueToDistance(gp2y0a60sz0f_value_sum / POSCTRL_UPDATE_RATE_DIVIDER);
		gp2y0a60sz0f_value_sum = 0;
		tilt = atanf(hypotf(tanf(rollActual *M_PI/180), tanf(pitchActual *M_PI/180)));
		irAlt = irAlt_raw * cosf(tilt);

		posCtrlCounter = 0;
	}

	return 0;
}


//saves desired control values to given pointers
uint8_t positionControl_getRPYT(float *roll, float *pitch, float *yaw, uint16_t *thrust)
{
	*roll = rollDesired;
	*pitch = pitchDesired;
	*yaw = yawDesired;
	*thrust = thrustDesired;
	return 0;
}


//returns the angle between two blobs
static float wmcBlobToBlobAngle(struct WmcBlob wmcBlob1, struct WmcBlob wmcBlob2)
{
	return hypotf((wmcBlob1.x_angle - wmcBlob2.x_angle), (wmcBlob1.y_angle - wmcBlob2.y_angle));
}


//returns distance from point to line segment in 2D, from http://stackoverflow.com/a/11172574/3658125
static float pointToLineSegmentDistance2D(float x, float y, float x1, float y1, float x2, float y2)
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


//finds blob id for each point in pattern, stores it in wmcPattern_F/L/M/R TODO: ugly, optimize
static void findWmcPatternBlobMapping(struct WmcBlob WMCBlobs[4])
{
	float distance;
	//2 0 1
	float shortestDistance = pointToLineSegmentDistance2D(WMCBlobs[2].x, WMCBlobs[2].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[1].x, WMCBlobs[1].y);
	uint8_t pattern_m = 2; uint8_t pattern_f = 3; uint8_t pattern_a = 0;
	//3 0 1
	distance = pointToLineSegmentDistance2D(WMCBlobs[3].x, WMCBlobs[3].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[1].x, WMCBlobs[1].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 3; pattern_f = 2; pattern_a = 0;}
	//1 0 2
	distance = pointToLineSegmentDistance2D(WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[2].x, WMCBlobs[2].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 1; pattern_f = 3; pattern_a = 0;}
	//3 0 2
	distance = pointToLineSegmentDistance2D(WMCBlobs[3].x, WMCBlobs[3].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[2].x, WMCBlobs[2].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 3; pattern_f = 1; pattern_a = 0;}
	//1 0 3
	distance = pointToLineSegmentDistance2D(WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 1; pattern_f = 2; pattern_a = 0;}
	//2 0 3
	distance = pointToLineSegmentDistance2D(WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 2; pattern_f = 1; pattern_a = 0;}
	//0 1 2
	distance = pointToLineSegmentDistance2D(WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[2].x, WMCBlobs[2].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 0; pattern_f = 3; pattern_a = 1;}
	//3 1 2
	distance = pointToLineSegmentDistance2D(WMCBlobs[3].x, WMCBlobs[3].y, WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[2].x, WMCBlobs[2].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 3; pattern_f = 0; pattern_a = 1;}
	//0 1 3
	distance = pointToLineSegmentDistance2D(WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 0; pattern_f = 2; pattern_a = 1;}
	//2 1 3
	distance = pointToLineSegmentDistance2D(WMCBlobs[2].x, WMCBlobs[2].y, WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 2; pattern_f = 0; pattern_a = 1;}
	//0 2 3
	distance = pointToLineSegmentDistance2D(WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[2].x, WMCBlobs[2].y, WMCBlobs[3].x, WMCBlobs[3].y);
	if(distance < shortestDistance) { shortestDistance = distance; pattern_m = 0; pattern_f = 1; pattern_a = 2;}
	//1 2 3
	distance = pointToLineSegmentDistance2D(WMCBlobs[1].x, WMCBlobs[1].y, WMCBlobs[2].x, WMCBlobs[2].y, WMCBlobs[3].x, WMCBlobs[3].y);
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
LOG_ADD(LOG_UINT16, blob_0_x, &wmcBlobs[0].x)
LOG_ADD(LOG_UINT16, blob_0_y, &wmcBlobs[0].y)
LOG_ADD(LOG_UINT16, blob_1_x, &wmcBlobs[1].x)
LOG_ADD(LOG_UINT16, blob_1_y, &wmcBlobs[1].y)
LOG_ADD(LOG_UINT16, blob_2_x, &wmcBlobs[2].x)
LOG_ADD(LOG_UINT16, blob_2_y, &wmcBlobs[2].y)
LOG_ADD(LOG_UINT16, blob_3_x, &wmcBlobs[3].x)
LOG_ADD(LOG_UINT16, blob_3_y, &wmcBlobs[3].y)
LOG_ADD(LOG_UINT8, pattern_f, &wmcPattern_F)
LOG_ADD(LOG_UINT8, pattern_l, &wmcPattern_L)
LOG_ADD(LOG_UINT8, pattern_m, &wmcPattern_M)
LOG_ADD(LOG_UINT8, pattern_r, &wmcPattern_R)
LOG_ADD(LOG_FLOAT, wmcYaw, &wmcYaw)
LOG_ADD(LOG_FLOAT, wmcAlt, &wmcAlt)
LOG_ADD(LOG_FLOAT, wmcX, &wmcX)
LOG_ADD(LOG_FLOAT, wmcY, &wmcY)
LOG_GROUP_STOP(wmc)
