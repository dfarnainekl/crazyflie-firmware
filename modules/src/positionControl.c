/*
 * wmcPosition.c
 *
 *  Created on: 01.05.2015
 *      Author: daniel
 */

#define DEBUG_MODULE "POSITIONCONTROL"

#include "positionControl.h"

#include <stdlib.h>
#include <math.h>
#include "debug.h"
#include "log.h"
#include "param.h"
#include "commander.h"
#include "imu.h"
#include "pid.h"
#include "sensfusion6.h"
#include "wiiMoteCam.h"
#include "gp2y0a60sz0f.h"
#include <stm32f4xx.h>


#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

//positionControl
bool positionControlActive = false;          // Currently in positionControl mode
bool setPositionControlActive = false;      // positionControl mode has just been activated

//mode defines if pattern or single point + ir altitude is used
static uint8_t posCtrlMode = POSCTRL_MODE_PATTERN;

//positionControl_update() gets called with IMU_UPDATE_FREQ Hz, gets divided down
static uint32_t posCtrlCounter = 0;
#define POSCTRL_UPDATE_RATE_DIVIDER 5 //100Hz
#define POSCTRL_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / POSCTRL_UPDATE_RATE_DIVIDER))

//actual RPY values
static float rollActual = 0;
static float pitchActual = 0;
static float yawActual = 0;

//raw (not compensated for yaw) desired R/P values
static float rollDesired_raw = 0;
static float pitchDesired_raw = 0;

//desired RPYT values, get read and applied by the stabilizer
static float rollDesired = 0;
static float pitchDesired = 0;
static float yawRateDesired = 0;
static uint16_t thrustDesired = 0;

// Infrared Althold stuff
static float gp2y0a60sz0f_value_sum = 0; //for averaging ir distance sensor readings
static float irAlt_raw = 0; //raw (i.e. not corrected for tilt) altitude above ground from ir distance sensor
static float tilt = 0; //tilt in rad used for correcting altitude reading
static float irAlt = 0; // altitude above ground from ir distance sensor

// wmcTracking stuff
static uint8_t wmcStatus = 0; //status of wmc stuff, see defines in positionControl.h
static struct WmcBlob wmcBlobs[4]={{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}; //all four wiiMote cam blobs
static uint8_t wmcBlobCount = 0; //number of recognized blobs
static uint8_t wmcBlobsVisible = 0; //bit 0-3 represent blob 0-3, 1 if visible, 0 if not
static uint8_t wmcPattern_F = 0; //blob id of front led in pattern
static uint8_t wmcPattern_L = 1; //blob id of left led in pattern
static uint8_t wmcPattern_M = 2; //blob id of middle led in pattern
static uint8_t wmcPattern_R = 3; //blob id of right led in pattern
static float wmcAlt1 = 0; //altitude calculated from wmc + pattern from L-R distance, in mm
static float wmcAlt2 = 0; //altitude calculated from wmc + pattern from L-F distance, in mm
static float wmcAlt3 = 0; //altitude calculated from wmc + pattern from R-F distance, in mm
static float wmcAlt4 = 0; //altitude calculated from wmc + pattern from M-F distance, in mm
static float wmcAlt = 0; //altitude calculated from wmc + pattern, in mm
static float wmcAltDeviationsSum = 0; //sum of devaiations of wmcAlt1/2/3/4 compared to wmcAlt, used to verify pattern recognition

//actual position and yaw
static float position_alt = 0; //altitude, in mm
static float position_yaw = 0; //yaw angle, in degree
static float position_x_raw = 0; //x position relative to cfn (i.e. not yaw compensated), in mm
static float position_y_raw = 0; //y position relative to cfn (i.e. not yaw compensated), in mm
static float position_x = 0; //x position relative to pattern when in pattern mode, relative to cnf when in pint mode, in mm
static float position_y = 0; //y position relative to pattern when in pattern mode, relative to cnf when in pint mode, in mm

//desired position and yaw
static float position_desired_alt = 700; //altitude, in mm
static float position_desired_yaw = 0; //yaw angle, in degree
static float position_desired_x = 0; //x position, in mm
static float position_desired_y = 0; //y position, in mm

//pid's
PidObject pidAlt;
PidObject pidYaw;
PidObject pidX;
PidObject pidY;

//out of view timeout
static uint32_t outOfView_counter = 0; //counts up when wmc_status is not WMC_STATUS_OK
static float outOfView_timeout = OUT_OF_VIEW_TIMEOUT; //time in s after which timeout occurs

//random stuff
static int i; //for for-loops


//static function prototypes
static float constrain(float value, const float minVal, const float maxVal);
static float wmcBlobToBlobAngle(struct WmcBlob wmcBlob1, struct WmcBlob wmcBlob2);
static float constrainAngle180(float angle);
static float pointToLineSegmentDistance2D(float x, float y, float x1, float y1, float x2, float y2);
static void findWmcPatternBlobMapping(struct WmcBlob WMCBlob[4]);


//initializes positionControl
uint8_t positionControl_init()
{
	wmc_init(); //settings in wiiMoteCam.h, alternatively: wmc_init_basic();
	gp2y0a60sz0f_init();

	pidInit(&pidAlt, position_desired_alt, PID_ALT_P, PID_ALT_I, PID_ALT_D, POSCTRL_UPDATE_DT);
	pidInit(&pidYaw, position_desired_yaw, PID_YAW_P, PID_YAW_I, PID_YAW_D, POSCTRL_UPDATE_DT);
	pidInit(&pidX, position_desired_x, PID_X_P, PID_X_I, PID_X_D, POSCTRL_UPDATE_DT);
	pidInit(&pidY, position_desired_y, PID_Y_P, PID_Y_I, PID_Y_D, POSCTRL_UPDATE_DT);
	pidSetIntegralLimit(&pidAlt, PID_ALT_INTEGRATION_LIMIT_HIGH);
	pidSetIntegralLimitLow(&pidAlt, PID_ALT_INTEGRATION_LIMIT_LOW);

	return 0;
}


//updates positionControl, has to get called at IMU_UPDATE_FREQ Hz
uint8_t positionControl_update()
{
	sensfusion6GetEulerRPY(&rollActual, &pitchActual, &yawActual); //get actual roll, pitch and yaw angles
	gp2y0a60sz0f_value_sum += gp2y0a60sz0f_getValue(); //add ir distance sensor reading to sum (to be divided later to get mean value)
	commanderGetPositionControl(&positionControlActive, &setPositionControlActive);

	//positionControl has just been activated
	if(setPositionControlActive)
	{
		DEBUG_PRINT("posCtrl on\n");

		//reset pid's
		pidReset(&pidAlt);
		pidReset(&pidYaw);
		pidReset(&pidX);
		pidReset(&pidY);

		thrustDesired = constrain(THRUST_HOVER + pidUpdate(&pidAlt, position_alt, true), THRUST_MIN, THRUST_MAX);
		yawRateDesired = constrain(pidUpdate(&pidYaw, position_yaw, true), YAWRATE_MIN, YAWRATE_MAX);
		pitchDesired = -constrain(pidUpdate(&pidX, position_x, true), PITCH_MIN, PITCH_MAX);
		rollDesired = constrain(pidUpdate(&pidY, position_y, true), ROLL_MIN, ROLL_MAX);

		outOfView_counter = 0;
	}

	if (++posCtrlCounter >= POSCTRL_UPDATE_RATE_DIVIDER) //100Hz
	{
		wmc_readBlobs(&wmcBlobs); //get wiiMoteCam blobs

		//find number of recognized blobs
		uint8_t wmcBlobCountTemp = 0;
		uint8_t wmcBlobsVisibleTemp = 0;
		for(i=0;i<4;i++)
		{
			if(wmcBlobs[i].isVisible)
			{
				wmcBlobCountTemp++;
				wmcBlobsVisibleTemp |= (1<<i);
			}
		}
		wmcBlobCount = wmcBlobCountTemp;
		wmcBlobsVisible = wmcBlobsVisibleTemp;

		//calculate mean sensor reading, smooth data, convert to altitude and correct for tilt
		irAlt_raw = 0.7*irAlt_raw + 0.3*gp2y0a60sz0f_valueToDistance(gp2y0a60sz0f_value_sum / POSCTRL_UPDATE_RATE_DIVIDER);
		gp2y0a60sz0f_value_sum = 0;
		tilt = atanf(hypotf(tanf(rollActual *M_PI/180), tanf(pitchActual *M_PI/180)));
		irAlt = irAlt_raw * cosf(tilt);

		if(posCtrlMode == POSCTRL_MODE_PATTERN)
		{
			if(wmcBlobCount < 4) wmcStatus = WMC_STATUS_BLOBCOUNT_LOW_ERROR; //not enough blobs found
			else //blobcount ok, calculate position & yaw angle relative to T-pattern (from http://www.cogsys.cs.uni-tuebingen.de/publikationen/2010/Wenzel2010imav.pdf)
			{
				findWmcPatternBlobMapping(wmcBlobs); //find blob id for each point in pattern

				//calculate altitude
				wmcAlt1 = PATTERN_DISTANCE_L_R / tanf(wmcBlobToBlobAngle(wmcBlobs[wmcPattern_L], wmcBlobs[wmcPattern_R]));
				wmcAlt2 = PATTERN_DISTANCE_L_F / tanf(wmcBlobToBlobAngle(wmcBlobs[wmcPattern_L], wmcBlobs[wmcPattern_F]));
				wmcAlt3 = PATTERN_DISTANCE_R_F / tanf(wmcBlobToBlobAngle(wmcBlobs[wmcPattern_R], wmcBlobs[wmcPattern_F]));
				wmcAlt4 = PATTERN_DISTANCE_M_F / tanf(wmcBlobToBlobAngle(wmcBlobs[wmcPattern_M], wmcBlobs[wmcPattern_F]));
				wmcAlt = (wmcAlt1 + wmcAlt2 + wmcAlt3 + wmcAlt4) / 4;

				//verify correct pattern allocation (from deviations in wmcAlt1 to wmcAlt4) TODO: other methods of verifying
				wmcAltDeviationsSum = fabsf(wmcAlt - wmcAlt1) + fabsf(wmcAlt - wmcAlt2) + fabsf(wmcAlt - wmcAlt3) + fabsf(wmcAlt - wmcAlt4);
				if(wmcAltDeviationsSum > wmcAlt*WMC_PATTERN_CORRECT_THRESHOLD) wmcStatus = WMC_STATUS_PATTERN_ERROR;//pattern allocation is probably not correct
				else
				{
					wmcStatus = WMC_STATUS_OK;
					//calculate position & yaw angle
					position_alt = POS_SMOOTHING_ALT*position_alt + (1-POS_SMOOTHING_ALT)*wmcAlt;
					position_x_raw = -position_alt * tanf((wmcBlobs[wmcPattern_M].x_angle + wmcBlobs[wmcPattern_F].x_angle)/2 + pitchActual*M_PI/180 + WMC_CAL_X);
					position_y_raw = position_alt * tanf((wmcBlobs[wmcPattern_M].y_angle + wmcBlobs[wmcPattern_F].y_angle)/2 + rollActual*M_PI/180 + WMC_CAL_Y);
					position_yaw = POS_SMOOTHING_YAW*position_yaw + (1-POS_SMOOTHING_YAW)*(-atan2f(wmcBlobs[wmcPattern_M].x - wmcBlobs[wmcPattern_F].x, wmcBlobs[wmcPattern_M].y - wmcBlobs[wmcPattern_F].y)*180/M_PI);

					//position relative to pattern
					position_x = POS_SMOOTHING_X*position_x + (1-POS_SMOOTHING_X)*(- position_y_raw*sin(position_yaw*M_PI/180) + position_x_raw*cos(position_yaw*M_PI/180));
					position_y = POS_SMOOTHING_Y*position_y + (1-POS_SMOOTHING_Y)*(position_x_raw*sin(position_yaw*M_PI/180) + position_y_raw*cos(position_yaw*M_PI/180));
				}
			}
		}
		else if(posCtrlMode == POSCTRL_MODE_POINT)
		{
			if(wmcBlobCount < 1) wmcStatus = WMC_STATUS_BLOBCOUNT_LOW_ERROR; //not enough blobs found
			else if(wmcBlobCount > 1) wmcStatus = WMC_STATUS_BLOBCOUNT_HIGH_ERROR; //too many blobs found
			else //blobcount ok
			{
				wmcStatus = WMC_STATUS_OK;
				//calculate position relative to ir point on surface, for altitude use ir altitude measurements
				position_alt = POS_SMOOTHING_ALT*position_alt + (1-POS_SMOOTHING_ALT)*irAlt;
				position_x = POS_SMOOTHING_X*position_x + (1-POS_SMOOTHING_X)*(-position_alt * tanf(wmcBlobs[0].x_angle + pitchActual*M_PI/180 + WMC_CAL_X));
				position_y = POS_SMOOTHING_Y*position_y + (1-POS_SMOOTHING_Y)*(position_alt * tanf(wmcBlobs[0].y_angle + rollActual*M_PI/180 + WMC_CAL_Y));
				position_yaw = 0; //pid output gets set to 0 when mode is POSCTRL_MODE_POINT
			}
		}
		else DEBUG_PRINT("unknown posCtrlMode [ERROR].\n");

		//positionControl is active, update pid
		if(positionControlActive)
		{
			if(wmcStatus == WMC_STATUS_OK) outOfView_counter = 0;
			else
			{
				outOfView_counter++;
				if(((float)outOfView_counter*POSCTRL_UPDATE_DT > outOfView_timeout) && (outOfView_timeout != 0.0))
				{
					DEBUG_PRINT("posCtrl off (out of view timeout)\n");
					commanderSetPositionControl(false);
		    		//TODO: proper landing?
				}
			}

			pidSetDesired(&pidAlt, position_desired_alt);
			pidSetDesired(&pidYaw, position_desired_yaw);
			pidSetDesired(&pidX, position_desired_x);
			pidSetDesired(&pidY, position_desired_y);

			pidSetError(&pidYaw,constrainAngle180(pidGetDesired(&pidYaw) - position_yaw)); //constrain/wrap yaw error to -180 to 180
			if(posCtrlMode == POSCTRL_MODE_POINT) yawRateDesired = 0;
			else yawRateDesired = constrain(pidUpdate(&pidYaw, position_yaw, false), YAWRATE_MIN, YAWRATE_MAX);
			thrustDesired = (uint16_t)constrain(THRUST_HOVER + pidUpdate(&pidAlt, position_alt, true), THRUST_MIN, THRUST_MAX);
			pitchDesired_raw = - constrain(pidUpdate(&pidX, position_x, true), PITCH_MIN, PITCH_MAX);
			rollDesired_raw = constrain(pidUpdate(&pidY, position_y, true), ROLL_MIN, ROLL_MAX);
			//when in pattern mode, the x/y position is relative to the pattern --> desired pitch/roll needs yaw compensation
			pitchDesired = - rollDesired_raw*sin(position_yaw*M_PI/180) + pitchDesired_raw*cos(position_yaw*M_PI/180);
			rollDesired = pitchDesired_raw*sin(position_yaw*M_PI/180) + rollDesired_raw*cos(position_yaw*M_PI/180);
		}

		posCtrlCounter = 0;
	}

	return 0;
}


//saves desired control values to given pointers
uint8_t positionControl_getRPYT(float *roll, float *pitch, float *yawRate, uint16_t *thrust)
{
	*roll = rollDesired;
	*pitch = pitchDesired;
	*yawRate = yawRateDesired;
	*thrust = thrustDesired;
	return 0;
}

uint8_t positionControl_getWmcStatus()
{
	return wmcStatus;
}


// Constrain value between min and max
static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

//returns the angle between two blobs
static float wmcBlobToBlobAngle(struct WmcBlob wmcBlob1, struct WmcBlob wmcBlob2)
{
	return hypotf((wmcBlob1.x_angle - wmcBlob2.x_angle), (wmcBlob1.y_angle - wmcBlob2.y_angle));
}

//constrains angle to -180 to 180 TODO doesnt work vith very large angles
static float constrainAngle180(float angle)
{
    if(angle < -180) return angle + 360;
    else if(angle > 180) return angle - 360;
    else return angle;
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

//finds blob id for each point in pattern, stores it in wmcPattern_F/L/M/R TODO: ugly --> optimize!
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
	distance = pointToLineSegmentDistance2D(WMCBlobs[2].x, WMCBlobs[2].y, WMCBlobs[0].x, WMCBlobs[0].y, WMCBlobs[3].x, WMCBlobs[3].y);
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
LOG_ADD(LOG_FLOAT, alt_raw, &irAlt_raw) //raw (i.e. not corrected for tilt) altitude above ground from ir distance sensor
LOG_ADD(LOG_FLOAT, tilt, &tilt) //tilt in rad used for correcting altitude reading
LOG_ADD(LOG_FLOAT, alt, &irAlt) //altitude above ground from ir distance sensor
LOG_GROUP_STOP(irAlt)

LOG_GROUP_START(wmc)
LOG_ADD(LOG_UINT8, blobCount, &wmcBlobCount) //number of recognized blobs
LOG_ADD(LOG_UINT8, blobsValid, &wmcBlobsVisible) //visibility status, bit 0-3 represent blob 0-3, 1 if visible, 0 if not
LOG_ADD(LOG_UINT16, blob_0_x, &wmcBlobs[0].x) //blob 0 x position
LOG_ADD(LOG_UINT16, blob_0_y, &wmcBlobs[0].y) //blob 0 y position
LOG_ADD(LOG_UINT16, blob_1_x, &wmcBlobs[1].x) //blob 1 x position
LOG_ADD(LOG_UINT16, blob_1_y, &wmcBlobs[1].y) //blob 1 y position
LOG_ADD(LOG_UINT16, blob_2_x, &wmcBlobs[2].x) //blob 2 x position
LOG_ADD(LOG_UINT16, blob_2_y, &wmcBlobs[2].y) //blob 2 y position
LOG_ADD(LOG_UINT16, blob_3_x, &wmcBlobs[3].x) //blob 3 x position
LOG_ADD(LOG_UINT16, blob_3_y, &wmcBlobs[3].y) //blob 3 y position
LOG_ADD(LOG_UINT8, pattern_f, &wmcPattern_F) //blob id of front led in pattern
LOG_ADD(LOG_UINT8, pattern_l, &wmcPattern_L) //blob id of left led in pattern
LOG_ADD(LOG_UINT8, pattern_m, &wmcPattern_M) //blob id of middle led in pattern
LOG_ADD(LOG_UINT8, pattern_r, &wmcPattern_R) //blob id of right led in pattern
LOG_ADD(LOG_FLOAT, wmcAlt, &wmcAlt) //altitude calculated from wmc + pattern, in mm
LOG_ADD(LOG_FLOAT, wmcAltDS, &wmcAltDeviationsSum) //sum of devaiations of wmcAlt1/2/3/4 compared to wmcAlt, used to verify pattern recognition
LOG_GROUP_STOP(wmc)

LOG_GROUP_START(pos)
LOG_ADD(LOG_UINT8, wmcStatus, &wmcStatus) //status of wmc stuff, see defines in positionControl.h
LOG_ADD(LOG_FLOAT, alt, &position_alt) //altitude, in mm
LOG_ADD(LOG_FLOAT, yaw, &position_yaw) //yaw angle, in degree
LOG_ADD(LOG_FLOAT, x, &position_x) //x position, in mm
LOG_ADD(LOG_FLOAT, y, &position_y) //y position, in mm
LOG_GROUP_STOP(pos)

LOG_GROUP_START(posCtrlPid)
LOG_ADD(LOG_FLOAT, err_yaw, &pidYaw.error) //yaw error
LOG_ADD(LOG_FLOAT, err_alt, &pidAlt.error) //altitude error
LOG_ADD(LOG_FLOAT, err_x, &pidX.error) //x error
LOG_ADD(LOG_FLOAT, err_y, &pidY.error) //y error
LOG_ADD(LOG_FLOAT, des_yawRate, &yawRateDesired) //output of yaw pid
LOG_ADD(LOG_UINT16, des_thrust, &thrustDesired) //output of altitude pid
LOG_ADD(LOG_FLOAT, des_pitch, &pitchDesired) //output of x pid
LOG_ADD(LOG_FLOAT, des_roll, &rollDesired) //output of y pid
LOG_GROUP_STOP(posCtrlPid)


PARAM_GROUP_START(posCtrl)
PARAM_ADD(PARAM_UINT8, mode, &posCtrlMode) //mode defines if pattern or single point + ir altitude is used
PARAM_ADD(PARAM_FLOAT, des_alt, &position_desired_alt) // desired altitude in mm
PARAM_ADD(PARAM_FLOAT, des_yaw, &position_desired_yaw) // desired yaw in deg -180 to 180
PARAM_ADD(PARAM_FLOAT, des_x, &position_desired_x) // desired x position in mm
PARAM_ADD(PARAM_FLOAT, des_y, &position_desired_y) // desired x position in mm
PARAM_ADD(PARAM_FLOAT, oov_to, &outOfView_timeout) // out of view timeout in s
PARAM_GROUP_STOP(posCtrl)

PARAM_GROUP_START(posCtrlPid)
PARAM_ADD(PARAM_FLOAT, alt_p, &pidAlt.kp) //p factor
PARAM_ADD(PARAM_FLOAT, alt_i, &pidAlt.ki) //i factor
PARAM_ADD(PARAM_FLOAT, alt_d, &pidAlt.kd) //d factor
PARAM_ADD(PARAM_FLOAT, yaw_p, &pidYaw.kp) //p factor
PARAM_ADD(PARAM_FLOAT, yaw_i, &pidYaw.ki) //i factor
PARAM_ADD(PARAM_FLOAT, yaw_d, &pidYaw.kd) //d factor
PARAM_ADD(PARAM_FLOAT, x_p, &pidX.kp) //p factor
PARAM_ADD(PARAM_FLOAT, x_i, &pidX.ki) //i factor
PARAM_ADD(PARAM_FLOAT, x_d, &pidX.kd) //d factor
PARAM_ADD(PARAM_FLOAT, y_p, &pidY.kp) //p factor
PARAM_ADD(PARAM_FLOAT, y_i, &pidY.ki) //i factor
PARAM_ADD(PARAM_FLOAT, y_d, &pidY.kd) //d factor
PARAM_GROUP_STOP(posCtrlPid)
