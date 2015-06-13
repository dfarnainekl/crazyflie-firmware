/*
 * wmcPosition.h
 *
 *  Created on: 01.05.2015
 *      Author: daniel
 */

#ifndef MODULES_INTERFACE_POSITIONCONTROL_H_
#define MODULES_INTERFACE_POSITIONCONTROL_H_


#include <stdint.h>

//physical pattern layout distances in mm
#define PATTERN_DISTANCE_L_R 90
#define PATTERN_DISTANCE_M_F 60
#define PATTERN_DISTANCE_L_M (PATTERN_DISTANCE_L_R / 2)
#define PATTERN_DISTANCE_M_R (PATTERN_DISTANCE_L_R / 2)
#define PATTERN_DISTANCE_L_F (hypotf(PATTERN_DISTANCE_L_M,PATTERN_DISTANCE_M_F))
#define PATTERN_DISTANCE_R_F (hypotf(PATTERN_DISTANCE_M_R,PATTERN_DISTANCE_M_F))

//calibration angles of wmc in rad(to account for non-perfect mounting)
#define WMC_CAL_X 0
#define WMC_CAL_Y 0

//threshold for pattern allocation verification
#define WMC_PATTERN_CORRECT_THRESHOLD 0.1

//wmc status
#define WMC_STATUS_OK 0
#define WMC_STATUS_BLOBCOUNT_LOW_ERROR 1
#define WMC_STATUS_BLOBCOUNT_HIGH_ERROR 2
#define WMC_STATUS_PATTERN_ERROR 3

//posCtrl modes
#define POSCTRL_MODE_PATTERN 0
#define POSCTRL_MODE_POINT 1

//PID factors TODO: optimize
#define PID_ALT_P 70.0
#define PID_ALT_I 10.0
#define PID_ALT_D 30.0
#define PID_YAW_P 5.0
#define PID_YAW_I 0.0
#define PID_YAW_D 0.0
#define PID_X_P 0.04
#define PID_X_I 0.01
#define PID_X_D 0.03
#define PID_Y_P 0.04
#define PID_Y_I 0.01
#define PID_Y_D 0.03
//alt PID integral limits TODO: optimize
#define PID_ALT_INTEGRATION_LIMIT_HIGH 200000
#define PID_ALT_INTEGRATION_LIMIT_LOW -200000

//thrust needed to hover TODO: optimize, make dependend on battery voltage?
#define THRUST_HOVER 40000

//min/max values for rpyt TODO: optimize
#define THRUST_MIN 10000
#define THRUST_MAX 60000
#define YAWRATE_MIN -400
#define YAWRATE_MAX 400
#define PITCH_MIN -10
#define PITCH_MAX 10
#define ROLL_MIN -10
#define ROLL_MAX 10


uint8_t positionControl_init(); //initializes positionControl
uint8_t positionControl_update(); //updates positionControl, has to get called at IMU_UPDATE_FREQ Hz
uint8_t positionControl_getRPYT(float *roll, float *pitch, float *yawRate, uint16_t *thrust); //saves desired control values to given pointers


#endif /* MODULES_INTERFACE_POSITIONCONTROL_H_ */
