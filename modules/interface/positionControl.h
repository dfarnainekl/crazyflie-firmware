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

//wmc status
#define WMC_STATUS_OK 0
#define WMC_STATUS_BLOBCOUNT_LOW_ERROR 1
#define WMC_STATUS_BLOBCOUNT_HIGH_ERROR 2
#define WMC_STATUS_PATTERN_ERROR 3

//posCtrl modes
#define POSCTRL_MODE_PATTERN 0
#define POSCTRL_MODE_POINT 1

uint8_t positionControl_init(); //initializes positionControl
uint8_t positionControl_update(); //updates positionControl, has to get called at IMU_UPDATE_FREQ Hz
uint8_t positionControl_getRPYT(float *roll, float *pitch, float *yaw, uint16_t *thrust); //saves desired control values to given pointers


#endif /* MODULES_INTERFACE_POSITIONCONTROL_H_ */
