/*
 * wmcPosition.h
 *
 *  Created on: 01.05.2015
 *      Author: daniel
 */

#ifndef MODULES_INTERFACE_POSITIONCONTROL_H_
#define MODULES_INTERFACE_POSITIONCONTROL_H_

#include <stdint.h>

uint8_t positionControl_init(); //initializes positionControl
uint8_t positionControl_update(); //updates positionControl, has to get called at IMU_UPDATE_FREQ Hz
uint8_t positionControl_getRPYT(float *roll, float *pitch, float *yaw, uint16_t *thrust); //saves desired control values to given pointers


#endif /* MODULES_INTERFACE_POSITIONCONTROL_H_ */
