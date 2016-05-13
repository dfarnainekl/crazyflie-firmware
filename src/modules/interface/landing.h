/*
 * landing.h
 *
 *  Created on: 15.08.2015
 *      Author: Franky333
 */

#ifndef MODULES_INTERFACE_LANDING_H_
#define MODULES_INTERFACE_LANDING_H_

#include <stdint.h>

#define LANDING_THRUST 30000
#define LANDING_YAWRATE 0.0
#define LANDING_PITCH 0.0
#define LANDING_ROLL 0.0
#define LANDING_DURATION 1.0 //time in s for slow descent, after which motors get turned off
#define LANDING_DELAY 1.0 //time in s for falling, after which landing is finished

void landing_update();
void landing_getRPYT(float *roll, float *pitch, float *yawRate, uint16_t *thrust);

#endif /* MODULES_INTERFACE_LANDING_H_ */
