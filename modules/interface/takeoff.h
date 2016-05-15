/*
 * takeoff.h
 *
 *  Created on: 15.08.2015
 *      Author: Franky333
 */

#ifndef MODULES_INTERFACE_TAKEOFF_H_
#define MODULES_INTERFACE_TAKEOFF_H_

#include <stdint.h>

#define TAKEOFF_THRUST 51000
#define TAKEOFF_YAWRATE 0.0
#define TAKEOFF_PITCH 0.0
#define TAKEOFF_ROLL 0.0
#define TAKEOFF_TIMEOUT 2.0 //time in s after which takeoff gets aborted if pattern not recognized, 0.0 == disabled

void takeoff_update();
void takeoff_getRPYT(float *roll, float *pitch, float *yawRate, uint16_t *thrust);

#endif /* MODULES_INTERFACE_TAKEOFF_H_ */
