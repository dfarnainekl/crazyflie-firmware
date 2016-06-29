/*
 * vl53l0x.h
 *
 *  Created on: 16.06.2016
 *      Author: daniel
 */

#ifndef DRIVERS_INTERFACE_VL53L0X_H_
#define DRIVERS_INTERFACE_VL53L0X_H_


#define VL_ADR 0x29

#define DISTANCE_ERROR_THRESHOLD 3000


uint8_t vl53l0x_init();
uint16_t vl53l0x_getDistance();


#endif /* DRIVERS_INTERFACE_VL53L0X_H_ */
