/*
 * vl53l0x.c
 *
 *  Created on: 16.06.2016
 *      Author: daniel
 */

#define DEBUG_MODULE "VL53L0X"

#include "stm32fxxx.h"
// TA: Maybe not so good to bring in these dependencies...
#include "debug.h"
#include "eprintf.h"
#include "i2cdev.h"

#include "vl53l0x.h"


uint8_t vl53l0x_init()
{
	if(!i2cdevWriteByte(I2C1_DEV, VL_ADR, 0x00, 0x02))
	{
		DEBUG_PRINT("I2C connection [FAIL].\n");
		return 0;
	}
	return 1;
}


uint16_t vl53l0x_getDistance()
{
	uint8_t distance_high, distance_low, status;

	if(!i2cdevRead(I2C1_DEV,VL_ADR, 0x14, 1, &status)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevRead(I2C1_DEV,VL_ADR, 0x1E, 1, &distance_high)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevRead(I2C1_DEV,VL_ADR, 0x1F, 1, &distance_low)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }

	uint16_t  distance = ((uint16_t)distance_high << 8) | distance_low;
	//distance &= 0x0FFF;

	if((status == 64)/* || (distance > DISTANCE_ERROR_THRESHOLD)*/)
	{
		DEBUG_PRINT("Status or Distance [ERROR]\n");
		return 0;
	}

	return distance;
}
