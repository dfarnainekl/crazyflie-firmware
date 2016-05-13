/*
 * gp2y0a60sz0f.c
 *
 *  Created on: 22.03.2015
 *      Author: Franky333
 */

#define DEBUG_MODULE "GP2Y0A60SZ0F"

#include "stm32fxxx.h"
// TA: Maybe not so good to bring in these dependencies...
#include "debug.h"
#include "eprintf.h"

#include "gp2y0a60sz0f.h"
#include "deck_analog.h"


uint8_t gp2y0a60sz0f_init()
{
	adcInit();

	DEBUG_PRINT("initialization [OK].\n");
	return 1;
}


uint16_t gp2y0a60sz0f_getValue() //FIXME: doesnt seem to work anymore after using bitcraze adc driver
{
	return analogRead(11);
}


float gp2y0a60sz0f_valueToDistance(uint16_t value)
{
	return (500000/(float)value)-200; //TODO: better calculation
}
