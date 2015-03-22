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
#include "adc_exp.h"


uint8_t gp2y0a60sz0f_init()
{
	if(!adc_exp_init()) return 0;

	DEBUG_PRINT("initialization [OK].\n");
	return 1;
}
