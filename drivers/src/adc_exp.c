/*
 * adc_exp.c
 *
 *  Created on: 22.03.2015
 *      Author: Franky333
 */

#define DEBUG_MODULE "ADC_EXP"

#include "stm32fxxx.h"
// TA: Maybe not so good to bring in these dependencies...
#include "debug.h"
#include "eprintf.h"

#include "adc_exp.h"


uint8_t adc_exp_init()
{

	DEBUG_PRINT("initialization [OK].\n");
	return 1;
}
