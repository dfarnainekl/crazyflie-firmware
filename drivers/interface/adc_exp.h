/*
 * adc_exp.h
 *
 *  Created on: 22.03.2015
 *      Author: Franky333
 */

#ifndef DRIVERS_INTERFACE_ADC_EXP_H_
#define DRIVERS_INTERFACE_ADC_EXP_H_

uint8_t adc_exp_init(uint8_t channel); //use channel 0-4 as shown on bitcraze wiki expansion board site
uint16_t adc_exp_getValue(uint8_t channel); //use channel 0-4 as shown on bitcraze wiki expansion board site


#endif /* DRIVERS_INTERFACE_ADC_EXP_H_ */
