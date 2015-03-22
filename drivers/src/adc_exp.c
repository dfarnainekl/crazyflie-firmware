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

bool adc_isInitialized = 0;

uint8_t adc_exp_init(uint8_t channel)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	switch(channel)
	{
		case 0: GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; break;
		case 1: GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; break;
		case 2: GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; break;
		case 3: GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; break;
		case 4: GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; break;
		default: DEBUG_PRINT("unknown analog channel [ERROR].\n"); return 0;
	}
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	if(!adc_isInitialized)
	{
		RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //enable adc 1 digital clock
		ADC->CCR |= ADC_CCR_ADCPRE; //APB2 divided by 8 to get adc analog clock
		ADC1->CR1 &= ~ADC_CR1_RES; //12bit
		uint32_t sampleTime = 0b010; //sample time; 000: 3 cycles 001: 15 cycles 010: 28 cycles 011: 56 cycles 100: 84 cycles 101: 112 cycles 110: 144 cycles 111: 480 cycles
		ADC1->SMPR2 = (sampleTime<<(3*7)) | (sampleTime<<(3*6)) | (sampleTime<<(3*5)) | (sampleTime<<(3*3)) | (sampleTime<<(3*2));
		ADC1->CR2 |= ADC_CR2_ADON; //adc enabled
		DEBUG_PRINT("initialization [OK].\n");
	}
	adc_isInitialized = 1;

	return 1;
}

uint16_t adc_exp_getValue(uint8_t channel)
{
	//set channel number to first place in conversion sequence (incl. conversion from bitcraze channel numbering to adc channels)
	switch(channel)
	{
		case 0: ADC1->SQR3 = 2; break;
		case 1: ADC1->SQR3 = 3; break;
		case 2: ADC1->SQR3 = 5; break;
		case 3: ADC1->SQR3 = 6; break;
		case 4: ADC1->SQR3 = 7; break;
		default: DEBUG_PRINT("unknown analog channel [ERROR].\n"); return 0;
	}
	ADC1->CR2 |= ADC_CR2_SWSTART;	//start conversion
	while(!(ADC1->SR & ADC_SR_EOC)); //wait until conversion finished
	return (uint16_t)ADC1->DR; //return  measured data
}
