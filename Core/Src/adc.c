/*
 * adc.c
 *
 *  Created on: Nov 1, 2024
 *      Author: pengu
 */

#include "adc.h"

#define SLOPE_CAL_LAPTOP 818
#define INTERCEPT_CAL_LAPTOP 12600
#define SLOPE_CAL_DESKTOP 818
#define INTERCEPT_CAL_DESKTOP 12600
#define SCALE_CAL 10000

/* Macro for which device I am using for calibration
 * 0: Laptop
 * 1: desktop
 */
#define DEVICE 0

void adc_init(void){
	RCC->AHB2ENR |= (RCC_AHB2ENR_ADCEN | RCC_AHB2ENR_GPIOAEN);
	ADC123_COMMON->CCR |= (1 << ADC_CCR_CKMODE_Pos);

	// configure GPIO for PA0
	// sets PA0 to analog mode
	GPIOA->MODER |= (GPIO_MODER_MODE0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0);
	GPIOA->ASCR |= (GPIO_ASCR_ASC0);

	  // Power up ADC and Voltage Regulator
	ADC1->CR &= ~(ADC_CR_DEEPPWD);
	ADC1->CR |= (ADC_CR_ADVREGEN);

	for (uint32_t i = 0; i < 50000; i++);

	// configure differential select
	// configure channel 5 for single ended (PA0)
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

	//calibrate
	// ensure ADC is disabled
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);
	// start calibration
	ADC1->CR |= (ADC_CR_ADCAL);
	while(ADC1->CR & ADC_CR_ADCAL);

	//enable ADC
	ADC1->ISR |= (ADC_ISR_ADRDY);
	ADC1->CR |= (ADC_CR_ADEN);
	while(!(ADC1->ISR & ADC_ISR_ADRDY));

	//configure sequence
	ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

	// sets CFGR register to 0
	// includes Sample time = 00 -> 2.5 clocks
	// includes Single Conversion mode enabled
	ADC1->CFGR = 0;
	ADC1->SMPR1 &= ~(ADC_SMPR1_SMP5);
	ADC1->SMPR1 |= (0x0 << ADC_SMPR1_SMP5_Pos);

	//configure interrupts
	// configures end of conversion interrupt
	ADC1->IER |= (ADC_IER_EOCIE);
	NVIC->ISER[0] = (1 << (ADC1_2_IRQn));


}

uint32_t calibrateValue(uint16_t in_val){
	uint32_t calibratedVal = 0;

	if(DEVICE){
		// Desktop calibration
		// outputs value in microvolts
		calibratedVal = SLOPE_CAL_DESKTOP*in_val - INTERCEPT_CAL_DESKTOP;
	}else {
		// laptop calibration
		// outputs value in microvolts
		calibratedVal = SLOPE_CAL_LAPTOP*in_val - INTERCEPT_CAL_LAPTOP;
	}
	// returns value in 10s of mV
	calibratedVal /= SCALE_CAL;

	return calibratedVal;


}


