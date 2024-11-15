/*
 * adc.h
 *
 *  Created on: Nov 1, 2024
 *      Author: pengu
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32l4xx_hal.h"

void adc_init(void);
uint32_t calibrateValue(uint16_t in_val);

#endif /* INC_ADC_H_ */
