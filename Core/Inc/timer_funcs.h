/*
 * timer_funcs.h
 *
 *  Created on: Oct 21, 2024
 *      Author: pengu
 */

#ifndef INC_TIMER_FUNCS_H_
#define INC_TIMER_FUNCS_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define CPU_FREQ 222

// CCR value, not currently used, copied from A3
#define RESET_CNT 500

void tim3_init(void);
void tim2_init(void);
void GPIO_init(void);

#endif /* INC_TIMER_FUNCS_H_ */
