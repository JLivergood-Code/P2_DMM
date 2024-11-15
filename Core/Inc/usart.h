/*
 * usarrt.h
 *
 *  Created on: Oct 31, 2024
 *      Author: pengu
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#define BRR_CALC (24000000 / 115200)

#include "adc.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <string.h>

void usart_init(void);
void usart_gpio_init(void);

// functions to write to USART terminal
void usart_print(char *in_string);
void usart_esc(char *in_string);
void updateFreq(uint32_t in_freq);
void updateDCValues(uint32_t avg);
void updateACValues(uint16_t p2p, uint16_t rms_avg, uint32_t freq);
void write_divider(void);
void write_terminal_int(uint32_t in_val, char* valType, uint8_t isFlt);
void clear_scrn(void);
void fltToStr(uint32_t N, char *str);
void intToStr(uint32_t N, char *str, uint8_t isFlt);

#endif /* INC_USART_H_ */
