/*
 * usart.c
 *
 *  Created on: Oct 31, 2024
 *      Author: pengu
 */

#include "usart.h"

#define NUM_DECIMAL 2
#define TRUE 1
#define FALSE 0

#define MAXmV 330
#define MAXGRAPH 24
#define NUMROWS 10

void usart_init(void){
	// calculates the baud rate
//	uint16_t baud_rate = CLK/RSCLK;

	// enables the clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	//enables receive interrupt
//	USART2->CR1 |= (USART_CR1_RXNEIE);
	//sets over sampling to 16 bits
	USART2->CR1 &= ~(USART_CR1_OVER8);
	//sets stop bits to 00
	USART2->CR2 &= ~(USART_CR2_STOP);


	// sets baud rate
	USART2->BRR = BRR_CALC;

	// enables USART to enable
	USART2->CR1 |= USART_CR1_UE;

	//enables receiving and transmitting
	USART2->CR1 |= (USART_CR1_TE);

	// enable interrupt in NVIC
//	NVIC -> ISER[1] = (1 << (USART2_IRQn & 0x1F));

}

void usart_gpio_init(void){
	// enables GPIO clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// sets mode to alternate mode
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOA->MODER |= (0x2 << GPIO_MODER_MODE2_Pos | 0x2 << GPIO_MODER_MODE3_Pos);

	// sets OTYPE to Push pull
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);

	// sets OPSEED to maximum value
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3);

	// sets alternate function to 7, which is USART
	GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL2_Pos | 0x7 << GPIO_AFRL_AFSEL3_Pos);
}

void usart_print(char *in_string){
	// sets index to 0 and gets the first
	uint8_t i;
//	uint8_t cur_char = in_string[index];

	// while not at the end of the string
	for(i = 0; in_string[i] != 0; i++){
		// waits until transmit is finished
		while(!(USART2->ISR & USART_ISR_TXE));
		// loads the character into the transmit register
		USART2->TDR = in_string[i];
	}

}

void usart_esc(char *in_string){
	uint8_t i;
	uint8_t esc_code = 0x1b;

	// sends escape code character
	while(!(USART2->ISR & USART_ISR_TXE));
	USART2->TDR = esc_code;


	// sends rest of string
	for(i = 0; in_string[i] != 0; i++){
		// waits until transmit is finished
		while(!(USART2->ISR & USART_ISR_TXE));
		// loads the character into the transmit register
		USART2->TDR = in_string[i];
	}
}
void clear_scrn(void){
	while(!(USART2->ISR & USART_ISR_TXE));
	usart_esc("[2J");	// clears screen
	usart_esc("[H"); // resets cursor position
}

void write_divider(void){
	usart_print("=====================================================");
	usart_esc("[#1");
	usart_print("=====================================================");
	usart_esc("[3;26H");

	for(int i = 0; i < NUMROWS; i++){
		usart_print("|");
		usart_esc("[1B");
		usart_esc("[1D");
	}

	usart_print("=====================================================");
	usart_esc("[#1");
	usart_print("=====================================================");

}


void write_terminal_int(uint32_t in_val, char *valType, uint8_t isFlt=TRUE){
	 // largest value is 65534 digits + 1 for safety
	 char *int_str = malloc(7);
	 intToStr(in_val, int_str, isFlt);

	 // ensures transmition register is empty when sending
	  while(!(USART2->ISR & USART_ISR_TXE));
	  usart_print(valType);
	  usart_print(int_str);


	  free(int_str);

}
void updateFreq(uint32_t in_freq){
	write_terminal_int(in_freq, "Freq: ", TRUE);
//	write_terminal_int(avg, " ;Uncalibrated: ");
	usart_esc("[1E");

}

void updateDCValues(uint32_t avg){
	uint32_t curCalVal = 0;
	uint8_t num_graph = 0;
	curCalVal = calibrateValue((uint16_t) avg);

	usart_esc("[4;22H");
	usart_esc("[1K");

	write_terminal_int(avg, "DC Avg (V): ", TRUE);

	num_graph = (MAXGRAPH * avg) / MAXmV;

	usart_esc("[0;27H");
	for(int i = 0; i < num_graph; i++) { usart_print("#"); }

	usart_esc("[0;27H");
	usart_print("|---|---|---|---|---|---|");
	usart_esc("[0;27H");
	usart_print("0  0.5 1.0 1.5 2.0 2.5 3.0");

}

void updateACValues(uint16_t p2p, uint16_t rms_avg, uint32_t freq){
	uint32_t curCalVal = 0;
	uint8_t numGraph = 0;

//	clear_scrn();

	curCalVal = calibrateValue((uint16_t) p2p);

	usart_esc("[4;27H");
	usart_esc("[0K");

	write_terminal_int(p2p, "Peak to Peak (V): ", TRUE);

	curCalVal = calibrateValue((uint16_t) freq);

	usart_esc("[5;27H");
	usart_esc("[0K");

	write_terminal_int(freq, "Frequency (Hz): ", TRUE);

	curCalVal = calibrateValue((uint16_t) rms_avg);

	usart_esc("[7;27H");
	usart_esc("[0K");

	write_terminal_int(rms_avg, "AC RMS Avg (V): ", TRUE);

	num_graph = (MAXGRAPH * rms_avg) / MAXmV;

	usart_esc("[9;27H");
	for(int i = 0; i < num_graph; i++) { usart_print("#"); }

	usart_esc("[10;27H");
	usart_print("|---|---|---|---|---|---|");
	usart_esc("[11;27H");
	usart_print("0  0.5 1.0 1.5 2.0 2.5 3.0");

}

// Algorithm found on Geeks for Geeks
/* partially copied and modified to imporve for assignment performance
 * https://www.geeksforgeeks.org/how-to-convert-an-integer-to-a-string-in-c/#1-manual-conversion-using-loop
 */
void intToStr(uint32_t N, char *str, uint8_t isFlt) {
	uint16_t i = 0;
//	uint8_t numLen = strlen(str);

    // If the number is negative, make it positive
    if (N < 0)
        N = -N;

    // while there is still a value left in N to compute
    while (N > 0) {
    	// adds a decimal point
    	if(isFlt && i == NUM_DECIMAL) { str[i++] = '.'; }
        // Convert integer digit to character and store
      	// it in the str
        str[i++] = N % 10 + '0';
      	N /= 10;
    }
    if(isFlt && i == NUM_DECIMAL) {
    	str[i++] = '.';
    	str[i++] = '0';
    }

    // Null-terminate the string
    str[i] = '\0';

    // Reverse the string to get the correct order
    for (int j = 0, k = i - 1; j < k; j++, k--) {
        char temp = str[j];
        str[j] = str[k];
        str[k] = temp;
    }
}
