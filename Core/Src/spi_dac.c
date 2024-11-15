
#include "spi_dac.h"

/* Written by: Joshua Livergood
 * Project: P1 Function Generator
 * File used for all functions that deal with SPI communication
 * Includes initialization functions to initialize GPIO pins and SPI registers
 * Also includes send_dac function respoinsible for formatting and sending data to DAC
 */

/*
 * Initializes all registers for the SPI
 */
void spi_init(){
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //enables SPI1 clock
	// sets Baud Rate to 000
	SPI1->CR1 &= ~(SPI_CR1_BR);
//	SPI1->CR1 |= (SPI_CR1_BR);
	//===================================
	// Register COnfiguration for SPI_CR1
	//===================================
	// sets clock phase shift and clock polarity to 0, 0
	SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
	SPI1->CR1 |= (SPI_CR1_BIDIMODE); // sets bidirectional mode to 1 (1-line bidirectional mode)
	SPI1->CR1 |= (SPI_CR1_BIDIOE);	// sets output enable High so data is only being sent, not received
	SPI1->CR1 &= ~(SPI_CR1_RXONLY); // set to transmit mode only
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST); //sets data transfer mode to MSB first
//	SPI1->CR1 &= ~(SPI_CR1_CRCEN); //re-enable this later
	SPI1->CR1 &= ~(SPI_CR1_SSM); //sets software CS management to hardware mode
	SPI1->CR1 |= (SPI_CR1_MSTR);
	//==================================
	//Register Configuration for SPI_CR2
	//==================================
	SPI1->CR2 |= (0xF << SPI_CR2_DS_Pos); // sets DS to 1111, or 16 bit Frame size
//	SPI1->CR2 &=
	SPI1->CR2 |= (SPI_CR2_SSOE); // Sets CS to output mode
	SPI1->CR2 |= SPI_CR2_NSSP_Msk; //mask nssp
	//disable interrupts
	SPI1->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
	// FRXTH enabled since DS is 12
	SPI1->CR2 &= ~SPI_CR2_FRXTH;
	SPI1->CR1 |= (SPI_CR1_SPE); //enables SPI1
}

// initializes all GPIO pins for the SPI, specifically pins A4, A5, A7
void GPIO_SPI_init(void) {
	// PA4: CS
	// PA5: SCLK
	// PA7: MOSI
	//initialize SPI for clock
	RCC -> AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA -> MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE7);
	GPIOA-> MODER |= (2 << GPIO_MODER_MODE4_Pos | 2 << GPIO_MODER_MODE5_Pos| 2 << GPIO_MODER_MODE7_Pos);
	GPIOA -> OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT7);
	GPIOA -> OSPEEDR |= (GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED7);
	GPIOA -> PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR7);
//	GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
	GPIOA -> AFR[0] &= ~((GPIO_AFRL_AFSEL4 << GPIO_AFRL_AFSEL4_Pos) | (GPIO_AFRL_AFSEL5 << GPIO_AFRL_AFSEL5_Pos) | (GPIO_AFRL_AFSEL7 << GPIO_AFRL_AFSEL7_Pos));
//clear AF, use RL because 0-7, use
	GPIOA -> AFR[0] |= ((5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos));
}

// formats the in_volt parameter in the data fram and sends the data over SPI to the DAC
void send_dac(uint16_t in_volt){
	uint16_t data_frame = 0x0000;
	data_frame &= ~(1 << DAC_DACA_Pos); // Sets DAC select to DAC A(0xF)
	data_frame &= ~(1 << DAC_BUFF_Pos);  // sets Buffer to disabled(0xE)
	data_frame |= (1 << DAC_GA_Pos); 	// Sets Gain to 1(0xD)
	data_frame |= (1 << DAC_SHDN_Pos); 	// Sets output enable to true(0xC)
	in_volt &= 0x0FFF; // keeps the first 12 bits
	data_frame |= in_volt; //sets 12 lowest bits to in_volt
	// if transmit FIFO is empty, load data
	SPI1->DR = data_frame;
	while(SPI1->SR & SPI_SR_BSY); //wait after sending
	while (!(SPI1->SR & SPI_SR_TXE)); //wait after sending
}



