/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "timer_funcs.h"

#define SAMPLE_CLK_CNT 1200
#define MAXARR -1
#define TIM2_PRESC 127
//

/*
 * Written by: Joshua Livergood
 * Project: P1 Function Generator
 * Includes all initialization functions for Timer registers
 */

// initializes registers for timers and interrupts
void tim2_init(void) {
	  // Configure TIM2
	  RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
	  
	  //this is a variable
	  TIM2 -> ARR = MAXARR;

	  // configuring CC4 to be an input (Capture/Compare 4 selection)
	  // CC4S[1:0]: = 01: CC4 channel is configured as input, IC4 is mapped on TI4
	  TIM2->CCMR2 |= TIM_CCMR2_CC4S_0;
	  
	  // configuring CC 4 to connect directly to the comparator (Input Capture 4 remap)
	  // TI4_RMP = 01: TIM2 input capture 4 is connected to COMP1_OUT
	  // this sets the interrupt flag CC4IF
	  TIM2->OR1 |= TIM2_OR1_TI4_RMP_0;
	  
	  /* Select the edge of the active transition on the channel by writing the CC4P and
		CC4NP and CC4NP bits to 000 in the TIMx_CCER register (rising edge in this case)
		| TIM_CCER_CC4NE_Msk */
	  TIM2->CCER &= ~(TIM_CCER_CC4P_Msk | TIM_CCER_CC4NP_Msk );

	  /* Program the input prescaler.*/
	  TIM2->CCMR2 &= ~(TIM_CCMR2_IC4PSC_Msk);

	  TIM2->CCMR2 &= ~(TIM_CCMR2_IC4F);
	  TIM2->CCMR2 |= (0x3 << TIM_CCMR2_IC4F_Pos);

	  /* Enable capture from the counter into the capture register by setting the CC4E bit in the
	  TIMx_CCER register.*/
	  TIM2->CCER |= (TIM_CCER_CC4E_Msk);

	  TIM2->PSC |= (TIM2_PRESC);

	  // enable update event interrupt in TIM2
	  TIM2->DIER |= TIM_DIER_UIE | TIM_DIER_CC4IE;
	  // clear the interrupt status register for Update event
	  TIM2->SR &= ~(TIM_SR_UIF);
	  // start the timer
	  TIM2->CR1 |= TIM_CR1_CEN;
	  // enable TIM2 interrupt in NVIC
	  NVIC->ISER[0] = (1 << TIM2_IRQn);

	  // stops timer in debug mode
	  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM2_STOP;
}

void tim3_init(void) {
	  // Configure TIM2
	  RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM3EN);

	  //this is a variable
	  TIM3 -> ARR = SAMPLE_CLK_CNT-1;

	  // enable update event interrupt in TIM2
	  TIM3->DIER |= TIM_DIER_UIE;
	  // clear the interrupt status register for Update event
	  TIM3->SR &= ~(TIM_SR_UIF);
	  // start the timer
	  TIM3->CR1 |= TIM_CR1_CEN;
	  // enable TIM2 interrupt in NVIC
	  NVIC->ISER[0] = (1 << TIM3_IRQn);
}

// initialize GPIO pins for ISR measurement
void GPIO_init(void) {
	  // Turn on clock for GPIOC
	  RCC -> AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);
	  // Set up Pin PC0 for output measurement
	  GPIOC -> MODER &= ~(GPIO_MODER_MODE0);
	  GPIOC -> MODER |=  (1 << GPIO_MODER_MODE0_Pos);
	  GPIOC -> OTYPER &= ~(GPIO_OTYPER_OT0);
//	  GPIOC -> OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0);
	  GPIOC -> OSPEEDR |= (GPIO_OSPEEDR_OSPEED0);
	  GPIOC -> PUPDR &= ~(GPIO_PUPDR_PUPDR0);


}
