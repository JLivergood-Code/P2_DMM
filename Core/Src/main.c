#include "main.h"
#include "spi_dac.h"
#include "adc.h"
#include "usart.h"
#include "timer_funcs.h"

#include <math.h>
//
#define COMP_HYST_VAL 0x2
#define SAMPLE_DC_RATE 100
#define SAMPLE_AC_RATE 10000
#define OUTPUT_RATE 100
#define SAMPLE_CLK_CNT 1200
#define MAX16BIT 65535
#define NUMFREQCNT 10

#define MAXDAC 4095
#define MINVAL 0

void SystemClock_Config(void);

void comp_init(void);
void TIM2_init(void);

// adc init globles
uint8_t adcFlag = 0;
uint16_t adcData = 0;

// frequency calculation globles
uint32_t prevTime = 0;
uint32_t currTime = 0;
uint32_t period = 0;
uint32_t freq_calc = 0;
uint32_t freq_avg = 0;
uint32_t freq_sum = 0;
uint8_t freq_cnt = 0;



int main(void)
{


  HAL_Init();


  SystemClock_Config();

  // Initializes ADC
    /* GPIO Pins:
       * PA0: ADC Input
    */
  adc_init();

  // initializes Comparator and Timer
  /* GPIO Pins:
   * PB1: Comp IN-
   * PB2: Comp IN+
   * PB10: Comp Ouptut
  */
  comp_init();
  tim3_init();
  tim2_init();

  // initializes DAC and SPI
  /* GPIO Pins:
	* PA4: DAC CS
	* PA5: DAC SCLK
	* PA7: DAC MOSI
	*/
  spi_init();
  GPIO_SPI_init();
//  send_dac(2000);

  // Initializes USART
  /* GPIO Pins:
	* PA2: USART Transmit
	* PA3: USART Receive
	*/
  usart_init();
  usart_gpio_init();
  __enable_irq();


  uint8_t numDCConv = 0;
  uint16_t arr_cnt = 0;
  uint16_t max = 0;
  uint16_t min = MAX16BIT;
  uint16_t p2p = 0;
  uint32_t avg = 0;
  uint32_t dc_avg = 0;
  uint64_t rms_avg = 0;
  uint32_t ac_avg = 0;

  clear_scrn();
  write_divider();

  while (1) {
	  // checks if adcFlag is set
	  if(adcFlag){

//		  send_dac((uint16_t)1000);
		  // sets current position in array to the measured ADC value
//		  adc_arr[arr_cnt] = adcData;


		  // checks to see if the measured value is the max or min value
		  if(adcData > max){
			  max = adcData;
		  }
		  if(adcData < min) {
			  min = adcData;
		  }

		  // adds current adc value to avg
		  avg += adcData;
		  ac_avg += adcData;

		  // adds the squared value to the rms_average
		  rms_avg += (adcData * adcData);

		  arr_cnt++;



		  // every 100 samples, computes average and rms_average
		  if((arr_cnt % SAMPLE_DC_RATE) == 0){
			  //calculate average
			  avg /= SAMPLE_DC_RATE;
			  if(avg > 4095) { send_dac(MAXDAC); }
			  //			  else if(avg < 0) { send_dac(MINDAC); }
			  else { send_dac((uint16_t)avg); }

			  dc_avg += avg;
			  numDCConv++;
			  avg = 0;

			  if(numDCConv == OUTPUT_RATE){
				  dc_avg /= numDCConv;
				  updateDCValues(dc_avg);

				  dc_avg = 0;
				  numDCConv = 0;
			  }
		  }


		  if(arr_cnt == SAMPLE_AC_RATE){

			  ac_avg /= SAMPLE_AC_RATE;
			  rms_avg /= SAMPLE_AC_RATE;
			  rms_avg = (uint32_t) sqrt(rms_avg);



			  if(freq_cnt == 0) freq_cnt = 1;
			  freq_avg = (freq_sum / freq_cnt);
			  p2p = 2 * (max - ac_avg);


			  updateACValues(p2p, rms_avg, freq_avg);

			  // reset values for next 20 readings
			  arr_cnt = 0;
			  max = 0;
			  min = MAX16BIT;
			  rms_avg = 0;


		  } // end cnt == SAMPLE_RATE

		  if(freq_cnt == NUMFREQCNT){
			  // ceiling rounds number instead of truncating
			  freq_avg = (freq_sum / NUMFREQCNT);
			  freq_sum = 0;
			  freq_cnt = 0;

		  }

		  adcData = 0;
		  adcFlag = 0;


	  } //end adcFlag
  } // end while(1)
} //end main

/*
 * TIM3 ISR
 * When ARR finishes counting, intiates an ADC Conversion
 */
void TIM3_IRQHandler(void){
	if(TIM3 -> SR & TIM_SR_UIF){

		ADC1->CR |= ADC_CR_ADSTART;
		TIM3->SR &= ~(TIM_SR_UIF);
	}
}

/*
 * TIM2 ISR
 * On CC4 input capture for the rising edge, it will read the current value in the register
 * will then take the current value and previous value to calculate the frequency
 */
void TIM2_IRQHandler(void) {
	//The TIMx_CCR1 register gets the value of the counter on the active transition.
	//rising edge detected
	// CC1OF is also set if at least two consecutive captures occurred whereas the flag was not cleared
	  if(TIM2->SR & TIM_SR_CC4IF){ //check flag for capture \ compare
		  //clear flag

		currTime = TIM2->CCR4;  // Read the current captured value
		if(currTime >= prevTime){
			// if the current value is greater than the previous, the period is the difference between them
			period = currTime - prevTime;
		}
		else{
			//if the current time is not greater than the previous time, we have either time-traveled or the timer has overflowed
			 period = -1 - prevTime + currTime;
		}
		prevTime = currTime;  // Update prevTime for next period measurement

		if (period != 0) {
			freq_calc =  (SystemCoreClock*100) / ( period);  // Calculate frequency (TIM2->PSC + 1) *(TIM2->PSC + 1) *
			freq_sum += freq_calc;
			freq_cnt++;
		}
		TIM2->SR &= ~(TIM_SR_CC4IF);
	  }
}

/*
 * ADC1 and ADC2 ISR handler
 * on end of conversion, reads the data from the Data register and sets an ADC Flag
 */
void ADC1_2_IRQHandler(void){
	// for testing purposes:  || (ADC1->ISR & ADC_ISR_EOS)
	if ((ADC1->ISR & ADC_ISR_EOC)){
		//set flag and set value to global variable
		adcData = ADC1->DR;
		adcFlag = 1;
	}
}

// if it doesn't work, enable interrupts again
void USART2_IRQHandler(void){
	if (USART2->ISR & USART_ISR_RXNE){
		// sends input back
		USART2->TDR = USART2->RDR;
				// clear interrupt flag
		USART2->ISR &= ~(USART_ISR_RXNE);
	}

}



void comp_init(void){
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// enables both GPIO C and GPIO B
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);

	COMP1->CSR &= ~(COMP_CSR_HYST);
	COMP1->CSR |= (COMP_HYST_VAL << COMP_CSR_HYST_Pos);

	// sets IN+ to PB2
	COMP1->CSR |= (COMP_CSR_INPSEL);
	// sets IN MINUS to PB1
	COMP1->CSR |= (0x6 << COMP_CSR_INMSEL_Pos);



	// sets mode to AF
	GPIOB->MODER &= ~(GPIO_MODER_MODE10);
	// sets PB10 to AF mode for COMP output
	// sets PB1 to analog input mode for IN-
	// set PB2 to analog input for IN+
	GPIOB->MODER |= (0x2 << GPIO_MODER_MODE10_Pos  | GPIO_MODER_MODE1 | GPIO_MODER_MODE2);

	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT10 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED1 | GPIO_OSPEEDR_OSPEED2);

	// sets PB10 to Comparator 1 output
	GPIOB->AFR[1] |= (12 << GPIO_AFRH_AFSEL10_Pos);
	GPIOB->ASCR |= (GPIO_ASCR_ASC1 | GPIO_ASCR_ASC2);

	// enables COMP1
	COMP1->CSR |= COMP_CSR_EN;

	// locks COMP1
	COMP1->CSR |= COMP_CSR_LOCK;

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
