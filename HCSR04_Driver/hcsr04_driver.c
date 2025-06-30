/******PINS OF TIM2 CHANNELS*****
 * PB2 - TIM2 CH4 INPUT CAPTURE MODE, ECHO PIN
 * PA1 - TIM2 CH2 OUTPUT COMPARE MODE, TRIG PIN
 ********************************/

#include "hcsr04_driver.h"

static void TIM2_GPIO_Init(void);
static void Error_Handler1(void);

/*
 * Global variable of TIM2 handle structure
 */
TIM_HandleTypeDef ultrasonic_TIM2;
float *pDistance1;

/**
 * @brief  Measures distance with ultrasonic sensor. Uses not blocking mode (interrupts) for trig pulse generation and distance measurement
 * @param  float *pDistanceBuffer, float pointer to distance data buffer
 * @retval none
 */
void ping_IT(float *pDistanceBuffer)
{
	//flag for calling ping function for 1st time
	static uint8_t ping_1st_time = 1;
	pDistance1 = pDistanceBuffer;

	if(ping_1st_time == 1)
	{
		//TO GENERATE AN INTERRUPT AFTER 10US PULSE
		__HAL_TIM_ENABLE_IT(&ultrasonic_TIM2, TIM_IT_UPDATE);

		ping_1st_time = 0;

		//START TIM2 CNT TO GENERATE 10US PULSE
		HAL_TIM_PWM_Start(&ultrasonic_TIM2, TIM_CHANNEL_2);

		//ENABLE INPUT CAPTURE INTERRUPT FROM TIM2 CH4
		HAL_TIM_IC_Start_IT(&ultrasonic_TIM2, TIM_CHANNEL_4);

		//WRITE NEW ARR FOR ECHO PIN PULSE MEASUREMENT (TAKES EFFECT AFTER 10US PULSE SINCE ARPE = 1)
		__HAL_TIM_SET_AUTORELOAD(&ultrasonic_TIM2, 0xFFFFFFFF);


	}else{
		//TO GENERATE AN INTERRUPT AFTER 10US PULSE
		__HAL_TIM_ENABLE_IT(&ultrasonic_TIM2, TIM_IT_UPDATE);

		//ENABLE INTERRUPT FROM TIM2 CH4
		__HAL_TIM_ENABLE_IT(&ultrasonic_TIM2, TIM_IT_CC4);

		//ENABLED CEN TO START THE TIMER FROM COUNTING AGAIN
		//CR1_CEN WAS CLEARED AFTER ONE PULSE DUE TO OPM ENABLED
		ultrasonic_TIM2.Instance->CR1 |= TIM_CR1_CEN;

		//WRITE NEW ARR FOR ECHO PIN PULSE MEASUREMENT (TAKES EFFECT AFTER 10US PULSE SINCE ARPE = 1)
		__HAL_TIM_SET_AUTORELOAD(&ultrasonic_TIM2, 0xFFFFFFFF);
	}

	//let sensor stabilize before initiating next pulse
	HAL_Delay(50);
}

/**
 * @brief  Initializes ultrasonic TIMER2 peripheral and ultrasonic GPIO alternate function pins
 * @param  none
 * @retval none
 */
void ultrasonic_init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	ultrasonic_TIM2.Instance = ULTRASONIC_TIMER;
	ultrasonic_TIM2.Init.Prescaler = ( HAL_RCC_GetHCLKFreq() / 1e6) - 1; //driver assumes that AHB1_Clock = APB1_TIM2_CLOCK
																		 //sets CNT_CLK as 1Mhz 1us period
	ultrasonic_TIM2.Init.CounterMode = TIM_COUNTERMODE_UP;
	ultrasonic_TIM2.Init.Period = 11-1;
	ultrasonic_TIM2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	ultrasonic_TIM2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	//initialize low level GPIO pin alternate function modes
	TIM2_GPIO_Init();

	//configure TIM2 time base
	if (HAL_TIM_Base_Init(&ultrasonic_TIM2) != HAL_OK)
	{
		Error_Handler1();
	}

	//configure TIM2 CH4 (ECHO PIN) as input capture mode
	sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&ultrasonic_TIM2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler1();
	}


	//configure TIM2 CH2 (TRIG PIN) as output compare mode
	sConfigOC.OCMode = TIM_OCMODE_PWM2;	//PWM2 MODE - In upcounting, CH2 is inactive/low as long as
										//TIMx_CNT<TIMx_CCR2 else active.

	//THIS VALUE WILL BE PROGRAMMED TO CCR2
	sConfigOC.Pulse = 1; //AT TIMx_CNT = 0 LOW STATE, BUT AT TIMx_CNT = 1 HIGH STATE UNTIL TIMx_CNT = 10
						 //THEN TIMx_CNT RESETS TO 0 (SINCE ARR=10) DRIVING THE OUTPUT LOW. TOTAL OF 10us PULSE
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	if (HAL_TIM_PWM_ConfigChannel(&ultrasonic_TIM2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler1();
	}

	//ENABLE ONE PULSE MODE
	ultrasonic_TIM2.Instance->CR1 |= TIM_CR1_OPM;


	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&ultrasonic_TIM2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler1();
	}
}


static void TIM2_GPIO_Init()
{
	/******PINS OF TIM2 CHANNELS*****
	 * PB2 - TIM2 CH4 INPUT CAPTURE MODE, ECHO PIN
	 * PA1 - TIM2 CH2 OUTPUT COMPARE MODE, TRIG PIN
	 ********************************/

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(ultrasonic_TIM2.Instance == ULTRASONIC_TIMER)
	{
		//enable TIM2  peripheral clock and GPIOA peripheral clock
		__HAL_RCC_TIM2_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		//low level hardware initializations
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		GPIO_InitStruct.Pin = TRIG_PIN; //initialize TRIG PIN TIM2 CH4 as alternate function
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = ECHO_PIN;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


		//enable interrupts from TIM2 processor side
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
		HAL_NVIC_SetPriority(TIM2_IRQn, 15, 0);
	}

}



/*******************INTERRUPT SERVICE ROUTINE SECTION************************/
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&ultrasonic_TIM2);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//1st interrupt flag
	static uint8_t is_1st_call = 1;
	static uint32_t cnt_read[2] = {0};
	static uint32_t difference;
	float distance;

	//ECHO PIN SOURCE OF INTERRUPT
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		if(is_1st_call)
		{
			//SAVE 1ST CAPTURED VALUE WHICH WILL BE USED FOR DISTANCE CALCULATION
			cnt_read[0] = htim->Instance->CCR4;


			/*****PREPARE FOR NEXT INTERRUPT*****/
			is_1st_call = 0;

			//CHANGE POLARITY TO DETECT THE UPCOMING FALLING EDGE
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);


		}else{
			//SAVES 2ND TIMER CNT WHICH WILL BE USED FOR DISTANCE CALCULATION
			cnt_read[1] =  htim->Instance->CCR4;

			if(cnt_read[1] > cnt_read[0])
			{
				difference = cnt_read[1] - cnt_read[0];
			}else{
				difference = (0xFFFFFFFF - cnt_read[0]) + (cnt_read[1]+1);
			}

			//343m/s convert to 0.034cm/us then div 2 because need only 1 way of distance travelled by sound
			distance = difference * (0.034/2);

			//store distance data into distance buffer
			if(distance <= MAX_DISTANCE)
			{
				*pDistance1 = distance;
			}else{
				*pDistance1 = 0;
			}

			/******CLEANUP ROUTINE PREPARE FOR NEXT PULSE*******/
			//DISABLE TIM2 TO STOP COUNTING
			htim->Instance->CR1 &= ~(TIM_CR1_CEN);

			//SET TIM2 CNT BACK TO ZERO
			htim->Instance->CNT = 0;

			//SET ARR BACK TO 10 FOR NEXT 10US PULSE (TAKES EFFECT IMMEDIATELY)
			__HAL_TIM_SET_AUTORELOAD(&ultrasonic_TIM2, 10);

			//ENABLE AUTO RELOAD PRELOAD FOR NEXT 10US PULSE
			htim->Instance->CR1 |= TIM_CR1_ARPE;

			//CHANGE POLARITY TO DETECT THE NEXT RISING EDGE
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);

			//DISABLE INTERRUPT FROM TIM2 CH4
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);

			//ENABLE TM2 CH2 AGAIN FOR NEXT 10US PULSE
			htim->Instance->CCER |= TIM_CCER_CC2E;

			is_1st_call = 1;
		}
	}
}


/*
 * AFTER 10US PULSE AN UPDATE EVENT IS TRIGGERED WHICH GENERATES AN INTERRUPT
 * WILL PERFORM CLEANUP ROUTINE FOR TIM2 CH2 AND PREPARE CH4 TO LISTEN ON ECHO PIN
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//STOP GENERATING INTERRUPT ON UPDATE EVENT
	__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);

	//DISABLE TIM2 CH2 TO PREVENT FROM SENDING PULSES WHEN TIM2 START UPCOUNTING AGAIN
	htim->Instance->CCER &= ~(TIM_CCER_CC2E);

	//DISABLE AUTO RELOAD PRELOAD BECAUSE NEED TO SET ARR = 10 IMMEDIATELY AFTER INPUT CAPTURE 2ND INTERRUPT
	htim->Instance->CR1 &= ~(TIM_CR1_ARPE);

	//ENABLE TIM2 TO START COUNTING AGAIN
	htim->Instance->CR1 |= TIM_CR1_CEN;
}



void Error_Handler1(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}


