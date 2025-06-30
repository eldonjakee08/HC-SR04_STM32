/*
 * hcsr04_driver.h
 *
 *  Created on: Jun 27, 2025
 *      Author: eldon
 */

#ifndef HCSR04_DRIVER_HCSR04_DRIVER_CONF_H_
#define HCSR04_DRIVER_HCSR04_DRIVER_CONF_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>


/*
 * ultrasonic configuration macros
 */
#define ECHO_PIN			GPIO_PIN_2	//default echo pin GPIOA PIN0 TIM2 CH4, NOTE: CH1 AND CH2 reserved for one pulse mode
#define TRIG_PIN			GPIO_PIN_1	//default trig pin GPIOA PIN1 TIM2 CH2, modify to suit specific application
#define ULTRASONIC_TIMER	TIM2		//default timer used TIM2, modify to suit specific application
#define MAX_DISTANCE		400			//MAX DISTANCE SET TO 400cm

/*
 * APIs supported by driver
 */
void ultrasonic_init(void);
void ping_IT(float *pDistance);

#endif /* HCSR04_DRIVER_HCSR04_DRIVER_CONF_H_ */
