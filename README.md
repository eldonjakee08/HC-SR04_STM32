# HC-SR04_STM32
HCSR04 ultrasonic driver for STM32F4. This driver uses STM32F4 HAL drivers.

TIMER2 peripheral is used for Trig pulse generation and distance measurement. 

You could modify the driver to use other timers as long as its a "General Purpose" timer, you'd also need to replace the GPIO pins to the respective timer alternate function pins. 


*DEFAULT ULTRASONIC SENSOR PIN ASSIGNMENTS*
 * PB2 - ECHO PIN, TIM2 CH4 INPUT CAPTURE MODE 
 * PA1 - TRIG PIN, TIM2 CH2 PWM ONE PULSE MODE


*HOW TO USE THE DRIVER*

Include "hcsr04_driver.h" in main.c.

![image](https://github.com/user-attachments/assets/ab5cce20-d1a3-426c-a36e-45af0952f52f)

Call ultrasonic_init() during hardware initialization phase. This function initializes TIMER2 for ultrasonic sensor use and sets GPIO pins PB2 & PA1 in alternate function mode.

![image](https://github.com/user-attachments/assets/57bd8b6f-6021-4f22-9cf5-4d733f690b9a)

To measure the distance, call ping_IT(float *pDistanceBuffer), pDistanceBuffer is a float pointer where the distance measurement will be stored.

![image](https://github.com/user-attachments/assets/87dffd08-dfca-4217-ba33-8483c3773a28)


*SAMPLE RESULT*
![image](https://github.com/user-attachments/assets/4052fad5-6b9e-4f2a-9e22-c0de90994eda)
I'd estimate the error to be at +/-1cm, which is fine for simple applications





