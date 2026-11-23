#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"

/*--------moto Interface Fun --------*/
//Interfaces open to the public
#define SERVO_INIT     1500                             //Servo zero point // servo zero point
#define FULL_DUTYCYCLE 16800                            //Wheel motor full-duty-cycle value; depends on initialization settings.
void MiniBalance_PWM_Init(u16 arr,u16 psc);            //DC motor initialization
void V1_0_MiniBalance_PWM_Init(u16 arr,u16 psc);      //DC motor initialization
void Servo_Senior_Init(u16 arr,u16 psc,int servomid); //Initialize high-spec Ackermann servo
void Servo_Top_Init(u16 arr,u16 psc,int servomid);    //Initialize top-spec Ackermann servo
/*--------------------------------------------*/


//User config configuration: When no functional changes are involved, if a pin change occurs, only modify the pin parameters in that file
/*--------Motor_A User config--------*/
//PWM pin configuration
#define ENABLE_PWMA_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE)  //PWMA clock enabled
#define ENABLE_PWMA_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE) //Enable the pin port corresponding to PWMA
	
#define PWMA_PORT      GPIOC            //PWMA pin port
#define PWMA_PIN       GPIO_Pin_9       //PWMA pin
#define PWMA_TIM       TIM8             //Timer used by PWMA
#define PWMA_Channel   4                //Which channel is the timer used for CCRx(1~4)?

//Motor direction pin configuration
#define ENABLE_AIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)
#define ENABLE_AIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)

#define AIN1 	            PBout(13)//AIN1
#define AIN1_PORT           GPIOB
#define AIN1_PIN            GPIO_Pin_13

#define AIN2 	            PBout(12)//AIN2
#define AIN2_PORT           GPIOB
#define AIN2_PIN            GPIO_Pin_12
/*----------------------------------*/

/*--------Motor_B User config--------*/
//PWM pin configuration
#define ENABLE_PWMB_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE)  //PWMB clock enabled
#define ENABLE_PWMB_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE) //Enable the pin ports corresponding to PWMB
	
#define PWMB_PORT      GPIOC            //PWMB pin port
#define PWMB_PIN       GPIO_Pin_8       //PWMB pin
#define PWMB_TIM       TIM8             //Timer used by PWMB
#define PWMB_Channel   3                //Which channel is the timer used for CCRx(1~4)?

//Motor direction pin configuration
#define ENABLE_BIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE)
#define ENABLE_BIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE)

#define BIN1 	            PCout(0)//BIN1
#define BIN1_PORT           GPIOC
#define BIN1_PIN            GPIO_Pin_0

#define BIN2 	            PEout(15)//BIN2
#define BIN2_PORT           GPIOE
#define BIN2_PIN            GPIO_Pin_15
/*----------------------------------*/

/*--------Motor_C User config--------*/
//PWM pin configuration
#define ENABLE_PWMC_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE) //PWMC clock enablement
#define ENABLE_PWMC_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE) //Enable the pin port corresponding to PWMC
	
#define PWMC_PORT      GPIOC            //PWMC pin port
#define PWMC_PIN       GPIO_Pin_7       //PWMC pin
#define PWMC_TIM       TIM8             //Timer used by PWMC
#define PWMC_Channel   2                //Which channel is the timer used for CCRx(1~4)?

//Motor direction pin configuration
#define ENABLE_CIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE)
#define ENABLE_CIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE)

#define CIN1 	        PDout(10)//CIN1
#define CIN1_PORT       GPIOD
#define CIN1_PIN        GPIO_Pin_10

#define CIN2 	        PDout(12)//CIN2
#define CIN2_PORT       GPIOD
#define CIN2_PIN        GPIO_Pin_12
/*----------------------------------*/

/*--------Motor_D User config--------*/
//PWM pin configuration
#define ENABLE_PWMD_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE) //PWMD clock enabled
#define ENABLE_PWMD_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE) //Enable the corresponding pin port of PWMD
	
#define PWMD_PORT      GPIOC            //PWMD pin port
#define PWMD_PIN       GPIO_Pin_6       //PWMD pin
#define PWMD_TIM       TIM8             //Timer used by PWMD
#define PWMD_Channel   1                //Which channel is the timer used for CCRx(1~4)?

//Motor direction pin configuration
#define ENABLE_DIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE)
#define ENABLE_DIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)

#define DIN1 	        PCout(12)//DIN1
#define DIN1_PORT       GPIOC
#define DIN1_PIN        GPIO_Pin_12

#define DIN2 	        PAout(8) //DIN2
#define DIN2_PORT       GPIOA
#define DIN2_PIN        GPIO_Pin_8
/*----------------------------------*/

/*--------Senior akm Servo User config--------*/
//Configure high-spec Ackermann servo
//PWM pin configuration
#define ENABLE_SERVO_SENIOR_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE)  //Servo timer clock enabled
#define ENABLE_SERVO_SENIOR_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE) //Enable the pin ports corresponding to servos
	
#define SERVO_SENIOR_PORT      GPIOA            //Servo pin port
#define SERVO_SENIOR_PIN       GPIO_Pin_2       //Servo pin
#define SERVO_SENIOR_TIM       TIM9             //Timer used by servos
#define SERVO_SENIOR_Channel   1                //Which channel is the timer used for CCRx(1~4)?
/*--------------------------------------------*/

/*--------Top akm Servo User config--------*/
//Configure top-spec Ackermann servo
#define ENABLE_SERVO_TOP_TIM_CLOCK   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE) //Servo clock enabled
#define ENABLE_SERVO_TOP_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE) //Enable the pin ports corresponding to servos
	
#define SERVO_TOP_PORT      GPIOB            //Servo pin port
#define SERVO_TOP_PIN       GPIO_Pin_7       //Servo pin
#define SERVO_TOP_TIM       TIM4             //Timer used by servos
#define SERVO_TOP_Channel   2                //Which channel is the timer used for CCRx(1~4)?
/*--------------------------------------------*/

/*-------- V1.0 Motor_B User config--------*/
//Motor direction pin configuration of V1.00 hardware
#define V1_0_ENABLE_BIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE)
#define V1_0_ENABLE_BIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)

#define V1_0_BIN1 	              PCout(0)//BIN1
#define V1_0_BIN1_PORT            GPIOC
#define V1_0_BIN1_PIN             GPIO_Pin_0

#define V1_0_BIN2 	              PBout(14)//BIN2
#define V1_0_BIN2_PORT            GPIOB
#define V1_0_BIN2_PIN             GPIO_Pin_14
/*----------------------------------*/



//Configuration generated based on user configuration, no user modification required
//The configuration generated based on user configuration does not require user modification
#if PWMA_Channel==1
	#define PWMA           PWMA_TIM->CCR1
#elif PWMA_Channel == 2
	#define PWMA           PWMA_TIM->CCR2
#elif PWMA_Channel == 3
	#define PWMA           PWMA_TIM->CCR3
#elif PWMA_Channel == 4
	#define PWMA           PWMA_TIM->CCR4
#else
	#error "PWM_A Channel configuration error"
#endif

#if PWMB_Channel==1
	#define PWMB           PWMB_TIM->CCR1
#elif PWMB_Channel == 2
	#define PWMB           PWMB_TIM->CCR2
#elif PWMB_Channel == 3
	#define PWMB           PWMB_TIM->CCR3
#elif PWMB_Channel == 4
	#define PWMB           PWMB_TIM->CCR4
#else
	#error "PWM_B Channel configuration error"
#endif

#if PWMC_Channel==1
	#define PWMC           PWMC_TIM->CCR1
#elif PWMC_Channel == 2
	#define PWMC           PWMC_TIM->CCR2
#elif PWMC_Channel == 3
	#define PWMC           PWMC_TIM->CCR3
#elif PWMC_Channel == 4
	#define PWMC           PWMC_TIM->CCR4
#else
	#error "PWM_C Channel configuration error"
#endif

#if PWMD_Channel==1
	#define PWMD           PWMD_TIM->CCR1
#elif PWMD_Channel == 2
	#define PWMD           PWMD_TIM->CCR2
#elif PWMD_Channel == 3
	#define PWMD           PWMD_TIM->CCR3
#elif PWMD_Channel == 4
	#define PWMD           PWMD_TIM->CCR4
#else
	#error "PWM_D Channel configuration error"
#endif

#if SERVO_SENIOR_Channel==1
	#define SERVO_SENIOR           SERVO_SENIOR_TIM->CCR1
#elif SERVO_SENIOR_Channel == 2
	#define SERVO_SENIOR           SERVO_SENIOR_TIM->CCR2
#elif SERVO_SENIOR_Channel == 3
	#define SERVO_SENIOR           SERVO_SENIOR_TIM->CCR3
#elif SERVO_SENIOR_Channel == 4
	#define SERVO_SENIOR           SERVO_SENIOR_TIM->CCR4
#else
	#error "SERVO_SENIOR_Channel Channel configuration error"
#endif

#if SERVO_TOP_Channel==1
	#define SERVO_TOP           SERVO_TOP_TIM->CCR1
#elif SERVO_TOP_Channel == 2
	#define SERVO_TOP           SERVO_TOP_TIM->CCR2
#elif SERVO_TOP_Channel == 3
	#define SERVO_TOP           SERVO_TOP_TIM->CCR3
#elif SERVO_TOP_Channel == 4
	#define SERVO_TOP           SERVO_TOP_TIM->CCR4
#else
	#error "SERVO_TOP_Channel Channel configuration error"
#endif


/*---------------------------------------------------------------*/

#endif
