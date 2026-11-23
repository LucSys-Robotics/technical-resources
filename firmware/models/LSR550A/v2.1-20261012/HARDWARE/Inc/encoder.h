#ifndef __ENCODER_H
#define __ENCODER_H 
#include "sys.h"

// No larger than 65535, because the timer of STM32F103 is 16 bit
//Must not exceed 65535 because the STM32F103 timer is 16 bits
#define ENCODER_TIM_PERIOD (u16)(65535)   

//Encoder enumeration
typedef enum{
	Encoder_A,
	Encoder_B,
	Encoder_C,
	Encoder_D
}ENCODER_t;

/*--------ENCODER Interface Fun --------*/
short Read_Encoder(ENCODER_t e);
/*----------------------------------*/


/*--------ENCODER_A config--------*/
#define ENABLE_ENCODER_A_TIM_CLOCK    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE)
#define ENBALE_ENCODER_A1_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)
#define ENBALE_ENCODER_A2_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)

#define ENCODER_A_TIM     TIM2             //Timer used by Encoder A interface

#define ENCODER_A1_PORT   GPIOA            //Encoder A port 1
#define ENCODER_A1_PIN    GPIO_Pin_15      //Encoder A port pin 1

#define ENCODER_A2_PORT   GPIOB            //Encoder A port 2
#define ENCODER_A2_PIN    GPIO_Pin_3       //Encoder A interface pin 2
/*----------------------------------*/

/*--------ENCODER_A Interface Fun --------*/
void EncoderA_Init(void);
/*----------------------------------*/


/*--------ENCODER_B config--------*/
#define ENABLE_ENCODER_B_TIM_CLOCK    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE)
#define ENBALE_ENCODER_B1_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)
#define ENBALE_ENCODER_B2_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)

#define ENCODER_B_TIM     TIM3             //Timer used by the encoder interface B

#define ENCODER_B1_PORT   GPIOA            //Encoder port B port 1
#define ENCODER_B1_PIN    GPIO_Pin_6       //Encoder B port pin 1

#define ENCODER_B2_PORT   GPIOA            //Encoder port B port 2
#define ENCODER_B2_PIN    GPIO_Pin_7       //Encoder interface B pin 2
/*----------------------------------*/

/*--------ENCODER_B Interface Fun --------*/
void EncoderB_Init(void);
/*----------------------------------*/

/*--------ENCODER_C config--------*/
#define ENABLE_ENCODER_C_TIM_CLOCK    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE)
#define ENBALE_ENCODER_C1_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)
#define ENBALE_ENCODER_C2_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)

#define ENCODER_C_TIM     TIM4             //Timer used by the encoder C interface

#define ENCODER_C1_PORT   GPIOB            //Encoder port C port 1
#define ENCODER_C1_PIN    GPIO_Pin_6       //Encoder C interface pin 1

#define ENCODER_C2_PORT   GPIOB            //Encoder port C port 2
#define ENCODER_C2_PIN    GPIO_Pin_7       //Encoder port C pin 2
/*----------------------------------*/

/*--------ENCODER_C Interface Fun --------*/
void EncoderC_Init(void);
/*----------------------------------*/

/*--------ENCODER_D config--------*/
#define ENABLE_ENCODER_D_TIM_CLOCK    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE)
#define ENBALE_ENCODER_D1_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)
#define ENBALE_ENCODER_D2_PORT_CLOCK  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)

#define ENCODER_D_TIM     TIM5             //Timer used by the encoder interface D

#define ENCODER_D1_PORT   GPIOA            //Encoder port D port 1
#define ENCODER_D1_PIN    GPIO_Pin_0       //Encoder interface D pin 1

#define ENCODER_D2_PORT   GPIOA            //Encoder port D port 2
#define ENCODER_D2_PIN    GPIO_Pin_1       //Encoder interface D pin 2
/*----------------------------------*/

/*--------ENCODER_D Interface Fun --------*/
void EncoderD_Init(void);
/*----------------------------------*/


#endif
