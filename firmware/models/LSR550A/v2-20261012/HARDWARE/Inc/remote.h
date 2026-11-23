#ifndef __REMOTE_H
#define __REMOTE_H
#include "sys.h"

//Threshold for interference judgment. When the active statistical value of the model aircraft exceeds this threshold, it is considered to be an interference signal or there is no model aircraft signal. Reset the remote control values to prevent the car from losing control.
//Smaller thresholds increase detection sensitivity; larger thresholds slow detection.
#define Filter_Threshold 10

//Remote-control variables
typedef struct  
{
	int ch1; //Values read from model aircraft channels 1~4, related to control
	int ch2; 
	int ch3;
	int ch4;
	uint16_t check_count; //Model aircraft remote controller activity statistics, used to verify whether the model aircraft signal is genuine and to filter out interference
}REMOTER_t;

//External interface
void Remoter_Init(void);
extern REMOTER_t remoter;
void Remoter_Param_Init(REMOTER_t *p);


/*--------Remote User config--------*/
//RC transmitter configuration
#define ENABLE_REMOTE_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE)  //PWMA clock enabled

#define ENABLE_REMOTE_CH1_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE) //Enable the pin port corresponding to PWMA
#define ENABLE_REMOTE_CH2_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE) //Enable the pin port corresponding to PWMA
#define ENABLE_REMOTE_CH3_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE) //Enable the pin port corresponding to PWMA
#define ENABLE_REMOTE_CH4_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE) //Enable the pin port corresponding to PWMA
	
#define REMOTE_TIM            TIM1	
#define REMOTE_TIM_IRQn       TIM1_CC_IRQn        //Timer interruption corresponding to model aircraft data acquisition
#define REMOTE_TIM_IRQHandler TIM1_CC_IRQHandler //Interrupt service functions corresponding to model aircraft data acquisition

#define REMOTE_CH1_PORT      GPIOE            
#define REMOTE_CH1_PIN       GPIO_Pin_9       

#define REMOTE_CH2_PORT      GPIOE            
#define REMOTE_CH2_PIN       GPIO_Pin_11

#define REMOTE_CH3_PORT      GPIOE           
#define REMOTE_CH3_PIN       GPIO_Pin_13 

#define REMOTE_CH4_PORT      GPIOE            
#define REMOTE_CH4_PIN       GPIO_Pin_14 

/*----------------------------------*/


//Timer channel corresponding to the RC transmitter receiving channel
#define Set_CH1_Rising  TIM_OC1PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Rising)  //Rising edge configuration
#define Set_CH2_Rising  TIM_OC2PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Rising)
#define Set_CH3_Rising  TIM_OC3PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Rising)
#define Set_CH4_Rising  TIM_OC4PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Rising)

#define Set_CH1_Falling TIM_OC1PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Falling)  //Descending Edge configuration
#define Set_CH2_Falling TIM_OC2PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Falling)
#define Set_CH3_Falling TIM_OC3PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Falling)
#define Set_CH4_Falling TIM_OC4PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Falling)

#define Get_CH1_CNT TIM_GetCapture1(REMOTE_TIM) //Get the value of the counter
#define Get_CH2_CNT TIM_GetCapture2(REMOTE_TIM)
#define Get_CH3_CNT TIM_GetCapture3(REMOTE_TIM)
#define Get_CH4_CNT TIM_GetCapture4(REMOTE_TIM)

#define Get_CH1_State TIM_GetITStatus(REMOTE_TIM, TIM_IT_CC1) //Get the status of the interrupt
#define Get_CH2_State TIM_GetITStatus(REMOTE_TIM, TIM_IT_CC2)
#define Get_CH3_State TIM_GetITStatus(REMOTE_TIM, TIM_IT_CC3)
#define Get_CH4_State TIM_GetITStatus(REMOTE_TIM, TIM_IT_CC4)

#define Clear_CH1_State TIM_ClearITPendingBit(REMOTE_TIM, TIM_IT_CC1) //Clear interrupt flags
#define Clear_CH2_State TIM_ClearITPendingBit(REMOTE_TIM, TIM_IT_CC2)
#define Clear_CH3_State TIM_ClearITPendingBit(REMOTE_TIM, TIM_IT_CC3)
#define Clear_CH4_State TIM_ClearITPendingBit(REMOTE_TIM, TIM_IT_CC4)


void Remoter_Init(void);


#endif
