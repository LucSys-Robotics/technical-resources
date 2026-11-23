#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"

//Ackermann steering data structure
typedef struct{
	int Bias;                      //Zero point value of front wheel rotation angle, which can be used as a corrective compensation adjustment for the steering structure
	int Mid;                       //Median servo value, used to correct the direction of the servo during car reset and restart
	int Max;                       //Maximum servo rotation
	int Min;                       //Minimum value of servo rotation
}AKM_SERVO_ADC;


/*--------ADC Interface Fun--------*/
//External interface
void ADC1_Init(void);
void ADC2_Init(void);
void Akm_ServoParam_Init(AKM_SERVO_ADC* p);
int get_DMA_SlideRes(void);
int get_DMA_ServoBias(void);
u16 Get_ADC1(u8 ch);
u16 Get_ADC1_Average(u8 chn, u8 times);
float Get_battery_volt(void);
extern AKM_SERVO_ADC Akm_Servo;
/*----------------------------------*/

/*--------Battery_PIN config--------*/
//Pin for battery voltage acquisition
#define ENABLE_BATTERY_PIN_CLOCK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define Battery_PORT  GPIOB
#define Battery_PIN   GPIO_Pin_0
#define Battery_Ch    ADC_Channel_8
/*----------------------------------*/

/*--------CarMode_PIN config--------*/
//Used to detect pins for model adjustment potentiometer
#define ENABLE_CarMode_PIN_CLOCK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define CarMode_PORT  GPIOC
#define CarMode_PIN   GPIO_Pin_2
#define CarMode_Ch    ADC_Channel_12
/*----------------------------------*/

/*--------TurnAdjust_PIN config--------*/
//Ackermann steering compensation potentiometer acquisition pin
#define ENABLE_ServoAdj_PIN_CLOCK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define ServoAdj_PORT  GPIOC
#define ServoAdj_PIN   GPIO_Pin_3
#define ServoAdj_Ch    ADC_Channel_13
/*----------------------------------*/

/*--------Slide_PIN config--------*/
//Steering slide-rail data acquisition pin
#define ENABLE_SLIDE_PIN_CLOCK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define SLIDE_PORT  GPIOC
#define SLIDE_PIN   GPIO_Pin_1
#define SLIDE_Ch    ADC_Channel_11

//Rail data pins correspond to DMA configurations
#define ENABLE_DMAx_CLOCK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE)
#define DMA_REG_ADDR      (uint32_t)(&ADC2->DR)
#define DMAx_Streamx      DMA2_Stream3
#define DMA_ChannelX      DMA_Channel_1
/*----------------------------------*/


#endif 


