#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO		3     //Task priority //Task Priority
#define LED_STK_SIZE 		128   //Task stack size //Task Stack Size


/*--------Buzzer control pin--------*/
#define Buzzer_PORT GPIOD
#define Buzzer_PIN GPIO_Pin_11
#define Buzzer PDout(11)
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED_PORT GPIOD
#define LED_R_PIN GPIO_Pin_14
#define LED_G_PIN GPIO_Pin_13
#define LED_B_PIN GPIO_Pin_15
#define LED_R PDout(14) 
#define LED_G PDout(13) 
#define LED_B PDout(15) 
/*----------------------------------*/

#define LED_OFF (LED_R=1,LED_G=1,LED_B=1)
#define LED_Red(n)    n?(LED_R=0,LED_G=1,LED_B=1):LED_OFF /* Red */
#define LED_Purple(n) n?(LED_R=0,LED_G=1,LED_B=0):LED_OFF /* Purple */
#define LED_Blue(n)   n?(LED_R=1,LED_G=1,LED_B=0):LED_OFF /* Blue */
#define LED_Cyan(n)   n?(LED_R=1,LED_G=0,LED_B=0):LED_OFF /* Cyan */
#define LED_Green(n)  n?(LED_R=1,LED_G=0,LED_B=1):LED_OFF /* Green */
#define LED_Yellow(n) n?(LED_R=0,LED_G=0,LED_B=1):LED_OFF /* Yellow */
#define LED_White(n)  n?(LED_R=0,LED_G=0,LED_B=0):LED_OFF /* White */

void LED_Init(void);  
void Buzzer_Init(void); 
void Led_Flash(u16 time);
void led_task(void *pvParameters);
extern int Led_Count;
#endif
