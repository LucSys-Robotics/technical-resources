#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO		3     //Task priority //Task Priority
#define LED_STK_SIZE 		128   //Task stack size //Task Stack Size


/*--------Buzzer control pin--------*/
#define Buzzer_PORT GPIOA
#define Buzzer_PIN GPIO_Pin_8
#define Buzzer PAout(8)
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED_PORT GPIOA
#define LED_PIN GPIO_Pin_12
//Control port C30DLED old board
#define V1_0_LED PAout(12) 
/*----------------------------------*/
//New board C30DLED control port
#define V1_1_LED PEout(8) 


void V1_1_LED_Init(void);
void V1_0_LED_Init(void);  
void Buzzer_Init(void); 
void Led_Flash(u16 time);
void led_task(void *pvParameters);
extern int Led_Count;
#endif
