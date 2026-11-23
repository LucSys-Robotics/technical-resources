#include "led.h"

int Led_Count=500; //LED flicker time control //LED flicker time control

/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
Function: Initialize the LED interface
Entry parameters: None
Return value: none
**************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//Enable the GPIOB clock
  GPIO_InitStructure.GPIO_Pin =  LED_R_PIN|LED_G_PIN|LED_B_PIN;//LED corresponding IO port
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Normal output mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOD, &GPIO_InitStructure);//Initialize GPIO
	GPIO_SetBits(GPIOD,LED_R_PIN);
	GPIO_SetBits(GPIOD,LED_G_PIN);
	GPIO_SetBits(GPIOD,LED_B_PIN);
}
/**************************************************************************
Function: Buzzer interface initialized
Input   : none
Output  : none
Function: Initialize buzzer interface
Entry parameters: None
Return value: none
**************************************************************************/
void Buzzer_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//Enable the GPIOB clock
  GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;//LED corresponding IO port
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Normal output mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOD, &GPIO_InitStructure);//Initialize GPIO
}
/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
Function: LED flashing task
Entry parameters: None
Return value: none
**************************************************************************/
void led_task(void *pvParameters)
{
    while(1)
    {
		static u8 led_state=0;
		
		led_state = !led_state;
		
		LED_Red(led_state);
			
		vTaskDelay(Led_Count); 
    }
}  

/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
Function: LED flashing
Entry parameter: blinking time
Return value: none
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED_R=0;
	  else		if(++temp==time)	LED_R=~LED_R,temp=0;
}

