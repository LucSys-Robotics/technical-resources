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
void V1_0_LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//Enable the GPIOB clock
  GPIO_InitStructure.GPIO_Pin =  LED_PIN;//LED corresponding IO port
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Normal output mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOA, &GPIO_InitStructure);//Initialize GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_12);
}

void V1_1_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // Enable the GPIOE clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    // Configure GPIO_InitStructure
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // PE8 corresponds to IO ports
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // Normal output mode
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull output
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // Do not use pull-up or down
    // Initialize GPIOE
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_SetBits(GPIOE, GPIO_Pin_8);
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
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//Enable the GPIOB clock
  GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;//LED corresponding IO port
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Normal output mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOA, &GPIO_InitStructure);//Initialize GPIO
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
		if(SysVal.HardWare_Ver==V1_0)//LED flashing on the old C30D
		{
			//The status of the LED is reversed. 0 is on and 1 is off
			//LED state inversion, 0 is on, 1 is off
			V1_0_LED=~V1_0_LED; 
		}
		else if( SysVal.HardWare_Ver==V1_1 )//LED flashing on the new C30D
		{
			//The status of the LED is reversed. 0 is on and 1 is off
			//LED state inversion, 0 is on, 1 is off
			V1_1_LED=~V1_1_LED; 
		}
      //The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function	
      //LED blinking task is very simple, with low frequency accuracy requirements, using relative delay functions
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
	  if(0==time) V1_0_LED=0;
	  else		if(++temp==time)	V1_0_LED=~V1_0_LED,temp=0;
}

