#include "enable_key.h"

/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
Function: Initialize the enable switch pin
Entry parameters: None
Return value: none
**************************************************************************/
void EnableKey_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    ENABLE_EnableKEY_PIN_CLOCK;
    GPIO_InitStructure.GPIO_Pin = ENABLE_KEY_PIN; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
    GPIO_Init(ENABLE_KEY_PORT, &GPIO_InitStructure);
}
