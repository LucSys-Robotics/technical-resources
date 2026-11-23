#ifndef __RC_JOYSTICK_H
#define __RC_JOYSTICK_H

#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct  
{
	int LX; //航模通道1~4读取到的数值,与控制相关
	int LY; 
	int RX;
	int RY;
	
	uint8_t freq;
}RC_REMOTE_t;


extern RC_REMOTE_t rc_remote;

void RCJoystick_IC_CaptureCallback(TIM_HandleTypeDef *htim,BaseType_t* woken);

#endif

