#ifndef __RC_JOYSTICK_H
#define __RC_JOYSTICK_H

#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct  
{
	int LX; //Values read from model aircraft channels 1~4, related to control
	int LY; 
	int RX;
	int RY;
	
	uint8_t freq;
}RC_REMOTE_t;


extern RC_REMOTE_t rc_remote;

void RCJoystick_IC_CaptureCallback(TIM_HandleTypeDef *htim,BaseType_t* woken);

#endif

