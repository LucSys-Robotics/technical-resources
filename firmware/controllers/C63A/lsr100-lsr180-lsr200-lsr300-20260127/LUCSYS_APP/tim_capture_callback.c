/**
 * @file    tim_capture_callback.c
 * @brief Timer input captures interrupt callback functions.
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - This file mainly contains timer input capture interrupt callback functions, which process the underlying signals of the RC transmitter within the callback.
 *   and the underlying signals for processing ultrasound. Both RC transmitter signals and ultrasonic ranging rely on this callback function to obtain data.
   - The callback function processing content is compatible with FreeRTOS, and whether to initiate a task is determined by xHigherPriorityTaskWoken.
     Dispatch tasks
 *      
 * @note
 * 
 * 
 */

#include "sensor_ranger.h"
#include "rc_joystick.h"

//Timer capture callback function, ultrasonic capture, and RC transmitter channel capture are both used
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//This interrupt involves the operating system's API and only executes the interrupt content when the scheduler starts
	BaseType_t schedulerState = xTaskGetSchedulerState();
	if( schedulerState!=taskSCHEDULER_RUNNING ) return;
	
	if( htim == &htim4 )
	{
		//Signal processing logic for model aircraft handles
		RCJoystick_IC_CaptureCallback(htim,&xHigherPriorityTaskWoken);
	}
	else
	{
		//Ultrasonic signal logic processing
		Ranger_IC_CaptureCallback(htim,&xHigherPriorityTaskWoken);
	}
	
	//Initiate scheduling based on specific circumstances
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

