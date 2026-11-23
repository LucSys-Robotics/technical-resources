/**
 * @file    sensor_ranger.c
 * @brief Logic files for obtaining ultrasonic sensor data.
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - This document mainly covers ultrasonic sensor data acquisition and relies on the timer's input capture function. Ultrasonic data acquisition process:
     Ultrasonic operation is triggered by the task ultrasonic_task, and the distance is calculated during the timer's capture interrupt.
 * - Users of this file mainly focus on the GetRangerGroup function, which can group ultrasounds, and ultrasounds in the same group will be grouped simultaneously
 *   When triggering distance measurement, be careful not to assign ultrasound signals that are too close to each other in the same group, otherwise the two ultrasonic signals may interfere with each other
 * - If you need to modify the ultrasonic trigger order or grouping, please define the group according to the format in GetRangerGroup. If you need to block a specific one
 *   ultrasounds, simply remove and group them
 * @note
 * 
 * 
 */

#include "sensor_ranger.h"

#include "robot_select_init.h"

//Ultrasonic sensor event group
EventGroupHandle_t g_xEventRangerSensor = NULL;

//Set the debug level
#define SENSOR_DEBUG_LEVEL 0

#if (SENSOR_DEBUG_LEVEL > 0U)
#define  SENSOR_UsrLog(...)   do { \
                            printf(__VA_ARGS__); \
                            printf("\r\n"); \
} while (0)
#else
#define SENSOR_UsrLog(...) do {} while (0)
#endif


//Ultrasonic completion callback function for acquisition _ Operating environment is interrupted
BaseType_t RangerSensor_CaptureCallBack(uint8_t index)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	//Set the flag for successful acquisition
	xEventGroupSetBitsFromISR(g_xEventRangerSensor,1<<index,&xHigherPriorityTaskWoken);
	
	return xHigherPriorityTaskWoken;
}


enum {
	WaitUp=0,
	WaitDown,
};

//Interface binding
RangerHALObj RangerHAL_A = {
	.index = 1,      //Index values: A to 1, B-2, C-3....
	.sm = WaitUp, //Built-in state machine
	.dis = 5.1f,  //Distance value
	.upvalue = 0,    //Internal collected variables
	.downvalue = 0,  //Internal collected variables
	.CaptureCompleted = RangerSensor_CaptureCallBack,//Acquisition-complete callback
	.timeoutflag = 1,//Acquisition timeout flag
	.timeoutCount = 0,//Acquisition timeout count
	
	//Bind hardware interfaces
	.tim = &htim3,
	.ch = TIM_CHANNEL_4,
	.TrigPort = TriA_GPIO_Port,
	.TrigPin = TriA_Pin,
};

RangerHALObj RangerHAL_B = {
	.index = 2,
	.sm = WaitUp,
	.dis = 5.1f,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim3,
	.ch = TIM_CHANNEL_3,
	.TrigPort = TriB_GPIO_Port,
	.TrigPin = TriB_Pin,
};

RangerHALObj RangerHAL_C = {
	.index = 3,
	.sm = WaitUp,
	.dis = 5.1f,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim3,
	.ch = TIM_CHANNEL_1,
	.TrigPort = TriC_GPIO_Port,
	.TrigPin = TriC_Pin,
};

RangerHALObj RangerHAL_D = {
	.index = 4,
	.sm = WaitUp,
	.dis = 5.1f,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim2,
	.ch = TIM_CHANNEL_1,
	.TrigPort = TriD_GPIO_Port,
	.TrigPin = TriD_Pin,
};

RangerHALObj RangerHAL_E = {
	.index = 5,
	.sm = WaitUp,
	.dis = 5.1f,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim2,
	.ch = TIM_CHANNEL_2,
	.TrigPort = TriE_GPIO_Port,
	.TrigPin = TriE_Pin,
};

RangerHALObj RangerHAL_F = {
	.index = 6,
	.sm = WaitUp,
	.dis = 5.1f,
	.upvalue = 0,
	.downvalue = 0,
	.CaptureCompleted = RangerSensor_CaptureCallBack,
	.timeoutflag = 1,
	.timeoutCount = 0,
	.tim = &htim3,
	.ch = TIM_CHANNEL_2,
	.TrigPort = TriF_GPIO_Port,
	.TrigPin = TriF_Pin,
};

//List of supported ultrasound counts, used for interrupting use
#define RANGER_NUM 6
static RangerHALObj* rangerlist[] = { 
	&RangerHAL_A,&RangerHAL_B,&RangerHAL_C,
	&RangerHAL_D,&RangerHAL_E,&RangerHAL_F 
};

//Counting frequency of the counter, related to timer configuration
#define TIM_COUNT_FREQ 1000000.0f

//Sensor reset and used when data collection times out
static void RangerReset(RangerHALObj* ranger)
{
	ranger->sm = WaitUp;
	ranger->dis = 5.1f;
	ranger->upvalue = 0;
	ranger->downvalue = 0;
	ranger->timeoutflag = 1;
	TIM_RESET_CAPTUREPOLARITY(ranger->tim,ranger->ch);
	TIM_SET_CAPTUREPOLARITY(ranger->tim,ranger->ch,TIM_ICPOLARITY_RISING);
	HAL_GPIO_WritePin(ranger->TrigPort,ranger->TrigPin,GPIO_PIN_SET);
}

//Returns the corresponding channel value based on the active timer channel, interrupting usage
static uint32_t get_tim_activate_ch(HAL_TIM_ActiveChannel active_ch)
{
	switch (active_ch) {
		case HAL_TIM_ACTIVE_CHANNEL_1: return TIM_CHANNEL_1;
		case HAL_TIM_ACTIVE_CHANNEL_2: return TIM_CHANNEL_2;
		case HAL_TIM_ACTIVE_CHANNEL_3: return TIM_CHANNEL_3;
		case HAL_TIM_ACTIVE_CHANNEL_4: return TIM_CHANNEL_4;
		default: return 0xFF;
	}
}

//Ultrasonic trigger, used to drive the ultrasonic sensor to operate
static void RangerTrigger(RangerHALObj* ranger)
{
	//Low-level triggered ultrasound operation
	HAL_GPIO_WritePin(ranger->TrigPort,ranger->TrigPin,GPIO_PIN_RESET);
	delay_us(15);
	HAL_GPIO_WritePin(ranger->TrigPort,ranger->TrigPin,GPIO_PIN_SET);
	
	//Set a timeout flag; if the flag is not set to 0, it means the data timed out
	ranger->timeoutflag = 1;
	
	//Clear the historical event group flags
	xEventGroupClearBits(g_xEventRangerSensor,1<<ranger->index);
	
	//Check whether element access is correct
//	SENSOR_UsrLog("sensor %c trigger!",0x40+ranger->index);
}

/*
 * Interrupt internal state machine and interrupt usage.
 * Ultrasonic operating mode: trig descending edge triggers operation, ranging signals from low to high, then from high to low, and the high-level time is calculated to measure distance
*/
static void RangerHandle_Result(RangerHALObj* ranger,BaseType_t* wokenTask)
{
	switch (ranger->sm)
	{
		case WaitUp:
		{
			//Read out the count value
			ranger->upvalue = HAL_TIM_ReadCapturedValue(ranger->tim,ranger->ch);
			
			//Set down edge capture
			TIM_RESET_CAPTUREPOLARITY(ranger->tim,ranger->ch);
			TIM_SET_CAPTUREPOLARITY(ranger->tim,ranger->ch,TIM_ICPOLARITY_FALLING);
			
			//State machine switching
			ranger->sm = WaitDown;
			break;
		}
		
		case WaitDown:
		{
			//Read count values
			ranger->downvalue = HAL_TIM_ReadCapturedValue(ranger->tim,ranger->ch);
			
			//Calculate distance
			if( ranger->upvalue > ranger->downvalue )//Spillover calculation
				ranger->dis = ranger->downvalue+0xFFFF - ranger->upvalue;
			else
				ranger->dis = ranger->downvalue - ranger->upvalue;
			
			// S = V*t/2
			ranger->dis = (ranger->dis/TIM_COUNT_FREQ)/2.0f * 340.0f;
			
			//Set rising edge capture
			TIM_RESET_CAPTUREPOLARITY(ranger->tim,ranger->ch);
			TIM_SET_CAPTUREPOLARITY(ranger->tim,ranger->ch,TIM_ICPOLARITY_RISING);
			
			//Trigger the collection callback function to complete
			*wokenTask = ranger->CaptureCompleted(ranger->index);
			
			//Successfully collected data and cleared timeout flags
			ranger->timeoutflag = 0;
			
			//State machine switching
			ranger->sm = WaitUp;
			
			break;
		}
	}		
}

//Ultrasonic capture callback function processing
void Ranger_IC_CaptureCallback(TIM_HandleTypeDef *htim,BaseType_t* woken)
{
	//Match the corresponding ultrasonic waves according to the hardware interface
	for(uint8_t i=0;i<RANGER_NUM;i++)
	{
		//Acquire data from different ultrasonic sensors by matching timer and timer channel
		if( rangerlist[i]->tim == htim && rangerlist[i]->ch == get_tim_activate_ch(htim->Channel) )
		{
			RangerHandle_Result(rangerlist[i],woken);
		}
	}
}

// Group of information structures
typedef struct {
    RangerHALObj** list; // Pointer to the group
    uint8_t size;        // Number of elements in the group
} RangerGroup_t;

//////////// User configuration area, where you can modify ultrasonic grouping and define group assignments; no additional code modifications are needed after configuration////////////
////Define which ultrasounds are used as Group 1
//RangerHALObj* RangerGroup1_List[] = { &RangerHAL_E,&RangerHAL_F }; 

////Define which ultrasounds are used as Group 2
//RangerHALObj* RangerGroup2_List[] = { &RangerHAL_A,&RangerHAL_B }; 

////Define which ultrasounds to use as Group X

////Add the groups defined above one by one to the following ultrasound sets
//static RangerGroup_t GroupManager_List[] = {
//	{ RangerGroup1_List,sizeof(RangerGroup1_List)/sizeof(RangerGroup1_List[0]) } , 
//	{ RangerGroup2_List,sizeof(RangerGroup2_List)/sizeof(RangerGroup2_List[0]) } , 
//};
//////////// User configuration area, where you can modify ultrasonic grouping and define group assignments; no additional code modifications are needed after configuration////////////


//Grouping ultrasound according to different vehicle models
void GetRangerGroup(RangerGroup_t** group,uint8_t* groupNum)
{
	//S260 six-wheeler and S200 outdoor autonomous driving robot are both suitable for this ultrasonic grouping
	static RangerHALObj* RangerGroup1_List_S260[] = { &RangerHAL_A,&RangerHAL_D,&RangerHAL_F };
	static RangerHALObj* RangerGroup2_List_S260[] = { &RangerHAL_B,&RangerHAL_E}; 
	static RangerHALObj* RangerGroup3_List_S260[] = { &RangerHAL_C}; 
	static RangerGroup_t GroupManager_List_S260[] = {
		{ RangerGroup1_List_S260,sizeof(RangerGroup1_List_S260)/sizeof(RangerGroup1_List_S260[0]) } , 
		{ RangerGroup2_List_S260,sizeof(RangerGroup2_List_S260)/sizeof(RangerGroup2_List_S260[0]) } , 
		{ RangerGroup3_List_S260,sizeof(RangerGroup3_List_S260)/sizeof(RangerGroup3_List_S260[0]) } , 
	};
	
	//S300 ultrasonic grouping
	static RangerHALObj* RangerGroup1_List_S300[] = { &RangerHAL_A,&RangerHAL_C,&RangerHAL_E };
	static RangerHALObj* RangerGroup2_List_S300[] = { &RangerHAL_B,&RangerHAL_D,&RangerHAL_F}; 
	static RangerGroup_t GroupManager_List_S300[] = {
		{ RangerGroup1_List_S300,sizeof(RangerGroup1_List_S300)/sizeof(RangerGroup1_List_S300[0]) } , 
		{ RangerGroup2_List_S300,sizeof(RangerGroup2_List_S300)/sizeof(RangerGroup2_List_S300[0]) } , 
	};
	
	//S300 Mini ultrasonic grouping
	static RangerHALObj* RangerGroup1_List_S150[] = { &RangerHAL_A,&RangerHAL_C,&RangerHAL_E };
	static RangerHALObj* RangerGroup2_List_S150[] = { &RangerHAL_B,&RangerHAL_D}; 
	static RangerGroup_t GroupManager_List_S150[] = {
		{ RangerGroup1_List_S150,sizeof(RangerGroup1_List_S150)/sizeof(RangerGroup1_List_S150[0]) } , 
		{ RangerGroup2_List_S150,sizeof(RangerGroup2_List_S150)/sizeof(RangerGroup2_List_S150[0]) } , 
	};
	
	//Select different groups according to the vehicle model
	if( RobotHardWareParam.CarType == S200_OUTDOOR || RobotHardWareParam.CarType == S260 )
	{
		*group = GroupManager_List_S260;
		*groupNum = sizeof(GroupManager_List_S260)/sizeof(GroupManager_List_S260[0]);
	}
	else if( RobotHardWareParam.CarType == S300 )
	{
		*group = GroupManager_List_S300;
		*groupNum = sizeof(GroupManager_List_S300)/sizeof(GroupManager_List_S300[0]);
	}
	else if( RobotHardWareParam.CarType == S300Mini )
	{
		*group = GroupManager_List_S150;
		*groupNum = sizeof(GroupManager_List_S150)/sizeof(GroupManager_List_S150[0]);
	}
	
	//For models without ultrasonic technology, grouping will not be performed; in this case, ultrasonic tasks will be deleted and the corresponding interrupts will be disabled to save system resources

}

//Ultrasonic ranging mission
void ultrasonic_task(void* param)
{
	static uint8_t MAX_Group=0;
	static RangerGroup_t* GroupManager_List = NULL;
	
	//Obtain the number of ultrasound groups and groups
	GetRangerGroup(&GroupManager_List,&MAX_Group);
	
    // Check if the group is valid; invalid groups will not perform this task
    if (GroupManager_List == NULL || MAX_Group == 0)
	{
		//For models not assigned to a group, ultrasonic tasks are deleted and ultrasonic interrupts are disabled to save system resources
		HAL_TIM_IC_Stop_IT(RangerHAL_A.tim,RangerHAL_A.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_B.tim,RangerHAL_B.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_C.tim,RangerHAL_C.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_D.tim,RangerHAL_D.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_E.tim,RangerHAL_E.ch);
		HAL_TIM_IC_Stop_IT(RangerHAL_F.tim,RangerHAL_F.ch);
		vTaskDelete(NULL);
    }
	
	//Event group instance
	g_xEventRangerSensor = xEventGroupCreate();
	EventBits_t waitbits = 0 , resultbits = 0;
	
	//Enable the ultrasonic-related timer capture function
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	
	uint8_t i=0,k=0;
	while(1)
	{
		for(i=0;i<MAX_Group;i++)
		{
			//Ultrasound within the same group triggers simultaneously
			for(k=0;k<GroupManager_List[i].size;k++)
			{
				waitbits |= (1<<GroupManager_List[i].list[k]->index);//Calculate events that require waiting
				RangerTrigger(GroupManager_List[i].list[k]);//Trigger ultrasonic operation
			}

			//Wait for the ultrasonic processing results; the timeout is 30ms
			resultbits = xEventGroupWaitBits(g_xEventRangerSensor,waitbits,pdTRUE,pdTRUE,pdMS_TO_TICKS(30));
			
			//Check for ultrasonic timeouts and perform timeout handling
			if( (resultbits&waitbits) != waitbits )
			{
				for(k=0;k<GroupManager_List[i].size;k++)
				{
					//Handle timeout ultrasound
					if( GroupManager_List[i].list[k]->timeoutflag == 1 ) 
					{
						GroupManager_List[i].list[k]->timeoutCount++;//Count timeout counts
						RangerReset(GroupManager_List[i].list[k]);
						SENSOR_UsrLog("sensor %c timeout...",0x40+GroupManager_List[i].list[k]->index);
					}
				}
			}
			
			waitbits = 0;
			//Next set triggers...
		}
	}
}



