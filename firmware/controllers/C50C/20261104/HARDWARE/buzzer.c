#include "buzzer.h"

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
	ENABLE_Buzzer_PIN_CLOCK;
	GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(Buzzer_PORT, &GPIO_InitStructure);
	
	Buzzer = BEEP_OFF;
}

/**************************************************************************
Function function: buzzer response function
Entry parameters: number of beeps, beep time interval, and frequency of executing the function
Return value: None
Author: LUCSYS
Function: buzzer response function
Entry parameters: number of beeps, beep interval, frequency of the function
Return value: none
Author: LUCSYS
**************************************************************************/
static uint8_t Buzzer_Response(uint8_t numt,uint8_t times,uint8_t rate)
{
	uint16_t real_times = times*10;       //The beep interval time requested by the user, in milliseconds
	uint8_t taskover = 0;                 //Used to track whether tasks are complete
	uint16_t time_ratio = (uint16_t)( (1.0f/(float)rate) * 1000 );//Smallest unit of time, related to task frequency. Unit: ms
	static uint16_t buzzer_timecore = 0; //Task time, accumulated from the start of the task
	static uint8_t done_count = 0;       //Number of times the beep has been completed
	
	if( numt==0 && times == 0 ) //Do not accept all zero parameters
	{
		taskover = 1;
		buzzer_timecore = 0;
		done_count = 0;
		Buzzer = BEEP_OFF;
		return taskover;
	}
	
	//Check whether the user's requested time interval exceeds the timing minimum resolution
	if( real_times < time_ratio ) 
	{
		//Unreachable time intervals, unresponsive to tasks
		taskover = 1;
		buzzer_timecore = 0;
		done_count = 0;
		Buzzer = BEEP_OFF;
		return taskover;
	}

	//Mission time
	buzzer_timecore += (1.0f/(float)rate) * 1000 ; // Time within the task, unit: ms
	
	//Execute tasks
	if( buzzer_timecore<real_times )
	{
		Buzzer = BEEP_ON;
	}
	else if( buzzer_timecore>=real_times && buzzer_timecore<real_times*2 )
	{
		Buzzer = BEEP_OFF;
	}
	else if( buzzer_timecore>=real_times*2 )
	{
		buzzer_timecore = 0;//Restart timing
		done_count ++ ;     //Record one beep completed
		if(done_count==numt)
		{
			Buzzer = BEEP_OFF;    //Ensure the buzzer does not emit sound
			done_count = 0;
			taskover = 1;
			return taskover;//Complete the task
		}
	}
	return taskover;
}


/* ------------------ Buzzer Task Queue Related Variables, Do Not Modify Directly. ------------------- */
#define BUZZER_QUEUE_LEN 10 //Length of the buzzer task queue

//Task queue for buzzer. A total of BUZZER_QUEUE_LEN groups of tasks (number of beeps (u8), beeping time (u8)).
static uint8_t buzzer_task_buf[BUZZER_QUEUE_LEN][2];
static uint8_t buzzer_NowLen = 0;//Buzzer current task queue length
static uint8_t buzzer_task_tail = 0;//Queue tail
static uint8_t buzzer_task_head = 0;//Queue head
/* ----------------------------------------------------------------------- */

/**************************************************************************
Functionality: Add a task to the buzzer execution queue.
Input Parameters: Number of buzzes, time interval between each buzz (in units of *10ms).
Return Value: 0: Task addition failed, 1: Task addition succeeded.
Author: LUCSYS
Function: Add tasks to the buzzer execution task
Entry parameters: Number of beeps, interval between each beep (unit: 10ms)
Return value: 0-task addition failed; 1-task addition succeeded
Author: LUCSYS
**************************************************************************/
uint8_t Buzzer_AddTask(uint8_t num_time,uint8_t times )
{
	//Check if the task queue is full
	if( buzzer_NowLen >= BUZZER_QUEUE_LEN )
	{
		return 0;
	}
	else
	{
		//Add tasks to the end of the circular queue
		buzzer_task_buf[buzzer_task_tail][0] = num_time;
		buzzer_task_buf[buzzer_task_tail][1] = times;
		
		//Increment queue tail circularly
		buzzer_task_tail = (buzzer_task_tail + 1)%BUZZER_QUEUE_LEN;
		
		//Count the length of data within the queue
		buzzer_NowLen++;
	}
	return 1;
}



/**************************************************************************
Functionality: Buzzer task execution, retrieving and executing tasks added by the
               user through the Buzzer_AddTask() function one by one from the task queue,
               with a fixed time interval of 1 second between each task.
Return Value: None.
Author: LUCSYS
Function: Buzzer tasks, which remove task queues added by users via function Buzzer_AddTask() and execute them side by side. The interval between each task is fixed at 0.5 seconds
Entry parameters: frequency at which the buzzing task is executed
Return value: none
Author: LUCSYS
**************************************************************************/
void Buzzer_task(uint8_t rate)
{
	static uint8_t task_over = 1;
	static uint8_t task_delay = 0;//The time interval between tasks
	static uint8_t start_delay = 0;//Task interval completion flag. 0 tasks executable, 1 waiting
	
	//Storage location for specific task content
	static uint8_t task[2]={0,0};
	
	if( start_delay==0 ) //Task interval delay completion
	{
		//If the queue is nonempty and no buzzer task is running, execute queued tasks in order
		if( buzzer_NowLen!=0 && task_over==1 ) 
		{
			//Retrieve a set of tasks
			task[0] = buzzer_task_buf[buzzer_task_head][0];
			task[1] = buzzer_task_buf[buzzer_task_head][1];
			
			//Circular queue head increment
			buzzer_task_head = (buzzer_task_head + 1)%BUZZER_QUEUE_LEN;
			
			//Count the length of data within the queue
			buzzer_NowLen --;
		}
	}

	//After the task is removed, start execution
	task_over = Buzzer_Response(task[0],task[1],rate);
	
	//Clear the task value after completing the task
	if( task_over==1 ) task[0]=0,task[1]=0,start_delay=1;
	
	//Task waiting: Wait 0.5 seconds between tasks to distinguish between tasks
	if(start_delay)
	{
		task_delay++;
		if( task_delay>=rate*0.5 ) task_delay = 0,start_delay=0;
	}
	
}
