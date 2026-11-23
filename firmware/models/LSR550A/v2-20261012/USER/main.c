/* LucSys Robotics prepares and maintains this distribution. */
#include "system.h"

//Task priority //Task Priority
#define START_TASK_PRIO	1

//Task stack size //Task Stack Size
#define START_STK_SIZE 	512  

//Task handle // Task handle
TaskHandle_t StartTask_Handler;

//Task function // Task function
void start_task(void *pvParameters);

//Main function // Main function
int main(void)
{ 
	systemInit(); //Hardware initialization // Hardware initialization

	//Create the start task // Create start task
	xTaskCreate((TaskFunction_t )start_task,            //Task function // Task function
							(const char*    )"start_task",          //Task name //Task Name
							(uint16_t       )START_STK_SIZE,        //Task stack size //Task Stack Size
							(void*          )NULL,                  //Arguments passed to the task function // Arguments passed to the task function
							(UBaseType_t    )START_TASK_PRIO,       //Task priority //Task Priority
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle // Task handle
	vTaskStartScheduler();  //Enable task scheduling
}
 
//Start task task function // Start task task function
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area
	
    //Create the task // Create the task
	xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);	//Vehicle motion control task // Vehicle motion control task
	xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL);  //User interaction tasks related to data display // User interaction tasks related to data display
	xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);	 //LED light flashing task //LED light flashing task
	xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     &data_TaskHandle);	 //Send data to each interface task //Send data to each interface task
	
	if(SysVal.HardWare_Ver==V1_0) 	//IMU data read task // IMU data reading task, start different tasks based on different hardware versions
	{
		xTaskCreate(MPU6050_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
		xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      &show_TaskHandle);	 //Read the PS2 controller task //Read PS2 controller task
	}
		
	else if( SysVal.HardWare_Ver==V1_1 )
	{
		xTaskCreate(ICM20948_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	}
	
	
    vTaskDelete(StartTask_Handler); //Delete the start task // Delete the start task

    taskEXIT_CRITICAL();            //Exit the critical section // Exit critical section
}






