/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//C Include File
#include <stdio.h>

//FreeRTOS Include File
#include "queue.h"

//BSP Include File
#include "bsp_oled.h"
#include "bsp_RTOSdebug.h"
#include "bsp_ServoDrive.h"
#include "bsp_RGBLight.h"
#include "bsp_can.h"
#include "bsp_ServoDrive.h"
#include "bsp_buzzer.h"

//APP Include File
#include "sensor_ranger.h"
#include "robot_select_init.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//FreeRTOS system debugging
#define userconfig_OPEN_STACK_CHECK     0  //Enable task stack size check printing
#define userconfig_OPEN_CPU_USAGE_CHECK 0  //Enable the CPU percentage check for printing
#define userconfig_OPEN_CHECK_HEAPSIZE  0  //Check the remaining heap size

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

QueueHandle_t g_xQueueCANopenCallback = NULL;

//Bluetooth data queue
QueueHandle_t g_xQueueBlueTooth = NULL;

//ROS data queue
QueueHandle_t  g_xQueueROSserial = NULL;

//Automatically refill queue data
QueueHandle_t g_xQueueAutoRecharge = NULL;

TaskHandle_t g_reportErrTaskHandle = NULL;

/* USER CODE END Variables */
/* Definitions for InitTask */
osThreadId_t InitTaskHandle;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

//FreeRTOS system debugging tasks
#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
void CpuUsageCheckTask(void *param);
#endif

//Task function declaration
void ultrasonic_task(void* param);
void show_task(void* param);
void ImuTask(void* param);
void BlueToothControlTask(void* param);
void SerialControlTask(void* param);
void RobotControl_task(void* param);
void AutoRechargeTask(void* param);
void data_task(void* param);
void RobotDataTransmitTask(void* param);
void APPshow_task(void* param);
void HeartbeatTask(void* param);
void ReportErrTask(void* param);
void RGBControl_task(void* param);

/* USER CODE END FunctionPrototypes */

void StartInitTask(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	//Initialize content before using the CPU share timer
	TIM6->CNT = 0;
}

__weak unsigned long getRunTimeCounterValue(void)
{
	static unsigned long time = 0 ;
	static uint16_t lasttime = 0;
	static uint16_t nowtime = 0;
	
	nowtime = TIM6->CNT; //Get the current count value
	
	//If the current count is less than the previous count, it indicates a timer count overflow
	if( nowtime < lasttime )
	{
		time += (nowtime + 0xffff - lasttime); //Time increment after overflow
	}		
	else time += ( nowtime - lasttime ) ; //No overflow occurred; increments are from the current time to the previous time
	
	lasttime = nowtime;
	
	return time;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
	
	//Idle task hook function
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	
	//Task stack overflow warning
	printf("%s stack overflow\r\n",pcTaskName);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	
	//Memory allocation failure warning
	printf("malloc failed.check heapsize\r\n");
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	g_xQueueBlueTooth = xQueueCreate(50,sizeof(char));//Bluetooth data queue
	g_xQueueROSserial = xQueueCreate(50,sizeof(char));//ROS data queue
	g_xQueueAutoRecharge = xQueueCreate(1,sizeof(CANmsgType_t));
	
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of InitTask */
  InitTaskHandle = osThreadNew(StartInitTask, NULL, &InitTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //Car control tasks
  xTaskCreate(RobotControl_task,"robotcontrol",128*4,NULL,osPriorityNormal1,NULL);
  
  //Ultrasonic data acquisition tasks
  xTaskCreate(ultrasonic_task,"ultraTask",128*2,NULL,osPriorityNormal,NULL);
  
  //OLED display tasks
  xTaskCreate(show_task,"showTask",128*4,NULL,osPriorityBelowNormal7,NULL);

  //APP displays tasks
  xTaskCreate(APPshow_task,"AppShow",128*2,NULL,osPriorityBelowNormal7,NULL);

  //Acquire gyroscope data
  xTaskCreate(ImuTask,"ImuTask",128*4,NULL,osPriorityNormal,NULL);
  
  //Bluetooth data parsing tasks
  xTaskCreate(BlueToothControlTask,"BTControlTask",128*2,NULL,osPriorityNormal,NULL);
  
  //ROS serial data parsing task
  xTaskCreate(SerialControlTask,"SerialConTask",128*2,NULL,osPriorityNormal,NULL);
  
  //Automatic recharge device data analysis task
  xTaskCreate(AutoRechargeTask,"AutoRechargeTask",128*2,NULL,osPriorityNormal,NULL);
  
  //Data outbound tasks
  xTaskCreate(RobotDataTransmitTask,"transmit_task",128*4,NULL,osPriorityNormal,NULL);
  
  //RGB light strip management task
  xTaskCreate(RGBControl_task,"RGB_task",128*2,NULL,osPriorityBelowNormal7,NULL);
  
  //CANopen heartbeat packet processing task
  xTaskCreate(HeartbeatTask,"HeartbeatTask",128,NULL,osPriorityNormal1,NULL);
  
  //Submit the vehicle self-inspection and debugging task
  xTaskCreate(ReportErrTask,"ReportErrTask",128*4,NULL,osPriorityNormal,&g_reportErrTaskHandle);
  
	//FreeRTOS debugging tasks
	#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
	 static uint16_t delaytime = 5000;//Print time interval, unit tick
	 xTaskCreate(CpuUsageCheckTask,"DebugTask",128, &delaytime ,osPriorityAboveNormal,NULL);
	#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartInitTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInitTask */
void StartInitTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartInitTask */
  /* Infinite loop */
	
	//Initialization tasks used to initialize projects that partially depend on the FreeRTOS kernel
	
	//Initialize the CAN filter
	pCANInterface_t can = &UserCAN1Dev;
	can->init();
	can = &UserCAN2Dev;
	can->init();
	
	//Power on and wait for the drive to be ready
	HAL_Delay(1200);
	
	//Close all CANopen mapping relationships to prevent data interference
	uint8_t PreOp[8]= {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	can = &UserCAN2Dev;
	can->sendStd(0x00,PreOp,8);
	//Enable feedback queue preparation configuration
	g_xQueueCANopenCallback = xQueueCreate(1,sizeof(CANmsgType_t));
	
	//Initialize the corresponding driver based on the number of drives for the vehicle model
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_Init(g_ServoDriveList[i]);//Load from the list of available drivers
	}

	//Power-on prompt
	pBuzzerInterface_t tips = &UserBuzzer;
	tips->AddTask(1,700);

	//Initialization complete, delete yourself
	vTaskDelete(NULL);
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartInitTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
void CpuUsageCheckTask(void *param)
{
	uint16_t* delaytime = (uint16_t*)param;
	static char showbuf[500];
	
	while( 1 )
	{
#if 1 == userconfig_OPEN_CPU_USAGE_CHECK
		//Print CPU proportion
		vTaskGetRunTimeStats(showbuf);
		printf("TaskName\tUseTime\tCPU\r\n");
		printf("%s\r\n",showbuf);
		vTaskDelay(*delaytime);
#endif

#if 1 == userconfig_OPEN_STACK_CHECK
		//Print the remaining task stack size, unit in word
		vTaskList(showbuf);
		printf("TaskName\tTaskState\tTaskPrio\tStackSize\tTaskNum\r\n");
		printf("%s\r\n",showbuf);
		vTaskDelay(*delaytime);
#endif
		
#if 1 == userconfig_OPEN_CHECK_HEAPSIZE
		//Print the remaining heap size, in bytes
		printf("free heap size : %d bytes\r\n\r\n",xPortGetFreeHeapSize());
		vTaskDelay(*delaytime);
#endif
	}
}
#endif


/* USER CODE END Application */

