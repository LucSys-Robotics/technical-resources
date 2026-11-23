/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "sys.h"
#include "usart.h"
//Include the appropriate stdint.h for each compiler
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
    #include <stdint.h>
    extern uint32_t SystemCoreClock;
#endif

//assert
#define vAssertCalled(char,int) printf("Error:%s,%d\r\n",char,int)
#define configASSERT(x) if((x)==0) vAssertCalled(__FILE__,__LINE__)

/// Priority values. //Priority definitions
typedef enum {
  osPriorityNone          =  0,         ///< No priority (not initialized).
  osPriorityIdle          =  1,         ///< Reserved for Idle thread.
  osPriorityLow           =  8,         ///< Priority: low
  osPriorityLow1          =  8+1,       ///< Priority: low + 1
  osPriorityLow2          =  8+2,       ///< Priority: low + 2
  osPriorityLow3          =  8+3,       ///< Priority: low + 3
  osPriorityLow4          =  8+4,       ///< Priority: low + 4
  osPriorityLow5          =  8+5,       ///< Priority: low + 5
  osPriorityLow6          =  8+6,       ///< Priority: low + 6
  osPriorityLow7          =  8+7,       ///< Priority: low + 7
  osPriorityBelowNormal   = 16,         ///< Priority: below normal
  osPriorityBelowNormal1  = 16+1,       ///< Priority: below normal + 1
  osPriorityBelowNormal2  = 16+2,       ///< Priority: below normal + 2
  osPriorityBelowNormal3  = 16+3,       ///< Priority: below normal + 3
  osPriorityBelowNormal4  = 16+4,       ///< Priority: below normal + 4
  osPriorityBelowNormal5  = 16+5,       ///< Priority: below normal + 5
  osPriorityBelowNormal6  = 16+6,       ///< Priority: below normal + 6
  osPriorityBelowNormal7  = 16+7,       ///< Priority: below normal + 7
  osPriorityNormal        = 24,         ///< Priority: normal
  osPriorityNormal1       = 24+1,       ///< Priority: normal + 1
  osPriorityNormal2       = 24+2,       ///< Priority: normal + 2
  osPriorityNormal3       = 24+3,       ///< Priority: normal + 3
  osPriorityNormal4       = 24+4,       ///< Priority: normal + 4
  osPriorityNormal5       = 24+5,       ///< Priority: normal + 5
  osPriorityNormal6       = 24+6,       ///< Priority: normal + 6
  osPriorityNormal7       = 24+7,       ///< Priority: normal + 7
  osPriorityAboveNormal   = 32,         ///< Priority: above normal
  osPriorityError         = -1,         ///< System cannot determine priority or illegal priority.
  osPriorityReserved      = 0x7FFFFFFF  ///< Prevents enum down-size compiler optimization.
} osPriority_t;

/***************************************************************************************************************/
/*                                        FreeRTOS basic configuration options                                              */
/***************************************************************************************************************/
#define configUSE_PREEMPTION					1                       //1 uses a preemptive kernel, 0 uses coroutines
#define configUSE_TIME_SLICING					1						//1 Enable time-slice scheduling (enabled by default)
#define configUSE_PORT_OPTIMISED_TASK_SELECTION	1                       //1 Enable special methods to select the next task to run
                                                                        //Generally, hardware computes leading zero instructions, if used
                                                                        //If the MCU doesn¡¯t have these hardware instructions, this macro should be set to 0!
#define configUSE_TICKLESS_IDLE					0                       //1 Enable low-power tickless mode
#define configUSE_QUEUE_SETS					1                       //is 1
#define configCPU_CLOCK_HZ						(SystemCoreClock)       //CPU frequency
#define configTICK_RATE_HZ						(1000)                  //Clock beat frequency, set here to 1000, with a period of 1ms
#define configMAX_PRIORITIES					(32)                    //Maximum available priority
#define configMINIMAL_STACK_SIZE				((unsigned short)130)   //Stack size used by idle tasks
#define configMAX_TASK_NAME_LEN					(16)                    //Length of the task name string

#define configUSE_16_BIT_TICKS					0                       //System beat counter variable data types,
                                                                        //1 indicates a 16-bit unsigned integer, 0 indicates a 32-bit unsigned integer
#define configIDLE_SHOULD_YIELD					1                       //Give up CPU usage rights for idle tasks at 1 hour to other user tasks of the same priority
#define configUSE_TASK_NOTIFICATIONS            1                       //Task notification function is enabled at 1, enabled by default
#define configUSE_MUTEXES						1                       //Use mutually exclusive semaphore when set to 1
#define configQUEUE_REGISTRY_SIZE				8                       //If not 0, it means queue recording is enabled; the specific value is yes
                                                                        //Maximum number of recorded queues and semaphore.
#define configCHECK_FOR_STACK_OVERFLOW			0                       //Enable stack overflow detection when greater than 0, if this feature is used
                                                                        //User must provide a stack overflow hook function, if used
                                                                        //This value can be either 1 or 2, as there are two stack overflow detection methods.
#define configUSE_RECURSIVE_MUTEXES				1                       //Use recursive mutex semaphore when set to 1
#define configUSE_MALLOC_FAILED_HOOK			0                       //1 Use memory request failed hook function
#define configUSE_APPLICATION_TASK_TAG			0                       
#define configUSE_COUNTING_SEMAPHORES			1                       //When 1, use the counting semaphore

/***************************************************************************************************************/
/*                                FreeRTOS and memory application configuration options                                                */
/***************************************************************************************************************/
#define configSUPPORT_DYNAMIC_ALLOCATION        1                       //Supports dynamic memory requests
#define configTOTAL_HEAP_SIZE					((size_t)(30*1024))     //Total heap size of all systems in the system

/***************************************************************************************************************/
/*                                Configuration options related to FreeRTOS hook functions                                              */
/***************************************************************************************************************/
#define configUSE_IDLE_HOOK						0                       //1, use the free hook; 0, do not use it
#define configUSE_TICK_HOOK						0                       //1. Use the time slice hook; 0. Do not use it

/***************************************************************************************************************/
/*                                Configuration options related to FreeRTOS runtime and task status collection                                 */
/***************************************************************************************************************/
#define configGENERATE_RUN_TIME_STATS	        0                       //is 1 Enable queue when
#define configUSE_TRACE_FACILITY				1                       //Enable visual tracking debugging for 1
#define configUSE_STATS_FORMATTING_FUNCTIONS	1                       //When both configUSE_TRACE_FACILITY and macro are 1, the following three functions will be compiled
                                                                        //prvWriteNameToBuffer(),vTaskList(),
                                                                        //vTaskGetRunTimeStats()
                                                                        
/***************************************************************************************************************/
/*                                Configuration options related to FreeRTOS and coroutines                                                  */
/***************************************************************************************************************/
#define configUSE_CO_ROUTINES 			        0                       //Enable the coroutine when set to 1. After enabling the coroutine, you must add the file croutine.c Enable the software timer when
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )                   //Number of valid priorities for coroutines

/***************************************************************************************************************/
/*                                Configuration options related to FreeRTOS and software timers                                            */
/***************************************************************************************************************/
#define configUSE_TIMERS				        1                               //is 1 Enable runtime statistics when
#define configTIMER_TASK_PRIORITY		        (configMAX_PRIORITIES-1)        //Software timer priority
#define configTIMER_QUEUE_LENGTH		        5                               //Software timer queue length
#define configTIMER_TASK_STACK_DEPTH	        (configMINIMAL_STACK_SIZE*2)    //Software timer task stack size

/***************************************************************************************************************/
/*                                FreeRTOS optional function configuration options                                                      */
/***************************************************************************************************************/
#define INCLUDE_xTaskGetSchedulerState          1                       
#define INCLUDE_vTaskPrioritySet		        1
#define INCLUDE_uxTaskPriorityGet		        1
#define INCLUDE_vTaskDelete				        1
#define INCLUDE_vTaskCleanUpResources	        1
#define INCLUDE_vTaskSuspend			        1
#define INCLUDE_vTaskDelayUntil			        1
#define INCLUDE_vTaskDelay				        1
#define INCLUDE_eTaskGetState			        1
#define INCLUDE_xTimerPendFunctionCall	        1

/***************************************************************************************************************/
/*                                FreeRTOS configuration options related to interrupts                                                  */
/***************************************************************************************************************/
#ifdef __NVIC_PRIO_BITS
	#define configPRIO_BITS       		__NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       		4                  
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			15                      //Lowest interrupt priority
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5                       //Highest interruption priority the system can manage
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/***************************************************************************************************************/
/*                                Configuration options related to FreeRTOS interrupt service functions                                          */
/***************************************************************************************************************/
#define xPortPendSVHandler 	PendSV_Handler
#define vPortSVCHandler 	SVC_Handler

//ms to OS Ticks
#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))
	
#endif /* FREERTOS_CONFIG_H */

