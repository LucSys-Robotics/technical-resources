#ifndef __SYSTEM_H
#define __SYSTEM_H

//Set the program start offset address. Program start address = 0x8000000 + VECT_TAB_OFFSET
#define VECT_TAB_OFFSET  0x00000

//Chip header file
#include "stm32f4xx.h"

//C library function related header file
//Header files related to C library functions
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"

//FreeRTOS related header files
// FreeRTOS related header files
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

//Board level support package related header files
//Board-level support package (hardware) related header files
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "adc.h"
#include "can.h"
#include "encoder.h"
#include "key.h"
#include "LED.h"
#include "motor.h"
#include "oled.h"
#include "pstwo.h"
#include "stmflash.h"
#include "auto_recharge.h"
#include "buzzer.h"
#include "enable_key.h"
#include "uartx.h"
#include "driver_d50a.h"
#include "I2C.h"
#include "MPU6050.h"
#include "ICM20948.h"
#include "remote.h"
#include "usb_host.h"

//Header files related to robot body parameters
#include "robot_init.h"

//Main logic code related header files
//Main logic related header files
#include "balance_task.h"
#include "imu_task.h"
#include "show_task.h"
#include "led_task.h"
#include "ps2_task.h"
#include "data_task.h"
#include "uartx_callback.h"

//Check the multiple vehicle model definitions and ensure that only one model is allowed to exist at a time.
//Define inspections for multiple vehicle types, preventing multiple models from coexisting simultaneously.
#if defined AKM_CAR + defined DIFF_CAR + defined MEC_CAR + defined _4WD_CAR + defined OMNI_CAR > 1
	#error "ERROR: multiple vehicle model definitions."
#endif

//External interface
void systemInit(void);

//Debug task handles
extern TaskHandle_t g_reportErrTaskHandle;

/***Macros define***/ /***Macro Definition***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//The car is only allowed to be controlled after turning on (1000/100Hz=10) seconds after startup

//TODO: Restores to 1000
#define CONTROL_DELAY		1000

//RTOS task frequency
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000


#endif 
