#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//Reference all header files that need to be used
#include "FreeRTOSConfig.h"
//FreeRTOS related header files
//FreeRTOS related header files
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//The associated header file for the peripheral 
//Related header files of the peripheral
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "show.h"								   
#include "pstwo.h"
#include "key.h"
#include "robot_select_init.h"
#include "I2C.h"
#include "HubDriver.h"
#include "HubDriverCan.h"
#include "MPU6050.h"
#include "StepperMotor.h"

//Is it an Ackermann car, or a differential car
//Is it an Ackermann-structured chassis or a differential chassis?
#define Omni 0
#define Mec  0
#define Diff 1

//Motor speed control related parameters of the structure
//Parameter structures related to motor speed control
typedef struct  
{
	float Encoder;     //Read the real-time speed of the motor by encoder //Encoder value, read motor real-time speed
	float Encoder_Rpm;   //Encoder value, motor real-time speed, unit: rpm
	float Control_Rpm;   //Control the target speed of the brushless motor // Control the target speed of the brushless motor
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor // Motor PWM value, control the motor's real-time speed
	float Target;      //Control the target speed of the motor // Motor target speed value, control motor target speed
	float Velocity_KP; //Speed control PID parameters // Speed control PID parameters
	float	Velocity_KI; //Speed control PID parameters // Speed control PID parameters
}Motor_parameter;

//Smoothed the speed of the three axes
//Smoothed three-axis speed
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** External variable definitions: When other C files reference system.h, variables defined by system.c can also be used******/
extern u8 Flag_Stop, Last_Flag_Stop;

extern int Divisor_Mode;   
extern u8 Car_Mode;
extern float RC_Velocity;
extern float Move_X,Move_Y,Move_Z; 
extern float Velocity_KP,Velocity_KI;	
extern Smooth_Control smooth_control;
extern Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
extern float Encoder_precision;
extern float Wheel_perimeter; 
extern float Wheel_spacing; 
extern float Wheel_axlespacing; 
extern float Omni_turn_radiaus; 
extern u8 PS2_ON_Flag, APP_ON_Flag, Remote_ON_Flag, CAN_ON_Flag, Usart_ON_Flag;   
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern u8 PID_Send; 
extern float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern int robot_mode_check_flag;
extern float Moveit_Angle1,Moveit_Angle2,Moveit_Angle3,Moveit_Angle4,Moveit_Angle5,Moveit_Angle6;
extern u8 motor1_flag,motor2_flag,motor3_flag;
extern float Angle_poor[3];
extern float Angle1_poor,Angle2_poor,Angle3_poor,Last_angleA,Last_angleB,Last_angleC,Speed1,Speed2,Speed3;
extern int Servo1;
extern short Moveit_Angle1_init;
extern float Position1,Velocity1;

extern u8 Get_Charging_HardWare;
extern u8  charger_check;
void Find_Charging_HardWare(void);

void systemInit(void);

/***Macros define***/ /***Macro Definition***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//The car is only allowed to be controlled after turning on (1000/100Hz=10) seconds after startup
#define CONTROL_DELAY		750
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//Number of robot models, which determines the value of the Divisor_Mode; currently, there are 6 types of cars
#define CAR_NUMBER    7      
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
/***Macros define***/ /***Macro Definition***/

//C library function related header file
//Header files related to C library functions
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
