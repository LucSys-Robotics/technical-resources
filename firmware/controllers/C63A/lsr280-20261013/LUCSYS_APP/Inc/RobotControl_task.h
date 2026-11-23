#ifndef __ROBOTCONTROL_TASK_H
#define __ROBOTCONTROL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_armcc.h"

#include "robot_select_init.h"

//Control instruction priority; the smaller the value, the higher the priority
//High-priority control can interrupt low-priority control
enum{
	NONE_CMD   = 0,  //User cannot modify. 0 means ignore the control value to prevent writing without a specified source
	BootLoader ,  //User cannot modify, BootLoader is the highest priority system control, used for wireless updates of the car \
	                   程序更新前最高优先级控制量将介入控制小车停止。
	
	//User-defined zones, with priority from highest to lowest
	GamePad_CMD , //Game controller
	RCJOY_CMD   , //RC transmitter
	APP_CMD     , //Mobile app
	ROS_CMD     , //ROS or serial port upper computer
	CAN_CMD     , //CAN1 interface control
	Charger_CMD , //Automatic equipment recharge control
	AutoTest_CMD, //Automated site testing tasks (lowest priority, any manual control can preemptive)
	
	UnKnownCMD     //User cannot modify, this enum must always stay at the last position to prevent writing control values without a specified source
};

//Data format for the cart control queue
typedef struct{
	uint8_t cmdsource;
	float Vx;
	float Vy;
	float Vz;
}RobotControlCMDType_t;

//The latest control source for small cars
extern volatile uint8_t RobotControl_CMDsource;

//Flags for automatic chassis disable conditions; any 1 condition disables the chassis and prohibits control
enum{
	errCode_LowPower        = (1<<0), //Low battery
	errCode_StopKeyEn       = (1<<1), //Emergency-stop switch pressed
	errCode_SoftWareStop    = (1<<2), //Software emergency stop is set
	errCode_Driver1_offline = (1<<3), //Drive 1 is offline
	errCode_Driver1_Err     = (1<<4), //Drive 1 has an error
	errCode_Driver2_offline = (1<<5), //Drive 2 is offline
	errCode_Driver2_Err     = (1<<6), //Drive 2 has an error
	errCode_Driver3_offline = (1<<7), //Drive 3 is offline
	errCode_Driver3_Err     = (1<<8), //Drive 3 has an error
	errCode_Driver4_offline = (1<<9), //Drive 4 is offline
	errCode_Driver4_Err     = (1<<10), //Drive 4 has an error
	errCode_SteeringHome    = (1<<11), //Steering homing timeout
};

#define Clear_RobotErrorCode(mask)  (RobotControlParam.ErrNum &= ~(mask))    //Clear error flags
#define Set_RobotErrorCode(mask)    (RobotControlParam.ErrNum |= (mask))     //Set error flags
#define Get_RobotErrorCode(mask)    (RobotControlParam.ErrNum & (mask))      //Read error flag bits


//Public use to the public
uint8_t WriteRobotControlQueue(RobotControlCMDType_t* cmd,BaseType_t* woken);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel);
void _System_Reset_FromAPP_RTOS(char uart_recv);
float rpm_to_linearVel(float rpm,float wheelper);
void RobotControl_SetDebugLevel(char uart_recv);


#endif

