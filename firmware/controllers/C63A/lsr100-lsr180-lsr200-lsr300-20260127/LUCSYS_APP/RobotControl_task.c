/**
 * @file    RobotControl_task.c
 * @brief Cart control logic file.
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - This file is the core content of the chassis's operation, RobotControl_task by the task, receiving various remote or serial ports from the control queue
 *   Velocity command data: use kinematics to calculate each wheel's rotational speed, then transmit the speeds
 *   Command to the drive to complete the chassis drive
 * @note
 * 
 * 
 */
#include "RobotControl_task.h"

//HAL Lib Include File
#include "main.h"

//C Include File
#include <stdio.h>
#include <math.h>

//BSP Include File
#include "bsp_adc.h"
#include "bsp_ServoDrive.h"
#include "bsp_RTOSdebug.h"
#include "bsp_icm20948.h"
#include "bsp_RGBLight.h"
#include "bsp_buzzer.h"


//APP Include File
#include "robot_select_init.h"
#include "sensor_ranger.h"
#include "xbox360_gamepad.h"
#include "show_task.h"

//Used to store the control queue data of the car
QueueHandle_t  g_xQueueRobotControl = NULL;

//Used to record the current control source of the car
volatile uint8_t RobotControl_CMDsource = NONE_CMD; 

//Internal use function
static float linearVel_to_rpm(float linearVel,float wheelper);
static void Robot_DriverRecovery(void);
static void robot_en_check(void);
static void Robot_DriverDisable(void);
static void Robot_DriverEnable(void);
static void Robot_DriverSetParkingMode(uint8_t setflag);
static void Robot_DriverCheckError(void);
static void Robot_DriverStop(void);
static void Linear_IMUHelp(RobotControlCMDType_t* target);

//IMU assists the cart in a straight line
static RobotControlCMDType_t LinearHelp_Target = { 0 };

void RobotControl_task(void* param)
{
	vTaskDelay(1000);
	
	//Create control command queue
	g_xQueueRobotControl = xQueueCreate(1,sizeof(RobotControlCMDType_t));
	
	//Obtain the timebase, which can run at a fixed frequency for auxiliary tasks
	TickType_t preTime = xTaskGetTickCount();
	
	//The control frequency for this task, in Hz
	const uint16_t TaskFreq = 100;

	//Debug variables
//	pRTOSDebugInterface_t debug = &RTOSDebugTimer;
//	RTOSDebugPrivateVar priv = { 0 };
	
	//Servo motor self-check variables
	uint8_t driverCheck = 0;
	
	//Idle count
	TickType_t ActiveTick = xTaskGetTickCount();
	
	//Empty-queue statistics
	uint8_t LostCmdCounts = 0;
	
	//Buzzer interface
	pBuzzerInterface_t buzzer = &UserBuzzer;
	
	RobotControlCMDType_t cmd = { 0 };
	while(1)
	{
		//Read control instructions from the queue
		BaseType_t control_queue_state = pdFAIL;
		control_queue_state = xQueueReceive(g_xQueueRobotControl,&cmd,0);
		if( control_queue_state == pdPASS )
			LostCmdCounts=0;
		else 
			LostCmdCounts++;

		//Ultrasonic obstacle avoidance
		if( RobotControlParam.RangerAvoidEN == 1 && RobotControlParam.EmergencyMode==0 )
			ranger_avoid(&cmd);
		
		//Gyroscope straight-line compensation
		if( RobotControlParam.ImuAssistedFlag == 1 )
			Linear_IMUHelp(&cmd);
		else
		{
			LinearHelp_Target.Vx=0;LinearHelp_Target.Vy=0;LinearHelp_Target.Vz=0;
		}
			
		//Inverse kinematics: obtain wheel linear speed in m/s
		Drive_Motor(&cmd);
		
		//Cart enable status check, determining whether to start the chassis based on self-test parameters
		robot_en_check();

		//When the car is enabled, the motor movement is controlled according to different vehicle models
		static uint8_t ErrTipsFlag = 0;
		if( RobotControlParam.en_flag == 1 )
		{	
			switch( RobotHardWareParam.CarType )
			{
				case S260: //6-wheeled vehicles
					set_MotorRPM(&Driver1,RobotControlParam.MotorA.target,RobotControlParam.MotorF.target);
					set_MotorRPM(&Driver2,RobotControlParam.MotorB.target,RobotControlParam.MotorE.target);
					set_MotorRPM(&Driver3,RobotControlParam.MotorC.target,RobotControlParam.MotorD.target);
					break;
				case S200:case S200_OUTDOOR: //Four-wheel drive vehicles
					set_MotorRPM(&Driver1,RobotControlParam.MotorA.target,RobotControlParam.MotorD.target);
					set_MotorRPM(&Driver2,RobotControlParam.MotorB.target,RobotControlParam.MotorC.target);
					break;
				case S300: case S100: //Two-wheel differential
					set_MotorRPM(&Driver1,RobotControlParam.MotorA.target,RobotControlParam.MotorB.target);
					break;
				case S300Mini:
					set_MotorRPM(&Driver1,-RobotControlParam.MotorA.target,-RobotControlParam.MotorB.target);
					break;
			}
			
			//Allow to disable error prompts after enabling them
			if( ErrTipsFlag==1 )
			{
				ErrTipsFlag = 0;
				//Cancel post-error action can be added
			}
		}
		else //Chassis disabled; transmit 0 speed to stop it
		{
			Robot_DriverStop();
			
			//When the car is disabled, a control command is received, and the buzzer alerts with an error
			//if( control_queue_state == pdPASS ) buzzer->AddTask(3,100);
			
			//After being disabled, an error prompt will be triggered.
			if( ErrTipsFlag == 0 )
			{
				ErrTipsFlag=1;
				//Post-error action can be added
			}
		}
		
		//Forward kinematics: calculate three-axis velocity from wheel speeds
		Robot_Forwardkinematics(&RobotControlParam,&RobotHardWareParam);
		
		//Determine whether there is currently an active control quantity
		if( cmd.Vx!=0 || cmd.Vy!=0 || cmd.Vz!=0 )
		{
			ActiveTick = xTaskGetTickCount();
			if( RobotControlParam.ParkingMode==1 )
			{
				RobotControlParam.ParkingMode=0;
				//Release parking mode
				Robot_DriverSetParkingMode(0);
			}
		}
		
		//Car loss of control protection settings (queue without data timeout)
		if( LostCmdCounts > TaskFreq )
		{
			LostCmdCounts=TaskFreq;
			
			switch( RobotControl_CMDsource )
			{
				//Non-host control types require protection by default
				//This protection would also apply to app control, but cannot be enabled for compatibility with the iOS app, which sends data intermittently.
				case GamePad_CMD:case RCJOY_CMD: case Charger_CMD:
					cmd.Vx=0;cmd.Vy=0;cmd.Vz=0;
					break;
				
				//Host type, protection enabled according to user-set safety levels
				case ROS_CMD:case CAN_CMD:
					if( RobotControlParam.SecurityLevel==0 ) 
						cmd.Vx=0;cmd.Vy=0;cmd.Vz=0;
					break;
			}
		}
		
		//Determine whether to enter parking mode based on whether the control amount is active. If the car stops for 10 seconds and there is no control mode, it will enter parking mode to save power
		if( RobotControlParam.ParkingMode==0 && xTaskGetTickCount() - ActiveTick > 10000 )
		{
			//Determine whether the site is on a slope based on the IMU angle
			if(fabs(AttitudeVal.pitch) < 10.0f && fabs(AttitudeVal.roll)<10.0f )
			{
				RobotControlParam.ParkingMode=1;
				//Send a hold command
				Robot_DriverSetParkingMode(1);
			}
			else
			{	//The car is on a slope, refreshing the time used for judgment
				ActiveTick = xTaskGetTickCount();
			}
		}
		
		//Poll driver self-test parameters at low frequency, including motor error codes and driver errors
		driverCheck++;
		if( driverCheck==10 )
		{
			driverCheck=0;
			Robot_DriverCheckError();
		}
		
		//Driver fault-recovery request; after a fault, users can set this flag to request an attempt to clear it
		if( RobotControlParam.DriveErrRecovery )
		{
			RobotControlParam.DriveErrRecovery = 0;
			Robot_DriverRecovery();
		}
		
		//Low battery buzzer alert for small cars; only models without light strips use buzzer alerts
		static uint32_t LowPowerTipsCnt = 0;
		switch( RobotHardWareParam.CarType )
		{
			case S200:case S200_OUTDOOR:case S260:
				if( RobotControlParam.LowPowerFlag )
				{
					LowPowerTipsCnt++;
					if( LowPowerTipsCnt>TaskFreq*4 )
					{
						LowPowerTipsCnt=0;
						buzzer->AddTask(1,750);
					}
				}
				break;
			default:
				break;
		}
		
		
		/* Delay the specified frequency */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

//Multi-mode intelligent control handle button callback functions, all related to the car's functions
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	pBuzzerInterface_t buzzer = &UserBuzzer;
	
	//LB top-left button: decelerate
	if( keyid == Xbox360KEY_LB )
	{
		RobotControlParam.defalutSpeed -= 50;
		if( RobotControlParam.defalutSpeed<0 ) RobotControlParam.defalutSpeed=0;
		buzzer->AddTask(1,200);
	}
	
	//RB upper-right button: accelerate
	if( keyid == Xbox360KEY_RB )
	{
		RobotControlParam.defalutSpeed += 50;
		if( RobotControlParam.defalutSpeed>3000 ) RobotControlParam.defalutSpeed=3000;
		buzzer->AddTask(1,200);
	}
	
	//Right joystick Rjoy: IMU marked zero
	if( keyid == Xbox360KEY_RJoy && event == GamePadKeyEvent_LONGCLICK)
	{
		pIMUInterface_t imu = &UserICM20948;
		imu->UpdateZeroPoint_axis();
		imu->UpdateZeroPoint_attitude();
		buzzer->AddTask(1,200);//Use the buzzer to indicate the operation was successful
	}
		
	//Y: Auto recharge on/off
	if( keyid == Xbox360KEY_Y && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.ChargeMode = !RobotControlParam.ChargeMode;
		buzzer->AddTask(1,200);
	}
	
	//B: Clear motor error requests
	if( keyid == Xbox360KEY_B && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.DriveErrRecovery=1;
		buzzer->AddTask(1,200);
	}
	
	//A: Obstacle avoidance request
	if( keyid == Xbox360KEY_A && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.RangerAvoidEN = !RobotControlParam.RangerAvoidEN;
		buzzer->AddTask(1,200);
	}
	
	//X: IMU assists straight switching
	if( keyid==Xbox360KEY_X && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.ImuAssistedFlag = !RobotControlParam.ImuAssistedFlag;
		buzzer->AddTask(1,200);
	}
	
	//Left analog stick LJoy: Emergency mode
	if( keyid == Xbox360KEY_LJoy && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.EmergencyMode = !RobotControlParam.EmergencyMode;
		buzzer->AddTask(1,200);
	}
	
	//Directional buttons for left and right OLED page turning
	if( keyid == Xbox360KEY_LEFT )
	{
		oled_page_down();
		buzzer->AddTask(1,200);
	}
	
	if( keyid == Xbox360KEY_RIGHT )
	{
		oled_page_up();
		buzzer->AddTask(1,200);
	}
}

//Linear speed m/s to RPM speed
static float linearVel_to_rpm(float linearVel,float wheelper)
{
	return linearVel*60.0f/wheelper;
}

//Driver fault-recovery function
static void Robot_DriverRecovery(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		//If the motor has errors, clear them
		if(g_ServoDriveList[i]->errFlag==1)
		{
			ServoDriver_ClearError(g_ServoDriveList[i]);
		}
			
		//Determine whether to enable the motor based on the state of the enable switch
		if( RobotControlParam.Enkeystate==1 ) ServoDriver_Enable(g_ServoDriveList[i]);
		else ServoDriver_Disable(g_ServoDriveList[i]);
		
		//Reset rotational speed to zero
		set_MotorRPM(g_ServoDriveList[i],0,0);
	}
}

//Set parking mode
static void Robot_DriverSetParkingMode(uint8_t setflag)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_ParkingMode(g_ServoDriveList[i],setflag);
	}
}

//Obtain driver error information
static void Robot_DriverCheckError(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_CheckError(g_ServoDriveList[i]);
	}
}

//Disable motor
static void Robot_DriverDisable(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_Disable(g_ServoDriveList[i]);
	}
}

//Enable motor
static void Robot_DriverEnable(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_Enable(g_ServoDriveList[i]);
	}
}

//Motor self-locking stops
static void Robot_DriverStop(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		set_MotorRPM(g_ServoDriveList[i],0,0);
	}
}

//Cart enable status check
static void robot_en_check(void)
{
	//The default error code is 0, set according to the following error situations
	RobotControlParam.ErrNum = 0;
	
	//Calculate the robot battery voltage; if the battery is insufficient, control of the chassis will be prohibited
	RobotControlParam.Vol = (float)USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL)/4095.0f*3.3f*11;
	
	//Software emergency-stop switch state
	if( RobotControlParam.softwareEnflag == 1 ) Set_RobotErrorCode(errCode_SoftWareStop);
	
	//Read the status of the robot's emergency stop switch and perform corresponding operations when switching buttons
	static uint8_t last_EnKey = 1;
	RobotControlParam.Enkeystate = get_EnKeyState();
	if( RobotControlParam.Enkeystate==0 ) Set_RobotErrorCode(errCode_StopKeyEn);
	
	//Enable switch is pressed
	if( RobotControlParam.Enkeystate==0 && last_EnKey==1 )
	{
		Robot_DriverDisable();
	}
	
	//Enable switch pops up
	else if( last_EnKey==0 && RobotControlParam.Enkeystate==1 )
	{
		Robot_DriverEnable();
	}
	
	last_EnKey = RobotControlParam.Enkeystate;
	
	//Robot low battery status acquisition
	static uint32_t LowPowerCnt=0;
	if( RobotControlParam.Vol>18.0f && RobotControlParam.Vol < 20.5f )
	{
		LowPowerCnt++;
		if( LowPowerCnt==100*10 ) 
		{
			RobotControlParam.LowPowerFlag=1;
		}
	}
	else
	{
		LowPowerCnt=0;
		if( RobotControlParam.Vol>21.5f) RobotControlParam.LowPowerFlag=0;
	}

	//Insufficient voltage. (If automatic recharge mode is enabled, voltage is ignored)
	if( RobotControlParam.Vol < 20.0f && RobotControlParam.ChargeMode==0 ) Set_RobotErrorCode(errCode_LowPower);
	
	//Generate different error codes based on the number of drivers
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		if( g_ServoDriveList[i]->errFlag==1 )
		{
			switch(g_ServoDriveList[i]->NodeID)
			{
				case 1: Set_RobotErrorCode(errCode_Driver1_Err); break;
				case 2: Set_RobotErrorCode(errCode_Driver2_Err);break;
				case 3: Set_RobotErrorCode(errCode_Driver3_Err);break;
			}
		}
		
		if( g_ServoDriveList[i]->onlineFlag==0 )
		{
			switch(g_ServoDriveList[i]->NodeID)
			{
				case 1: Set_RobotErrorCode(errCode_Driver1_offline); break;
				case 2: Set_RobotErrorCode(errCode_Driver2_offline); break;
				case 3: Set_RobotErrorCode(errCode_Driver3_offline); break;
			}
		}
	}

	if( RobotControlParam.ErrNum!=0 && RobotControlParam.EmergencyMode==0 ) RobotControlParam.en_flag = 0;
	else RobotControlParam.en_flag = 1;
}


/**
 * @brief This function is used to write control queues; all tasks that need to control the cart must write control quantities through this task.
 * @param cmd: The parameter must include the instruction source and specify the three-axis target speed for the cart
           woken: if called in an interrupt, this parameter needs to be passed. For non-interrupt calls, just write 0
 * @retval 0 write successful, 1 write failed. Reason for write failure: No source of control instruction specified or a higher-priority control is currently active
 *
 * @details
 * - When users need to add control methods, they must first add an enumeration of control types in the RobotControl_task.h header file and determine their priority.
 *   Then control the cart via the WriteControlQueue write control queue
 */
uint8_t WriteRobotControlQueue(RobotControlCMDType_t* cmd,BaseType_t* woken)
{
	static uint32_t lastTick = 0;
	uint8_t writeflag = 0;
	
	//Ignore writes if the queue is not created or the control source is unspecified
	if(g_xQueueRobotControl == NULL || cmd->cmdsource == NONE_CMD || 
		cmd->cmdsource >= UnKnownCMD ) return 1;
	
	//High-priority commands can preempt control directly
	if( cmd->cmdsource <= RobotControl_CMDsource )
	{
		RobotControl_CMDsource = cmd->cmdsource;
		writeflag = 1;
	}
	else
	{
		//Receive low-priority control instructions and check active timestamps
		uint32_t tick = 0;
		if( __get_IPSR()!=0 ) tick = xTaskGetTickCountFromISR();
		else tick = xTaskGetTickCount();
		
		//If a high-priority command is inactive for more than 2000ms, allow low-priority commands
		if( tick - lastTick >= 2000 )
		{
			RobotControl_CMDsource = cmd->cmdsource;
			writeflag = 1;
		}
	}
	
	//Allow writing to the queue
	if( writeflag )
	{
		if( __get_IPSR()!=0 ) //Interrupt contextual judgment
		{
			lastTick = xTaskGetTickCountFromISR();
			xQueueOverwriteFromISR(g_xQueueRobotControl,cmd,woken);
		}			
		else 
		{
			lastTick = xTaskGetTickCount();
			xQueueOverwrite(g_xQueueRobotControl,cmd);
		}
		return 0;
	}
	
	return 1;
}

/**
 * @brief This function provides the forward kinematics for different vehicle models. Different correct formulas are applied according to different vehicle models, ultimately obtaining the three-axis speed of the chassis
 * @param robot: Object related to car control robot_hw: Object related to car hardware parameters
 * @retval None
 *
 * @details
 * 
 *   
 */
static void Robot_Forwardkinematics(RobotControlParmentType_t* robot,const RobotParmentType_t* robot_hw)
{
	switch( robot_hw->CarType )
	{
		case S260:
		{
			robot->feedbackVx = (robot->MotorA.feedback + robot->MotorB.feedback + robot->MotorC.feedback + robot->MotorD.feedback + robot->MotorE.feedback + robot->MotorF.feedback )/6.0f;
			robot->feedbackVy = 0;
			robot->feedbackVz = (-robot->MotorA.feedback - robot->MotorB.feedback - robot->MotorC.feedback + robot->MotorD.feedback + robot->MotorE.feedback + robot->MotorF.feedback)/6.0f/(robot_hw->WheelSpacing+robot_hw->AxleSpacing);
			break;
		}
		case S200:case S200_OUTDOOR:
		{
			robot->feedbackVx = (robot->MotorA.feedback + robot->MotorB.feedback + robot->MotorC.feedback + robot->MotorD.feedback )/4.0f;
			robot->feedbackVy = 0;
			robot->feedbackVz = (-robot->MotorA.feedback - robot->MotorB.feedback + robot->MotorC.feedback + robot->MotorD.feedback )/4.0f/(robot_hw->WheelSpacing+robot_hw->AxleSpacing);
			break;
		}
		case S300:case S300Mini:case S100:
		{
			robot->feedbackVx = (robot->MotorA.feedback+robot->MotorB.feedback)/2.0f;
			robot->feedbackVy = 0;
			robot->feedbackVz = (robot->MotorB.feedback-robot->MotorA.feedback)/robot_hw->WheelSpacing;
			break;
		}
	}
}

/**
 * @brief This function is used to calculate the correction factor and to correct the effect of the cart moving in a straight line
 * @param Correction coefficient value, left wheel/right wheel
 * @retval Correction multiplier
 *
 * @details
 * When the car can't go straight, you can adjust the correction factor to improve straight-line performance.
 * 0-100 is an adjustable value, 50 means no correction effect. If the car is more than 50, it will automatically veer to the right,
 * Cars less than 50 meters automatically veer to the left.
 *   
 */
float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //Left-wheel correction, corresponding to a correction factor of 50~100 and 1.0~1.2 times
	{
		if( diffparam>50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //Right wheel correction: 50~0 corresponds to a correction factor of 1.0~1.2 times
	{
		if( diffparam<50 )
			return 1.0f + 0.004f*(50-diffparam);
	}
	
	return 1.0f;//If the condition is not met, the default is 1.
}


/**
 * @brief This function is used for speed smoothing control and kinematic inverse solutions. Different inverse formulas are applied depending on the vehicle model, ultimately obtaining the wheel speed
 * @param Vx, Vy, Vz: The speed of the cart's X, Y, and Z axes, unit m/s
 * @retval None
 *
 * @details
 * 
 *   
 */
static void Drive_Motor(const RobotControlCMDType_t* target)
{
	//Smoothed velocity value
	static RobotControlCMDType_t smooth = { 0 }; 
	
	//Wheel target speed limit 
	//Wheel target speed limit, unit: m/s
	float amplitude=3.5;
	
	//The actual incoming control is user control + gyroscope compensation
	RobotControlCMDType_t control_val = { 0 };
	control_val.Vx = target->Vx ;
	control_val.Vy = target->Vy ;
	control_val.Vz = target->Vz + LinearHelp_Target.Vz;
	
	//In automatic recharge mode, only commands from the recharge device and ROS host computer are received; other commands are set to speed 0
	if( RobotControlParam.ChargeMode==1 )
	{
		if( target->cmdsource!=Charger_CMD && target->cmdsource!=ROS_CMD )
		{
			control_val.Vx=0;control_val.Vy=0;control_val.Vz=0;
		}
	}
	
	//Velocity smoothing
	Smooth_control(&smooth,&control_val,0.02f);
	
	//Calculation of correction coefficient
	float LeftWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,0);
	
	//Kinematic solutions for six-wheelers
	if( RobotHardWareParam.CarType == S260 )
	{
		//Four-wheel drive model Wheelbase is not a true wheelbase; it needs to be coordinated to an appropriate value based on IMU data
		RobotControlParam.MotorA.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorB.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorC.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorD.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorE.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorF.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);

		//Assignment of correction coefficients
		RobotControlParam.MotorA.target *= LeftWheelDiff;
		RobotControlParam.MotorB.target *= LeftWheelDiff;
		RobotControlParam.MotorC.target *= LeftWheelDiff;
		RobotControlParam.MotorD.target *= RightWheelDiff;
		RobotControlParam.MotorE.target *= RightWheelDiff;
		RobotControlParam.MotorF.target *= RightWheelDiff;
		
		//Maximum wheel speed limit
		RobotControlParam.MotorA.target = target_limit_float(RobotControlParam.MotorA.target,-amplitude,amplitude);
		RobotControlParam.MotorB.target = target_limit_float(RobotControlParam.MotorB.target,-amplitude,amplitude);
		RobotControlParam.MotorC.target = target_limit_float(RobotControlParam.MotorC.target,-amplitude,amplitude);
		RobotControlParam.MotorD.target = target_limit_float(RobotControlParam.MotorD.target,-amplitude,amplitude);
		RobotControlParam.MotorE.target = target_limit_float(RobotControlParam.MotorE.target,-amplitude,amplitude);
		RobotControlParam.MotorF.target = target_limit_float(RobotControlParam.MotorF.target,-amplitude,amplitude);
		
		//Convert wheel speed to rpm
		RobotControlParam.MotorA.target = linearVel_to_rpm(RobotControlParam.MotorA.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorB.target = linearVel_to_rpm(RobotControlParam.MotorB.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorC.target = linearVel_to_rpm(RobotControlParam.MotorC.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorD.target = -linearVel_to_rpm(RobotControlParam.MotorD.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorE.target = -linearVel_to_rpm(RobotControlParam.MotorE.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorF.target = -linearVel_to_rpm(RobotControlParam.MotorF.target,RobotHardWareParam.WheelPerimeter);
	}
	
	//Four-wheel drive vehicles
	else if( RobotHardWareParam.CarType == S200 || RobotHardWareParam.CarType == S200_OUTDOOR )
	{
		//Obtain the linear velocity
		RobotControlParam.MotorA.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorB.target = smooth.Vx - smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorC.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		RobotControlParam.MotorD.target = smooth.Vx + smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
		
		//Assignment of correction coefficients
		RobotControlParam.MotorA.target *= LeftWheelDiff;
		RobotControlParam.MotorB.target *= LeftWheelDiff;
		RobotControlParam.MotorC.target *= RightWheelDiff;
		RobotControlParam.MotorD.target *= RightWheelDiff;
		
		//Maximum wheel speed limit
		RobotControlParam.MotorA.target = target_limit_float(RobotControlParam.MotorA.target,-amplitude,amplitude);
		RobotControlParam.MotorB.target = target_limit_float(RobotControlParam.MotorB.target,-amplitude,amplitude);
		RobotControlParam.MotorC.target = target_limit_float(RobotControlParam.MotorC.target,-amplitude,amplitude);
		RobotControlParam.MotorD.target = target_limit_float(RobotControlParam.MotorD.target,-amplitude,amplitude);
		
		//Convert wheel speed to rpm
		RobotControlParam.MotorA.target = linearVel_to_rpm(RobotControlParam.MotorA.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorB.target = linearVel_to_rpm(RobotControlParam.MotorB.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorC.target = -linearVel_to_rpm(RobotControlParam.MotorC.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorD.target = -linearVel_to_rpm(RobotControlParam.MotorD.target,RobotHardWareParam.WheelPerimeter);
	}
	
	//Two-wheel differential
	else if( RobotHardWareParam.CarType == S300 || RobotHardWareParam.CarType == S300Mini ||
              RobotHardWareParam.CarType == S100 )
	{
		RobotControlParam.MotorA.target = smooth.Vx - smooth.Vz * RobotHardWareParam.WheelSpacing / 2.0f; 
		RobotControlParam.MotorB.target = smooth.Vx + smooth.Vz * RobotHardWareParam.WheelSpacing / 2.0f; 
		RobotControlParam.MotorA.target *= LeftWheelDiff;
		RobotControlParam.MotorB.target *= RightWheelDiff;
		RobotControlParam.MotorA.target = target_limit_float(RobotControlParam.MotorA.target,-amplitude,amplitude);
		RobotControlParam.MotorB.target = target_limit_float(RobotControlParam.MotorB.target,-amplitude,amplitude);
		RobotControlParam.MotorA.target = -linearVel_to_rpm(RobotControlParam.MotorA.target,RobotHardWareParam.WheelPerimeter);
		RobotControlParam.MotorB.target = linearVel_to_rpm(RobotControlParam.MotorB.target,RobotHardWareParam.WheelPerimeter);
		
		//The S300 wheels are installed in the opposite direction
		if( RobotHardWareParam.CarType == S300 || RobotHardWareParam.CarType == S300Mini)
		{
			RobotControlParam.MotorA.target=-RobotControlParam.MotorA.target;
			RobotControlParam.MotorB.target=-RobotControlParam.MotorB.target;
		}
	}
	
}


//Velocity smoothing control
static void Smooth_control(RobotControlCMDType_t* cmd,const RobotControlCMDType_t* target,float step)
{
	//Smooth X-axis speed
	if(target->Vx>cmd->Vx)
	{
		cmd->Vx+=step;
		if(cmd->Vx>target->Vx) cmd->Vx=target->Vx;
	}
	else if (target->Vx<cmd->Vx)
	{
		cmd->Vx-=step;
		if(cmd->Vx<target->Vx) cmd->Vx=target->Vx;
	}
	else
		 cmd->Vx=target->Vx;

	//Smooth Y-axis speed
	if(target->Vy>cmd->Vy)
	{
		cmd->Vy+=step;
		if(cmd->Vy>target->Vy) cmd->Vy=target->Vy;
	}
	else if (target->Vy<cmd->Vy)
	{
		cmd->Vy-=step;
		if(cmd->Vy<target->Vy) cmd->Vy=target->Vy;
	}
	else
		 cmd->Vy=target->Vy;
	
	//Smooth Z-axis speed
//	if(target->Vz>cmd->Vz)
//	{
//		cmd->Vz+=step*2;
//		if(cmd->Vz>target->Vz) cmd->Vz=target->Vz;
//	}
//	else if (target->Vz<cmd->Vz)
//	{
//		cmd->Vz-=step*2;
//		if(cmd->Vz<target->Vz) cmd->Vz=target->Vz;
//	}
//	else
	
	//Z axis does not use smoothing
	cmd->Vz=target->Vz;
}

/**
 * @brief This function executes ultrasonic obstacle avoidance logic
 * @param Car target speed value
 * @retval None
 *
 * @details
 * 
 *   
 */
static void ranger_avoid(RobotControlCMDType_t* target)
{
	//Obstacle-avoidance flag: front
	uint8_t avoid_front = 0;
	
	//Obstacle-avoidance flag: rear
	uint8_t avoid_back = 0;
	
	//Reference Z-axis velocity used during obstacle avoidance
	float base_vz = PI/4.0f;
	
	//Buzzer prompt interface
	pBuzzerInterface_t tips = &UserBuzzer;
	
	//Filter value
	static uint8_t FilterCnt = 0;
	
	//Determine whether obstacle avoidance is needed, and set the obstacle avoidance distance accordingly
	if( RobotHardWareParam.CarType == S300 )
	{
		if( RangerHAL_A.dis < 0.4f || RangerHAL_B.dis < 0.4f || RangerHAL_C.dis < 0.4f || 
			RangerHAL_D.dis < 0.4f || RangerHAL_E.dis < 0.4f || RangerHAL_F.dis < 0.4f )
		{
			FilterCnt++;
		}
		else FilterCnt = 0;
		
		if( FilterCnt >= 5 ) FilterCnt=5,avoid_front=1;
	}
	else if( RobotHardWareParam.CarType == S300Mini )
	{
		if( RangerHAL_A.dis < 0.4f || RangerHAL_B.dis < 0.4f || RangerHAL_C.dis < 0.4f || 
			RangerHAL_D.dis < 0.4f || RangerHAL_E.dis < 0.4f )
		{
			FilterCnt++;
		}
		else FilterCnt = 0;
		
		if( FilterCnt >= 5 ) FilterCnt=5,avoid_front=1;
	}
	else if( RobotHardWareParam.CarType == S200_OUTDOOR || RobotHardWareParam.CarType == S260 )
	{
		static uint8_t BackFilterCnt = 0 ; //Rear obstacle avoidance sensor
		if( RangerHAL_A.dis < 0.4f || RangerHAL_B.dis < 0.4f || RangerHAL_C.dis < 0.4f || 
			RangerHAL_D.dis < 0.4f )
		{
			FilterCnt++;
		}
		else FilterCnt=0;
		
		if( RangerHAL_E.dis < 0.4f || RangerHAL_F.dis < 0.4f )
		{
			BackFilterCnt++;
		}
		else BackFilterCnt = 0;
		
		if( FilterCnt >= 5 ) FilterCnt=5,avoid_front=1;
		if( BackFilterCnt >= 5 ) BackFilterCnt=5,avoid_back=1;
	}
	
	
	//Terminate the program directly without obstacle avoidance
	if( avoid_front == 0 && avoid_back==0 ) return;
	
	//Obstacle-avoidance logic varies by chassis type
	if( RobotHardWareParam.CarType == S300 )
	{
		if( target->Vx > 0 )
		{
			if( RangerHAL_A.dis+RangerHAL_B.dis+RangerHAL_C.dis > RangerHAL_D.dis+RangerHAL_E.dis+RangerHAL_F.dis)
			{
				//Turn left
				target->Vz = (target->Vx/0.5f)*base_vz;
			}
			else
			{
				//Turn right
				target->Vz = -(target->Vx/0.5f)*base_vz;
			}
			target->Vx = 0;
			tips->AddTask(3,100);
		}
	}
	else if( RobotHardWareParam.CarType == S300Mini )
	{
		if( target->Vx > 0 )
		{
			if( RangerHAL_A.dis+RangerHAL_B.dis > RangerHAL_D.dis+RangerHAL_E.dis )
			{
				//Turn left
				target->Vz = (target->Vx/0.5f)*base_vz;
			}
			else
			{
				//Turn right
				target->Vz = -(target->Vx/0.5f)*base_vz;
			}
			target->Vx = 0;
			tips->AddTask(3,100);
		}
	}
	else if( RobotHardWareParam.CarType == S200_OUTDOOR || RobotHardWareParam.CarType == S260 )
	{
		//Avoid obstacles in front
		if( target->Vx > 0 && avoid_front==1 )
		{
			if( RangerHAL_A.dis+RangerHAL_B.dis > RangerHAL_C.dis+RangerHAL_D.dis )
			{
				//Turn right
				target->Vz = -(target->Vx/0.5f)*base_vz;
			}
			else
			{
				//Turn left
				target->Vz = (target->Vx/0.5f)*base_vz;
			}
			target->Vx=0;
			tips->AddTask(3,100);
		}
		
		//Rear obstacle avoidance (does not avoid obstacles when automatically recharging equipment control)
		else if( target->Vx < 0 && avoid_back==1 && RobotControl_CMDsource!=Charger_CMD )
		{
			target->Vx = 0;//Rear obstacle avoidance is only prohibited from backward
			tips->AddTask(3,100);
		}
	}
}



/**
 * @brief This function performs the IMU data line routing function
 * @param Car target speed value
 * @retval None
 *
 * @details
 *  IMU and cart control direction: When rotating counterclockwise, yaw is positive, yaw speed is positive. Vz is the correct time, the cart rotates counterclockwise
 *   
 */
static void Linear_IMUHelp(RobotControlCMDType_t* target)
{
	//Used to delay the acquisition of the target angle
	static uint8_t TargetCnt = 0;
	#define DelayCNT 70
	
	//Target angle
	static float target_angle = 0;
	
	//PID parameters
	float linear_helpkp = 0.08f;
	float linear_helpkd = 0.05f;
	
	switch( RobotHardWareParam.CarType )
	{
		//Different models have different PID parameters when adjusting in a straight line, which is related to body weight, body structure, and motor
		case S100: linear_helpkp = 0.03f; linear_helpkd = 0.007f; break;
		case S300Mini: linear_helpkp = 0.04f; linear_helpkd = 0.005f; break;
		case S300: linear_helpkp = 0.06f; linear_helpkd = 0.015f; break;
		default:break;
	}
	
	static uint8_t giveup_flag = 0;

	//Only compensates when Vx has speed; otherwise, no compensation and target angle is updated
	if( target->Vx!=0 && target->Vy==0 && target->Vz==0 && giveup_flag==0 ) 
	{
		//Delay obtaining the target angle
		TargetCnt++;
		if( TargetCnt==DelayCNT )  target_angle = AttitudeVal.yaw;

		//After obtaining the target angle, execute PID control
		if( TargetCnt >= DelayCNT )
		{
			//Lock the counter value to prevent cycling
			TargetCnt=DelayCNT+1;
			
			//Error calculation
			float error = target_angle-AttitudeVal.yaw;
			
			//Normalize error angles
			if( error >  180.0f ) error -= 360.0f;
			if( error < -180.0f ) error += 360.0f;
			
			//When the deviation is too large, abandon adjustments (such as suspension control).
			if( fabs(error)> 30 ) 
			{
				giveup_flag = 1;
				LinearHelp_Target.Vz=0;
				return;
			}
			
			//PID adjusts the straight-line effect of the car (only adjusts when the speed exceeds a certain level)
			static uint8_t AllowFilterCnt;
			if( fabs(RobotControlParam.feedbackVx)>=0.05f )
			{
				AllowFilterCnt++;
				if( AllowFilterCnt>80 )
				{
					AllowFilterCnt=81;
					LinearHelp_Target.Vz = linear_helpkp * error - linear_helpkd * axis_9Val.gyro.z*57.3f;
				}
				else LinearHelp_Target.Vz=0;
			}
			else
			{
				LinearHelp_Target.Vz=0;
				AllowFilterCnt=0;
			}
				
			
			//Limit adjustment value
			LinearHelp_Target.Vz = target_limit_float(LinearHelp_Target.Vz,-1.57f,1.57f);
		}

	}
	else if( target->Vx==0 && target->Vy==0 && target->Vz==0 )
	{
		//Refresh target angles after remaining stationary with all velocities at 0 for a while
		TargetCnt++;
		if( TargetCnt>100 )  target_angle = AttitudeVal.yaw,TargetCnt=101;
		LinearHelp_Target.Vx = 0;
		LinearHelp_Target.Vy = 0;
		LinearHelp_Target.Vz = 0;
		giveup_flag=0;
	}
	else
	{	
		//In other cases, the refresh of the target angle is delayed to prevent inaccurate refresh during dynamic switching
		TargetCnt=0;
		LinearHelp_Target.Vx = 0;
		LinearHelp_Target.Vy = 0;
		LinearHelp_Target.Vz = 0;
	}
	
}


float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

float rpm_to_linearVel(float rpm,float wheelper)
{
	return rpm/60.0f*wheelper;
}

//Reset to enter the bootloader function
void _System_Reset_FromAPP_RTOS(char uart_recv)
{
	static char res_buf[5];
	static char res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//Upon receiving the reset character "reset" requested by the host computer, perform a software reset
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			RobotControlCMDType_t cmd = {
				.cmdsource = BootLoader,
				0,0,0
			};
			
			//Write the highest priority instruction to stop the chassis's movement
			for(uint8_t i=0;i<50;i++)
			{
				WriteRobotControlQueue(&cmd,0);
				vTaskDelay( pdMS_TO_TICKS(10) );
			}
			
			//Reset system
			NVIC_SystemReset();
		}
	}
}

//Debug mode
void RobotControl_SetDebugLevel(char uart_recv)
{
	//Debug task handles
	extern TaskHandle_t g_reportErrTaskHandle;
	
	static char res_buf[4];
	static char res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='L'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==4)
	{
		res_count = 0;
		
		if( res_buf[0]=='L'&&res_buf[1]=='O'&&res_buf[2]=='G')
		{
			switch( res_buf[3] )
			{
				case '0': RobotControlParam.DebugLevel = 0; break;
				case '1': RobotControlParam.DebugLevel = 1; break;
				case '2': RobotControlParam.DebugLevel = 2; break;
				case '3': RobotControlParam.DebugLevel = 3; break;
				default: RobotControlParam.DebugLevel = 0; break;
			}
			
			//Operation prompt sound
			pBuzzerInterface_t tips = &UserBuzzer;
			tips->AddTask(1,200);
			
			//Send task notifications
			xTaskNotifyGive(g_reportErrTaskHandle);
		}
	}
}


