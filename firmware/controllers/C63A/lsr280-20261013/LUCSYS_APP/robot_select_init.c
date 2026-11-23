#include "robot_select_init.h"
#include "bsp_adc.h"
#include "bsp_ServoDrive.h"

//Define robot parameters
RobotParmentType_t RobotHardWareParam = { 0 };

//Parameters of the robot's control software
RobotControlParmentType_t RobotControlParam = { 
	.en_flag = 1,          //Chassis enable flag
	.ErrNum = 0,           //Error code for the chassis
	.defalutSpeed = 500,   //Default remote control speed for small cars, unit: mm/s
	.ChargeMode = 0,       //Auto recharge mode mode, turned off by default
	.EmergencyMode = 0,    //Emergency mode, turned off by default
	.softwareEnflag = 0,   //Software enable bit; enabled by default
	.LineDiffParam = 50,   //Correction factor, used to adjust the effect of straight lines
	.RangerAvoidEN = 0 ,   //Ultrasonic obstacle avoidance position
	.SecurityLevel = 0 ,   //safety level, 0 is the highest
	.LowPowerFlag = 0  ,   //Low battery level marker 1 indicates low battery
	.ImuAssistedFlag = 0,  //Enable IMU assistance for straight-line motion by default
	.DebugLevel = 0        //Debug level
};

static void Robot_Init(float wheelspacing,float axlespacing,float tyre_diameter,uint8_t WheelType);

void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
	//ADC value segment variable, depending on the number of car models
	uint16_t Divisor_Mode = 4095/(Number_of_CAR);
	
	//Read potentiometer settings, reserved for future T series model expansion and production line diagnostics
	RobotHardWareParam.PotCarType = USER_ADC_Get_AdcBufValue(userconfigADC_CARMODE_CHANNEL)/Divisor_Mode;

	//This project is solidified to LSR280, with the vehicle not changing with the potentiometer, preventing misreading of the potentiometer and causing parameter zero to remain active
	RobotHardWareParam.CarType = T280;
	
	//Model initialization
	switch( RobotHardWareParam.CarType )
	{
		case S300:
			Robot_Init(S300_Wheelspacing,0,S300_Diameter,DoubleAxis_8inch);
			RobotHardWareParam.driveCounts=1;
			Driver1.initMode=SpeedMode;Driver1.motorType=DoubleAxis_8inch;
			break;
		case S300Mini:
			Robot_Init(S150_Wheelspacing,0,S150_Diameter,DoubleAxis_5inch);
			RobotHardWareParam.driveCounts=1;
			Driver1.initMode=SpeedMode;Driver1.motorType=DoubleAxis_5inch;
			break;
		case S260:
			Robot_Init(S260_Wheelspacing,S260_axlespacing,S260_Diameter,SingleAxis_8inch);
			RobotHardWareParam.driveCounts=3;
			Driver1.initMode=SpeedMode;Driver1.motorType=SingleAxis_8inch;
			Driver2.initMode=SpeedMode;Driver2.motorType=SingleAxis_8inch;
			Driver3.initMode=SpeedMode;Driver3.motorType=SingleAxis_8inch;
			break;
		case S200_OUTDOOR:
			Robot_Init(S200_OUTDOOR_Wheelspacing,S200_OUTDOOR_axlespacing,S200_OUTDOOR_Diameter,SingleAxis_8inch);
			RobotHardWareParam.driveCounts=2;
			Driver1.initMode=SpeedMode;Driver1.motorType=SingleAxis_8inch;
			Driver2.initMode=SpeedMode;Driver2.motorType=SingleAxis_8inch;
			break;
		case S200:
			Robot_Init(S200_Wheelspacing,S200_axlespacing,S200_Diameter,SingleAxis_8inch);
			RobotHardWareParam.driveCounts=2;
			Driver1.initMode=SpeedMode;Driver1.motorType=SingleAxis_8inch;
			Driver2.initMode=SpeedMode;Driver2.motorType=SingleAxis_8inch;
			break;
		case S100:
			Robot_Init(S100_Wheelspacing,0,S100_Diameter,SingleAxis_5inch);
			RobotHardWareParam.driveCounts=1;
			Driver1.initMode=SpeedMode;Driver1.motorType=SingleAxis_5inch;
			break;
		case T280:
			Robot_Init(T280_Wheelspacing,T280_axlespacing,T280_Diameter,SingleAxis_5inch);
			RobotHardWareParam.driveCounts=4;
			Driver1.initMode=PosMode;  Driver1.motorType=SteeringMotor;
			Driver2.initMode=PosMode;  Driver2.motorType=SteeringMotor;
			Driver3.initMode=SpeedMode;Driver3.motorType=SingleAxis_5inch;
			Driver4.initMode=SpeedMode;Driver4.motorType=SingleAxis_5inch;
			break;
		default:
			break;
	}
	
}

static void Robot_Init(float wheelspacing,float axlespacing,float tyre_diameter,uint8_t WheelType) 
{
	RobotHardWareParam.WheelSpacing = wheelspacing;
	RobotHardWareParam.AxleSpacing = axlespacing;
	RobotHardWareParam.WheelPerimeter = tyre_diameter*PI;
	RobotHardWareParam.wheeltype = WheelType;
}

