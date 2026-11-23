#include "robot_select_init.h"
#include "bsp_adc.h"

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
	.ImuAssistedFlag = 1,  //Enable IMU assistance for straight-line motion by default
	.DebugLevel = 0        //Debug level
};

static void Robot_Init(float wheelspacing,float axlespacing,float tyre_diameter,uint8_t WheelType);

void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
	//ADC value segment variable, depending on the number of car models
	uint16_t Divisor_Mode = 4095/(Number_of_CAR-1);
	
	//The S Series is tentatively planned for 0~5 in six models
	RobotHardWareParam.CarType = USER_ADC_Get_AdcBufValue(userconfigADC_CARMODE_CHANNEL)/Divisor_Mode; //Collect the pin information of potentiometer // Collect potentiometer pin information
	
	//Set different parameters according to the vehicle configuration: track width, wheelbase, wheel diameter, wheel model
	if( RobotHardWareParam.CarType == S300 )      Robot_Init(S300_Wheelspacing,0,S300_Diameter,DoubleAxis_8inch);
	else if( RobotHardWareParam.CarType == S200 ) Robot_Init(S200_Wheelspacing,S200_axlespacing,S200_Diameter,SingleAxis_8inch);
	else if( RobotHardWareParam.CarType == S300Mini ) Robot_Init(S150_Wheelspacing,0,S150_Diameter,DoubleAxis_5inch);
	else if( RobotHardWareParam.CarType == S100 ) Robot_Init(S100_Wheelspacing,0,S100_Diameter,SingleAxis_5inch);
	
	//S260 6-wheeler, model with differential
	else if( RobotHardWareParam.CarType == S260 ) Robot_Init(S260_Wheelspacing,S260_axlespacing,S260_Diameter,SingleAxis_8inch);
	
	//Outdoor version of the S200
	else if( RobotHardWareParam.CarType == S200_OUTDOOR ) Robot_Init(S200_OUTDOOR_Wheelspacing,S200_OUTDOOR_axlespacing,S200_OUTDOOR_Diameter,SingleAxis_8inch);
	
	
	//Initialize the number of drivers according to chassis type
	switch( RobotHardWareParam.CarType )
	{
		case S300:case S300Mini:case S100:
			RobotHardWareParam.driveCounts=1;
			break;
		case S200:case S200_OUTDOOR:
			RobotHardWareParam.driveCounts=2;
			break;
		case S260:
			RobotHardWareParam.driveCounts=3;
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

