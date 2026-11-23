#include "robot_select_init.h"
#include "bsp_adc.h"
#include "bsp_ServoDrive.h"

//定义机器人参数
RobotParmentType_t RobotHardWareParam = { 0 };

//机器人的控制软件参数
RobotControlParmentType_t RobotControlParam = { 
	.en_flag = 1,          //小车使能标志位
	.ErrNum = 0,           //小车报错码
	.defalutSpeed = 500,   //小车默认遥控速度,单位mm/s
	.ChargeMode = 0,       //自动回充模式状态,默认关闭
	.EmergencyMode = 0,    //应急模式,默认关闭
	.softwareEnflag = 0,   //软件使能位,默认使能
	.LineDiffParam = 50,   //纠偏系数,用于调整走直线效果
	.RangerAvoidEN = 0 ,   //超声波避障位
	.SecurityLevel = 0 ,   //安全等级,0为最高
	.LowPowerFlag = 0  ,   //小车电量低标志位 1表示低电量
	.ImuAssistedFlag = 0,  //默认启用小车走直线时用IMU辅助功能
	.DebugLevel = 0        //调试等级
};

static void Robot_Init(float wheelspacing,float axlespacing,float tyre_diameter,uint8_t WheelType);

void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
	//ADC值分段变量，取决于小车型号数量
	uint16_t Divisor_Mode = 4095/(Number_of_CAR);
	
	//读取电位器档位,保留供后续T系列车型扩展与产线诊断使用
	RobotHardWareParam.PotCarType = USER_ADC_Get_AdcBufValue(userconfigADC_CARMODE_CHANNEL)/Divisor_Mode;

	//本工程固化为T280,车型不随电位器变化,避免电位器误读导致参数全零仍使能
	RobotHardWareParam.CarType = T280;
	
	//车型初始化
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

