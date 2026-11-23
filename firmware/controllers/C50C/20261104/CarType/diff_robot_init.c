#define ROBOT_INIT_SOURCE
#include "robot_init.h"

//小车硬件结构和电机相关变量
ROBOT_t robot;

/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
	uint16_t TypeNum = 4096/CAR_NUMBER;//根据车型数量,获取ADC采集的分频系数
	
	TypeNum = Get_ADC1_Average(CarMode_Ch,10)/TypeNum;
	
	set_RobotType(TypeNum);//根据板子电位器选择确定不同的车型
	
	//D50A自动管理：旧驱动无有效回复时只记录通信失败，不因缺数据停机.
	robot.D50A_Enable = 1;
	robot.D50A_Number = 1;
	
	switch( TypeNum )
	{
		//高配常规、重载
		case 0: Robot_Init(TOP_DIFF_wheelspacing , MD36N_27 , GMR_500 , TOP_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		case 1: Robot_Init(TOP_DIFF_wheelspacing , MD36N_51 , GMR_500 , TOP_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		
		//顶配摆式常规、重载
		case 2: Robot_Init(FOUR_WHEEL_DIFF_BS_wheelspacing , MD60N_18 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		case 3: Robot_Init(FOUR_WHEEL_DIFF_BS_wheelspacing , MD60N_47 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		
		//顶配独立常规、重载
		case 4: Robot_Init(FOUR_WHEEL_DIFF_DL_wheelspacing , MD60N_18 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		case 5: Robot_Init(FOUR_WHEEL_DIFF_DL_wheelspacing , MD60N_47 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		
		//无匹配档位时保留D50A原默认值，不额外初始化机械参数.
		default: robot.D50A_OverCurrent_mA = 10000; robot.D50A_OverTime_ms = 1500;break;
	}
	
}


/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
static void set_RobotType(uint8_t type)
{
	robot.type = type;
}

//入口参数：轮距、电机减速比、编码器精度、轮径、kp、ki、D50A电流mA、过流时间ms
static void Robot_Init(float wheelspacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,int vkp,int vki,uint16_t overcurrent_ma,uint16_t overtime_ms)
{
    robot.HardwareParam.WheelSpacing = wheelspacing;   //轮距
	robot.HardwareParam.AxleSpacing = 0;               //轴距
	robot.HardwareParam.GearRatio = gearratio;         //电机减速比
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //电机编码器精度
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //轮子周长(PI*D)
	//编码器精度*电机减速比*倍频数 = 电机旋转一圈的编码器读数
	robot.HardwareParam.Encoder_precision = EncoderMultiples * robot.HardwareParam.EncoderAccuracy * robot.HardwareParam.GearRatio;
	robot.V_KP = vkp;                                  //默认kp参数
	robot.V_KI = vki;                                  //默认ki参数
	robot.D50A_OverCurrent_mA = overcurrent_ma;          //D50A启动过流阈值，单位mA
	robot.D50A_OverTime_ms = overtime_ms;                //D50A启动过流时间，单位ms

}

