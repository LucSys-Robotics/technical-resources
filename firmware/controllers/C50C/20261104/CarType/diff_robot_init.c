#define ROBOT_INIT_SOURCE
#include "robot_init.h"

//Variables related to the chassis's hardware structure and motor
ROBOT_t robot;

/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
Function: Switch the type of cart to be controlled according to the potentiometer
Entry parameters: None
Return value: none
**************************************************************************/
void Robot_Select(void)
{
	uint16_t TypeNum = 4096/CAR_NUMBER;//Obtain the ADC frequency division coefficient collected by the ADC based on the number of vehicle models
	
	TypeNum = Get_ADC1_Average(CarMode_Ch,10)/TypeNum;
	
	set_RobotType(TypeNum);//Determine different models based on board potentiometer selection
	
	//D50A automatic management: When the old driver has no effective response, only communication failures are recorded, without downtime due to missing data.
	robot.D50A_Enable = 1;
	robot.D50A_Number = 1;
	
	switch( TypeNum )
	{
		//High-spec standard/heavy-duty
		case 0: Robot_Init(TOP_DIFF_wheelspacing , MD36N_27 , GMR_500 , TOP_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		case 1: Robot_Init(TOP_DIFF_wheelspacing , MD36N_51 , GMR_500 , TOP_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		
		//Top-spec oscillating-axle chassis, standard/heavy-duty
		case 2: Robot_Init(FOUR_WHEEL_DIFF_BS_wheelspacing , MD60N_18 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		case 3: Robot_Init(FOUR_WHEEL_DIFF_BS_wheelspacing , MD60N_47 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		
		//Top-spec independent suspension, standard/heavy-duty
		case 4: Robot_Init(FOUR_WHEEL_DIFF_DL_wheelspacing , MD60N_18 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		case 5: Robot_Init(FOUR_WHEEL_DIFF_DL_wheelspacing , MD60N_47 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI , 10000 , 1500); break;
		
		//When no matching gear is available, retain the original default value of D50A without additional mechanical parameter initialization.
		default: robot.D50A_OverCurrent_mA = 10000; robot.D50A_OverTime_ms = 1500;break;
	}
	
}


/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
Function: Initialize car parameters
Entry parameters: track width, wheelbase, rotation radius, motor reduction ratio, motor encoder accuracy, tire diameter
Return value: none
**************************************************************************/
static void set_RobotType(uint8_t type)
{
	robot.type = type;
}

//Input parameters: track width, motor reduction ratio, encoder accuracy, wheel diameter, kp, ki, D50A current mA, overcurrent time ms
static void Robot_Init(float wheelspacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,int vkp,int vki,uint16_t overcurrent_ma,uint16_t overtime_ms)
{
    robot.HardwareParam.WheelSpacing = wheelspacing;   //Track width
	robot.HardwareParam.AxleSpacing = 0;               //Wheelbase
	robot.HardwareParam.GearRatio = gearratio;         //Motor reduction ratio
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //Motor encoder accuracy
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //Wheel circumference (PI*D)
	//Encoder accuracy * motor reduction ratio * frequency = encoder reading after one rotation of the motor
	robot.HardwareParam.Encoder_precision = EncoderMultiples * robot.HardwareParam.EncoderAccuracy * robot.HardwareParam.GearRatio;
	robot.V_KP = vkp;                                  //Default kp parameter
	robot.V_KI = vki;                                  //Default ki parameter
	robot.D50A_OverCurrent_mA = overcurrent_ma;          //D50A sets the overcurrent threshold, unit: mA
	robot.D50A_OverTime_ms = overtime_ms;                //D50A startup overcurrent time, unit: ms

}

