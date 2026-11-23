#include "robot_init.h"

static void set_RobotType(uint8_t type);
static void Robot_Init(float wheelspacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,int vkp,int vki);

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
	
	switch( TypeNum )
	{
		//High-spec standard/heavy-duty
		case 0: Robot_Init(TOP_DIFF_wheelspacing , MD36N_27 , GMR_500 , TOP_DIFF_WheelDiameter , VEL_KP , VEL_KI); break;
		case 1: Robot_Init(TOP_DIFF_wheelspacing , MD36N_51 , GMR_500 , TOP_DIFF_WheelDiameter , VEL_KP , VEL_KI); break;
		
		//Top-spec oscillating-axle chassis, standard/heavy-duty
		case 2: Robot_Init(FOUR_WHEEL_DIFF_BS_wheelspacing , MD60N_18 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI); break;
		case 3: Robot_Init(FOUR_WHEEL_DIFF_BS_wheelspacing , MD60N_47 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI); break;
		
		//Top-spec independent suspension, standard/heavy-duty
		case 4: Robot_Init(FOUR_WHEEL_DIFF_DL_wheelspacing , MD60N_18 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI); break;
		case 5: Robot_Init(FOUR_WHEEL_DIFF_DL_wheelspacing , MD60N_47 , GMR_500 , FOUR_WHEEL_DIFF_WheelDiameter , VEL_KP , VEL_KI); break;
		
		default:break;
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

//Entry parameters: vehicle model, track width, motor reduction ratio, motor encoder accuracy, wheel diameter, default KP, default KI
static void Robot_Init(float wheelspacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,int vkp,int vki)
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

}

