#include "robot_init.h"

static void set_RobotType(uint8_t type);

static void Robot_Init(float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
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
		//Reserved
		case 0: Robot_Init( SENIOR_MEC_BS_wheelspacing , SENIOR_MEC_BS_axlespacing , MD36N_27 , GMR_500 , Mecanum_100 , VEL_KP , VEL_KI ); break;
		
		//High-spec standard oscillating-axle chassis
		case 1: Robot_Init( SENIOR_MEC_BS_wheelspacing , SENIOR_MEC_BS_axlespacing , MD36N_27 , GMR_500 , Mecanum_100 , VEL_KP , VEL_KI ); break;
		
		//High-spec standard independent-suspension chassis
		case 2: Robot_Init( SENIOR_MEC_DL_wheelspacing , SENIOR_MEC_DL_axlespacing , MD36N_27 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
		//Top-spec oscillating-axle chassis, standard/heavy-duty
		case 3: Robot_Init( TOP_MEC_BS_wheelspacing , TOP_MEC_BS_axlespacing , MD60N_18 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		case 4: Robot_Init( TOP_MEC_BS_wheelspacing , TOP_MEC_BS_axlespacing , MD60N_47 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
		//Top-spec independent suspension, standard/heavy-duty
		case 5: Robot_Init( TOP_MEC_DL_wheelspacing , TOP_MEC_DL_axlespacing , MD60N_18 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		case 6: Robot_Init( TOP_MEC_DL_wheelspacing , TOP_MEC_DL_axlespacing , MD60N_47 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
		//Flagship pendulum type, conventional, heavy-duty load
		case 7: Robot_Init( FLAGSHIP_MEC_BS_wheelspacing , FLAGSHIP_MEC_BS_axlespacing , MD60N_18 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		case 8: Robot_Init( FLAGSHIP_MEC_BS_wheelspacing , FLAGSHIP_MEC_BS_axlespacing , MD60N_47 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
		//Flagship standalone and heavy-duty models
		case 9 : Robot_Init( FLAGSHIP_MEC_DL_wheelspacing , FLAGSHIP_MEC_DL_axlespacing , MD60N_18 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		case 10: Robot_Init( FLAGSHIP_MEC_DL_wheelspacing , FLAGSHIP_MEC_DL_axlespacing , MD60N_47 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
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

//Entry parameters: vehicle model, track width, wheelbase, motor reduction ratio, motor encoder accuracy, wheel diameter, Ackermann minimum turning radius, omnidirectional wheel rotation radius, default KP, default KI
static void Robot_Init(float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,int vkp,int vki)
{
    robot.HardwareParam.WheelSpacing = wheelspacing;   //Track width
	robot.HardwareParam.AxleSpacing = axlespacing;     //Wheelbase
	robot.HardwareParam.GearRatio = gearratio;         //Motor reduction ratio
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //Motor encoder accuracy
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //Wheel circumference (PI*D)
	//Encoder accuracy * motor reduction ratio * frequency = encoder reading after one rotation of the motor
	robot.HardwareParam.Encoder_precision = EncoderMultiples * robot.HardwareParam.EncoderAccuracy * robot.HardwareParam.GearRatio;
	robot.V_KP = vkp;                                  //Default kp parameter
	robot.V_KI = vki;                                  //Default ki parameter
}

