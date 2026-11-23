#include "robot_init.h"

static void set_RobotType(uint8_t type);
static void Robot_Init(float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,float akm_min_turn,int vkp,int vki);


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
/*
Ackermann chassis code descriptions:
0: High-end Ackermann Regular SENIOR_AKM
1: high configuration Ackermann overload SENIOR_AKM
2: Top-tier Ackermann pendulum suspension standard TOP_AKM_BS
3: Top-tier Ackermann pendulum suspension with heavy load TOP_AKM_BS
4: Top-tier Ackermann independent suspension standard TOP_AKM_DL
5: Top-tier Ackermann independent suspension with heavy load TOP_AKM_DL
6: Non-standard custom models
7: Models used for installation and debugging
8: Models used for installation and commissioning
9: High-end Ackermann High-Speed Edition SENIOR_AKM
*/
void Robot_Select(void)
{
	uint16_t TypeNum = 4096/CAR_NUMBER;//Obtain the ADC frequency division coefficient collected by the ADC based on the number of vehicle models
	
	TypeNum = Get_ADC1_Average(CarMode_Ch,10)/TypeNum;
	
	set_RobotType(TypeNum);//Determine different models based on board potentiometer selection
	
	switch( TypeNum )
	{
		//High-spec standard/heavy-duty
		case 0: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI); break;
		case 1: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_51,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		
		//Top-spec oscillating-axle chassis, standard/heavy-duty
		case 2: Robot_Init(TOP_AKM_BS_wheelspacing,TOP_AKM_BS_axlespacing,MD60N_18,GMR_500,TOP_AKM_BS_WheelDiameter,TOP_AKM_BS_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		case 3: Robot_Init(TOP_AKM_BS_wheelspacing,TOP_AKM_BS_axlespacing,MD60N_47,GMR_500,TOP_AKM_BS_WheelDiameter,TOP_AKM_BS_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		
		//Top-spec independent suspension, standard/heavy-duty
		case 4: Robot_Init(TOP_AKM_DL_wheelspacing,TOP_AKM_DL_axlespacing,MD60N_18,GMR_500,TOP_AKM_DL_WheelDiameter,TOP_AKM_DL_MIN_TURN_RADIUS,400,100);break;
		case 5: Robot_Init(TOP_AKM_DL_wheelspacing,TOP_AKM_DL_axlespacing,MD60N_47,GMR_500,TOP_AKM_DL_WheelDiameter,TOP_AKM_DL_MIN_TURN_RADIUS,50,200);break;
		
		//Reserved nonstandard custom chassis type
		case 6: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		
		//Models 7 and 8 are installed and tested for use
		case 7: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		case 8: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		
		//High-speed Ackermann (high-spec chassis, motor reduction ratio 5.18)
		case 9: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_5_18,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,200,50);break;
		
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
	                    float tyre_diameter,float akm_min_turn,int vkp,int vki)
{
    robot.HardwareParam.WheelSpacing = wheelspacing;   //Track width
	robot.HardwareParam.AxleSpacing = axlespacing;     //Wheelbase
	robot.HardwareParam.GearRatio = gearratio;         //Motor reduction ratio
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //Motor encoder accuracy
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //Wheel circumference (PI*D)
	robot.HardwareParam.MIN_turn_radius = akm_min_turn;//Minimum turning radius
	//Encoder accuracy * motor reduction ratio * frequency = encoder reading after one rotation of the motor
	robot.HardwareParam.Encoder_precision = EncoderMultiples * robot.HardwareParam.EncoderAccuracy * robot.HardwareParam.GearRatio;
	robot.V_KP = vkp;                                  //Default kp parameter
	robot.V_KI = vki;                                  //Default ki parameter

}

