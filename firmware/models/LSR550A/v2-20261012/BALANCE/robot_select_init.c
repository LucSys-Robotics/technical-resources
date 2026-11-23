#include "robot_select_init.h"

static void Robot_Init(uint8_t type,float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,float akm_min_turn,float omin_turnR,int vkp,int vki);

//Define a robot containing various parameters of the robot
static ROBOT robot;

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
	
	TypeNum = Get_ADC1_Average(CarMode_Ch,10)/TypeNum;//Determine different models based on board potentiometer selection
	
	#if defined AKM_CAR
	
	switch( TypeNum )
	{
		//High-spec standard/heavy-duty
		case 0: Robot_Init(TypeNum,SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,0,VEL_KP,VEL_KI);break;
		case 1: Robot_Init(TypeNum,SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_51,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,0,VEL_KP,VEL_KI);break;
		
		//Top-spec oscillating-axle chassis, standard/heavy-duty
		case 2: Robot_Init(TOP_AKM_BS_wheelspacing,TOP_AKM_BS_axlespacing,MD60N_18,GMR_500,TOP_AKM_BS_WheelDiameter,VEL_KP,VEL_KI);break;
		case 3: Robot_Init(TOP_AKM_BS_wheelspacing,TOP_AKM_BS_axlespacing,MD60N_47,GMR_500,TOP_AKM_BS_WheelDiameter,VEL_KP,VEL_KI);break;
		
		//Top-spec independent suspension, standard/heavy-duty
		case 4: Robot_Init(TOP_AKM_DL_wheelspacing,TOP_AKM_DL_axlespacing,MD60N_18,GMR_500,TOP_AKM_DL_WheelDiameter,400,100);break;
		case 5: Robot_Init(TOP_AKM_DL_wheelspacing,TOP_AKM_DL_axlespacing,MD60N_47,GMR_500,TOP_AKM_DL_WheelDiameter,50,200);break;
		
		//Reserved nonstandard custom chassis type
		case 6: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,VEL_KP,VEL_KI);break;
		
		//Models 7 and 8 are installed and tested for use
		case 7: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,VEL_KP,VEL_KI);break;
		case 8: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,VEL_KP,VEL_KI);break;
		
		//High-speed Ackermann (high-spec chassis, motor reduction ratio 5.18)
		case 9: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_5_18,GMR_500,SENIOR_AKM_WheelDiameter,200,50);break;
		
		default:break;
	}
	
	
	#elif defined DIFF_CAR
	
	#elif defined MEC_CAR
	
	#elif defined _4WD_CAR
	
	#elif defined OMNI_CAR
	
	#endif
    
}


/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
Function: Initialize car parameters
Entry parameters: track width, wheelbase, rotation radius, motor reduction ratio, motor encoder accuracy, tire diameter
Return value: none
**************************************************************************/

void robot_param_setKP(int setKP)
{
	robot.V_KP = setKP;
}

void robot_param_setKI(int setKI)
{
	robot.V_KI = setKI;
}


//Entry parameters: vehicle model, track width, wheelbase, motor reduction ratio, motor encoder accuracy, wheel diameter, Ackermann minimum turning radius, omnidirectional wheel rotation radius, default KP, default KI
static void Robot_Init(uint8_t type,float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,float akm_min_turn,float omin_turnR,int vkp,int vki)
{
	robot.HardwareParam.type = type;                   //Model
    robot.HardwareParam.WheelSpacing = wheelspacing;   //Track width
	robot.HardwareParam.AxleSpacing = axlespacing;     //Wheelbase
	robot.HardwareParam.GearRatio = gearratio;         //Motor reduction ratio
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //Motor encoder accuracy
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //Wheel circumference (PI*D)
	robot.HardwareParam.MIN_turn_radius = akm_min_turn;//Minimum turning radius
	
	
	robot_param_setKP(vkp);
	robot_param_setKI(vki);
}

//Track width
float get_wheelspacing(void)
{
	return robot.HardwareParam.WheelSpacing;
}

//Wheelbase
float get_AxleSpacing(void)
{
	return robot.HardwareParam.AxleSpacing;
}

//Reduction ratio
float get_GearRatio(void)
{
	return robot.HardwareParam.GearRatio;
}

//Encoder accuracy
uint16_t get_EncoderAccuracy(void)
{
	return robot.HardwareParam.EncoderAccuracy;
}

//Wheel circumference
float get_Wheel_Circ(void)
{
	return robot.HardwareParam.Wheel_Circ;
}

int get_robot_KP(void)
{
	return robot.V_KP;
}

int get_robot_KI(void)
{
	return robot.V_KI;
}
