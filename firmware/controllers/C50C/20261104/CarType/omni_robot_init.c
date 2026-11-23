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
	robot.D50A_Number = 2;
	
	switch( TypeNum )
	{
		//New all-directional wheels

		//High-spec standard/heavy-duty variants
		case 0: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_27 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI , 10000 , 1500); break;
		case 1: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_51 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI , 10000 , 1500); break;
		
		//Top-spec standard variant
		case 2: Robot_Init(Omni_Turn_Radiaus_290 , MD60N_18 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI , 10000 , 1500); break;

		
		//Old version
//		//High-speed high-spec triangular omni chassis
//		case 0: Robot_Init(Omni_Turn_Radiaus_164 , MD36N_5_18 , GMR_500 , FullDirecion_75 ,  VEL_KP , VEL_KI , 10000 , 1500); break;
//		case 1: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_27 ,   GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI , 10000 , 1500); break;
//		
//		//High-spec triangular omni chassis, standard/heavy-duty
//		case 2: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_27 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI , 10000 , 1500); break;
//		case 3: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_51 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI , 10000 , 1500); break;
//		
//		//Top-spec omni chassis with three standard wheel diameters
//		case 4: Robot_Init(Omni_Turn_Radiaus_290 , MD60N_18 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI , 10000 , 1500); break;
//		case 5: Robot_Init(Omni_Turn_Radiaus_290 , MD60N_18 , GMR_500 , FullDirecion_152 , VEL_KP , VEL_KI , 10000 , 1500); break;
//		case 6: Robot_Init(Omni_Turn_Radiaus_290 , MD60N_18 , GMR_500 , FullDirecion_203 , VEL_KP , VEL_KI , 10000 , 1500); break;
		
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

//Inlet parameters: omnidirectional wheel rotation radius, motor reduction ratio, encoder accuracy, wheel diameter, kp, ki, D50A current mA, overcurrent time ms
static void Robot_Init(float omni_turn,float gearratio,int Accuracy,\
	                    float tyre_diameter,int vkp,int vki,uint16_t overcurrent_ma,uint16_t overtime_ms)
{
    robot.HardwareParam.WheelSpacing = 0;              //Track width
	robot.HardwareParam.AxleSpacing = 0;               //Wheelbase
	robot.HardwareParam.GearRatio = gearratio;         //Motor reduction ratio
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //Motor encoder accuracy
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //Wheel circumference (PI*D)
	robot.HardwareParam.TurnRadiaus = omni_turn;       //Radius of rotation of the omnidirectional wheel
	
	//Parameters related to kinematic analysis of omnidirectional wheels
	robot.HardwareParam.X_PARAMETER = sqrt(3)/2.0f;
	robot.HardwareParam.Y_PARAMETER = 0.5f;
	robot.HardwareParam.L_PARAMETER = 1.0f;
	
	//Encoder accuracy * motor reduction ratio * frequency = encoder reading after one rotation of the motor
	robot.HardwareParam.Encoder_precision = EncoderMultiples * robot.HardwareParam.EncoderAccuracy * robot.HardwareParam.GearRatio;
	robot.V_KP = vkp;                                  //Default kp parameter
	robot.V_KI = vki;                                  //Default ki parameter
	robot.D50A_OverCurrent_mA = overcurrent_ma;          //D50A sets the overcurrent threshold, unit: mA
	robot.D50A_OverTime_ms = overtime_ms;                //D50A startup overcurrent time, unit: ms

}

