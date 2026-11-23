#include "robot_select_init.h"

//Initialize the robot parameter structure
//Initialize the robot parameter structure
Robot_Parament_InitTypeDef  Robot_Parament; 
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
		//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
  //ADC value segmental variable, depending on the number of car models. Currently, there are 6 car models, CAR_NUMBER=6
	Divisor_Mode=330;
	Car_Mode=0; //Collect the pin information of potentiometer // Collect potentiometer pin information

	#if _4WD
	{	
		if (Car_Mode==0)   Robot_Init(SENIOR_4WD_BS_wheelspacing, SENIOR_4WD_BS_axlespacing, MD36N_27, Photoelectric_500, _4WD_152); //SENIOR_4WD_BS - High-end 4WD swing suspension standard // BS: Pendulum suspension
		if (Car_Mode==1)   Robot_Init(SENIOR_4WD_BS_wheelspacing, SENIOR_4WD_BS_axlespacing, MD36N_51, Photoelectric_500, _4WD_152); //SENIOR_4WD_BS - High-end 4WD swing suspension heavy-duty model
		
		if (Car_Mode==2)   Robot_Init(SENIOR_4WD_DL_wheelspacing, SENIOR_4WD_DL_axlespacing, MD36N_27, Photoelectric_500, _4WD_152); //SENIOR_4WD_DL - High-end 4WD independent suspension standard // DL: Independent suspension
		if (Car_Mode==3)   Robot_Init(SENIOR_4WD_DL_wheelspacing, SENIOR_4WD_DL_axlespacing, MD36N_51, Photoelectric_500, _4WD_152); //SENIOR_4WD_DL - High-end 4WD with independent suspension and heavy load
		
		if (Car_Mode==4)   Robot_Init(TOP_4WD_BS_wheelspacing,    TOP_4WD_BS_axlespacing,    MD60N_18, Photoelectric_500, _4WD_225); //TOP_4WD_BS - Top trim standard 4WD swing suspension
		if (Car_Mode==5)   Robot_Init(TOP_4WD_BS_wheelspacing,    TOP_4WD_BS_axlespacing,    MD60N_47, Photoelectric_500, _4WD_225); //TOP_4WD_BS - Top-tier 4WD swing suspension heavy-duty model
		
		if (Car_Mode==6)   Robot_Init(TOP_4WD_DL_wheelspacing,    TOP_4WD_DL_axlespacing,    MD60N_18, Photoelectric_500, _4WD_225); //TOP_4WD_DL - Top trim with 4WD independent suspension standard model
		if (Car_Mode==7)   Robot_Init(TOP_4WD_DL_wheelspacing,    TOP_4WD_DL_axlespacing,    MD60N_47, Photoelectric_500, _4WD_225); //TOP_4WD_DL - Top-tier 4WD with independent suspension and heavy load

		if (Car_Mode==8)   Robot_Init(FlagShip_4WD_BS_wheelspacing, FlagShip_4WD_BS_axlespacing, MD60N_18, Photoelectric_500, _4WD_225); //FlagShip_4WD_BS - Flagship 4WD swing suspension standard type
		if (Car_Mode==9)   Robot_Init(FlagShip_4WD_BS_wheelspacing, FlagShip_4WD_BS_axlespacing, MD60N_47, Photoelectric_500, _4WD_225); //FlagShip_4WD_BS - Flagship 4WD swing suspension heavy-duty type
		
		if (Car_Mode==10)  Robot_Init(FlagShip_4WD_DL_wheelspacing, FlagShip_4WD_DL_axlespacing, MD60N_18, Photoelectric_500, _4WD_225); //FlagShip_4WD_DL - Flagship 4WD independent suspension standard type
		if (Car_Mode==11)  Robot_Init(FlagShip_4WD_DL_wheelspacing, FlagShip_4WD_DL_axlespacing, MD60N_47, Photoelectric_500, _4WD_225); //FlagShip_4WD_DL - Flagship 4WD independent suspension heavy-duty type
		
		if (Car_Mode==12)  Robot_Init(TOP_4WD_BS_wheelspacing,    TOP_4WD_BS_axlespacing,    MD60N_47, Photoelectric_500, _4WD_225); //Customized_4WD - Custom models
	}
	#elif Omni
	{
		if (Car_Mode==0)  Robot_Init(Omni_Turn_Radiaus_164, MD36N_5_18, Photoelectric_500, FullDirecion_75); //SENIOR_OMNI_5_18 - High-end all-directional wheel triangular speed model
	  if (Car_Mode==1)  Robot_Init(Omni_Turn_Radiaus_180, MD36N_27,   Photoelectric_500, FullDirecion_127);//SENIOR_OMNI_27 - High-end all-directional wheel triangular standard
		if (Car_Mode==2)  Robot_Init(Omni_Turn_Radiaus_180, MD36N_27,   Photoelectric_500, FullDirecion_127);//SENIOR_OMNI_27 - High-end all-round wheels with regular rounds
		if (Car_Mode==3)  Robot_Init(Omni_Turn_Radiaus_180, MD36N_51,   Photoelectric_500, FullDirecion_127);//SENIOR_OMNI_51 - High-end all-round wheels with round heavy load
		
		if (Car_Mode==4)  Robot_Init(Omni_Turn_Radiaus_290, MD60N_18,   Photoelectric_500, FullDirecion_127);//TOP_OMNI_18 - Top configuration omnidirectional wheels with a regular diameter of 127
		if (Car_Mode==5)  Robot_Init(Omni_Turn_Radiaus_290, MD60N_18,   Photoelectric_500, FullDirecion_152);//TOP_OMNI_18 - Top configuration omnidirectional wheels with a regular diameter of 152
		if (Car_Mode==6)  Robot_Init(Omni_Turn_Radiaus_290, MD60N_18,   Photoelectric_500, FullDirecion_203);//TOP_OMNI_18 - Top-spec omnidirectional wheels with a diameter of 203
	}
	#endif
}


#if _4WD
/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
Function: Initialize car parameters
Entry parameters: track width, wheelbase, rotation radius, motor reduction ratio, motor encoder accuracy, tire diameter
Return value: none
**************************************************************************/
void Robot_Init(float wheelspacing,float axlespacing,int gearratio,int Accuracy,float tyre_diameter) 
{
  Robot_Parament.WheelSpacing=wheelspacing;   //half wheelspacing // half wheelspace
	Robot_Parament.AxleSpacing=axlespacing;     //half axlespacing // Half wheelbase
  Robot_Parament.GearRatio=gearratio;         //motor_gear_ratio // Motor reduction ratio
  Robot_Parament.EncoderAccuracy=Accuracy;    //Number_of_encoder_lines // Encoder accuracy (number of encoder lines)
  Robot_Parament.WheelDiameter=tyre_diameter; //Diameter of driving wheel // Diameter of driving wheel

	//Encoder value corresponding to 1 turn of motor (wheel)
	//The encoder value corresponding to one rotation of the motor (wheel).
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference // Driving wheel circumference
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
	//half wheelspacing // half wheelspace
  Wheel_spacing=Robot_Parament.WheelSpacing;
	//half axlespacing // Half wheelbase
	Wheel_axlespacing=Robot_Parament.AxleSpacing; 
}

#elif Omni
/**************************************************************************
Function: Initialize cart parameters
Input   : omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
Function: Initialize car parameters
Entry parameters: rotation radius, motor reduction ratio, motor encoder accuracy, tire diameter
Return value: none
**************************************************************************/
void Robot_Init(float omni_turn_radiaus,int gearratio,int Accuracy,float tyre_diameter)
{
	//Rotation radius of omnidirectional trolley
  //Radius of rotation of the omnidirectional wheel cart
  Robot_Parament.OmniTurnRadiaus=omni_turn_radiaus;
	//motor_gear_ratio
	//Motor reduction ratio
  Robot_Parament.GearRatio=gearratio;
	//Number_of_encoder_lines
  //Encoder Accuracy (Number of Encoder Lines)
  Robot_Parament.EncoderAccuracy=Accuracy;
	//Diameter of driving wheel
  //Active wheel diameter
  Robot_Parament.WheelDiameter=tyre_diameter;
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//The encoder value corresponding to one rotation of the motor (wheel).
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference
  //Active wheel circumference
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
	//Rotation radius of omnidirectional trolley
  //Radius of rotation of the omnidirectional wheel cart
  Omni_turn_radiaus=Robot_Parament.OmniTurnRadiaus;
}

#endif

