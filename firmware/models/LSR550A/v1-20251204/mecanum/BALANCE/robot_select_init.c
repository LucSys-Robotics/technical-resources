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
	Divisor_Mode=4096/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(CAR_MODE_ADC,10))/Divisor_Mode); //Collect the pin information of potentiometer // Collect potentiometer pin information
  if(Car_Mode>10)Car_Mode=10;
	#if Mec
	{	
		if (Car_Mode==0)  Robot_Init(SENIOR_MEC_BS_wheelspacing, SENIOR_MEC_BS_axlespacing, MD36N_27, Photoelectric_500, Mecanum_100); //SENIOR_MEC_BS - High-end wheat wheel pendulum suspension // BS: Pendulum suspension
		if (Car_Mode==1)  Robot_Init(SENIOR_MEC_BS_wheelspacing, SENIOR_MEC_BS_axlespacing, MD36N_27, Photoelectric_500, Mecanum_100); //SENIOR_MEC_BS - High-end microphone swing suspension
		
		if (Car_Mode==2)  Robot_Init(SENIOR_MEC_DL_wheelspacing, SENIOR_MEC_DL_axlespacing, MD36N_27, Photoelectric_500, Mecanum_152); //SENIOR_MEC_DL - High-end microphone independent suspension // DL: Independent suspension
		if (Car_Mode==3)  Robot_Init(TOP_MEC_BS_wheelspacing,    TOP_MEC_BS_axlespacing,    MD60N_18, Photoelectric_500, Mecanum_152); //TOP_MEC_BS_18 - Top-tier standard model with tilting suspension on the mic wheels
		
		if (Car_Mode==4)  Robot_Init(TOP_MEC_BS_wheelspacing,    TOP_MEC_BS_axlespacing,    MD60N_47, Photoelectric_500, Mecanum_152); //TOP_MEC_BS_47 - Top-tier wheat wheel swing suspension heavy-duty model
		
		if (Car_Mode==5)  Robot_Init(TOP_MEC_DL_wheelspacing,    TOP_MEC_DL_axlespacing,    MD60N_18, Photoelectric_500, Mecanum_152); //TOP_MEC_DL_18 - Top-tier standard with independent suspension on wheat wheels
		if (Car_Mode==6)  Robot_Init(TOP_MEC_DL_wheelspacing,    TOP_MEC_DL_axlespacing,    MD60N_47, Photoelectric_500, Mecanum_152); //TOP_MEC_DL_18 - Top-tier wheat wheel independent suspension heavy-duty model
		
		if (Car_Mode==7)  Robot_Init(FLAGSHIP_MEC_BS_wheelspacing,    FLAGSHIP_MEC_BS_axlespacing,    MD60N_18, Photoelectric_500, Mecanum_152); //FlagShip_MEC_DL_18 - Flagship MecWheel swing suspension standard type
		if (Car_Mode==8)  Robot_Init(FLAGSHIP_MEC_BS_wheelspacing,    FLAGSHIP_MEC_BS_axlespacing,    MD60N_47, Photoelectric_500, Mecanum_152); //FlagShip_MEC_DL_47 - Flagship MecWheel swing suspension heavy-duty type
		
		if (Car_Mode==9)   Robot_Init(FLAGSHIP_MEC_DL_wheelspacing,FLAGSHIP_MEC_DL_axlespacing,MD60N_18, Photoelectric_500, Mecanum_152); //flagship_MEC_dl_18 - flagship Mecanum independent suspension regular type
		if (Car_Mode==10)  Robot_Init(FLAGSHIP_MEC_DL_wheelspacing,FLAGSHIP_MEC_DL_axlespacing,MD60N_47, Photoelectric_500, Mecanum_152); //flagship_MEC_dl_47 - flagship Mecanum independent suspension heavy-duty type
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


#if Mec
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
  Robot_Parament.WheelSpacing=wheelspacing;   //Wheelspacing // Wheel spacing
	Robot_Parament.AxleSpacing=axlespacing;     //Axlespacing // Wheelbase
  Robot_Parament.GearRatio=gearratio;         //motor_gear_ratio // Motor reduction ratio
  Robot_Parament.EncoderAccuracy=Accuracy;    //Number_of_encoder_lines // Encoder accuracy (number of encoder lines)
  Robot_Parament.WheelDiameter=tyre_diameter; //Diameter of driving wheel // Diameter of driving wheel
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//The encoder value corresponding to one rotation of the motor (wheel).
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference // Driving wheel circumference
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI; 
	//Wheelspacing // Wheel spacing
  Wheel_spacing=Robot_Parament.WheelSpacing;  
 //Wheel_axlespacing (Wheel_axlespacing is not required for motion analysis of differential trolleys) // Axle distance (Not needed for motion analysis of differential trolleys)
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

