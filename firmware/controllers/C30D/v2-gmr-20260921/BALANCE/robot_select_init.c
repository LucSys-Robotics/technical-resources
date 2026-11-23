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
  //ADC value segmental variable, depending on the number of car models; currently there are 8 car models, CAR_NUMBER=8
	Divisor_Mode=2048/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); //Collect the pin information of potentiometer // Collect potentiometer pin information
  if(Car_Mode>7)Car_Mode=7;

	//Car_Mode=0;
	
	switch(Car_Mode)
	{
		case Mec_Car:       Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          0,                     HALL_30F, Photoelectric_500, Mecanum_75);            break; //Mecanum chassis
		case Omni_Car:      Robot_Init(0,                        0,                        Omni_Turn_Radiaus_109, HALL_30F, Photoelectric_500, FullDirecion_60);       break; //Omnidirectional wheel chassis
		case Akm_Car:       Robot_Init(Akm_wheelspacing,         Akm_axlespacing,          0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //Ackermann chassis
		case Diff_Car:      Robot_Init(Diff_wheelSpacing,        0,                        0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //Two-wheel differential small cars
		case FourWheel_Car: Robot_Init(Four_Mortor_wheelSpacing, Four_Mortor__axlespacing, 0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //Four-wheel drive vehicles
		case Tank_Car:      Robot_Init(Tank_wheelSpacing,        0,                        0,                     HALL_30F, Photoelectric_500, Tank_WheelDiameter);    break; //Tracked vehicles
		case Mec_Car_V550:  Robot_Init(V550_MEC_wheelspacing,    V550_MEC_axlespacing,     0,                     HALL_30F, Photoelectric_500, Mecanum_75);            break; //V550 - Wheat wheel
		case FourWheel_Car_V550:  Robot_Init(V550_FourMortorWheelSpacing, V550_FourMortorAxlespacing,  0,         HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //V550 - four-wheel drive
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
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter) // 
{
	//wheelspacing, Mec_Car is half wheelspacing
	//Track width; half track width for Mecanum chassis
  Robot_Parament.WheelSpacing=wheelspacing; 
	//axlespacing, Mec_Car is half axlespacing
  //Wheelbase; half wheelbase for Mecanum chassis
  Robot_Parament.AxleSpacing=axlespacing;   
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
	//wheelspacing, Mec_Car is half wheelspacing
  //Track width; half track width for Mecanum chassis
  Wheel_spacing=Robot_Parament.WheelSpacing; 
  //axlespacing, Mec_Car is half axlespacing	
  //Wheelbase; half wheelbase for Mecanum chassis
	Axle_spacing=Robot_Parament.AxleSpacing; 
	//Rotation radius of omnidirectional trolley
  //Radius of rotation of the omnidirectional wheel cart
	Omni_turn_radiaus=Robot_Parament.OmniTurnRadiaus; 
}


