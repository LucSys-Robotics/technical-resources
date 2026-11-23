#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//Robot parameter structure
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, Mec_Car is half wheelspacing // Wheel spacing, for mecanum car it's half wheel spacing
  float AxleSpacing;       //Axlespacing, Mec_Car is half axlespacing // Wheelbase Wheat wheel carts have half axles
  int GearRatio;           //Motor_gear_ratio // Motor reduction ratio
  int EncoderAccuracy;     //Number_of_encoder_lines // Encoder accuracy (number of encoder lines)
  float WheelDiameter;     //Diameter of driving wheel // Diameter of the driving wheel
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional chassis // Rotation radius of omnidirectional chassis
}Robot_Parament_InitTypeDef;

// Encoder structure
//Encoder structure
typedef struct  
{
  int A;      
  int B; 
	int C; 
	int D; 
}Encoder;

//The minimum turning radius of Ackermann models is determined by the mechanical structure: 
//the maximum Angle of the wheelbase, wheelbase and front wheels
//Minimum Ackermann turning radius is determined by geometry: track width, wheelbase and maximum front-wheel steering angle
#define MINI_AKM_MIN_TURN_RADIUS 0.350f 

//Wheelspacing, Mec_Car is half wheelspacing
//Track width; half-width for Mecanum wheels
//#define MEC_wheelspacing         0.109
#define MEC_wheelspacing         0.0930 //Revised on 2021.03.30
#define V550_MEC_wheelspacing    0.115
#define Akm_wheelspacing         0.162f
#define Diff_wheelSpacing        0.16f
#define Four_Mortor_wheelSpacing 0.187f
#define V550_FourMortorWheelSpacing 0.214f
#define Tank_wheelSpacing        0.235f

//Axlespacing, Mec_Car is half axlespacing
//Wheelbase; half-length for Mecanum wheels
#define MEC_axlespacing           0.085
#define V550_MEC_axlespacing      0.079
#define Akm_axlespacing           0.144f
#define Diff_axlespacing          0.155f
#define Four_Mortor__axlespacing  0.173f
#define V550_FourMortorAxlespacing  0.157f
#define Tank_axlespacing          0.222f

//Motor_gear_ratio
//Motor reduction ratio
#define   HALL_30F    30
#define   MD36N_5_18  5.18
#define   MD36N_27    27
#define   MD36N_51    51
#define   MD36N_71    71
#define   MD60N_18    18
#define   MD60N_47    47

//Number_of_encoder_lines
//Encoder accuracy
#define		Photoelectric_500 500
#define	  Hall_13           13

//Mecanum wheel tire diameter series
//Mecanum tire diameter
#define		Mecanum_60  0.060f
#define		Mecanum_75  0.075f
#define		Mecanum_100 0.100f
#define		Mecanum_127 0.127f
#define		Mecanum_152 0.152f
 
//Omni wheel tire diameter series
//Omni-wheel diameter series
#define	  FullDirecion_60  0.060
#define	  FullDirecion_75  0.075
#define	  FullDirecion_127 0.127
#define	  FullDirecion_152 0.152
#define	  FullDirecion_203 0.203
#define	  FullDirecion_217 0.217

//Black tire, tank_car wheel diameter
//Black-tire and tracked-wheel diameter
#define	  Black_WheelDiameter   0.065
//#define	  Tank_WheelDiameter 0.047
#define	  Tank_WheelDiameter 0.043

//Rotation radius of omnidirectional trolley
//Radius of rotation of the omnidirectional wheel cart
#define   Omni_Turn_Radiaus_109 0.109
#define   Omni_Turn_Radiaus_164 0.164
#define   Omni_Turn_Radiaus_180 0.180
#define   Omni_Turn_Radiaus_290 0.290

//The encoder octave depends on the encoder initialization Settings
//The number of encoder multipliers, depending on the encoder initialization settings
#define   EncoderMultiples  4
//Encoder data reading frequency
//Encoder data read frequency
#define   CONTROL_FREQUENCY 100

//#define PI 3.1415f //PI //Pi

void Robot_Select(void);
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter);

#endif
