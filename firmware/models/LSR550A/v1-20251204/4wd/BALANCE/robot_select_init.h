#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//Robot parameter structure
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, 4wd_Car is half wheelspacing // Wheel spacing, for 4WD car it's half wheel spacing
  float AxleSpacing;       //Axlespacing, for 4wd car it is half of the wheelbase // Wheelbase, half wheelbase for 4wd car
  int GearRatio;           //Motor_gear_ratio // Motor reduction ratio
  int EncoderAccuracy;     //Number_of_encoder_lines // Encoder accuracy (number of encoder lines)
  float WheelDiameter;     //Diameter of driving wheel // Diameter of the driving wheel
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional chassis // Rotation radius of omnidirectional chassis
}Robot_Parament_InitTypeDef;


//half wheelspacing // half wheelspace
#define SENIOR_4WD_BS_wheelspacing   0.176 
//#define SENIOR_4WD_DL_wheelspacing   0.247 
#define SENIOR_4WD_DL_wheelspacing   0.230
#define TOP_4WD_BS_wheelspacing      0.311
#define TOP_4WD_DL_wheelspacing      0.295
#define FlagShip_4WD_BS_wheelspacing 0.286

#define FlagShip_4WD_DL_wheelspacing 0.24

//half axlespacing // Half wheelbase
//#define SENIOR_4WD_BS_axlespacing   0.156
#define SENIOR_4WD_BS_axlespacing   0.150
//#define SENIOR_4WD_DL_axlespacing   0.214 
#define SENIOR_4WD_DL_axlespacing   0.20 
#define TOP_4WD_BS_axlespacing      0.308
#define TOP_4WD_DL_axlespacing      0.201
#define FlagShip_4WD_BS_axlespacing 0.167

#define FlagShip_4WD_DL_axlespacing 0.14

//Motor_gear_ratio
//Motor reduction ratio
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

//4WD wheel tire diameter series
//Diameter of four-wheel drive tires
#define		_4WD_225  0.225f
#define		_4WD_250  0.250f
#define   _4WD_152  0.152f

 
//Omni wheel tire diameter series
//Omni-wheel diameter series
#define	  FullDirecion_75  0.075
#define	  FullDirecion_127 0.127
#define	  FullDirecion_152 0.152
#define	  FullDirecion_203 0.203
#define	  FullDirecion_217 0.217

//Rotation radius of omnidirectional trolley
//Radius of rotation of the omnidirectional wheel cart
#define   Omni_Turn_Radiaus_164 0.164
#define   Omni_Turn_Radiaus_180 0.160 //Revised to 160
#define   Omni_Turn_Radiaus_290 0.280 //Revised to 280

//The encoder octave depends on the encoder initialization Settings
//The number of encoder multipliers, depending on the encoder initialization settings
#define   EncoderMultiples 4
//Encoder data reading frequency
//Encoder data read frequency
#define CONTROL_FREQUENCY 100

//#define PI 3.1415f //PI //Pi

void Robot_Select(void);
#if _4WD
void Robot_Init(float wheelspacing, float axlespacing, int gearratio, int Accuracy, float tyre_diameter);
#elif Omni
void Robot_Init(float omni_turn_radiaus, int gearratio, int Accuracy, float tyre_diameter);
#endif

#endif
