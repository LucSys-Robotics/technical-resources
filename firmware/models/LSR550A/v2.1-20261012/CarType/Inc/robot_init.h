#ifndef __ROBOT_INIT_H
#define __ROBOT_INIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//Parameters related to robot structure
typedef struct  
{
	float WheelSpacing;           //Wheelspacing, Mec_Car is half wheelspacing // Wheel spacing parameter, for mecanum and 4WD it's half wheel spacing
	float AxleSpacing;            //Axlespacing, Mec_Car is half axlespacing // Wheelbase parameters Wheat wheels and four-wheel drive have half wheelbase
	float Wheel_Circ;             //Wheel circumference // Wheel circumference
	float GearRatio;              //Motor_gear_ratio // Motor reduction ratio parameters
	float Encoder_precision;      //The encoder value corresponding to one wheel rotation
	uint16_t   EncoderAccuracy;   //Number_of_encoder_lines // Encoder accuracy (number of encoder lines)
	
	//Exclusive parameters for different types of vehicles
	#if defined AKM_CAR
		float MIN_turn_radius;    //Minimum turning-radius limit for Ackermann chassis
	#endif
	
	#if defined OMNI_CAR
		float TurnRadiaus;        //Rotation radius of omnidirectional chassis // Rotation radius of omnidirectional chassis
		float X_PARAMETER;
		float Y_PARAMETER;
		float L_PARAMETER;
	#endif
	
}Robot_Parament_InitTypeDef;

//Robot servo motor related parameters
//Parameters related to robot servo motors
typedef struct  
{
	float Target;                //Control the target speed of the motor // Motor target speed value, control motor target speed
	float Encoder;               //Read the real-time speed of the motor by encoder //Encoder value, read motor real-time speed
	int Output;                  //Output PWM values to motors for controlling rotation
}Moto_parameter;

typedef struct
{
	Robot_Parament_InitTypeDef HardwareParam;//Structural parameters of the robot
	
	Moto_parameter MOTOR_A;
	Moto_parameter MOTOR_B;
	Moto_parameter MOTOR_C;
	Moto_parameter MOTOR_D;
	
	//Add servo definition for Ackermann chassis
	#if defined AKM_CAR
		Moto_parameter SERVO;
	#endif
	
	//Robot model 
	//Car model
	uint8_t    type;      
	
	//Robot battery voltage
	float voltage;
	
	//PID parameters used to control robot movement
	int V_KP;
	int V_KI;
	
	uint8_t LowPower;
}ROBOT_t;


//External interface
extern ROBOT_t robot;
void Robot_Select(void);

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
#define		GMR_500  500
#define	    Hall_13  13

//The encoder octave depends on the encoder initialization Settings
//The number of encoder multipliers, depending on the encoder initialization settings
#define   EncoderMultiples 4

//Default PID parameters
#define VEL_KP 300
#define VEL_KI 300

//Parameters under different categories of models
#if defined AKM_CAR
	//Number of Ackermann chassis types
	#define CAR_NUMBER    10    
	//Wheel_spacing // Wheel spacing
	#define   SENIOR_AKM_wheelspacing  0.322f //High-spec Ackermann chassis
	#define   TOP_AKM_BS_wheelspacing  0.510f //Top-spec oscillating-axle Ackermann chassis
	#define   TOP_AKM_DL_wheelspacing  0.595f //Top-spec independent-suspension Ackermann chassis
	
	//Axle_spacing // Wheelbase
	#define   SENIOR_AKM_axlespacing   0.322f //High-spec Ackermann chassis
	#define   TOP_AKM_BS_axlespacing   0.594f //Top-spec oscillating-axle Ackermann chassis
	#define   TOP_AKM_DL_axlespacing   0.525f //Top-spec independent-suspension Ackermann chassis
	
	//Diameter of trolley tire
	//Diameter of the car tire
	#define   SENIOR_AKM_WheelDiameter  0.125 //High-spec Ackermann chassis
	#define   TOP_AKM_BS_WheelDiameter  0.180 //Top-spec oscillating-axle Ackermann chassis
	#define   TOP_AKM_DL_WheelDiameter  0.254 //Top-spec independent-suspension Ackermann chassis

	//The minimum turning radius of different Ackermann models is determined by the mechanical structure:
	//the maximum Angle of the wheelbase, wheelbase and front wheel
	//The minimum turning radius of different Ackermann models is determined by the mechanical structure: track width, wheelbase, and maximum front wheel turning angle
	#define   SENIOR_AKM_MIN_TURN_RADIUS  0.750f //High-spec Ackermann chassis
	#define   TOP_AKM_BS_MIN_TURN_RADIUS  1.400f //Top-spec oscillating-axle Ackermann chassis
	#define   TOP_AKM_DL_MIN_TURN_RADIUS  1.200f //Top-spec independent-suspension Ackermann chassis
	
#elif defined DIFF_CAR
	//Number of differential car models
	#define   CAR_NUMBER    9   
	
	//Track width
	#define   TOP_DIFF_wheelspacing            0.329f //Differential cars
	#define   FOUR_WHEEL_DIFF_BS_wheelspacing  0.573f //Four-wheel differential swing suspension
	#define   FOUR_WHEEL_DIFF_DL_wheelspacing  0.605f //Four-wheel differential independent suspension
	
	//Differential cars do not have wheelbase data
	
	//Tire diameter
	#define   TOP_DIFF_WheelDiameter        0.125 //Diameter of the two-wheel differential driving wheel
	#define   FOUR_WHEEL_DIFF_WheelDiameter 0.254 //Four-wheel differential driving wheel diameter
	
#elif defined MEC_CAR
	//Number of Mecanum chassis types
	#define CAR_NUMBER    11     
	//Half track width
	#define SENIOR_MEC_BS_wheelspacing    0.176 
	#define SENIOR_MEC_DL_wheelspacing    0.247 
	#define TOP_MEC_BS_wheelspacing       0.311
	#define TOP_MEC_DL_wheelspacing       0.295
	#define FLAGSHIP_MEC_DL_wheelspacing  0.285
	#define FLAGSHIP_MEC_BS_wheelspacing  0.292
	#define TOP_MEC_DL_wheelspacing_Customized    0.446 //Custom size // Custom size
	#define Senior_MEC_DL_wheelspacing_Customized 0.3   //Custom size // Custom size
	//Half wheelbase
	#define SENIOR_MEC_BS_axlespacing    0.156
	#define SENIOR_MEC_DL_axlespacing    0.214 
	#define TOP_MEC_BS_axlespacing       0.308
	#define TOP_MEC_DL_axlespacing       0.201
	#define FLAGSHIP_MEC_DL_axlespacing  0.178
	#define FLAGSHIP_MEC_BS_axlespacing  0.178
	#define TOP_MEC_DL_axlespacing_Customized     0.401 //Custom size // Custom size
	#define Senior_MEC_DL_axlespacing_Customized  0.24  //Custom size // Custom size
	//Tire diameter
	#define		Mecanum_60  0.060f
	#define		Mecanum_75  0.075f
	#define		Mecanum_100 0.100f
	#define		Mecanum_127 0.127f
	#define		Mecanum_152 0.152f
	
#elif defined _4WD_CAR
	//Number of models of four-wheel-drive small cars
	#define CAR_NUMBER    13    
	//half wheelspacing // half wheelspace
	#define SENIOR_4WD_BS_wheelspacing   0.176 
	#define SENIOR_4WD_DL_wheelspacing   0.230
	#define TOP_4WD_BS_wheelspacing      0.311
	#define TOP_4WD_DL_wheelspacing      0.295
	#define FlagShip_4WD_BS_wheelspacing 0.286
	#define FlagShip_4WD_DL_wheelspacing 0.24

	//half axlespacing // Half wheelbase
	#define SENIOR_4WD_BS_axlespacing   0.150
	#define SENIOR_4WD_DL_axlespacing   0.20 
	#define TOP_4WD_BS_axlespacing      0.308
	#define TOP_4WD_DL_axlespacing      0.201
	#define FlagShip_4WD_BS_axlespacing 0.167
	#define FlagShip_4WD_DL_axlespacing 0.14

	//4WD wheel tire diameter series
	//Diameter of four-wheel drive tires
	#define  WheelDiameter_4WD_152  0.152f
	#define  WheelDiameter_4WD_225  0.225f
	#define	 WheelDiameter_4WD_250  0.250f
	
#elif defined OMNI_CAR
	//Number of models of omnidirectional wheel trolleys
	#define CAR_NUMBER    7     
	
	//Rotation radius of omnidirectional trolley
	//Radius of rotation of the omnidirectional wheel cart
	#define   Omni_Turn_Radiaus_164 0.164
	#define   Omni_Turn_Radiaus_180 0.180 
	#define   Omni_Turn_Radiaus_290 0.290 
	
	//Omni wheel tire diameter series
	//Diameter of the all-wheel tire
	#define	  FullDirecion_75  0.075
	#define	  FullDirecion_127 0.127
	#define	  FullDirecion_152 0.152
	#define	  FullDirecion_203 0.203
	#define	  FullDirecion_217 0.217

#endif




#endif
