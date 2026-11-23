#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//Parameters related to robot structure
typedef struct  
{
	float WheelSpacing;           //Wheelspacing, Mec_Car is half wheelspacing // Wheel spacing parameter, for mecanum car it's half wheel spacing
	float AxleSpacing;            //Axlespacing, Mec_Car is half axlespacing // Wheelbase parameter Wheat wheel carts have half a wheelbase
	float Wheel_Circ;             //Wheel circumference // Wheel circumference
	float GearRatio;              //Motor_gear_ratio // Motor reduction ratio parameters
	uint16_t   EncoderAccuracy;   //Number_of_encoder_lines // Encoder accuracy (number of encoder lines)
	uint8_t    type;              //Robot model // Model
	
	//Exclusive parameters for different types of vehicles
	#if defined AKM_CAR
		float MIN_turn_radius;    //Minimum turning-radius limit for Ackermann chassis
	#endif
	
	#if defined OMNI_CAR
		float TurnRadiaus;        //Rotation radius of omnidirectional chassis // Rotation radius of omnidirectional chassis
	#endif
	
}Robot_Parament_InitTypeDef;

//Robot servo motor related parameters
//Parameters related to robot servo motors
typedef struct  
{
	float Target;                //Control the target speed of the motor // Motor target speed value, control motor target speed
	float Encoder;               //Read the real-time speed of the motor by encoder //Encoder value, read motor real-time speed
}Moto_parameter;


typedef struct
{
	Robot_Parament_InitTypeDef HardwareParam;//Structural parameters of the robot
	
	Moto_parameter MOTOR_A;
	Moto_parameter MOTOR_B;

	#if defined AKM_CAR
		Moto_parameter MOTOR_SERVO;
	#elif defined OMNI_CAR
		Moto_parameter MOTOR_C;
	#elif defined MEC_CAR || defined _4WD_CAR
		Moto_parameter MOTOR_C;
		Moto_parameter MOTOR_D;
	#endif
	
	//PID parameters used to control robot movement
	int V_KP;
	int V_KI;
	
}ROBOT;


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
	#define   TOP_AKM_BS_wheelspacing  0.508f //Top-spec oscillating-axle Ackermann chassis
	#define   TOP_AKM_DL_wheelspacing  0.585f //Top-spec independent-suspension Ackermann chassis
	
	//Axle_spacing // Wheelbase
	#define   SENIOR_AKM_axlespacing   0.322f //High-spec Ackermann chassis
	#define   TOP_AKM_BS_axlespacing   0.590f //Top-spec oscillating-axle Ackermann chassis
	#define   TOP_AKM_DL_axlespacing   0.530f //Top-spec independent-suspension Ackermann chassis
	
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
	#define   FOUR_WHEEL_DIFF_DL_wheelspacing  0.573f //Four-wheel differential independent suspension
	
	//Differential cars do not have wheelbase data
	
	//Tire diameter
	#define   TOP_DIFF_WheelDiameter        0.125 //Diameter of the two-wheel differential driving wheel
	#define   FOUR_WHEEL_DIFF_WheelDiameter 0.215 //Four-wheel differential driving wheel diameter
	
#elif defined MEC_CAR

	#define CAR_NUMBER    11     
	
#elif defined _4WD_CAR

	#define CAR_NUMBER    12    
	
#elif defined OMNI_CAR

	#define CAR_NUMBER    7     
	
#endif




#endif
