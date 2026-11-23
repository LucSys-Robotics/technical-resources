#ifndef __4WS_H
#define __4WS_H

#include <stdint.h>

//Steering wheel model parameters: track width, wheelbase, wheel turning radius
typedef struct{
	float WheelSpacing; //Track width
	float AxleSpacing;  //Wheelbase
	float WheelTurnR;   //Wheel turning radius
	float WheelPerimeter;//Wheel circumference
}RobotHardWareParam_t;


typedef struct{
	float radian; //radians
	float speed;  //Velocity
}WheelInfo_t;

typedef struct{
	//Car Hardware Parameters
	RobotHardWareParam_t hardware;
	
	//Actual speed of the cart's 3-axle axis
	float feedback_Vx;
	float feedback_Vy;
	float feedback_Vz;
	
	//Target wheel data
	WheelInfo_t TargetMotorA;
	WheelInfo_t TargetMotorB;
	WheelInfo_t TargetMotorC;
	WheelInfo_t TargetMotorD;
	
	//Actual wheel data
	WheelInfo_t RealMotorA;
	WheelInfo_t RealMotorB;
	WheelInfo_t RealMotorC;
	WheelInfo_t RealMotorD;

	//Inverse-kinematics branch memory, indices 0=A 1=B 2=C 3=D; 1 means the previous choice was target+180 degrees with reversed wheel speed.
	//This criterion uses the measured wheel angle as input, and the wheel is rotating toward the target; at zero hysteresis, it will oscillate on its own:
	//Cross the 90-degree threshold - > the target and flip 180 degrees - > wheel turns around - > cross back over the threshold again - > flip again. Remember the last choice to interrupt it.
	//Keep state per instance rather than static so multiple chassis models (such as the deceleration hold gate reference copy) do not interfere.
	uint8_t pathFlip[4];
}RobotParam_t;

/*
*    Function: FourWSCar_param_init
* Description: Set cart parameters
*       Input: *car - car structure
*               m_W - Small track width
*               m_L - Wheelbase of the chassis
*               wp - Circumference of the cart wheels
*      Output: None
*     Returns: None
*/
void FourWSCar_param_init(RobotParam_t *car,float m_W,float m_L,float WheelTurnR,float wp);

/*
*    Function: four_ws_kinematics
* Description: forward kinematics for a steerable-wheel chassis
*       Input: *car - car structure that needs calculation
*              
*      Output: Calculates the three-axis speed of the cart, accessible via car->feedback_Vx, car->feedback_Vy, and car->feedback_Vz
*     Returns: None
*/
void four_ws_kinematics(RobotParam_t* car);

/*
*    Function: four_ws_InverseKinematics
* Description: inverse kinematics for a steerable-wheel chassis
*       Input: Vx - target speed along the car's X-axis
*              Vy - Target speed on the vehicle's Y-axis
*              Vz - Target speed on the vehicle's Z-axis
*              *car - The structure of the chassis that needs to be solved
*      Output: Calculates the wheel speed and angle of the four wheels of the cart, accessible via car->MotoX->WheelSpeed, car->MotoX->angle
*     Returns: None
*/
void four_ws_InverseKinematics(float Vx,float Vy,float Vz,RobotParam_t *car);

/*
*    Function: four_ws_WheelSpeedProjection
* Description: According to the target steering angle already calculated in car, project the speed vectors corresponding to (Vx, Vy, Vz) for each wheel onto the target angle direction,
*              Get signed wheel speed, ensuring it matches the steering angle already sent
*       Input: Vx/Vy/Vz - target speeds along the car's three axes
*              *car - Car structure (using TargetMotorX.radian as the projection direction)
*      Output: Write the result to car->TargetMotorX.speed, keeping the angle unchanged
*     Returns: None
*/
void four_ws_WheelSpeedProjection(float Vx,float Vy,float Vz,RobotParam_t *car);

#endif

