#ifndef __ROBOT_SELECT_INIT_H
#define __ROBOT_SELECT_INIT_H

#include <stdint.h>

#define PI  3.14159265358979f

typedef struct{
	uint8_t CarType;     //Model
	uint8_t PotCarType;  //Vehicle gear reads by potentiometer (diagnostic use only)
	uint8_t driveCounts; //Number of drivers
	uint8_t wheeltype;   //Wheel types: 5-inch single-shaft, 5-inch double-shaft, 8-inch single-shaft, 8-inch double-shaft
	float WheelSpacing; //Track width
	float AxleSpacing;  //Wheelbase
	float WheelPerimeter;//Wheel circumference
}RobotParmentType_t;

//S300 track and diameter
#define S300_Wheelspacing 0.398
#define S300_Diameter 0.20

//S150 track and diameter
#define S150_Wheelspacing 0.26
#define S150_Diameter 0.13

//S100 track and track width
#define S100_Wheelspacing 0.2272f
#define S100_Diameter 0.13

//S200 track and diameter
#define S200_Wheelspacing 0.23 //Half track width
#define S200_axlespacing  0.15//Half wheelbase
#define S200_Diameter 0.205

#define S200_OUTDOOR_Wheelspacing 0.23 //Half track width
#define S200_OUTDOOR_axlespacing  0.15//Half wheelbase
#define S200_OUTDOOR_Diameter 0.205

//S260 track and diameter
#define S260_Wheelspacing 0.29  //Half track width

//The S260's wheelbase is not a true wheelbase in the strict sense; it needs to be coordinated and adjusted to an appropriate value based on IMU data
#define S260_axlespacing  0.08  //Half wheelbase
#define S260_Diameter 0.205

//Four-wheel steering chassis, wheelbase, and wheelbase
#define T280_Wheelspacing 0.335
#define T280_axlespacing  0.43
#define T280_Diameter     0.13

#define T280_WheelTurnSpeed 40

//Enumeration of models
enum CAR_MODE{
	S300,
	S200,
	S200_OUTDOOR, //S200 Outdoor Edition
	S300Mini,
	S100,
	S260,
	T280,
	Number_of_CAR
};

//Robot hardware parameters
extern RobotParmentType_t RobotHardWareParam;

//Wheel variable, including the target speed and actual speed of the wheel
typedef struct{
	float targetSpeed;
	float feedbackSpeed; //Velocity
	float feedbackCurrent;     // Wheel motor current, unit: A
	float feedbackTemperature; // Wheel motor temperature, unit: degC
	float targetRadian;
	float feedbackRadian;//radians
	
	float ori_pos;
	
}RobotMotorType_t;

//Robot control parameters
typedef struct{
	uint8_t en_flag;         //Vehicle global enable marker position: 1 enables control, 0 disabled prohibits control
	uint32_t ErrNum ;        //Error code for the chassis
	short defalutSpeed;     //Default remote control speed for small cars, unit: mm/s
	uint8_t Enkeystate;      //The state of the car's hardware enable switch: switch 1 pops up, 0 switch presses
	float Vol;              //Car battery voltage
	uint8_t DriveErrRecovery;//The cart requests to clear driver errors. Set 1 to request driver clearing errors; this bit will automatically be set to 0.
	uint8_t ChargeMode;      //Automatic recharge mode: 1 Enable auto recharge, 0 Turn auto recharge off
	uint8_t SecurityLevel;   //Safety Level 0: If the car does not continuously receive target velocity, it will automatically stop 1: The chassis will maintain the final target speed response
	uint8_t EmergencyMode;   //Emergency mode: When this position is set to 1, even if the motor cannot drive the error system, you can forcibly control the car to handle unexpected outdoor situations. If there is still a motor that can be driven, you can also pull the motor that cannot be driven to proceed
	uint8_t softwareEnflag;  //Software emergency-stop flag; set to 1 to put the chassis into the emergency-stop state
	uint8_t RangerAvoidEN;   //Ultrasonic obstacle avoidance enable position: 1 enables ultrasonic obstacle avoidance, 0 closes
	uint32_t LineDiffParam;  //Robot correction coefficient, adjustable from 0 to 100
	uint8_t ImuAssistedFlag; //imu flag to assist the car in moving straight. 0 means no assignment. When set to 1, the car will use imu data to help move straight while being controlled
	uint8_t DebugLevel;      //The chassis reports error debugging level. 0 Normal control; when set to 1, the chassis will report error messages via serial port 1 and Bluetooth serial port
	uint8_t LedTickState;    //Onboard LED status indicator on the chassis
	uint8_t LowPowerFlag;    //Low battery mark for the chassis
	
	uint8_t ParkingMode;     //Private variable, parking mode marker (user do not operate)
	
	//Robot wheel target value and feedback value
	RobotMotorType_t MotorA;  //The target speed and feedback speed of the wheels are stored here
	RobotMotorType_t MotorB;
	RobotMotorType_t MotorC;
	RobotMotorType_t MotorD;
	RobotMotorType_t MotorE;
	RobotMotorType_t MotorF;
	
	//The actual speed of the robot's three axes is stored here, obtained from the forward kinematics
	float feedbackVx;
	float feedbackVy;
	float feedbackVz;
	
}RobotControlParmentType_t;

void Robot_Select(void);
extern RobotControlParmentType_t RobotControlParam;

#endif

