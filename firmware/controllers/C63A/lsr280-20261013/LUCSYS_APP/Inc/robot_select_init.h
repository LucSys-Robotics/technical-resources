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

//LSR300 track and diameter
#define LSR300_Wheelspacing 0.398
#define LSR300_Diameter 0.20

//REF1 track and diameter
#define REF1_Wheelspacing 0.26
#define REF1_Diameter 0.13

//LSR100 track and track width
#define LSR100_Wheelspacing 0.2272f
#define LSR100_Diameter 0.13

//LSR200 track and diameter
#define LSR200_Wheelspacing 0.23 //Half track width
#define LSR200_axlespacing  0.15//Half wheelbase
#define LSR200_Diameter 0.205

#define LSR200_OUTDOOR_Wheelspacing 0.23 //Half track width
#define LSR200_OUTDOOR_axlespacing  0.15//Half wheelbase
#define LSR200_OUTDOOR_Diameter 0.205

//REF2 track and diameter
#define REF2_Wheelspacing 0.29  //Half track width

//The REF2's wheelbase is not a true wheelbase in the strict sense; it needs to be coordinated and adjusted to an appropriate value based on IMU data
#define REF2_axlespacing  0.08  //Half wheelbase
#define REF2_Diameter 0.205

//Four-wheel steering chassis, wheelbase, and wheelbase
#define LSR280_Wheelspacing 0.335
#define LSR280_axlespacing  0.43
#define LSR280_Diameter     0.13

#define LSR280_WheelTurnSpeed 40

//Enumeration of models
enum CAR_MODE{
	LSR300,
	LSR200,
	LSR200_OUTDOOR, //LSR200 Outdoor Edition
	LSR300Mini,
	LSR100,
	REF2,
	LSR280,
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

