#ifndef __SERVODRIVERIO_H
#define __SERVODRIVERIO_H

#include <stdint.h>

//#define ZeroC (-7000)
#define ZeroC (0)

//typedef struct{
//	uint32_t id;
//	uint8_t buffer[8];
//}CANOpenType_t;


//Enumeration of motor types
enum{
	SingleAxis_5inch = 0,  //5-inch single-shaft motor, compatible with model S100
	DoubleAxis_5inch ,     //5-inch dual shaft motor, compatible with model S300Mini (S150)
	SingleAxis_8inch ,     //8-inch single-shaft motor, compatible with models S200, S200 outdoor version (S200_OUTDOOR), and S260
	DoubleAxis_8inch ,     //8-inch dual-shaft motor, compatible with model S300
	SteeringMotor ,        //Hollow motor for steering
};

//Motor operating mode
enum{
	SpeedMode = 0,
	PosMode = 1,
};

//Servo motor driver structure
typedef struct{
	uint8_t NodeID;       //Driver node ID
	uint8_t  onlineFlag;  //Online Flag Spots
	uint8_t  errFlag;     //Driver fault flag
	uint8_t  motorEnflag; //Motor enables or releases marker positions
	uint8_t  initMode;    //Initialization types (speed mode, position mode, torque mode)
	uint8_t  motorType;   //Motor type
	uint16_t SoftWareVer; //Driver software version
	uint16_t LeftMotorErr;//Revolver error code
	uint16_t RightMotorErr;//Right-wheel error code
	float comm_qua;      //Communication quality as a percentage
	uint32_t comm_cnt;   //Private variable, which counts the number of communications per unit time
	uint32_t comm_Errcnt;//Private variables that count the number of communication errors per unit time
	uint32_t time_tick;  //Private variable, statistical time
	uint32_t online_tick;//Private variables to count heartbeat times
}ServoDriverHWType_t;


//Servo drive part PID used to adjust the tightness of the wheel when it locks
typedef struct{
	uint16_t SpeedKp;
	uint16_t SpeedKi;
	uint16_t CurKp;
	uint16_t CurKi;
}ServoDriverPIDType_t;

int8_t CANopenRSDO_Write_RTOS(ServoDriverHWType_t* driver,uint16_t addr,uint8_t index,uint8_t byteNum,uint32_t writedata,uint8_t retrytime);
int8_t CANopenRSDO_Read_RTOS(ServoDriverHWType_t* driver,uint16_t addr,uint8_t index,uint8_t byteNum,uint32_t* readdata,uint8_t retrytime);
uint8_t ServoDriver_Init(ServoDriverHWType_t* driver);
int8_t set_MotorRPM(ServoDriverHWType_t* driver,short Lm,short Rm);

extern ServoDriverHWType_t Driver1,Driver2,Driver3;
void ServoDriver_OnLineCheck(uint8_t nodeid);
int8_t ServoDriver_CheckError(ServoDriverHWType_t* driver);
int8_t ServoDriver_Disable(ServoDriverHWType_t* driver);
int8_t ServoDriver_Enable(ServoDriverHWType_t* driver);
int8_t ServoDriver_ClearError(ServoDriverHWType_t* driver);
int8_t ServoDriver_ParkingMode(ServoDriverHWType_t* driver,uint8_t set);

int8_t ServoDrive_SetSpeedkp(uint16_t param);
int8_t ServoDrive_SetSpeedki(uint16_t param);
int8_t ServoDrive_SetCurkp(uint16_t param);
int8_t ServoDrive_SetCurki(uint16_t param);
int8_t ServoDriver_SaveEEPROM(ServoDriverHWType_t* driver);

int8_t set_MotorPosStop(ServoDriverHWType_t* driver);
int8_t set_MotorPos(ServoDriverHWType_t* driver,short Lm,uint32_t Lrpm,short Rm,uint32_t Rrpm);
int8_t set_MotorPos_Relative(ServoDriverHWType_t* driver,short Lm,uint32_t Lrpm,short Rm,uint32_t Rrpm);
int8_t set_MotorPos_Zero(ServoDriverHWType_t* driver);

extern ServoDriverHWType_t Driver1,Driver2,Driver3,Driver4;
extern ServoDriverPIDType_t DrivePid;
extern ServoDriverHWType_t* g_ServoDriveList[];
#endif

