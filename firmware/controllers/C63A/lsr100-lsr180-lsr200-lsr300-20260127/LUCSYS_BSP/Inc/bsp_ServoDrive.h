#ifndef __SERVODRIVERIO_H
#define __SERVODRIVERIO_H

#include <stdint.h>

//typedef struct{
//	uint32_t id;
//	uint8_t buffer[8];
//}CANOpenType_t;

//Servo motor driver structure
typedef struct{
	uint8_t NodeID;       //Driver node ID
	float comm_qua;      //Communication quality as a percentage
	uint16_t SoftWareVer; //Driver software version
	uint8_t  onlineFlag;  //Online Flag Spots
	uint8_t  errFlag;     //Driver fault flag
	uint8_t  motorEnflag; //Motor enables or releases marker positions
	uint16_t LeftMotorErr;//Revolver error code
	uint16_t RightMotorErr;//Right-wheel error code
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

extern ServoDriverHWType_t Driver1,Driver2,Driver3;
extern ServoDriverPIDType_t DrivePid;
extern ServoDriverHWType_t* g_ServoDriveList[];
#endif

