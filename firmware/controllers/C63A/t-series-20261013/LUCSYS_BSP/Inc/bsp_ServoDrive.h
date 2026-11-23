#ifndef __SERVODRIVERIO_H
#define __SERVODRIVERIO_H

#include <stdint.h>

//#define ZeroC (-7000)
#define ZeroC (0)

//typedef struct{
//	uint32_t id;
//	uint8_t buffer[8];
//}CANOpenType_t;


//电机类型枚举
enum{
	SingleAxis_5inch = 0,  //5寸单出轴电机,适配型号S100
	DoubleAxis_5inch ,     //5寸双出轴电机,适配型号S300Mini(S150)
	SingleAxis_8inch ,     //8寸单出轴电机,适配型号S200、S200户外版本(S200_OUTDOOR)、S260
	DoubleAxis_8inch ,     //8寸双出轴电机,适配型号S300
	SteeringMotor ,        //中空电机,用于转向
};

//电机运行模式
enum{
	SpeedMode = 0,
	PosMode = 1,
};

//伺服电机驱动器结构体
typedef struct{
	uint8_t NodeID;       //驱动器的节点ID
	uint8_t  onlineFlag;  //在线标志位
	uint8_t  errFlag;     //驱动报错标志位
	uint8_t  motorEnflag; //电机使能或释放标志位
	uint8_t  initMode;    //初始化类型(速度模式、位置模式、转矩模式)
	uint8_t  motorType;   //电机类型
	uint16_t SoftWareVer; //驱动器软件版本
	uint16_t LeftMotorErr;//左轮报错码
	uint16_t RightMotorErr;//右轮报错码
	float comm_qua;      //通信质量,百分比表示
	uint32_t comm_cnt;   //私有变量,统计单位时间内的通信次数
	uint32_t comm_Errcnt;//私有变量,统计单位时间内的通信错误次数
	uint32_t time_tick;  //私有变量,统计时间
	uint32_t online_tick;//私有变量,统计心跳时间
}ServoDriverHWType_t;


//伺服驱动器部分PID,用于调节轮子上锁时松紧度
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

