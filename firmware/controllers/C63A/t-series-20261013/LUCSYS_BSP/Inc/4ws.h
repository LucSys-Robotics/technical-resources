#ifndef __4WS_H
#define __4WS_H

#include <stdint.h>

//舵轮车型参数：轮距、轴距、轮子转向半径
typedef struct{
	float WheelSpacing; //轮距
	float AxleSpacing;  //轴距
	float WheelTurnR;   //轮子转向半径
	float WheelPerimeter;//轮子周长
}RobotHardWareParam_t;


typedef struct{
	float radian; //弧度
	float speed;  //速度
}WheelInfo_t;

typedef struct{
	//小车硬件参数
	RobotHardWareParam_t hardware;
	
	//小车3轴实际速度
	float feedback_Vx;
	float feedback_Vy;
	float feedback_Vz;
	
	//轮子目标数据
	WheelInfo_t TargetMotorA;
	WheelInfo_t TargetMotorB;
	WheelInfo_t TargetMotorC;
	WheelInfo_t TargetMotorD;
	
	//轮子实际数据
	WheelInfo_t RealMotorA;
	WheelInfo_t RealMotorB;
	WheelInfo_t RealMotorC;
	WheelInfo_t RealMotorD;

	//逆解分支记忆,下标 0=A 1=B 2=C 3=D,1 表示上次选了"目标+180度并反转轮速"那条路径.
	//该判据以实测轮角为输入,而轮子又在朝目标转,零迟滞时会自持振荡:
	//越过90度门限->目标翻180度->轮子掉头->又越回门限->再翻.记住上次的选择才能打断它.
	//随实例走而不用static,是为了让同一份代码里的多个小车模型(如泄压闸判据用的副本)互不干扰.
	uint8_t pathFlip[4];
}RobotParam_t;

/*
*    Function: FourWSCar_param_init
* Description: 设置小车参数
*       Input: *car - 小车结构体
*               m_W - 小车轮距
*               m_L - 小车轴距  
*               wp - 小车车轮周长
*      Output: None
*     Returns: None
*/
void FourWSCar_param_init(RobotParam_t *car,float m_W,float m_L,float WheelTurnR,float wp);

/*
*    Function: four_ws_kinematics
* Description: 舵轮小车运动学正解
*       Input: *car - 需要解算的小车结构体
*              
*      Output: 计算小车的xyz三轴速度,通过 car->feedback_Vx、car->feedback_Vy、car->feedback_Vz可访问
*     Returns: None
*/
void four_ws_kinematics(RobotParam_t* car);

/*
*    Function: four_ws_InverseKinematics
* Description: 舵轮小车运动学逆解
*       Input: Vx - 小车X轴目标速度
*              Vy - 小车Y轴目标速度
*              Vz - 小车Z轴目标速度
*              *car    - 需要解算的小车结构体
*      Output: 计算小车的4个轮子的轮速与角度,通过 car->MotoX->WheelSpeed、car->MotoX->angle即可访问
*     Returns: None
*/
void four_ws_InverseKinematics(float Vx,float Vy,float Vz,RobotParam_t *car);

/*
*    Function: four_ws_WheelSpeedProjection
* Description: 按car中已解算的目标转向角,把(Vx,Vy,Vz)对应的各轮速度矢量投影到目标角方向,
*              得到带符号轮速,保证轮速符号与已下发的转向角一致
*       Input: Vx/Vy/Vz - 小车三轴目标速度
*              *car     - 小车结构体(使用TargetMotorX.radian作为投影方向)
*      Output: 结果写入 car->TargetMotorX.speed,角度保持不变
*     Returns: None
*/
void four_ws_WheelSpeedProjection(float Vx,float Vy,float Vz,RobotParam_t *car);

#endif

