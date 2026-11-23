#ifndef __DATALOG_TASK_H
#define __DATALOG_TASK_H

#include <stdint.h>

/**
 * @file    datalog_task.h
 * @brief   临时数据观察任务:通过串口4(huart4)以CSV文本上报整车运行数据
 *
 * @note
 * - 本功能仅用于研发阶段的长时间数据采集,出货固件请将 DATALOG_ENABLE 置0
 * - 启用本功能时 data_task 的串口4发送已被注释,两者不能同时占用huart4
 * - 所有数值均为定点整数上报(避免浮点printf),缩放系数见下方宏定义与H行说明
 */

//调试总开关
#define DATALOG_ENABLE          0

//上报周期:数据行10Hz,事件检测50Hz,统计行每10秒
#define DATALOG_TICK_MS         20      //任务周期(ms),事件检测频率
#define DATALOG_DATA_DIV        5       //数据行分频:20ms x 5 = 100ms (10Hz)
#define DATALOG_STAT_DIV        500     //统计行分频:20ms x 500 = 10s

//定点缩放系数(上位机按此还原)
#define DATALOG_SCALE_VEL       1000    //线速度 m/s   -> mm/s
#define DATALOG_SCALE_OMG       1000    //角速度 rad/s -> mrad/s
#define DATALOG_SCALE_RPM       10      //转速   rpm   -> 0.1rpm
#define DATALOG_SCALE_CUR       10      //电流   A     -> 0.1A
#define DATALOG_SCALE_TMP       10      //温度   degC  -> 0.1degC
#define DATALOG_SCALE_VOL       100     //电压   V     -> 0.01V
#define DATALOG_SCALE_ANG       100     //角度   deg   -> 0.01deg
#define DATALOG_SCALE_SCL       1000    //限速比例     -> 千分比

/**
 * @brief 控制任务快照
 *
 * RobotControl_task 内部的局部/静态变量对日志任务不可见,由控制任务在每个
 * 周期末尾一次性拷贝到此结构体,避免日志任务读到半新半旧的撕裂数据.
 * 其余数据(实际转向位置/轮速/电流/温度/IMU/驱动器状态)已在全局结构体中,
 * 由日志任务直接读取.
 */
typedef struct{
	uint32_t tick;          //本控制周期时间戳(ms)
	uint8_t  reorientHold;  //重定向泄压闸状态
	float    steerScale;    //本周期实际施加的转向限速比例(0=下发停止)
	int32_t  maxProgress;   //四轮中最大转向偏差(counts)
	float    cmdVx,cmdVy,cmdVz;  //队列收到的原始指令
	float    smVx,smVy,smVz;     //平滑限速后的实际执行值
	float    stVx,stVy,stVz;     //转向解算实际使用的限幅平滑指令(SteerCmd)
	uint32_t steerSendCnt;       //set_MotorPos累计调用次数,相邻采样之差即CAN下发速率
	float    steerTgt[4];   //转向目标位置(counts),顺序A/B/C/D
	float    rpmTgt[4];     //轮毂目标转速(rpm),顺序A/B/C/D
}RobotLogSnap_t;

extern volatile RobotLogSnap_t g_RobotLogSnap;

void DataLog_task(void* param);

#endif
