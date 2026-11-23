#ifndef __AUTOTEST_TASK_H
#define __AUTOTEST_TASK_H

#include <stdint.h>

//场地自动测试属于研发设施,与数据观察共用同一个总开关.
//DATALOG_ENABLE=0 时本文件不导出任何符号,任务也不会被创建 ——
//出货固件里客户长按手柄上键不会误触发测试脚本.
#include "datalog_task.h"

#if DATALOG_ENABLE

/**
 * 航向控制内部量,用于诊断平移段 Vz 的振荡
 *
 * 日志任务按 10Hz 采样而本任务跑 50Hz,直接读瞬时值会混叠 —— 曾经就因此
 * 只看到 Vz 在 +0.30/-0.03 之间跳,查不出中间发生了什么。
 * 改为本任务每拍累积包络(vzMin/vzMax)与符号翻转次数,日志读走后调用
 * AutoTest_DbgWindowReset 清窗口,这样 100ms 内每一拍都被计入,与采样时机无关。
 */
typedef struct{
	float   yaw0;     //测试坐标系基准航向(度).启动后应恒定,若变化说明被重采了
	float   yawErr;   //本拍航向误差(度),PD的输入
	float   vzRaw;    //本拍限幅前的PD输出(rad/s),看它是否真的冲到限幅
	float   vzLast;   //本拍实际下发的Vz
	float   vzMin;    //窗口内下发Vz的下包络
	float   vzMax;    //窗口内下发Vz的上包络
	uint8_t flipCnt;  //窗口内Vz符号翻转次数,振荡的直接指标
	uint8_t seg;      //当前段索引
	uint8_t state;    //状态机状态 0=待机 1=执行 2=段间停顿 3=收尾
}AutoTestDbg_t;

extern volatile AutoTestDbg_t g_AutoTestDbg;

//日志任务读走窗口量之后调用,重开下一个统计窗口
void AutoTest_DbgWindowReset(void);

//场地自动测试任务:在2.4m x 2.4m场地内循环执行 直行/横移/倒退/斜移/旋转
void AutoTest_task(void* param);

//请求启动测试(手柄上键长按触发,可在任意任务/回调中调用)
void AutoTest_Start(void);

//请求停止测试(手柄上键单击/双击触发)
void AutoTest_Stop(void);

//查询测试是否在运行
uint8_t AutoTest_IsRunning(void);

#endif //DATALOG_ENABLE

#endif
