/**
 * @file    datalog_task.c
 * @brief   临时数据观察任务:通过串口4以CSV文本上报整车运行数据
 * @date    2026-08-03
 *
 * @details
 * 输出四种行,首字段区分,上位机按首字段分流保存:
 *   H  会话头  开机发一次,含固件编译时间、车型、机械参数、列名与缩放说明
 *   D  数据行  10Hz  周期数据
 *   E  事件行  状态跳变时立即发送(50Hz检测),不受10Hz采样错过
 *   S  统计行  每10秒,累计里程/运行时间/峰值
 *
 * @note
 * - 全部定点整数上报,不使用浮点printf
 * - DMA未发完则丢弃本帧并计数(drop字段),绝不阻塞
 * - 任务优先级须低于控制任务
 */
#include "datalog_task.h"

//HAL Include File
#include "main.h"
#include "usart.h"

//C Include File
#include <stdio.h>
#include <math.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_ServoDrive.h"
#include "bsp_icm20948.h"

//APP Include File
#include "robot_select_init.h"
#include "RobotControl_task.h"
#include "AutoTest_task.h"

//控制任务快照,由RobotControl_task每周期填充
volatile RobotLogSnap_t g_RobotLogSnap = { 0 };

#if DATALOG_ENABLE

//发送缓冲区,DMA直接引用,重建前必须确认上一帧已发完
static char logbuf[640];

//累计统计量
static float    s_distAccum = 0.0f;   //累计里程(m)
static uint32_t s_holdCnt   = 0;      //重定向泄压闸触发次数
static uint32_t s_errCnt    = 0;      //报错码由0变为非0的次数
static float    s_curPeak   = 0.0f;   //四轮电流绝对值峰值(A)
static float    s_tmpPeak   = 0.0f;   //四轮温度峰值(degC)
static uint32_t s_dropCnt   = 0;      //因DMA忙丢弃的帧数
static uint8_t  s_pendingStat = 0;    //统计行待发标志:与数据行冲突时挂起,下个空闲周期补发

//事件检测用的上一拍状态
static uint32_t s_lastErrNum   = 0;
static uint8_t  s_lastEnFlag   = 0xFF;
static uint8_t  s_lastHold     = 0xFF;
static uint8_t  s_lastParking  = 0xFF;
static uint8_t  s_lastOnlineBm = 0xFF;
static uint8_t  s_lastSource   = 0xFF;

//浮点转定点,四舍五入
static int32_t sc(float v,int32_t k)
{
	float t = v * (float)k;
	return (int32_t)( t>=0.0f ? (t+0.5f) : (t-0.5f) );
}

//串口4是否空闲
static uint8_t DataLog_TxIdle(void)
{
	return ( huart4.gState == HAL_UART_STATE_READY ) ? 1 : 0;
}

//提交DMA发送
static void DataLog_Commit(int n)
{
	if( n <= 0 ) return;
	if( n > (int)sizeof(logbuf) ) n = (int)sizeof(logbuf);
	HAL_UART_Transmit_DMA(&huart4,(uint8_t*)logbuf,(uint16_t)n);
}

//采集四路驱动器在线/报错位图与最低通信质量
static void DataLog_GetDriverState(uint8_t* onlineBm,uint8_t* errBm,int32_t* commQuaMin)
{
	uint8_t on = 0, er = 0;
	float   qmin = 100.0f;
	uint8_t i;

	for(i=0;i<RobotHardWareParam.driveCounts && i<4;i++)
	{
		if( g_ServoDriveList[i]->onlineFlag ) on |= (uint8_t)(1u<<i);
		if( g_ServoDriveList[i]->errFlag )    er |= (uint8_t)(1u<<i);
		if( g_ServoDriveList[i]->comm_qua < qmin ) qmin = g_ServoDriveList[i]->comm_qua;
	}

	*onlineBm   = on;
	*errBm      = er;
	*commQuaMin = (int32_t)qmin;
}

//会话头:开机发一次,使CSV文件自描述
static void DataLog_SendHeader(void)
{
	int n = 0;

	if( !DataLog_TxIdle() ) return;

	n += snprintf(logbuf+n,sizeof(logbuf)-n,
		"H,fw_build,%s %s,car,T280,wheelspacing_mm,%d,axlespacing_mm,%d,wheeldia_mm,%d\r\n",
		__DATE__,__TIME__,
		(int)sc(RobotHardWareParam.WheelSpacing,1000),
		(int)sc(RobotHardWareParam.AxleSpacing,1000),
		(int)sc(T280_Diameter,1000));

	n += snprintf(logbuf+n,sizeof(logbuf)-n,
		"H,cols,t_ms,seq,src,en,err,hold,park,scl,mprg,"
		"cvx,cvy,cvz,svx,svy,svz,tvx,tvy,tvz,fvx,fvy,fvz,"
		"stA,stB,stC,stD,saA,saB,saC,saD,"
		"rtA,rtB,rtC,rtD,spA,spB,spC,spD,"
		"cA,cB,cC,cD,tA,tB,tC,tD,"
		"vol,pit,rol,yaw,gyz,onbm,erbm,cq,drop,send,"
		"at_yaw0,at_yerr,at_vzr,at_vzmin,at_vzmax,at_flip,at_seg,at_st\r\n");

	n += snprintf(logbuf+n,sizeof(logbuf)-n,
		"H,scale,vel_mm_s,%d,omg_mrad_s,%d,rpm_0p1,%d,cur_0p1A,%d,tmp_0p1C,%d,vol_0p01V,%d,ang_0p01deg,%d,scl_permil,%d\r\n",
		DATALOG_SCALE_VEL,DATALOG_SCALE_OMG,DATALOG_SCALE_RPM,DATALOG_SCALE_CUR,
		DATALOG_SCALE_TMP,DATALOG_SCALE_VOL,DATALOG_SCALE_ANG,DATALOG_SCALE_SCL);

	DataLog_Commit(n);
}

//数据行:10Hz
static void DataLog_SendData(uint32_t seq)
{
	RobotLogSnap_t snap;
	uint8_t  onbm=0, erbm=0;
	int32_t  cq=0;
	int n = 0;

	if( !DataLog_TxIdle() ){ s_dropCnt++; return; }

	//先整体拷贝快照,避免逐字段读取期间被控制任务改写
	snap = *(RobotLogSnap_t*)&g_RobotLogSnap;

	DataLog_GetDriverState(&onbm,&erbm,&cq);

	//时基与状态
	n += snprintf(logbuf+n,sizeof(logbuf)-n,"D,%lu,%lu,%u,%u,%lu,%u,%u,%d,%d",
		(unsigned long)snap.tick,
		(unsigned long)seq,
		(unsigned)RobotControl_CMDsource,
		(unsigned)RobotControlParam.en_flag,
		(unsigned long)RobotControlParam.ErrNum,
		(unsigned)snap.reorientHold,
		(unsigned)RobotControlParam.ParkingMode,
		(int)sc(snap.steerScale,DATALOG_SCALE_SCL),
		(int)snap.maxProgress);

	//指令链:原始指令 -> 驱动平滑后 -> 转向平滑后 -> 运动学正解实际值
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		(int)sc(snap.cmdVx,DATALOG_SCALE_VEL),
		(int)sc(snap.cmdVy,DATALOG_SCALE_VEL),
		(int)sc(snap.cmdVz,DATALOG_SCALE_OMG),
		(int)sc(snap.smVx,DATALOG_SCALE_VEL),
		(int)sc(snap.smVy,DATALOG_SCALE_VEL),
		(int)sc(snap.smVz,DATALOG_SCALE_OMG),
		(int)sc(snap.stVx,DATALOG_SCALE_VEL),
		(int)sc(snap.stVy,DATALOG_SCALE_VEL),
		(int)sc(snap.stVz,DATALOG_SCALE_OMG),
		(int)sc(RobotControlParam.feedbackVx,DATALOG_SCALE_VEL),
		(int)sc(RobotControlParam.feedbackVy,DATALOG_SCALE_VEL),
		(int)sc(RobotControlParam.feedbackVz,DATALOG_SCALE_OMG));

	//转向目标位置(counts)
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d",
		(int)snap.steerTgt[0],(int)snap.steerTgt[1],
		(int)snap.steerTgt[2],(int)snap.steerTgt[3]);

	//转向实际位置(counts)
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d",
		(int)RobotControlParam.MotorA.ori_pos,(int)RobotControlParam.MotorB.ori_pos,
		(int)RobotControlParam.MotorC.ori_pos,(int)RobotControlParam.MotorD.ori_pos);

	//轮毂目标转速(0.1rpm)
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d",
		(int)sc(snap.rpmTgt[0],DATALOG_SCALE_RPM),(int)sc(snap.rpmTgt[1],DATALOG_SCALE_RPM),
		(int)sc(snap.rpmTgt[2],DATALOG_SCALE_RPM),(int)sc(snap.rpmTgt[3],DATALOG_SCALE_RPM));

	//轮毂实际线速度(mm/s)
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d",
		(int)sc(RobotControlParam.MotorA.feedbackSpeed,DATALOG_SCALE_VEL),
		(int)sc(RobotControlParam.MotorB.feedbackSpeed,DATALOG_SCALE_VEL),
		(int)sc(RobotControlParam.MotorC.feedbackSpeed,DATALOG_SCALE_VEL),
		(int)sc(RobotControlParam.MotorD.feedbackSpeed,DATALOG_SCALE_VEL));

	//轮毂电流(0.1A)与温度(0.1degC),底层TPDO1周期200ms,实际更新率5Hz
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d,%d,%d,%d,%d",
		(int)sc(RobotControlParam.MotorA.feedbackCurrent,DATALOG_SCALE_CUR),
		(int)sc(RobotControlParam.MotorB.feedbackCurrent,DATALOG_SCALE_CUR),
		(int)sc(RobotControlParam.MotorC.feedbackCurrent,DATALOG_SCALE_CUR),
		(int)sc(RobotControlParam.MotorD.feedbackCurrent,DATALOG_SCALE_CUR),
		(int)sc(RobotControlParam.MotorA.feedbackTemperature,DATALOG_SCALE_TMP),
		(int)sc(RobotControlParam.MotorB.feedbackTemperature,DATALOG_SCALE_TMP),
		(int)sc(RobotControlParam.MotorC.feedbackTemperature,DATALOG_SCALE_TMP),
		(int)sc(RobotControlParam.MotorD.feedbackTemperature,DATALOG_SCALE_TMP));

	//整车:电压 姿态 Z轴角速度 驱动器状态
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d,%d,%u,%u,%d,%lu,%lu",
		(int)sc(RobotControlParam.Vol,DATALOG_SCALE_VOL),
		(int)sc(AttitudeVal.pitch,DATALOG_SCALE_ANG),
		(int)sc(AttitudeVal.roll,DATALOG_SCALE_ANG),
		(int)sc(AttitudeVal.yaw,DATALOG_SCALE_ANG),
		(int)sc(axis_9Val.gyro.z,DATALOG_SCALE_OMG),
		(unsigned)onbm,(unsigned)erbm,(int)cq,
		(unsigned long)s_dropCnt,
		(unsigned long)snap.steerSendCnt);

	//AutoTest航向控制内部量.vzmin/vzmax/flip 是过去100ms窗口内按50Hz累积的,
	//不是采样瞬时值 —— 平移段Vz的振荡只能从包络和翻转次数上看出来
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d,%d,%u,%u,%u\r\n",
		(int)sc(g_AutoTestDbg.yaw0,DATALOG_SCALE_ANG),
		(int)sc(g_AutoTestDbg.yawErr,DATALOG_SCALE_ANG),
		(int)sc(g_AutoTestDbg.vzRaw,DATALOG_SCALE_OMG),
		(int)sc(g_AutoTestDbg.vzMin,DATALOG_SCALE_OMG),
		(int)sc(g_AutoTestDbg.vzMax,DATALOG_SCALE_OMG),
		(unsigned)g_AutoTestDbg.flipCnt,
		(unsigned)g_AutoTestDbg.seg,
		(unsigned)g_AutoTestDbg.state);

	DataLog_Commit(n);

	//本窗口已读走,重开下一个
	AutoTest_DbgWindowReset();
}

//事件行:状态跳变立即发送
//返回1=已提交发送,0=串口忙(调用方须保留旧快照,下个周期重试,否则事件会被永久丢失)
static uint8_t DataLog_SendEvent(const char* type,int32_t oldv,int32_t newv)
{
	int n = 0;

	if( !DataLog_TxIdle() ){ s_dropCnt++; return 0; }

	n = snprintf(logbuf,sizeof(logbuf),"E,%lu,%s,%d,%d\r\n",
		(unsigned long)xTaskGetTickCount(),type,(int)oldv,(int)newv);

	DataLog_Commit(n);
	return 1;
}

//统计行:每10秒
static void DataLog_SendStat(void)
{
	int n = 0;

	if( !DataLog_TxIdle() ){ s_dropCnt++; return; }

	n = snprintf(logbuf,sizeof(logbuf),
		"S,%lu,%lu,%d,%lu,%lu,%d,%d,%lu\r\n",
		(unsigned long)xTaskGetTickCount(),
		(unsigned long)(xTaskGetTickCount()/1000),
		(int)sc(s_distAccum,100),
		(unsigned long)s_holdCnt,
		(unsigned long)s_errCnt,
		(int)sc(s_curPeak,DATALOG_SCALE_CUR),
		(int)sc(s_tmpPeak,DATALOG_SCALE_TMP),
		(unsigned long)s_dropCnt);

	DataLog_Commit(n);
}

//50Hz事件检测:优先于数据行发送,保证短暂跳变不被10Hz采样错过
static void DataLog_CheckEvent(void)
{
	uint8_t onbm=0,erbm=0;
	int32_t cq=0;
	uint8_t hold = g_RobotLogSnap.reorientHold;

	DataLog_GetDriverState(&onbm,&erbm,&cq);

	if( RobotControlParam.ErrNum != s_lastErrNum )
	{
		if( DataLog_SendEvent("ERR",(int32_t)s_lastErrNum,(int32_t)RobotControlParam.ErrNum) )
		{
			if( s_lastErrNum==0 && RobotControlParam.ErrNum!=0 ) s_errCnt++;
			s_lastErrNum = RobotControlParam.ErrNum;
		}
		return;
	}

	if( RobotControlParam.en_flag != s_lastEnFlag )
	{
		if( DataLog_SendEvent("EN",s_lastEnFlag,RobotControlParam.en_flag) )
			s_lastEnFlag = RobotControlParam.en_flag;
		return;
	}

	if( hold != s_lastHold )
	{
		if( DataLog_SendEvent("HOLD",s_lastHold,hold) )
		{
			if( hold ) s_holdCnt++;
			s_lastHold = hold;
		}
		return;
	}

	if( onbm != s_lastOnlineBm )
	{
		if( DataLog_SendEvent("ONLINE",s_lastOnlineBm,onbm) )
			s_lastOnlineBm = onbm;
		return;
	}

	if( RobotControlParam.ParkingMode != s_lastParking )
	{
		if( DataLog_SendEvent("PARK",s_lastParking,RobotControlParam.ParkingMode) )
			s_lastParking = RobotControlParam.ParkingMode;
		return;
	}

	if( RobotControl_CMDsource != s_lastSource )
	{
		if( DataLog_SendEvent("SRC",s_lastSource,RobotControl_CMDsource) )
			s_lastSource = RobotControl_CMDsource;
		return;
	}
}

//累计量更新,按任务周期积分
static void DataLog_UpdateStat(void)
{
	const float dt = (float)DATALOG_TICK_MS/1000.0f;
	float vx = RobotControlParam.feedbackVx;
	float vy = RobotControlParam.feedbackVy;
	float spd = sqrtf(vx*vx + vy*vy);
	float cur[4],tmp[4];
	uint8_t i;

	s_distAccum += spd*dt;

	cur[0]=fabsf(RobotControlParam.MotorA.feedbackCurrent);
	cur[1]=fabsf(RobotControlParam.MotorB.feedbackCurrent);
	cur[2]=fabsf(RobotControlParam.MotorC.feedbackCurrent);
	cur[3]=fabsf(RobotControlParam.MotorD.feedbackCurrent);
	tmp[0]=RobotControlParam.MotorA.feedbackTemperature;
	tmp[1]=RobotControlParam.MotorB.feedbackTemperature;
	tmp[2]=RobotControlParam.MotorC.feedbackTemperature;
	tmp[3]=RobotControlParam.MotorD.feedbackTemperature;

	for(i=0;i<4;i++)
	{
		if( cur[i] > s_curPeak ) s_curPeak = cur[i];
		if( tmp[i] > s_tmpPeak ) s_tmpPeak = tmp[i];
	}
}

#endif //DATALOG_ENABLE


void DataLog_task(void* param)
{
#if DATALOG_ENABLE

	TickType_t preTime;
	uint32_t   divData = 0;
	uint32_t   divStat = 0;
	uint32_t   seq = 0;

	//等待驱动器初始化与控制任务回零完成后再开始采集
	vTaskDelay(pdMS_TO_TICKS(3000));

	DataLog_SendHeader();
	vTaskDelay(pdMS_TO_TICKS(50));

	preTime = xTaskGetTickCount();

	while(1)
	{
		//累计统计每周期更新
		DataLog_UpdateStat();

		//事件优先:状态跳变立即上报
		DataLog_CheckEvent();

		//数据行10Hz
		divData++;
		if( divData >= DATALOG_DATA_DIV )
		{
			divData = 0;
			seq++;
			DataLog_SendData(seq);
		}

		//统计行每10秒.STAT_DIV是DATA_DIV的整数倍,必然与数据行同拍触发,
		//故先挂起再择空闲周期补发,否则统计行会被DMA忙判定永久丢弃
		divStat++;
		if( divStat >= DATALOG_STAT_DIV )
		{
			divStat = 0;
			s_pendingStat = 1;
		}

		if( s_pendingStat && DataLog_TxIdle() )
		{
			s_pendingStat = 0;
			DataLog_SendStat();
		}

		vTaskDelayUntil(&preTime,pdMS_TO_TICKS(DATALOG_TICK_MS));
	}

#else
	(void)param;
	vTaskDelete(NULL);
	while(1) vTaskDelay(1000);
#endif
}
