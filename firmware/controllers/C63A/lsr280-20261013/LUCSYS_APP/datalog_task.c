/**
 * @file    datalog_task.c
 * @brief Temporary data observation task: Reporting vehicle operation data via serial port 4 via CSV text
 * @date    2026-08-03
 *
 * @details
 * Output four row types identified by the first field; host routes and saves rows by that field:
 *   H session header sent once at startup, including firmware compile time, model, mechanical parameters, column names, and scaling instructions
 *   D data line 10Hz period data
 *   E event line Send immediately upon status transition (50Hz detection), preventing 10Hz sampling misses
 *   S statistics row: Accumulated mileage/running time/peak every 10 seconds
 *
 * @note
 * - All fixed-point integers are reported, without using floating-point printf
 * - If DMA is not fully sent, the frame is discarded and counted (drop field), never blocking
 * - Task priority must be lower than control tasks
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

//Controls task snapshots, populated by RobotControl_task each cycle
volatile RobotLogSnap_t g_RobotLogSnap = { 0 };

#if DATALOG_ENABLE

//Send buffer, DMA direct reference; before rebuilding, confirm that the previous frame has been fully sent
static char logbuf[640];

//Cumulative statistics
static float    s_distAccum = 0.0f;   //Cumulative mileage (m)
static uint32_t s_holdCnt   = 0;      //Redirection deceleration hold gate trigger count
static uint32_t s_errCnt    = 0;      //Number of times the error code changes from 0 to non-zero
static float    s_curPeak   = 0.0f;   //Peak absolute value of four-wheel current (A)
static float    s_tmpPeak   = 0.0f;   //Four-Round Temperature Peak (degC)
static uint32_t s_dropCnt   = 0;      //Number of frames dropped due to DMA busyness
static uint8_t  s_pendingStat = 0;    //Stats pending flag: Suspended when conflicting with data rows, reissued in the next idle cycle

//Previous state used for event detection
static uint32_t s_lastErrNum   = 0;
static uint8_t  s_lastEnFlag   = 0xFF;
static uint8_t  s_lastHold     = 0xFF;
static uint8_t  s_lastParking  = 0xFF;
static uint8_t  s_lastOnlineBm = 0xFF;
static uint8_t  s_lastSource   = 0xFF;

//Floating-point to fixed point, rounded
static int32_t sc(float v,int32_t k)
{
	float t = v * (float)k;
	return (int32_t)( t>=0.0f ? (t+0.5f) : (t-0.5f) );
}

//Is serial port 4 free?
static uint8_t DataLog_TxIdle(void)
{
	return ( huart4.gState == HAL_UART_STATE_READY ) ? 1 : 0;
}

//Submit DMA send
static void DataLog_Commit(int n)
{
	if( n <= 0 ) return;
	if( n > (int)sizeof(logbuf) ) n = (int)sizeof(logbuf);
	HAL_UART_Transmit_DMA(&huart4,(uint8_t*)logbuf,(uint16_t)n);
}

//Acquire online/error bitmaps and minimum communication quality for four drivers
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

//Session header: Send once on startup to make the CSV file self-describe
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

//Data row: 10Hz
static void DataLog_SendData(uint32_t seq)
{
	RobotLogSnap_t snap;
	uint8_t  onbm=0, erbm=0;
	int32_t  cq=0;
	int n = 0;

	if( !DataLog_TxIdle() ){ s_dropCnt++; return; }

	//Copy the entire snapshot first to avoid overwriting by controlled tasks during field-by-field reading
	snap = *(RobotLogSnap_t*)&g_RobotLogSnap;

	DataLog_GetDriverState(&onbm,&erbm,&cq);

	//Time base and state
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

	//Instruction chain: Original instruction -> after driving smoothing -> after steering smoothing -> Kinematics correct actual value
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

	//Target steering position (counts)
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d",
		(int)snap.steerTgt[0],(int)snap.steerTgt[1],
		(int)snap.steerTgt[2],(int)snap.steerTgt[3]);

	//Turn to actual positions (counts)
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d",
		(int)RobotControlParam.MotorA.ori_pos,(int)RobotControlParam.MotorB.ori_pos,
		(int)RobotControlParam.MotorC.ori_pos,(int)RobotControlParam.MotorD.ori_pos);

	//Target hub rotational speed (0.1rpm)
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d",
		(int)sc(snap.rpmTgt[0],DATALOG_SCALE_RPM),(int)sc(snap.rpmTgt[1],DATALOG_SCALE_RPM),
		(int)sc(snap.rpmTgt[2],DATALOG_SCALE_RPM),(int)sc(snap.rpmTgt[3],DATALOG_SCALE_RPM));

	//Actual hub linear speed (mm/s)
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d",
		(int)sc(RobotControlParam.MotorA.feedbackSpeed,DATALOG_SCALE_VEL),
		(int)sc(RobotControlParam.MotorB.feedbackSpeed,DATALOG_SCALE_VEL),
		(int)sc(RobotControlParam.MotorC.feedbackSpeed,DATALOG_SCALE_VEL),
		(int)sc(RobotControlParam.MotorD.feedbackSpeed,DATALOG_SCALE_VEL));

	//Hub current (0.1A) and temperature (0.1degC); underlying TPDO1 period 200ms, actual update rate 5Hz
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d,%d,%d,%d,%d",
		(int)sc(RobotControlParam.MotorA.feedbackCurrent,DATALOG_SCALE_CUR),
		(int)sc(RobotControlParam.MotorB.feedbackCurrent,DATALOG_SCALE_CUR),
		(int)sc(RobotControlParam.MotorC.feedbackCurrent,DATALOG_SCALE_CUR),
		(int)sc(RobotControlParam.MotorD.feedbackCurrent,DATALOG_SCALE_CUR),
		(int)sc(RobotControlParam.MotorA.feedbackTemperature,DATALOG_SCALE_TMP),
		(int)sc(RobotControlParam.MotorB.feedbackTemperature,DATALOG_SCALE_TMP),
		(int)sc(RobotControlParam.MotorC.feedbackTemperature,DATALOG_SCALE_TMP),
		(int)sc(RobotControlParam.MotorD.feedbackTemperature,DATALOG_SCALE_TMP));

	//Vehicle: voltage, attitude, Z-axis angular velocity, drive status
	n += snprintf(logbuf+n,sizeof(logbuf)-n,",%d,%d,%d,%d,%d,%u,%u,%d,%lu,%lu",
		(int)sc(RobotControlParam.Vol,DATALOG_SCALE_VOL),
		(int)sc(AttitudeVal.pitch,DATALOG_SCALE_ANG),
		(int)sc(AttitudeVal.roll,DATALOG_SCALE_ANG),
		(int)sc(AttitudeVal.yaw,DATALOG_SCALE_ANG),
		(int)sc(axis_9Val.gyro.z,DATALOG_SCALE_OMG),
		(unsigned)onbm,(unsigned)erbm,(int)cq,
		(unsigned long)s_dropCnt,
		(unsigned long)snap.steerSendCnt);

	//AutoTest heading control internal variables vzmin/vzmax/flip are accumulated at 50Hz over the past 100ms window.
	//Not a sampling instantaneous value ¡ª the oscillation of the translation segment Vz can only be seen from the envelope and flip count
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

	//This window is read, reopen the next one
	AutoTest_DbgWindowReset();
}

//Event line: state changes sent immediately
//Return 1-transmission submitted; 0-serial port busy (caller must retain the old snapshot and retry next cycle, or the event is permanently lost)
static uint8_t DataLog_SendEvent(const char* type,int32_t oldv,int32_t newv)
{
	int n = 0;

	if( !DataLog_TxIdle() ){ s_dropCnt++; return 0; }

	n = snprintf(logbuf,sizeof(logbuf),"E,%lu,%s,%d,%d\r\n",
		(unsigned long)xTaskGetTickCount(),type,(int)oldv,(int)newv);

	DataLog_Commit(n);
	return 1;
}

//Statistics row: every 10 seconds
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

//50Hz event detection: Sent before data lines to ensure brief transitions are not missed by 10Hz sampling
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

//Cumulative volume updates, with points based on task cycles
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

	//Wait until the drive initialization and control tasks are reset to zero before starting data collection
	vTaskDelay(pdMS_TO_TICKS(3000));

	DataLog_SendHeader();
	vTaskDelay(pdMS_TO_TICKS(50));

	preTime = xTaskGetTickCount();

	while(1)
	{
		//Cumulative statistics updated every cycle
		DataLog_UpdateStat();

		//Event priority: state changes reported immediately
		DataLog_CheckEvent();

		//Data row 10Hz
		divData++;
		if( divData >= DATALOG_DATA_DIV )
		{
			divData = 0;
			seq++;
			DataLog_SendData(seq);
		}

		//Statistics rows every 10 seconds. STAT_DIV are integer multiples of DATA_DIV and must be triggered simultaneously with data rows,
		//Therefore, the data line will be suspended first and then reissued during an idle period; otherwise, the statistics line will be permanently discarded by DMA busily
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
