/**
 * @file    RobotControl_task.c
 * @brief   小车控制逻辑文件.
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - 本文件是小车运行的核心内容,由任务RobotControl_task,从控制队列中接收各种遥控或串口的
 *   速度指令信息,使用速度信息进行运动学分析解算后,获得小车各个轮子的转速,进而下发转速
 *   指令到驱动器,完成小车的驱动
 * @note
 *
 */
#include "RobotControl_task.h"

//HAL Lib Include File
#include "main.h"

//C Include File
#include <stdio.h>
#include <math.h>

//BSP Include File
#include "bsp_adc.h"
#include "bsp_ServoDrive.h"
#include "bsp_icm20948.h"
#include "bsp_RGBLight.h"
#include "bsp_buzzer.h"
#include "4ws.h"

//APP Include File
#include "robot_select_init.h"
#include "sensor_ranger.h"
#include "xbox360_gamepad.h"
#include "show_task.h"
#include "AutoTest_task.h"
#include "datalog_task.h"

//用于存放小车的控制队列数据
QueueHandle_t  g_xQueueRobotControl = NULL;

//用于记录小车当前的控制来源
volatile uint8_t RobotControl_CMDsource = NONE_CMD;

//内部使用函数
static float linearVel_to_rpm(float linearVel,float wheelper);
static void Robot_DriverRecovery(void);
static void robot_en_check(void);
static void Robot_DriverDisable(void);
static void Robot_DriverEnable(void);
static void Robot_DriverSetParkingMode(uint8_t setflag);
static void Robot_DriverCheckError(void);
static void Robot_UpdateVoltageEstimate(const RobotControlCMDType_t* cmd);
static void Linear_IMUHelp(RobotControlCMDType_t* cmdtarget);
static uint8_t _4ws_HasMoveCmd(const RobotControlCMDType_t* cmd);
static uint8_t _4ws_TurnSpeedReady(void);
static void _4ws_SpeedSmoothFastBrake(RobotControlCMDType_t* smoothcmd,const RobotControlCMDType_t* cmdtarget,uint16_t RunFreq);
static void _4ws_SpeedSmooth(RobotControlCMDType_t* smoothcmd,const RobotControlCMDType_t* cmdtarget,uint16_t RunFreq);
static void _4ws_SpeedSmoothLimit(RobotControlCMDType_t* smoothcmd,const RobotControlCMDType_t* cmdtarget,uint16_t RunFreq,
float accel_start_x,float accel_brake_x,float accel_start_y,float accel_brake_y,float accel_start_w,float accel_brake_w);
static float _4ws_SteerScaleByProgress(int max_progress);
static void _4ws_LimitWheelRPM(float* rpm,float max_rpm);
static uint8_t SteeringHoming(uint32_t timeout_ms);

//转向回零失败持久标志:置位后小车失能,可通过错误恢复(DriveErrRecovery)重试回零
static uint8_t SteeringHomeFailed = 0;

//4轮转向保护参数
#define ROBOT_CONTROL_FREQ_HZ      100u
#define TURN_WAIT_ANGLE_DEG        40.0f
#define TURN_DRIVE_ALLOW_DEG       60.0f
#define TURN_SAFE_VX               0.2f
#define TURN_SAFE_VY               0.2f
#define TURN_SAFE_VZ               0.27f
#define TURN_MOVE_CMD_EPS          0.0001f
#define TURN_FULL_SPEED_DEG        20.0f
#define CMD_CHANGE_EPS_V           0.005f
#define CMD_CHANGE_EPS_W           0.01f
#define T280_MAX_DRIVE_RPM         1000.0f
#define ENCODER_COUNTS_PER_REV     16384.0f  //转向电机编码器每圈counts
#define COUNTS_PER_DEG             (ENCODER_COUNTS_PER_REV/360.0f)
#define HOMING_TIMEOUT_MS          5000      //开机回零超时时间(ms)
#define HOMING_RESEND_MS           500       //回零指令重发周期(ms)
#define HOMING_TOL_COUNTS          15.0f     //回零到位容差(counts)
#define STARTUP_STOP_EPS           0.01f     //静止判定:s_drive.smooth三轴均小于此值视为完全静止
#define STARTUP_ALIGN_TOL_DEG      5.0f      //静止起步对齐容差(度),轮子误差小于此值才放行给速
//大重定向门限:45°对应counts(45/360*16384),超过视为切自转/横移
#define REORIENT_JUMP_COUNTS       (45.0f*COUNTS_PER_DEG)
//转向指令平滑:上游(ROS/CAN/手柄)的速度抖动会被逆解放大成转向角突跳,
//低速时最剧烈(Vx=0.06/Vz=0.3 对应转弯半径0.2m,轮子接近±90度),故先限幅再送逆解.
//
//ACC_W 是本平滑唯一的强度旋钮,取值受两头夹逼:
//  往下调 -> 手柄响应变迟钝.20 时满打Vz=1.294rad/s需65ms到位,降到5要259ms,实车能明显感觉到
//  往上调 -> 对高频噪声的滤除能力下降
//实测该平滑对转向抖动的抑制有限(衰减仅1%量级),压住抖动靠的是下面的时间节流与
//泄压闸意图判据.本平滑的定位是对上游异常噪声(如客户导航栈输出)的兜底防线.
#define STEER_ACC_XY               2.0f      //转向指令XY线加速度限值(m/s2),驱动起步为1.0
#define STEER_ACC_W                20.0f     //转向指令Z角加速度限值(rad/s2),驱动起步为5.0
//下发下限门:仅用于避免零变化的无效下发,不承担限流职责(限流由时间节流负责).
//门槛过大会造成永久量化误差 —— 取2度时稳态误差落在2~5度的比例达20%.
#define STEER_SEND_EPS_COUNTS      (0.5f*COUNTS_PER_DEG)
//最小重发间隔(ms).转向驱动器自带位置环,收到目标会自己走到位,每10ms刷新纯属浪费.
//这是CAN负载的硬上界:位置门槛拦不住峰值(可达206次/秒),只有时间节流能给出确定值.
#define STEER_SEND_MIN_INTERVAL_MS 25

//泄压闸门限.判据用 intent_jump(未平滑指令算出的意图目标 vs 当前实际轮角),
//与转向平滑解耦 —— 若改用平滑后目标的单周期跳变量,渐变的目标够不到门限,
//保护会静默失效(实测触发率从1.44次/分掉到0.23次/分).
#define REORIENT_HOLD_TIMEOUT_MS   500       //重定向减速等待最长时间(ms),到点强制放行,防止锁死

/**
 * 车轮编号 —— 四轮同构量的统一下标
 *
 * 车头朝前时的物理位置:
 *     B(左前)   C(右前)
 *     A(左后)   D(右后)
 * 即 A/B 为左侧两轮,C/D 为右侧两轮 —— 一个转向驱动器管一整侧,不是管一根轴.
 *
 * 与逆解输出的 TargetMotorA~D 一一对应.本文件所有四轮数组
 * (target/speed/progress/lastSent/rpm 等)都用这套下标,不写裸数字.
 *
 * 硬件绑定:A/B 由 Driver1(NodeID=1)驱动,C/D 由 Driver2(NodeID=2)驱动,
 * 每个驱动器的通道1接前者,通道2接后者.右侧(C/D)转向电机镜像安装,
 * 零点比左侧差 180 度,故解算时 C/D 要先补 180 再转成电机位置.
 */
typedef enum{
	WHEEL_A = 0,
	WHEEL_B,
	WHEEL_C,
	WHEEL_D,
	WHEEL_COUNT
}WheelIndex_t;

/**
 * 转向控制器状态
 *
 * 四轮同构量一律用数组,下标见 WheelIndex_t.
 * 实例为静态存储且全局唯一,需要整体复位时对本结构体清零即可.
 */
typedef struct{
	float      target[WHEEL_COUNT];        //已解算的目标位置(counts)
	uint16_t   speed[WHEEL_COUNT];         //转向速度,按等时到达分配
	float      lastSent[WHEEL_COUNT];      //上次实际下发的目标,用于下发节流判据
	int32_t    progress[WHEEL_COUNT];      //本周期各轮距目标的剩余偏差(counts)
	int32_t    lastProgress[WHEEL_COUNT];  //看门狗比较快照,每100ms刷新一次
	int32_t    maxProgress;                //四轮中的最大偏差
	float      scale;                      //本周期施加到指令上的限速比例(0=下发停止)
	uint8_t    reorientHold;               //重定向泄压闸
	uint8_t    lowfreCnt;                  //看门狗分频计数
	uint8_t    lastEnFlag;                 //使能边沿检测
	TickType_t reorientHoldStart;          //泄压闸起始时刻,用于减速超时判断
	TickType_t lastSendTick;               //上次实际下发时刻,用于最小间隔节流
}SteerCtrl_t;

/** 驱动控制器状态 */
typedef struct{
	RobotControlCMDType_t smooth;            //平滑限速后的实际执行速度
	float                 rpm[WHEEL_COUNT];  //四轮目标转速
}DriveCtrl_t;

//转向下发出口(需在 SteerCtrl_t 定义之后声明)
static void Steer_SendDriver(SteerCtrl_t* s,ServoDriverHWType_t* drv,WheelIndex_t i0,WheelIndex_t i1);
static void Steer_Send(SteerCtrl_t* s);

#if DATALOG_ENABLE
//set_MotorPos 累计调用次数.相邻两次采样之差即转向下发速率,用于量化CAN负载.
//下发已收敛到 Steer_SendDriver 单一出口,故此处一处自增即覆盖全部下发路径.
static uint32_t s_steerSendCnt = 0;
#endif

/**
 * 控制器持久状态
 *
 * 主循环各职责函数共享这几项.仅本文件可见,不对外暴露.
 */
static SteerCtrl_t           s_steer;     //转向控制器状态
static DriveCtrl_t           s_drive;     //驱动控制器状态
static RobotParam_t          s_car;       //四轮转向小车模型(逆解/正解/投影共用)
static uint8_t s_steerSendFault = 0; // One fault bit per steering driver.
static RobotParam_t          s_sentCar; // Wheel-speed projection uses successfully sent steering targets.
static RobotParam_t          s_rawCar;    //泄压闸判据专用实例,用未平滑指令算意图目标
static RobotControlCMDType_t s_steerCmd;  //转向解算专用指令(对原始指令限幅平滑后)
static RobotControlCMDType_t s_lastCmd;   //上次触发转向重算时的指令,用于变化检测
static const RobotControlCMDType_t s_stopCmd = {0,0,0,0};

//IMU辅助小车走直线变量
static RobotControlCMDType_t LinearHelp_Target = { 0 };

//主循环职责函数.RobotControl_task 按此顺序逐个调用,实现在本文件后半部分.
//顺序不可随意调整:转向解算的结果被驱动平滑读取,驱动平滑又负责放行泄压闸,
//而转向看门狗以泄压闸未生效为前提.
static void Robot_CmdReceive(RobotControlCMDType_t* cmd,uint8_t* lostCnt);
static void Steer_Solve(const RobotControlCMDType_t* cmd,uint16_t taskFreq);
static void Drive_Update(const RobotControlCMDType_t* cmd,uint16_t taskFreq);
static void Steer_Watchdog(void);
static void Drive_Output(void);
static void Robot_Odometry(void);
static void Robot_ActiveUpdate(const RobotControlCMDType_t* cmd,TickType_t* activeTick);
static void Robot_FailSafe(RobotControlCMDType_t* cmd,uint8_t* lostCnt,uint16_t taskFreq);
static void Robot_ParkingEnter(TickType_t* activeTick);
static void Robot_Housekeeping(uint8_t* driverCheck,uint16_t taskFreq,TickType_t* preTime);

#if DATALOG_ENABLE
/**
 * 日志快照 —— 把控制器内部状态一次性拷给数据观察任务
 *
 * s_steer/s_drive 等为本文件私有,日志任务看不到.每周期末尾整体拷贝一次,
 * 避免日志任务逐字段读取期间被下一周期改写而拿到半新半旧的数据.
 * 仅在 DATALOG_ENABLE=1 时编译,出货固件不产生任何代码.
 */
static void Robot_LogSnapshot(const RobotControlCMDType_t* cmd)
{
	g_RobotLogSnap.tick         = xTaskGetTickCount();
	g_RobotLogSnap.reorientHold = s_steer.reorientHold;
	g_RobotLogSnap.steerScale   = s_steer.scale;
	g_RobotLogSnap.maxProgress  = s_steer.maxProgress;
	g_RobotLogSnap.steerSendCnt = s_steerSendCnt;

	g_RobotLogSnap.cmdVx = cmd->Vx;            g_RobotLogSnap.cmdVy = cmd->Vy;            g_RobotLogSnap.cmdVz = cmd->Vz;
	g_RobotLogSnap.smVx  = s_drive.smooth.Vx;  g_RobotLogSnap.smVy  = s_drive.smooth.Vy;  g_RobotLogSnap.smVz  = s_drive.smooth.Vz;
	g_RobotLogSnap.stVx  = s_steerCmd.Vx;      g_RobotLogSnap.stVy  = s_steerCmd.Vy;      g_RobotLogSnap.stVz  = s_steerCmd.Vz;

	for(uint8_t i=0;i<WHEEL_COUNT;i++)
	{
		g_RobotLogSnap.steerTgt[i] = s_steer.target[i];
		g_RobotLogSnap.rpmTgt[i]   = s_drive.rpm[i];
	}
}
#endif

//针对4轮转向的控制函数
void RobotControl_task(void* param)
{
	vTaskDelay(2000);

	//创建控制指令队列
	g_xQueueRobotControl = xQueueCreate(1,sizeof(RobotControlCMDType_t));

	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();

	//本任务的控制频率,单位为Hz
	const uint16_t TaskFreq = ROBOT_CONTROL_FREQ_HZ;

	//伺服电机自检变量
	uint8_t driverCheck = 0;

	//空闲计数
	TickType_t ActiveTick = xTaskGetTickCount();

	//队列无数据统计
	uint8_t LostCmdCounts = 0;

	//队列控制命令
	RobotControlCMDType_t cmd = { 0 };

	//上电初值.控制器状态(s_steer/s_drive/s_car 等)为静态存储,已由启动代码清零,
	//这里只补两项非零初值:小车机械参数,转向速度默认值.
	FourWSCar_param_init(&s_car,RobotHardWareParam.WheelSpacing,RobotHardWareParam.AxleSpacing,0,RobotHardWareParam.WheelPerimeter);
	s_sentCar = s_car;
	for(uint8_t si=0;si<WHEEL_COUNT;si++) s_steer.speed[si]=T280_WheelTurnSpeed;

	//等待所有转向电机复位
	if( SteeringHoming(HOMING_TIMEOUT_MS)!=0 )
	{
		//回零超时:置持久错误标志,小车失能,可通过错误恢复(B键长按)重试
		SteeringHomeFailed = 1;
	}
	vTaskDelay(200);

	while(1)
	{
		//取控制指令:队列读取 + IMU直线补偿
		Robot_CmdReceive(&cmd,&LostCmdCounts);

		//电压更新
		Robot_UpdateVoltageEstimate(&cmd);
		
		//小车使能状态检查,根据自检参数判断是否允许启动小车
		robot_en_check();

		//转向链路:解算目标 -> 驱动平滑(顺带放行泄压闸) -> 执行看门狗
		Steer_Solve(&cmd,TaskFreq);
		Drive_Update(&cmd,TaskFreq);
		Steer_Watchdog();

		//驱动链路:轮速下发 -> 正解回填三轴速度
		Drive_Output();
		Robot_Odometry();

		//状态机:活跃刷新 -> 失控保护 -> 驻车判定
		Robot_ActiveUpdate(&cmd,&ActiveTick);
		// Command expiry is handled before IMU correction in Robot_CmdReceive.
		Robot_ParkingEnter(&ActiveTick);

		//低频巡检:驱动器自检,报错恢复,低电量提示
		Robot_Housekeeping(&driverCheck,TaskFreq,&preTime);

		#if DATALOG_ENABLE
		//本周期状态快照,供数据观察任务读取
		Robot_LogSnapshot(&cmd);
		#endif

		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );

	}
}

//判断两次控制命令是否有变化
uint8_t cmd_is_change(const RobotControlCMDType_t* cmd1,const RobotControlCMDType_t* cmd2)
{
	uint8_t ischange = 0;

	//增加控制死区,避免遥控/上位机微小抖动反复触发转向重算
	if( fabsf(cmd1->Vx - cmd2->Vx) > CMD_CHANGE_EPS_V  )
		ischange=1;

	if( fabsf(cmd1->Vy - cmd2->Vy) > CMD_CHANGE_EPS_V )
		ischange=1;

	if( fabsf(cmd1->Vz - cmd2->Vz) > CMD_CHANGE_EPS_W )
		ischange=1;

	return ischange;
}

//根据转向剩余误差计算牵引速度比例,避免轮子未对准时直接满速拖拽
static float _4ws_SteerScaleByProgress(int max_progress)
{
	float err_deg = (1.0f/COUNTS_PER_DEG) * (float)max_progress;
	if( err_deg <= TURN_FULL_SPEED_DEG ) return 1.0f;
	if( err_deg >= TURN_DRIVE_ALLOW_DEG ) return 0.0f;
	//smoothstep(S-curve): zero slope at both ends, smooths drive re-engagement after a turn
	float t = (TURN_DRIVE_ALLOW_DEG - err_deg) / (TURN_DRIVE_ALLOW_DEG - TURN_FULL_SPEED_DEG);
	return t * t * (3.0f - 2.0f * t);
}

//四个轮子转速统一等比例限幅,保持底盘速度方向与旋转比例不被单轮限速破坏
static void _4ws_LimitWheelRPM(float* rpm,float max_rpm)
{
	float max_abs_rpm = 0.0f;
	for(uint8_t i=0;i<WHEEL_COUNT;i++)
	{
		if( fabsf(rpm[i]) > max_abs_rpm ) max_abs_rpm = fabsf(rpm[i]);
	}

	if( max_abs_rpm > max_rpm && max_abs_rpm > 0.0001f )
	{
		float scale = max_rpm / max_abs_rpm;
		for(uint8_t i=0;i<WHEEL_COUNT;i++) rpm[i] *= scale;
	}
}

//判断当前是否仍存在有效运动控制量
static uint8_t _4ws_HasMoveCmd(const RobotControlCMDType_t* cmd)
{
	if( fabsf(cmd->Vx) > TURN_MOVE_CMD_EPS || fabsf(cmd->Vy) > TURN_MOVE_CMD_EPS || fabsf(cmd->Vz) > TURN_MOVE_CMD_EPS )
		return 1;
	return 0;
}

//判断当前车速是否已满足大角度转向保护条件
static uint8_t _4ws_TurnSpeedReady(void)
{
	if( fabsf(RobotControlParam.feedbackVx) < TURN_SAFE_VX && fabsf(RobotControlParam.feedbackVy) < TURN_SAFE_VY &&
		fabsf(RobotControlParam.feedbackVz) < TURN_SAFE_VZ )
		return 1;
	return 0;
}

/**
 * 转向目标下发 —— 底层出口
 *
 * 全工程唯一知道"哪个驱动器管哪两个轮"的地方(Driver1->A/B, Driver2->C/D).
 * 换驱动器编号或调整轮序时只需改这里的调用参数.
 */
static void Steer_SendDriver(SteerCtrl_t* s,ServoDriverHWType_t* drv,WheelIndex_t i0,WheelIndex_t i1)
{
	uint8_t faultBit = (drv == &Driver1) ? 1u : 2u;
	if( set_MotorPos(drv,s->target[i0],s->speed[i0],s->target[i1],s->speed[i1]) != 0 )
	{
		s_steerSendFault |= faultBit;
		return;
	}
	s_steerSendFault &= (uint8_t)~faultBit;
	// Commit only the pair whose complete position transaction succeeded.
	s->lastSent[i0] = s->target[i0];
	s->lastSent[i1] = s->target[i1];
	s->lastSendTick = xTaskGetTickCount();
	if( drv == &Driver1 )
	{
		s_sentCar.TargetMotorA.radian = s_car.TargetMotorA.radian;
		s_sentCar.TargetMotorB.radian = s_car.TargetMotorB.radian;
	}
	else
	{
		s_sentCar.TargetMotorC.radian = s_car.TargetMotorC.radian;
		s_sentCar.TargetMotorD.radian = s_car.TargetMotorD.radian;
	}

	#if DATALOG_ENABLE
	s_steerSendCnt++;
	#endif
}

/**
 * 转向目标下发 —— 完整四轮,并同步记账
 *
 * 下发与记账(lastSent / lastSendTick)必须成对发生,否则节流判据会失准.
 * 需要下发就调本函数,不要在别处直接碰这两个字段.
 */
static void Steer_Send(SteerCtrl_t* s)
{
	Steer_SendDriver(s,&Driver1,WHEEL_A,WHEEL_B);
	Steer_SendDriver(s,&Driver2,WHEEL_C,WHEEL_D);

	// Per-driver successful sends update lastSent in Steer_SendDriver.
	s->lastSendTick = xTaskGetTickCount();
}

int angle_to_pos(float angle)
{
	return COUNTS_PER_DEG*(angle-90.0f);
}

float pos_to_angle(int pos)
{
	return pos*(1.0f/COUNTS_PER_DEG) + 90.0f;
}

/* ==========================================================================
 * 主循环职责函数实现
 *
 * 每个函数只做一件事.调用顺序及其约束见文件前部的声明块.
 * ========================================================================== */

/**
 * 取指令 —— 队列读取与IMU直线补偿
 *
 * 队列空时不改写cmd,沿用上一周期的值;连续空的次数由lostCnt累计,
 * 交由失控保护判定是否超时.
 */
static void Robot_CmdReceive(RobotControlCMDType_t* cmd,uint8_t* lostCnt)
{
	//从队列中读取控制指令
	BaseType_t control_queue_state = pdFAIL;
	static RobotControlCMDType_t rawCmd = {0};
	control_queue_state = xQueueReceive(g_xQueueRobotControl,&rawCmd,0);
	if( control_queue_state == pdPASS )
		(*lostCnt)=0;
	else
		if( *lostCnt < 255 ) (*lostCnt)++;

	// Expire the original command before applying this cycle's IMU correction.
	Robot_FailSafe(&rawCmd,lostCnt,ROBOT_CONTROL_FREQ_HZ);
	*cmd = rawCmd;

	//陀螺仪直线补偿
	if( RobotControlParam.ImuAssistedFlag == 1 )
	{
		Linear_IMUHelp(cmd);

		//直线Z速度补偿
		float final_vz = cmd->Vz + LinearHelp_Target.Vz;
		cmd->Vz = final_vz;
	}
	else
	{
		LinearHelp_Target.Vx=0;LinearHelp_Target.Vy=0;LinearHelp_Target.Vz=0;
	}
}

/**
 * 转向解算 —— 由速度指令算出四轮转向目标并按需下发
 *
 * 固定顺序:限幅平滑 -> 逆解目标 -> 等时到达的速度分配 -> 泄压闸判据 -> 下发 -> 偏差统计.
 * 目标只在指令变化时重算,下发却每周期评估:被节流挡掉的一次下发必须还有下一次机会,
 * 否则误差会停在门槛与看门狗之间的死区里.
 */
static void Steer_Solve(const RobotControlCMDType_t* cmd,uint16_t taskFreq)
{
	//转向输入限幅平滑:拦住上游抖动,且不受任何闸门影响
	_4ws_SpeedSmoothLimit(&s_steerCmd,cmd,taskFreq,
		STEER_ACC_XY,STEER_ACC_XY,STEER_ACC_XY,STEER_ACC_XY,STEER_ACC_W,STEER_ACC_W);

	//运动学逆解,获取车轮线速度m/s
	//命令变化标志位
	uint8_t changeFlag = 0;

	//检查速度是否发生变化
	changeFlag = cmd_is_change(&s_steerCmd,&s_lastCmd);

	//超过死区才更新历史数据,使缓慢变化的控制量可累积触发转向重算
	if( changeFlag )
		memcpy(&s_lastCmd,&s_steerCmd,sizeof(RobotControlCMDType_t));

	if( changeFlag )
	{
		//用未平滑的原始指令算一份'意图目标',仅供泄压闸判据使用.
		//平滑只改变执行过程,不该改变'是否需要先泄掉动量'的判断.
		//逆解是纯函数(只读RealMotor*.radian与hardware),两个实例互不干扰.
		s_rawCar = s_car;
		four_ws_InverseKinematics(cmd->Vx,cmd->Vy,cmd->Vz,&s_rawCar);

		//意图目标(counts).CD电机镜像安装,零点与AB相差180度,故C/D先补180再转位置
		float rawPos[WHEEL_COUNT];
		rawPos[WHEEL_A] = angle_to_pos(s_rawCar.TargetMotorA.radian*57.2958f);
		rawPos[WHEEL_B] = angle_to_pos(s_rawCar.TargetMotorB.radian*57.2958f);
		rawPos[WHEEL_C] = angle_to_pos(s_rawCar.TargetMotorC.radian*57.2958f+180);
		rawPos[WHEEL_D] = angle_to_pos(s_rawCar.TargetMotorD.radian*57.2958f+180);

		//再用平滑后的指令算实际要下发的目标
		four_ws_InverseKinematics(s_steerCmd.Vx,s_steerCmd.Vy,s_steerCmd.Vz,&s_car);

		//弧度转角度,再转换为转向电机的位置
		s_steer.target[WHEEL_A] = angle_to_pos(s_car.TargetMotorA.radian*57.2958f);
		s_steer.target[WHEEL_B] = angle_to_pos(s_car.TargetMotorB.radian*57.2958f);
		s_steer.target[WHEEL_C] = angle_to_pos(s_car.TargetMotorC.radian*57.2958f+180);
		s_steer.target[WHEEL_D] = angle_to_pos(s_car.TargetMotorD.radian*57.2958f+180);

		//获得所有电机实际总路径
		int total_path[WHEEL_COUNT];
		total_path[WHEEL_A] = fabsf(s_steer.target[WHEEL_A]-RobotControlParam.MotorA.ori_pos);
		total_path[WHEEL_B] = fabsf(s_steer.target[WHEEL_B]-RobotControlParam.MotorB.ori_pos);
		total_path[WHEEL_C] = fabsf(s_steer.target[WHEEL_C]-RobotControlParam.MotorC.ori_pos);
		total_path[WHEEL_D] = fabsf(s_steer.target[WHEEL_D]-RobotControlParam.MotorD.ori_pos);

		//获得最大路径的轮子
		int max_path = total_path[WHEEL_A];
		if( total_path[WHEEL_B] > max_path ) max_path = total_path[WHEEL_B];
		if( total_path[WHEEL_C] > max_path ) max_path = total_path[WHEEL_C];
		if( total_path[WHEEL_D] > max_path ) max_path = total_path[WHEEL_D];

		//最远的一路在最大速度下的耗时即本次转向的总时长(S=V*t),
		//其余各路按各自行程除以该时长分配速度,四轮同时到位.
		float tmpT = 0;
		if( max_path > 5 )
		{
			tmpT = (float)max_path/(float)T280_WheelTurnSpeed;
		}

		for(uint8_t i=0;i<WHEEL_COUNT;i++)
		{
			//行程太短(tmpT为0)时不重算速度,沿用上次的值即可
			if( tmpT!=0 ) s_steer.speed[i] = total_path[i]/tmpT;

			if( s_steer.speed[i] < 5 ) s_steer.speed[i] = 5;
			if( s_steer.speed[i] > T280_WheelTurnSpeed ) s_steer.speed[i] = T280_WheelTurnSpeed;
		}

		//泄压闸判据只在重算目标时评估(依赖本周期算出的rawPos)
		if( RobotControlParam.en_flag==1 )
		{
			//'意图目标'与当前实际轮角之差 = 用户要求轮子还得摆多远.
			//用未平滑的原始指令算,平滑前后同一动作算出的幅度一致,不会因平滑而漏判.
			float intent_jump = fabsf(rawPos[WHEEL_A]-RobotControlParam.MotorA.ori_pos);
			if( fabsf(rawPos[WHEEL_B]-RobotControlParam.MotorB.ori_pos) > intent_jump ) intent_jump = fabsf(rawPos[WHEEL_B]-RobotControlParam.MotorB.ori_pos);
			if( fabsf(rawPos[WHEEL_C]-RobotControlParam.MotorC.ori_pos) > intent_jump ) intent_jump = fabsf(rawPos[WHEEL_C]-RobotControlParam.MotorC.ori_pos);
			if( fabsf(rawPos[WHEEL_D]-RobotControlParam.MotorD.ori_pos) > intent_jump ) intent_jump = fabsf(rawPos[WHEEL_D]-RobotControlParam.MotorD.ori_pos);

			//大重定向 且 车身仍快:先进减速等待闸,泄掉动量再放轮子过去
			if( intent_jump > REORIENT_JUMP_COUNTS && _4ws_TurnSpeedReady()==0 )
			{
				if( s_steer.reorientHold==0 ){ s_steer.reorientHold=1; s_steer.reorientHoldStart=xTaskGetTickCount(); }
			}
		}
	}

	//转向目标下发:独立于changeFlag每周期评估.
	//若关在changeFlag里,一旦某次下发被时间节流或门槛挡掉就再无第二次机会
	//(指令已稳定,changeFlag不会再为真),误差会停在2~5度死区:
	//看门狗只在>5度时才重发,下面的门槛以下又不发,中间这段无人负责.
	if( RobotControlParam.en_flag==1 && s_steer.reorientHold==0 )
	{
		float send_jump = 0.0f;
		for(uint8_t i=0;i<WHEEL_COUNT;i++)
		{
			float d = fabsf(s_steer.target[i]-s_steer.lastSent[i]);
			if( d > send_jump ) send_jump = d;
		}

		//目标变化够大 且 距上次下发够久.发完send_jump归零,自然停止,不会持续占用总线.
		if( (send_jump > STEER_SEND_EPS_COUNTS || s_steerSendFault) &&
		    (xTaskGetTickCount()-s_steer.lastSendTick) >= pdMS_TO_TICKS(STEER_SEND_MIN_INTERVAL_MS) )
		{
			Steer_Send(&s_steer);
		}
	}

	//转向电机原始值:逆时针为正.
	//剩余偏差同时供起步对齐闸,连续限速与执行看门狗三处判据使用.
	s_steer.progress[WHEEL_A] = abs((int)s_steer.target[WHEEL_A]-RobotControlParam.MotorA.ori_pos);
	s_steer.progress[WHEEL_B] = abs((int)s_steer.target[WHEEL_B]-RobotControlParam.MotorB.ori_pos);
	s_steer.progress[WHEEL_C] = abs((int)s_steer.target[WHEEL_C]-RobotControlParam.MotorC.ori_pos);
	s_steer.progress[WHEEL_D] = abs((int)s_steer.target[WHEEL_D]-RobotControlParam.MotorD.ori_pos);

	//找出最大值
	s_steer.maxProgress = s_steer.progress[WHEEL_A];
	if( s_steer.progress[WHEEL_B] > s_steer.maxProgress ) s_steer.maxProgress = s_steer.progress[WHEEL_B];
	if( s_steer.progress[WHEEL_C] > s_steer.maxProgress ) s_steer.maxProgress = s_steer.progress[WHEEL_C];
	if( s_steer.progress[WHEEL_D] > s_steer.maxProgress ) s_steer.maxProgress = s_steer.progress[WHEEL_D];

	//本周期实际施加到指令上的转向限速比例(0=本周期下发停止指令),供数据观察使用
	s_steer.scale = 0.0f;

	//使能恢复(0->1)时主动重发一次当前转向目标,不再依赖看门狗的副作用
	if( RobotControlParam.en_flag==1 && s_steer.lastEnFlag==0 && s_steer.reorientHold==0 )
	{
		Steer_Send(&s_steer);
	}
	s_steer.lastEnFlag = RobotControlParam.en_flag;
}

/**
 * 驱动平滑 —— 决定本周期实际执行的三轴速度
 *
 * 三条互斥路径:泄压闸急刹 / 起步对齐等待 / 按轮角偏差连续限速.
 * 泄压闸的放行在本函数内完成,而 Steer_Watchdog 的守卫以 reorientHold==0 为条件,
 * 故调用顺序必须是 Steer_Solve -> Drive_Update -> Steer_Watchdog.
 */
static void Drive_Update(const RobotControlCMDType_t* cmd,uint16_t taskFreq)
{
	//转向逻辑处理
	if( s_steer.reorientHold )
	{
		//重定向闸:高速切自转/横移时先直线急刹泄掉动量,再放轮子过去,防止打滑
		if( _4ws_TurnSpeedReady() )
		{
			s_steer.reorientHold = 0;
			if( RobotControlParam.en_flag==1 )
			{
				Steer_Send(&s_steer);
			}
		}
		//等待期间全力急刹,泄掉前冲动量
		_4ws_SpeedSmoothFastBrake(&s_drive.smooth,&s_stopCmd,taskFreq);
		// A timeout is not evidence that the chassis has slowed down.
		if( s_steer.reorientHold && xTaskGetTickCount()-s_steer.reorientHoldStart > pdMS_TO_TICKS(REORIENT_HOLD_TIMEOUT_MS) )
			s_drive.smooth = s_stopCmd;
	}
	else if( RobotControlParam.en_flag==1 && _4ws_HasMoveCmd(cmd) )
	{
		//静止起步对齐闸:上一刻完全静止时,先等轮子摆到位(误差<容差)再给速,消除起步抖动
		uint8_t was_static = ( fabsf(s_drive.smooth.Vx)<STARTUP_STOP_EPS && fabsf(s_drive.smooth.Vy)<STARTUP_STOP_EPS && fabsf(s_drive.smooth.Vz)<STARTUP_STOP_EPS );
		if( was_static && s_steer.maxProgress > COUNTS_PER_DEG * STARTUP_ALIGN_TOL_DEG )
		{
			//轮子尚未就位,保持不动直到摆到位
			_4ws_SpeedSmooth(&s_drive.smooth,&s_stopCmd,taskFreq);
		}
		else
		{
			//车速按实时轮角偏差连续限速:偏差大(舵机未追上)则慢,对齐则快,杜绝转来转去的反复刹停
			RobotControlCMDType_t SteerLimitCmd = *cmd;
			s_steer.scale = _4ws_SteerScaleByProgress(s_steer.maxProgress);
			SteerLimitCmd.Vx *= s_steer.scale;
			SteerLimitCmd.Vy *= s_steer.scale;
			SteerLimitCmd.Vz *= s_steer.scale;
			_4ws_SpeedSmooth(&s_drive.smooth,&SteerLimitCmd,taskFreq);
		}
	}
	else
	{
		_4ws_SpeedSmooth(&s_drive.smooth,&s_stopCmd,taskFreq);
	}
}

/**
 * 转向执行看门狗 —— 每100ms比对一次进度
 *
 * 驱动器偶发丢帧会让目标停在半路,靠比较窗口内的偏差增量发现并补发.
 * 泄压闸期间不介入,否则会顶着急刹把轮子推过去.
 */
static void Steer_Watchdog(void)
{
	//检查转向电机指令是否失效
	if( s_steer.reorientHold==0 && RobotControlParam.en_flag==1 && RobotControlParam.ParkingMode==0 && s_steer.maxProgress > COUNTS_PER_DEG * 5.0f ) //目标未到达时检测
	{
		s_steer.lowfreCnt++;
		if( s_steer.lowfreCnt == 10 )
		{
			s_steer.lowfreCnt=0;
			//如检测到任意一路转向电机未运动,则重新发送目标指令
			if( (s_steer.progress[WHEEL_A] > COUNTS_PER_DEG*5.0f && abs(s_steer.progress[WHEEL_A]-s_steer.lastProgress[WHEEL_A]) < 10) || (s_steer.progress[WHEEL_B] > COUNTS_PER_DEG*5.0f && abs(s_steer.progress[WHEEL_B]-s_steer.lastProgress[WHEEL_B]) < 10) )
			{
				Steer_SendDriver(&s_steer,&Driver1,WHEEL_A,WHEEL_B);
			}
			if((s_steer.progress[WHEEL_C] > COUNTS_PER_DEG*5.0f && abs(s_steer.progress[WHEEL_C]-s_steer.lastProgress[WHEEL_C]) < 10) || (s_steer.progress[WHEEL_D] > COUNTS_PER_DEG*5.0f && abs(s_steer.progress[WHEEL_D]-s_steer.lastProgress[WHEEL_D]) < 10) )
			{
				Steer_SendDriver(&s_steer,&Driver2,WHEEL_C,WHEEL_D);
			}

			//快照只在每次比较时刷新,保证比较的是完整100ms窗口的增量
			for(uint8_t i=0;i<WHEEL_COUNT;i++) s_steer.lastProgress[i] = s_steer.progress[i];
		}
	}
	else
	{
		s_steer.lowfreCnt = 0;
		for(uint8_t i=0;i<WHEEL_COUNT;i++) s_steer.lastProgress[i] = s_steer.progress[i];
	}
}

/**
 * 轮速输出 —— 三轴速度投影到四轮转速并下发
 *
 * 沿已下发的转向目标投影,轮速符号与转角天然一致.整体归一化保证单轮限幅
 * 不破坏四轮之间的运动比例.失能时显式下发0转速,不依赖驱动器自行停车.
 */
static void Drive_Output(void)
{
	//使用平滑值计算当前的速度
	//轮速沿已下发的目标转向角投影,符号与角度天然一致,不再二次逆解
	four_ws_WheelSpeedProjection(s_drive.smooth.Vx,s_drive.smooth.Vy,s_drive.smooth.Vz,&s_sentCar);

	//轮速m/s转为转速rpm
	s_drive.rpm[WHEEL_A] = linearVel_to_rpm(s_sentCar.TargetMotorA.speed,s_car.hardware.WheelPerimeter);
	s_drive.rpm[WHEEL_B] = linearVel_to_rpm(s_sentCar.TargetMotorB.speed,s_car.hardware.WheelPerimeter);
	s_drive.rpm[WHEEL_C] = linearVel_to_rpm(s_sentCar.TargetMotorC.speed,s_car.hardware.WheelPerimeter);
	s_drive.rpm[WHEEL_D] = linearVel_to_rpm(s_sentCar.TargetMotorD.speed,s_car.hardware.WheelPerimeter);
	//轮速整体归一化,防止单个轮子超过驱动限制后破坏运动比例
	_4ws_LimitWheelRPM(s_drive.rpm,T280_MAX_DRIVE_RPM);

	if( RobotControlParam.en_flag==1 && s_steerSendFault==0 )
	{
		//轮毂电机执行
		set_MotorRPM(&Driver3,-s_drive.rpm[WHEEL_A],-s_drive.rpm[WHEEL_B]);
		set_MotorRPM(&Driver4,-s_drive.rpm[WHEEL_C],-s_drive.rpm[WHEEL_D]);
	}
	else
	{
		set_MotorRPM(&Driver3,0,0);
		set_MotorRPM(&Driver4,0,0);
	}
}

/**
 * 里程正解 —— 由实测轮角与轮速反推三轴速度
 *
 * 结果写回 RobotControlParam.feedbackV*,供上位机上报与IMU直线补偿读取.
 */
static void Robot_Odometry(void)
{
	//编码器位置转轮角
	RobotControlParam.MotorA.feedbackRadian = pos_to_angle(RobotControlParam.MotorA.ori_pos);
	RobotControlParam.MotorB.feedbackRadian = pos_to_angle(RobotControlParam.MotorB.ori_pos);
	RobotControlParam.MotorC.feedbackRadian = pos_to_angle(RobotControlParam.MotorC.ori_pos)-180;//CD电机镜像安装,减去180度
	RobotControlParam.MotorD.feedbackRadian = pos_to_angle(RobotControlParam.MotorD.ori_pos)-180;

	//赋值实际数据到小车模型,用于计算3轴速度
	s_car.RealMotorA.radian = RobotControlParam.MotorA.feedbackRadian*0.0174533f;
	s_car.RealMotorB.radian = RobotControlParam.MotorB.feedbackRadian*0.0174533f;
	s_car.RealMotorC.radian = RobotControlParam.MotorC.feedbackRadian*0.0174533f;
	s_car.RealMotorD.radian = RobotControlParam.MotorD.feedbackRadian*0.0174533f;
	s_car.RealMotorA.speed = RobotControlParam.MotorA.feedbackSpeed;
	s_car.RealMotorB.speed = RobotControlParam.MotorB.feedbackSpeed;
	s_car.RealMotorC.speed = RobotControlParam.MotorC.feedbackSpeed;
	s_car.RealMotorD.speed = RobotControlParam.MotorD.feedbackSpeed;

	//运动学正解
	four_ws_kinematics(&s_car);

	//输出3轴速度
	RobotControlParam.feedbackVx = s_car.feedback_Vx;
	RobotControlParam.feedbackVy = s_car.feedback_Vy;
	RobotControlParam.feedbackVz = s_car.feedback_Vz;
}

/**
 * 活跃刷新 —— 收到运动指令则刷新时间戳并退出驻车
 */
static void Robot_ActiveUpdate(const RobotControlCMDType_t* cmd,TickType_t* activeTick)
{
	//判断当前是否有活跃的控制量
	if( _4ws_HasMoveCmd(cmd) )
	{
		(*activeTick) = xTaskGetTickCount();
		if( RobotControlParam.ParkingMode==1 )
		{
			RobotControlParam.ParkingMode=0;
			//驻车模式解除
			Robot_DriverSetParkingMode(0);
		}
	}
}

/**
 * 失控保护 —— 指令超时后按控制源决定是否清零速度
 *
 * lostCnt 在此封顶,避免长时间无指令时计数回绕导致保护间歇性失效.
 */
static void Robot_FailSafe(RobotControlCMDType_t* cmd,uint8_t* lostCnt,uint16_t taskFreq)
{
	//小车失控保护设置(队列无数据超时情况)
	if( (*lostCnt) > taskFreq )
	{
		(*lostCnt)=taskFreq;

		switch( RobotControl_CMDsource )
		{
			//非上位机类型的控制默认需要保护
			//此处原本APP控制也应执行,但为了兼容IOS APP(数据非连续发送),保护无法开启.
			case GamePad_CMD:case RCJOY_CMD: case Charger_CMD: case AutoTest_CMD:
				cmd->Vx=0;cmd->Vy=0;cmd->Vz=0;
				break;

			//上位机类型,根据用户设置的安全级别启用保护
			case ROS_CMD:case CAN_CMD:
				if( RobotControlParam.SecurityLevel==0 )
				{
					cmd->Vx=0;cmd->Vy=0;cmd->Vz=0;
				}
				break;
		}
	}
}

/**
 * 驻车判定 —— 静止满10秒且车身水平则进入省电驻车
 *
 * 斜坡上不驻车(驻车会松开保持力矩),改为刷新时间戳继续等待.
 */
static void Robot_ParkingEnter(TickType_t* activeTick)
{
	//根据控制量是否活跃来判断是否进入驻车模式,若小车停止10秒后无控制量,将进入驻车模式省电
	if( RobotControlParam.ParkingMode==0 && xTaskGetTickCount() - (*activeTick) > 10000 )
	{
		//根据imu角度判断是否位于斜坡上
		if(fabs(AttitudeVal.pitch) < 2.5f && fabs(AttitudeVal.roll)<2.5f )
		{
			RobotControlParam.ParkingMode=1;
			//发送驻车指令
			Robot_DriverSetParkingMode(1);
		}
		else
		{	//小车位于斜坡上,刷新用于判断的时间
			(*activeTick) = xTaskGetTickCount();
		}
	}
}

/**
 * 巡检 —— 驱动器自检,报错恢复,低电量提示
 *
 * 报错恢复可能阻塞数百毫秒,故在此重置周期基准,避免 vTaskDelayUntil
 * 事后补偿性地连跑几个周期.
 */
static void Robot_Housekeeping(uint8_t* driverCheck,uint16_t taskFreq,TickType_t* preTime)
{
	pBuzzerInterface_t buzzer = &UserBuzzer;

	//驱动器低频获取自检参数,包含电机报错码,驱动报错情况
	(*driverCheck)++;
	if( (*driverCheck)==10 )
	{
		(*driverCheck)=0;
		Robot_DriverCheckError();
	}

	//驱动报错恢复请求,驱动器报错后,用户可通过此标志位请求尝试清除报错
	if( RobotControlParam.DriveErrRecovery )
	{
		RobotControlParam.DriveErrRecovery = 0;
		Robot_DriverRecovery();

		//恢复过程可能阻塞较久(含回零重试),重置周期基准避免vTaskDelayUntil补偿性连跑
		(*preTime) = xTaskGetTickCount();
	}

	//小车低电量蜂鸣器提示,仅无灯带车型使用蜂鸣器提示
	static uint32_t LowPowerTipsCnt = 0;
	switch( RobotHardWareParam.CarType )
	{
		case S200:case S200_OUTDOOR:case S260:
			if( RobotControlParam.LowPowerFlag )
			{
				LowPowerTipsCnt++;
				if( LowPowerTipsCnt>taskFreq*4 )
				{
					LowPowerTipsCnt=0;
					buzzer->AddTask(1,750);
				}
			}
			break;
		default:
			break;
	}
}

//多模智控手柄按键回调函数,均与小车功能相关
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	pBuzzerInterface_t buzzer = &UserBuzzer;

	//LB左上按键:减速
	if( keyid == Xbox360KEY_LB )
	{
		RobotControlParam.defalutSpeed -= 50;
		if( RobotControlParam.defalutSpeed<0 ) RobotControlParam.defalutSpeed=0;
		buzzer->AddTask(1,200);
	}

	//RB右上按键:加速
	if( keyid == Xbox360KEY_RB )
	{
		RobotControlParam.defalutSpeed += 50;
		if( RobotControlParam.defalutSpeed>3000 ) RobotControlParam.defalutSpeed=3000;
		buzzer->AddTask(1,200);
	}

	//右摇杆Rjoy:IMU标零
	if( keyid == Xbox360KEY_RJoy && event == GamePadKeyEvent_LONGCLICK)
	{
		pIMUInterface_t imu = &UserICM20948;
		imu->UpdateZeroPoint_axis();
		imu->UpdateZeroPoint_attitude();
		buzzer->AddTask(1,200);//使用蜂鸣器提示操作成功
	}

	//Y:自动回充开启/关闭
	if( keyid == Xbox360KEY_Y && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.ChargeMode = !RobotControlParam.ChargeMode;
		buzzer->AddTask(1,200);
	}

	//B:清除电机报错请求
	if( keyid == Xbox360KEY_B && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.DriveErrRecovery=1;
		buzzer->AddTask(1,200);
	}

	//A:超声波避障开关.仅切换RangerAvoidEN标志位与蜂鸣提示,避障决策逻辑尚未实现
	if( keyid == Xbox360KEY_A && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.RangerAvoidEN = !RobotControlParam.RangerAvoidEN;
		buzzer->AddTask(1,200);
	}

	//X:IMU辅助走直线开关
	if( keyid==Xbox360KEY_X && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.ImuAssistedFlag = !RobotControlParam.ImuAssistedFlag;
		buzzer->AddTask(1,200);
	}

	//左摇杆LJoy:应急模式
	if( keyid == Xbox360KEY_LJoy && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.EmergencyMode = !RobotControlParam.EmergencyMode;
		buzzer->AddTask(1,200);
	}

	//方向按键,左右对应OLED翻页
	if( keyid == Xbox360KEY_LEFT )
	{
		oled_page_down();
		buzzer->AddTask(1,200);
	}

	if( keyid == Xbox360KEY_RIGHT )
	{
		oled_page_up();
		buzzer->AddTask(1,200);
	}

	#if DATALOG_ENABLE
	//上键:场地自动测试,长按启动,单击/双击停止(蜂鸣反馈由测试任务发出)
	if( keyid == Xbox360KEY_UP )
	{
		if( event == GamePadKeyEvent_LONGCLICK )
			AutoTest_Start();
		else if( event == GamePadKeyEvent_SINGLECLICK || event == GamePadKeyEvent_DOUBLECLICK )
			AutoTest_Stop();
	}
	#endif

}

//线速度m/s转RPM转速
static float linearVel_to_rpm(float linearVel,float wheelper)
{
	return linearVel*60.0f/wheelper;
}

//转向电机回零:周期性重发回零指令并等待全部到位;成功返回0,超时返回1
static uint8_t SteeringHoming(uint32_t timeout_ms)
{
	TickType_t start = xTaskGetTickCount();
	TickType_t lastSend = start;

	//先发一次回零指令
	set_MotorPos(&Driver1,0,T280_WheelTurnSpeed,0,T280_WheelTurnSpeed);
	set_MotorPos(&Driver2,0,T280_WheelTurnSpeed,0,T280_WheelTurnSpeed);

	while(1)
	{
		vTaskDelay(20);

		//四轮全部回到零点(容差内)才算完成
		float zeroA = fabsf(RobotControlParam.MotorA.ori_pos);
		float zeroB = fabsf(RobotControlParam.MotorB.ori_pos);
		float zeroC = fabsf(RobotControlParam.MotorC.ori_pos);
		float zeroD = fabsf(RobotControlParam.MotorD.ori_pos);
		if( zeroA <= HOMING_TOL_COUNTS && zeroB <= HOMING_TOL_COUNTS && zeroC <= HOMING_TOL_COUNTS && zeroD <= HOMING_TOL_COUNTS )
			return 0;

		if( xTaskGetTickCount() - start > pdMS_TO_TICKS(timeout_ms) )
			return 1;

		//周期性重发,防止上电初期总线未就绪导致指令丢失
		if( xTaskGetTickCount() - lastSend >= pdMS_TO_TICKS(HOMING_RESEND_MS) )
		{
			lastSend = xTaskGetTickCount();
			set_MotorPos(&Driver1,0,T280_WheelTurnSpeed,0,T280_WheelTurnSpeed);
			set_MotorPos(&Driver2,0,T280_WheelTurnSpeed,0,T280_WheelTurnSpeed);
		}
	}
}

//驱动报错恢复函数
static void Robot_DriverRecovery(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		//如果电机存在报错,则清除
		if(g_ServoDriveList[i]->errFlag==1)
		{
			ServoDriver_ClearError(g_ServoDriveList[i]);
		}

		//根据使能开关的状态确定是否要使能电机
		if( RobotControlParam.Enkeystate==1 ) ServoDriver_Enable(g_ServoDriveList[i]);
		else ServoDriver_Disable(g_ServoDriveList[i]);

		//转速清零
		if( g_ServoDriveList[i]->initMode==SpeedMode )
			set_MotorRPM(g_ServoDriveList[i],0,0);
	}

	//回零失败后,借错误恢复动作重试回零,成功则清除持久错误标志
	if( SteeringHomeFailed )
	{
		if( SteeringHoming(HOMING_TIMEOUT_MS)==0 )
			SteeringHomeFailed = 0;
	}
}

//设置驻车模式
static void Robot_DriverSetParkingMode(uint8_t setflag)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		//驻车只对轮毂电机(速度模式)下发,转向电机不处理
		if( g_ServoDriveList[i]->initMode==SpeedMode )
			ServoDriver_ParkingMode(g_ServoDriveList[i],setflag);
	}
}

//获取驱动报错信息
static void Robot_DriverCheckError(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_CheckError(g_ServoDriveList[i]);
	}
}

//失能电机
static void Robot_DriverDisable(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_Disable(g_ServoDriveList[i]);
	}
}

//使能电机
static void Robot_DriverEnable(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_Enable(g_ServoDriveList[i]);
	}
}

//电机自锁停转
//static void Robot_DriverStop(void)
//{
//	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
//	{
//		if( g_ServoDriveList[i]->initMode == SpeedMode )
//			set_MotorRPM(g_ServoDriveList[i],0,0);
//		else if( g_ServoDriveList[i]->initMode == PosMode ) //TODO:待测试
//			set_MotorPosStop(g_ServoDriveList[i]);
//	}
//}

//根据运动状态低频更新电池电压估计值
static void Robot_UpdateVoltageEstimate(const RobotControlCMDType_t* cmd)
{
	const float eps = 0.001f;
	float rawVol = (float)USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL)/4095.0f*3.3f*11;
	float weight = 0.5f;//默认权重
	static uint8_t volInit = 0;
	static TickType_t lastVolUpdateTick = 0;
	TickType_t nowTick = xTaskGetTickCount();
	
	if( volInit == 0 )
	{
		RobotControlParam.Vol = rawVol;
		lastVolUpdateTick = nowTick;
		volInit = 1;
		return;
	}
	
	if( nowTick - lastVolUpdateTick < pdMS_TO_TICKS(500) ) return;
	lastVolUpdateTick = nowTick;
	
	//驻车模式下的权重最高
	if( RobotControlParam.en_flag == 1 && RobotControlParam.ParkingMode==1 )
	{
		weight = 0.9f;
	}
	
	if( RobotControlParam.en_flag == 1 && cmd != NULL )
	{
		if( (fabsf(cmd->Vx) > eps || fabsf(cmd->Vy) > eps) && (fabsf(cmd->Vz) > eps) )
		{
			weight = 0.3f; //可能存在滑动摩擦的运动场景,降低权重
		}
	}
	
	RobotControlParam.Vol = RobotControlParam.Vol*(1.0f-weight) + rawVol*weight;
}

//小车使能状态检查
static void robot_en_check(void)
{
	//报错码默认为0,根据下列报错情况而设置
	RobotControlParam.ErrNum = 0;

	//软件急停开关状态
	if( RobotControlParam.softwareEnflag == 1 ) Set_RobotErrorCode(errCode_SoftWareStop);

	//转向回零失败,持续置错误码使小车失能
	if( SteeringHomeFailed ) Set_RobotErrorCode(errCode_SteeringHome);

	//读取机器人急停开关的状态,按键切换时作出对应操作
	static uint8_t last_EnKey = 1;
	RobotControlParam.Enkeystate = get_EnKeyState();
	if( RobotControlParam.Enkeystate==0 ) Set_RobotErrorCode(errCode_StopKeyEn);

	//使能开关按下
	if( RobotControlParam.Enkeystate==0 && last_EnKey==1 )
	{
		Robot_DriverDisable();
	}

	//使能开关弹起
	else if( last_EnKey==0 && RobotControlParam.Enkeystate==1 )
	{
		Robot_DriverEnable();
	}

	last_EnKey = RobotControlParam.Enkeystate;

	//机器人电量低状态获取
	static uint8_t LowPowerTiming=0U;
	static TickType_t LowPowerStartTick=0U;
	if( RobotControlParam.Vol>18.0f && RobotControlParam.Vol < 20.2f )
	{
		TickType_t nowTick = xTaskGetTickCount();
		if( LowPowerTiming == 0U )
		{
			LowPowerTiming = 1U;
			LowPowerStartTick = nowTick;
		}
		if( nowTick - LowPowerStartTick >= pdMS_TO_TICKS(10000U) )
		{
			RobotControlParam.LowPowerFlag=1;
			//电压不足.(若开启了自动回充模式,则忽略电压)
			if( RobotControlParam.Vol < 20.0f && RobotControlParam.ChargeMode==0 ) Set_RobotErrorCode(errCode_LowPower);
		}
	}
	else
	{
		LowPowerTiming=0U;
		LowPowerStartTick=0U;
		if( RobotControlParam.Vol>21.5f) RobotControlParam.LowPowerFlag=0;
	}

	//根据不同的驱动个数生成不同的报错码
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		if( g_ServoDriveList[i]->errFlag==1 )
		{
			switch(g_ServoDriveList[i]->NodeID)
			{
				case 1: Set_RobotErrorCode(errCode_Driver1_Err);break;
				case 2: Set_RobotErrorCode(errCode_Driver2_Err);break;
				case 3: Set_RobotErrorCode(errCode_Driver3_Err);break;
				case 4: Set_RobotErrorCode(errCode_Driver4_Err);break;
			}
		}

		if( g_ServoDriveList[i]->onlineFlag==0 )
		{
			switch(g_ServoDriveList[i]->NodeID)
			{
				case 1: Set_RobotErrorCode(errCode_Driver1_offline); break;
				case 2: Set_RobotErrorCode(errCode_Driver2_offline); break;
				case 3: Set_RobotErrorCode(errCode_Driver3_offline); break;
				case 4: Set_RobotErrorCode(errCode_Driver4_offline); break;
			}
		}
	}

	if( RobotControlParam.ErrNum!=0 && RobotControlParam.EmergencyMode==0 ) RobotControlParam.en_flag = 0;
	else RobotControlParam.en_flag = 1;
}

/**
 * @brief  此函数用于写控制队列,所有需要控制小车的任务均需要通过此任务写入控制量.
 * @param  cmd:参数需要包含指令来源,以及指定小车的三轴目标速度大小
 *         woken:如果是在中断中调用,需要传入此参数.非中断中调用直接写入0
 * @retval 0写入成功,1写入失败.写入失败原因:未指定控制指令来源或当前有更高优先级控制在活跃
 *
 * @details
 * - 用户需要新增控制方式时,需要先在RobotControl_task.h头文件新增一个控制类型的枚举并确定优先级,
 *   然后通过 WriteControlQueue 写控制队列进行小车的控制
 */
uint8_t WriteRobotControlQueue(RobotControlCMDType_t* cmd,BaseType_t* woken)
{
	static uint32_t lastTick = 0;
	uint8_t writeflag = 0;

	//队列未创建或未指定的控制源,忽略写入
	if(g_xQueueRobotControl == NULL || cmd->cmdsource == NONE_CMD ||
		cmd->cmdsource >= UnKnownCMD ) return 1;

	//高优先级的控制指令,可直接抢占控制
	if( cmd->cmdsource <= RobotControl_CMDsource )
	{
		RobotControl_CMDsource = cmd->cmdsource;
		writeflag = 1;
	}
	else
	{
		//接受到低优先级控制指令,检查活跃时间戳
		uint32_t tick = 0;
		if( __get_IPSR()!=0 ) tick = xTaskGetTickCountFromISR();
		else tick = xTaskGetTickCount();

		//高优先级指令超2000ms未活跃,则允许低优先级指令执行
		if( tick - lastTick >= 2000 )
		{
			RobotControl_CMDsource = cmd->cmdsource;
			writeflag = 1;
		}
	}

	//允许写入队列
	if( writeflag )
	{
		if( __get_IPSR()!=0 ) //中断上下文判断
		{
			lastTick = xTaskGetTickCountFromISR();
			xQueueOverwriteFromISR(g_xQueueRobotControl,cmd,woken);
		}
		else
		{
			lastTick = xTaskGetTickCount();
			xQueueOverwrite(g_xQueueRobotControl,cmd);
		}
		return 0;
	}

	return 1;
}

/**
 * @brief  此函数用于计算纠偏系数,用于纠正小车走直线的效果
 * @param  纠偏系数值,左轮/右轮
 * @retval 纠偏倍数
 *
 * @details
 * 当小车无法走直线时,可通过调整纠偏系数来改善走直线的效果.
 * 0-100为可调值,50表示无纠偏效果.大于50小车主动往右偏,
 * 小于50小车主动往左偏.
 */
float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //左轮纠偏,对应50~100对应1.0~1.2倍的纠偏系数
	{
		if( diffparam>50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //右轮纠偏,50~0对应1.0~1.2倍的纠偏系数
	{
		if( diffparam<50 )
			return 1.0f + 0.004f*(50-diffparam);
	}

	return 1.0f;//不满足条件时,默认是1.
}

static void _4ws_SpeedSmooth(RobotControlCMDType_t* smoothcmd,const RobotControlCMDType_t* cmdtarget,uint16_t RunFreq)
{
	#define ACCEL_START_X 1.0f  //启动(加速)时最大线加速度 m/s2
	#define ACCEL_BRAKE_X 1.5f  //停车(减速)时最大线加速度 m/s2
	#define ACCEL_START_Y 1.0f  //启动(加速)时最大线加速度 m/s2
	#define ACCEL_BRAKE_Y 1.5f  //停车(减速)时最大线加速度 m/s2
	#define ACCEL_START_W 5.0f  //自转启动角加速度 rad/s2
	#define ACCEL_BRAKE_W 7.0f  //自转刹车角加速度

	//载人参数
//	#define ACCEL_START_X 0.5f  //启动(加速)时最大线加速度 m/s2
//	#define ACCEL_BRAKE_X 0.5f  //停车(减速)时最大线加速度 m/s2
//	#define ACCEL_START_Y 0.5f  //启动(加速)时最大线加速度 m/s2
//	#define ACCEL_BRAKE_Y 0.5f  //停车(减速)时最大线加速度 m/s2
//	#define ACCEL_START_W 1.5f  //自转启动角加速度 rad/s2
//	#define ACCEL_BRAKE_W 1.5f  //自转刹车角加速度

	_4ws_SpeedSmoothLimit(smoothcmd,cmdtarget,RunFreq,
		ACCEL_START_X,ACCEL_BRAKE_X,ACCEL_START_Y,ACCEL_BRAKE_Y,ACCEL_START_W,ACCEL_BRAKE_W);
}

static void _4ws_SpeedSmoothFastBrake(RobotControlCMDType_t* smoothcmd,const RobotControlCMDType_t* cmdtarget,uint16_t RunFreq)
{
	#define FAST_ACCEL_START_X 1.0f   //快速减速模式下仍使用正常启动加速度
	#define FAST_BRAKE_X       5.0f   //转向保护等待时快速线减速度 m/s2
	#define FAST_ACCEL_START_Y 1.0f   //快速减速模式下仍使用正常启动加速度
	#define FAST_BRAKE_Y       5.0f   //转向保护等待时快速线减速度 m/s2
	#define FAST_ACCEL_START_W 5.0f   //快速减速模式下仍使用正常自转启动角加速度
	#define FAST_BRAKE_W       15.0f  //转向保护等待时快速自转减速度 rad/s2

	//仅在转向保护等待阶段调用,避免影响正常驾驶手感
	_4ws_SpeedSmoothLimit(smoothcmd,cmdtarget,RunFreq,
		FAST_ACCEL_START_X,FAST_BRAKE_X,FAST_ACCEL_START_Y,FAST_BRAKE_Y,FAST_ACCEL_START_W,FAST_BRAKE_W);
}

static void _4ws_SpeedSmoothLimit(RobotControlCMDType_t* smoothcmd,const RobotControlCMDType_t* cmdtarget,uint16_t RunFreq,
	float accel_start_x,float accel_brake_x,float accel_start_y,float accel_brake_y,float accel_start_w,float accel_brake_w)
{
	//计算速度增量
	float dvx = cmdtarget->Vx - smoothcmd->Vx;
	float dvy = cmdtarget->Vy - smoothcmd->Vy;
	float dvz = cmdtarget->Vz - smoothcmd->Vz;

	//当前控制周期
	const float dt = 1.0f/(float)RunFreq;

	//XY平面速度按向量整体限加速度,避免斜向运动时合成加速度超限
	float dxy_len = sqrtf(dvx * dvx + dvy * dvy);
	float current_dot_delta = smoothcmd->Vx * dvx + smoothcmd->Vy * dvy;
	if (dxy_len > 0.00001f)
	{
		//沿变化方向做椭圆合成,X/Y限值不同时斜向加速度限值仍准确
		float accel_x = (current_dot_delta >= 0.0f) ? accel_start_x : accel_brake_x;
		float accel_y = (current_dot_delta >= 0.0f) ? accel_start_y : accel_brake_y;
		float ux = dvx / dxy_len;
		float uy = dvy / dxy_len;
		float accel_xy_limit = 1.0f / sqrtf((ux*ux)/(accel_x*accel_x) + (uy*uy)/(accel_y*accel_y));
		float max_dxy = accel_xy_limit * dt;
		if (dxy_len > max_dxy)
		{
			float scale = max_dxy / dxy_len;
			dvx *= scale;
			dvy *= scale;
		}
	}

	//对Z轴的加减速判断
	float max_dw;
	if( dvz * smoothcmd->Vz >= 0.0f )
	{
		max_dw = accel_start_w * dt;
	}
	else
	{
		//反向指令:先按brake减速到0;若本帧内过零,剩余时间按start反向加速
		float t_brake = fabsf(smoothcmd->Vz) / accel_brake_w;
		if( t_brake >= dt ) max_dw = accel_brake_w * dt;
		else max_dw = fabsf(smoothcmd->Vz) + accel_start_w * (dt - t_brake);
	}

	//检查是否超出平滑加速度限制值
	if (fabsf(dvz) > max_dw + 0.00001f)
	{
		dvz = (dvz > 0.0f) ? max_dw : -max_dw;
	}

	//速度平滑
	smoothcmd->Vx += dvx;
	smoothcmd->Vy += dvy;
	smoothcmd->Vz += dvz;

	//死区清零,防止遥杆零飘导致车子轻微抖动
//	if (fabsf(smoothcmd->Vx) < 0.008f) smoothcmd->Vx = 0.0f;
//	if (fabsf(smoothcmd->Vy) < 0.008f) smoothcmd->Vy = 0.0f;
//	if (fabsf(smoothcmd->Vz) < 0.005f) smoothcmd->Vz = 0.0f;
}

/**
 * @brief  此函数执行结合IMU数据走直线功能
 * @param  小车目标速度值
 * @retval 无
 *
 * @details
 *  IMU和小车控制方向:逆时针旋转时,yaw为正,yaw速度为正. Vz为正时小车逆时针旋转
 */
static void Linear_IMUHelp(RobotControlCMDType_t* cmdtarget)
{
	//用于延迟获取目标角度
	static uint8_t TargetCnt = 0;
	#define DelayCNT 70

	//目标角度
	static float target_angle = 0;

	//PID参数
	float linear_helpkp = 0.08f;
	float linear_helpkd = 0.05f;

	switch( RobotHardWareParam.CarType )
	{
		//不同车型在直线调节时PID参数不相同,与车身重量,车身结构,以及电机都有关系
		case S100: linear_helpkp = 0.03f; linear_helpkd = 0.007f; break;
		case S300Mini: linear_helpkp = 0.08f; linear_helpkd = 0.01f; break;
		case S300: linear_helpkp = 0.06f; linear_helpkd = 0.015f; break;
		case T280: linear_helpkp = 0.6f; linear_helpkd = 0.15f; break;
		default:break;
	}

	static uint8_t giveup_flag = 0;

	//仅在Vx有速度时补偿,其余情况不补偿并更新目标角度
	if( (cmdtarget->Vx!=0 || cmdtarget->Vy!=0) && cmdtarget->Vz==0 && giveup_flag==0 )
	{
		//延迟获得目标角度
		TargetCnt++;
		if( TargetCnt==DelayCNT )  target_angle = AttitudeVal.yaw;

		//获取到目标角度后再执行PID控制
		if( TargetCnt >= DelayCNT )
		{
			//锁住计数值防止循环
			TargetCnt=DelayCNT+1;

			//误差计算
			float error = target_angle-AttitudeVal.yaw;

			//对误差角度归一化
			if( error >  180.0f ) error -= 360.0f;
			if( error < -180.0f ) error += 360.0f;

			//当偏差过大时,放弃调整(如悬空控制).
			if( fabs(error)> 30 )
			{
				giveup_flag = 1;
				LinearHelp_Target.Vz=0;
				return;
			}

			//PID调整小车走直线效果(仅超过一定速度才调整)
			static uint8_t AllowFilterCnt;
			if( fabs(RobotControlParam.feedbackVx)>=0.05f || fabs(RobotControlParam.feedbackVy)>=0.05f )
			{
				AllowFilterCnt++;
				if( AllowFilterCnt>80 )
				{
					AllowFilterCnt=81;
					LinearHelp_Target.Vz = linear_helpkp * error - linear_helpkd * axis_9Val.gyro.z*57.3f;
				}
				else LinearHelp_Target.Vz=0;
			}
			else
			{
				LinearHelp_Target.Vz=0;
				AllowFilterCnt=0;
			}

			//限制调整值
			LinearHelp_Target.Vz = target_limit_float(LinearHelp_Target.Vz,-1.57f,1.57f);
		}

	}
	else if( cmdtarget->Vx==0 && cmdtarget->Vy==0 && cmdtarget->Vz==0 )
	{
		//速度全0时静止一段时间则刷新目标角度
		TargetCnt++;
		if( TargetCnt>100 )  target_angle = AttitudeVal.yaw,TargetCnt=101;
		LinearHelp_Target.Vx = 0;
		LinearHelp_Target.Vy = 0;
		LinearHelp_Target.Vz = 0;
		giveup_flag=0;
	}
	else
	{
		//其他情况则延迟目标角度的刷新,防止动态切换的过程目标角度刷新不准确
		TargetCnt=0;
		LinearHelp_Target.Vx = 0;
		LinearHelp_Target.Vy = 0;
		LinearHelp_Target.Vz = 0;
	}

}

float target_limit_float(float insert,float low,float high)
{
	if (insert < low)
		return low;
	else if (insert > high)
		return high;
	else
		return insert;
}

int target_limit_int(int insert,int low,int high)
{
	if (insert < low)
		return low;
	else if (insert > high)
		return high;
	else
		return insert;
}

float rpm_to_linearVel(float rpm,float wheelper)
{
	return rpm/60.0f*(wheelper);
}

//复位进入bootloader函数
void _System_Reset_FromAPP_RTOS(char uart_recv)
{
	static char res_buf[5];
	static char res_count=0;

	res_buf[res_count]=uart_recv;

	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;

	if(res_count==5)
	{
		res_count = 0;
		//接受到上位机请求的复位字符"reset",执行软件复位
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			RobotControlCMDType_t cmd = {
				.cmdsource = BootLoader,
				0,0,0
			};

			//写入最高优先级指令停止小车运动
			for(uint8_t i=0;i<10;i++)
			{
				WriteRobotControlQueue(&cmd,0);
				vTaskDelay( pdMS_TO_TICKS(10) );
			}

			//复位系统
			NVIC_SystemReset();
		}
	}
}

//调试模式
void RobotControl_SetDebugLevel(char uart_recv)
{
	//调试任务句柄
	extern TaskHandle_t g_reportErrTaskHandle;

	static char res_buf[4];
	static char res_count=0;

	res_buf[res_count]=uart_recv;

	if( uart_recv=='L'||res_count>0 )
		res_count++;
	else
		res_count = 0;

	if(res_count==4)
	{
		res_count = 0;

		if( res_buf[0]=='L'&&res_buf[1]=='O'&&res_buf[2]=='G')
		{
			switch( res_buf[3] )
			{
				case '0': RobotControlParam.DebugLevel = 0; break;
				case '1': RobotControlParam.DebugLevel = 1; break;
				case '2': RobotControlParam.DebugLevel = 2; break;
				case '3': RobotControlParam.DebugLevel = 3; break;
				default: RobotControlParam.DebugLevel = 0; break;
			}

			//操作提示音
			pBuzzerInterface_t tips = &UserBuzzer;
			tips->AddTask(1,200);

			//发送任务通知
			xTaskNotifyGive(g_reportErrTaskHandle);
		}
	}
}
