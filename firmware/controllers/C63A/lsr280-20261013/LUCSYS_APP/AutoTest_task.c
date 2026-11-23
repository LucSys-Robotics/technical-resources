/**
 * @file    AutoTest_task.c
 * @brief   2.4m x 2.4m 场地自动耐久测试任务
 * @date    2026-07-08
 *
 * @details
 * - 通过控制队列(WriteRobotControlQueue)以最低优先级发布预设动作序列,
 *   循环执行 直行/横移/倒退/斜移/原地旋转,轨迹为"正方形四边+两条对角线X".
 * - 简单闭环:
 *     平移段 = 航迹推算(里程计速度按IMU航向积分)路点导航 + IMU航向保持;
 *     旋转段 = IMU计圈;
 *     时间仅作为每段的兜底超时,超时强制切下一段.
 * - 脚本按回文编排,每条边/对角线正反各走一遍,系统性漂移首阶对消.
 * - 有人监督场景,不做虚拟围栏:位置漂出场地由人工搬回(测试继续跑).
 *
 * @note
 * - 启动: 手柄方向上键长按(记当前位置为原点,当前车头方向为+X)
 * - 停止: 手柄方向上键单击/双击;急停/失能自动停止;
 *         摇杆一动(高优先级源持续抢占队列)也会自动停止.
 */
#include "AutoTest_task.h"

//DATALOG_ENABLE=0(出货)时整个实现不参与编译,不占Flash也不会被创建
#if DATALOG_ENABLE

//HAL Lib Include File
#include "main.h"

//C Include File
#include <math.h>

//BSP Include File
#include "bsp_icm20948.h"
#include "bsp_buzzer.h"

//APP Include File
#include "RobotControl_task.h"
#include "robot_select_init.h"

//======== 测试参数(按需调整) ========
#define AT_RUN_FREQ_HZ     50       //任务运行频率
#define AT_EDGE_M          1.6f     //虚拟正方形边长(2.4m场地内四周各留0.4m裕量)
#define AT_SPEED_MPS       0.3f     //平移巡航速度
#define AT_MIN_SPEED_MPS   0.08f    //接近路点时的最低速度
#define AT_DECEL_DIST_M    0.35f    //开始线性减速的剩余距离
#define AT_ARRIVE_DIST_M   0.05f    //到点判定半径
#define AT_ROT_SPEED_RAD   0.4f     //旋转段角速度 rad/s
#define AT_ROT_TURNS       2.0f     //每个旋转段圈数
#define AT_PAUSE_MS        800      //段间停顿(等转向对准,同时反复考验转向保护逻辑)
#define AT_YAW_KP          0.05f    //平移段航向保持P增益 (rad/s)/度
#define AT_YAW_KD          0.02f    //平移段航向保持D增益(对角速度deg/s)
#define AT_YAW_VZ_MAX      0.3f     //航向修正角速度限幅 rad/s
#define AT_TIMEOUT_SCALE   1.5f     //段兜底超时 = 标称耗时 x 该系数 + 基数
#define AT_TIMEOUT_BASE_MS 500     //段兜底超时附加基数(ms)
#define AT_WRITE_FAIL_MS   500      //队列持续写入失败超过此时长,判定控制权被抢占,自动停止
#define AT_STOP_HOLD_MS    200      //停止时发零速收尾的时长

//段类型
typedef enum{
	SEG_MOVE = 0,   //平移到路点
	SEG_ROT,        //原地旋转N圈
}SegType_t;

//测试脚本单段:SEG_MOVE时p1/p2为路点坐标(m);SEG_ROT时p1为带符号圈数
typedef struct{
	uint8_t type;
	float p1;
	float p2;
}TestSeg_t;

//测试脚本:世界系以启动点为原点,启动时车头方向为+X,车体左侧为+Y
//四角: A(0,0) B(E,0) C(E,E) D(0,E)
//回文编排:每条边/对角线正反各走一遍
#define E_ AT_EDGE_M
static const TestSeg_t TestScript[] = {
	{ SEG_MOVE, E_,  0   },          //S1  A->B 前进直行
	{ SEG_MOVE, E_,  E_  },          //S2  B->C 左横移
	{ SEG_MOVE, 0,   E_  },          //S3  C->D 倒退
	{ SEG_MOVE, 0,   0   },          //S4  D->A 右横移
	{ SEG_MOVE, E_,  E_  },          //S5  A->C 斜移(前左45度)
	{ SEG_ROT,  +AT_ROT_TURNS, 0 },  //S6  原地旋转+2圈
	{ SEG_MOVE, 0,   0   },          //S7  C->A 斜移返回(后右)
	{ SEG_MOVE, 0,   E_  },          //S8  A->D 左横移
	{ SEG_MOVE, E_,  E_  },          //S9  D->C 前进直行
	{ SEG_MOVE, E_,  0   },          //S10 C->B 右横移
	{ SEG_MOVE, 0,   0   },          //S11 B->A 倒退
	{ SEG_MOVE, E_,  0   },          //S12 A->B 前进直行
	{ SEG_MOVE, 0,   E_  },          //S13 B->D 斜移(后左,另一条对角线)
	{ SEG_ROT,  -AT_ROT_TURNS, 0 },  //S14 原地旋转-2圈
	{ SEG_MOVE, E_,  0   },          //S15 D->B 斜移返回(前右)
	{ SEG_MOVE, 0,   0   },          //S16 B->A 倒退,回到起点
};
#undef E_
#define SEG_COUNT  ( sizeof(TestScript)/sizeof(TestScript[0]) )

//任务状态机
typedef enum{
	AT_IDLE = 0,     //待机,不写控制队列
	AT_RUN,          //执行当前段
	AT_SEG_PAUSE,    //段间停顿
	AT_STOPPING,     //发零速收尾后释放控制权
}ATState_t;

//启停请求(手柄按键回调置位,本任务消费)
static volatile uint8_t StartReq = 0;
static volatile uint8_t StopReq  = 0;

//运行状态(供调试/显示查看)
static volatile ATState_t at_state = AT_IDLE;
static uint16_t at_seg   = 0;    //当前段索引
static uint32_t at_cycle = 0;    //已完成的循环数

//航迹推算(世界系,原点=启动点,+X=启动时车头方向)
static float odo_x = 0, odo_y = 0;

//启动时的IMU航向(度).全程不刷新是刻意的:它同时充当测试世界系的基准方向与航向
//保持的目标,一旦中途重采,世界系会跟着IMU漂移一起转,航迹推算的坐标随之失去意义。
static float yaw0  = 0;

//诊断量,供数据观察任务读取,详见 AutoTest_task.h
volatile AutoTestDbg_t g_AutoTestDbg = { 0 };

void AutoTest_DbgWindowReset(void)
{
	g_AutoTestDbg.vzMin   = g_AutoTestDbg.vzLast;
	g_AutoTestDbg.vzMax   = g_AutoTestDbg.vzLast;
	g_AutoTestDbg.flipCnt = 0;
}

//每个控制周期调用一次,累积本窗口的Vz包络与符号翻转次数
static void AutoTest_DbgUpdate(float vz)
{
	float prev = g_AutoTestDbg.vzLast;

	if( vz < g_AutoTestDbg.vzMin ) g_AutoTestDbg.vzMin = vz;
	if( vz > g_AutoTestDbg.vzMax ) g_AutoTestDbg.vzMax = vz;

	//两拍都不接近零才算一次翻转,避免零附近的噪声被计入
	if( vz*prev < 0.0f && fabsf(vz) > 0.02f && fabsf(prev) > 0.02f )
	{
		if( g_AutoTestDbg.flipCnt < 255 ) g_AutoTestDbg.flipCnt++;
	}

	g_AutoTestDbg.vzLast = vz;
}

//角度归一化到(-180,180]
static float at_wrap180(float a)
{
	while( a >  180.0f ) a -= 360.0f;
	while( a < -180.0f ) a += 360.0f;
	return a;
}

static float at_clampf(float v,float lo,float hi)
{
	if( v < lo ) return lo;
	if( v > hi ) return hi;
	return v;
}

//计算某段的兜底超时时间(ms):标称耗时x系数+基数,时间仅兜底不作主判据
static uint32_t at_seg_timeout_ms(const TestSeg_t* seg)
{
	float nominal_s;
	if( seg->type == SEG_MOVE )
	{
		float dx = seg->p1 - odo_x;
		float dy = seg->p2 - odo_y;
		nominal_s = sqrtf(dx*dx + dy*dy) / AT_SPEED_MPS;
	}
	else
	{
		nominal_s = fabsf(seg->p1) * 2.0f * 3.14159f / AT_ROT_SPEED_RAD;
	}
	return (uint32_t)(nominal_s * AT_TIMEOUT_SCALE * 1000.0f) + AT_TIMEOUT_BASE_MS;
}

void AutoTest_Start(void){ StartReq = 1; }
void AutoTest_Stop(void) { StopReq  = 1; }
uint8_t AutoTest_IsRunning(void){ return (at_state != AT_IDLE); }

void AutoTest_task(void* param)
{
	const float dt = 1.0f/(float)AT_RUN_FREQ_HZ;
	pBuzzerInterface_t buzzer = &UserBuzzer;

	//段执行过程量
	TickType_t segStart   = 0;   //当前段开始时刻
	TickType_t pauseStart = 0;   //停顿开始时刻
	TickType_t stopStart  = 0;   //收尾开始时刻
	uint32_t   segTimeoutMs = 0; //当前段兜底超时
	float      rotAcc  = 0;      //旋转段累计角度(度)
	float      lastYaw = 0;      //上周期航向,用于计圈
	TickType_t writeFailStart = 0; //队列首次写失败时刻,0表示写入正常

	//等系统与IMU稳定
	vTaskDelay(3000);
	TickType_t preTime = xTaskGetTickCount();

	while(1)
	{
		RobotControlCMDType_t cmd = { 0 };
		cmd.cmdsource = AutoTest_CMD;

		//---- 停止请求 ----
		if( StopReq )
		{
			StopReq = 0;
			if( at_state==AT_RUN || at_state==AT_SEG_PAUSE )
			{
				at_state = AT_STOPPING;
				stopStart = xTaskGetTickCount();
			}
		}

		//---- 启动请求 ----
		if( StartReq )
		{
			StartReq = 0;
			if( at_state == AT_IDLE )
			{
				if( RobotControlParam.en_flag == 1 )
				{
					//以当前位置为原点,当前车头方向为+X建立测试坐标系
					odo_x = 0; odo_y = 0;
					yaw0 = AttitudeVal.yaw;
					lastYaw = AttitudeVal.yaw;
					at_seg = 0; at_cycle = 0;
					rotAcc = 0;
					writeFailStart = 0;
					segStart = xTaskGetTickCount();
					segTimeoutMs = at_seg_timeout_ms(&TestScript[0]);
					at_state = AT_RUN;
					buzzer->AddTask(1,300); //启动提示:1声
				}
				else
				{
					buzzer->AddTask(3,100); //小车失能,拒绝启动:3短声
				}
			}
		}

		//---- 航迹推算(测试运行期间) ----
		if( at_state==AT_RUN || at_state==AT_SEG_PAUSE )
		{
			float psi = at_wrap180(AttitudeVal.yaw - yaw0) * 0.0174533f;
			float vx = RobotControlParam.feedbackVx;
			float vy = RobotControlParam.feedbackVy;
			odo_x += ( vx*cosf(psi) - vy*sinf(psi) ) * dt;
			odo_y += ( vx*sinf(psi) + vy*cosf(psi) ) * dt;
		}

		//---- 安全:急停/失能时立即结束测试 ----
		if( (at_state==AT_RUN || at_state==AT_SEG_PAUSE) && RobotControlParam.en_flag==0 )
		{
			at_state = AT_IDLE;
			buzzer->AddTask(2,200); //停止提示:2声
		}

		//---- 状态机 ----
		switch( at_state )
		{
			case AT_RUN:
			{
				const TestSeg_t* seg = &TestScript[at_seg];
				uint8_t arrived = 0;

				if( seg->type == SEG_MOVE )
				{
					float dx = seg->p1 - odo_x;
					float dy = seg->p2 - odo_y;
					float dist = sqrtf(dx*dx + dy*dy);

					if( dist < AT_ARRIVE_DIST_M )
					{
						arrived = 1;
					}
					else
					{
						//接近路点线性减速,防过冲
						float v = AT_SPEED_MPS;
						if( dist < AT_DECEL_DIST_M )
						{
							v = AT_SPEED_MPS * (dist/AT_DECEL_DIST_M);
							if( v < AT_MIN_SPEED_MPS ) v = AT_MIN_SPEED_MPS;
						}

						//世界系速度指向路点(位置闭环),再按航向旋转到车体系
						float vwx = v * dx/dist;
						float vwy = v * dy/dist;
						float psi = at_wrap180(AttitudeVal.yaw - yaw0) * 0.0174533f;
						cmd.Vx =  cosf(psi)*vwx + sinf(psi)*vwy;
						cmd.Vy = -sinf(psi)*vwx + cosf(psi)*vwy;

						//航向保持(PD,符号约定与Linear_IMUHelp一致):车头锁定启动方向,
						//保证倒退段真在倒退/横移段真在横移
						float yaw_err = at_wrap180(yaw0 - AttitudeVal.yaw);
						float vz = AT_YAW_KP*yaw_err - AT_YAW_KD*axis_9Val.gyro.z*57.3f;
						cmd.Vz = at_clampf(vz,-AT_YAW_VZ_MAX,AT_YAW_VZ_MAX);

						//诊断:记下限幅前的原始输出与PD输入,用于判断振荡来自哪一环
						g_AutoTestDbg.yawErr = yaw_err;
						g_AutoTestDbg.vzRaw  = vz;
					}
				}
				else //SEG_ROT:原地旋转,IMU计圈
				{
					float dyaw = at_wrap180(AttitudeVal.yaw - lastYaw);
					rotAcc += dyaw;

					if( fabsf(rotAcc) >= fabsf(seg->p1)*360.0f )
						arrived = 1;
					else
						cmd.Vz = (seg->p1 >= 0) ? AT_ROT_SPEED_RAD : -AT_ROT_SPEED_RAD;
				}

				//兜底超时:强制切下一段,3短声提醒观察者
				if( !arrived && xTaskGetTickCount()-segStart > pdMS_TO_TICKS(segTimeoutMs) )
				{
					arrived = 1;
					buzzer->AddTask(3,100);
				}

				if( arrived )
				{
					at_state = AT_SEG_PAUSE;
					pauseStart = xTaskGetTickCount();
					//cmd保持零速,进入停顿
					cmd.Vx = 0; cmd.Vy = 0; cmd.Vz = 0;
				}
				break;
			}

			case AT_SEG_PAUSE:
			{
				//段间停顿发零速,等转向对准
				if( xTaskGetTickCount()-pauseStart >= pdMS_TO_TICKS(AT_PAUSE_MS) )
				{
					at_seg++;
					if( at_seg >= SEG_COUNT )
					{
						at_seg = 0;
						at_cycle++;
					}
					rotAcc = 0;
					segStart = xTaskGetTickCount();
					segTimeoutMs = at_seg_timeout_ms(&TestScript[at_seg]);
					at_state = AT_RUN;
				}
				break;
			}

			case AT_STOPPING:
			{
				//发零速收尾,之后释放控制权(2s后低优先级源可接管)
				if( xTaskGetTickCount()-stopStart >= pdMS_TO_TICKS(AT_STOP_HOLD_MS) )
				{
					at_state = AT_IDLE;
					buzzer->AddTask(2,200); //停止提示:2声
				}
				break;
			}

			case AT_IDLE:
			default:
				break;
		}

		//---- 控制指令下发与抢占检测 ----
		if( at_state != AT_IDLE )
		{
			if( WriteRobotControlQueue(&cmd,0) != 0 )
			{
				//写入失败=更高优先级控制源正在活跃(如手柄摇杆)
				if( writeFailStart == 0 )
				{
					writeFailStart = xTaskGetTickCount();
				}
				else if( xTaskGetTickCount()-writeFailStart > pdMS_TO_TICKS(AT_WRITE_FAIL_MS) )
				{
					//控制权被持续抢占,自动结束测试
					at_state = AT_IDLE;
					buzzer->AddTask(2,200);
				}
			}
			else
			{
				writeFailStart = 0;
			}
		}

		//诊断量每周期更新,窗口由日志任务读走后清零
		g_AutoTestDbg.yaw0  = yaw0;
		g_AutoTestDbg.seg   = (uint8_t)at_seg;
		g_AutoTestDbg.state = (uint8_t)at_state;
		AutoTest_DbgUpdate(cmd.Vz);

		//计圈基准每周期更新
		lastYaw = AttitudeVal.yaw;

		vTaskDelayUntil(&preTime, pdMS_TO_TICKS(1000/AT_RUN_FREQ_HZ));
	}
}

#endif //DATALOG_ENABLE
