/**
 * @file    RobotControl_task.c
 * @brief Cart control logic file.
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - This file is the core content of the chassis's operation, RobotControl_task by the task, receiving various remote or serial ports from the control queue
 *   Velocity command data: use kinematics to calculate each wheel's rotational speed, then transmit the speeds
 *   Command to the driver to complete the chassis's drive
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

//Used to store the control queue data of the car
QueueHandle_t  g_xQueueRobotControl = NULL;

//Used to record the current control source of the car
volatile uint8_t RobotControl_CMDsource = NONE_CMD;

//Internal use function
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

//Persistent steering-homing failure flag: setting it disables the chassis; retry homing through error recovery (DriveErrRecovery)
static uint8_t SteeringHomeFailed = 0;

//4-wheel steering protection parameters
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
#define LSR280_MAX_DRIVE_RPM         1000.0f
#define ENCODER_COUNTS_PER_REV     16384.0f  //Steering motor encoder counts per revolution
#define COUNTS_PER_DEG             (ENCODER_COUNTS_PER_REV/360.0f)
#define HOMING_TIMEOUT_MS          5000      //Power-on zero timeout (ms)
#define HOMING_RESEND_MS           500       //Resend Cycle (ms) for Zeroback Instructions
#define HOMING_TOL_COUNTS          15.0f     //Zero-to-Position Counts
#define STARTUP_STOP_EPS           0.01f     //Stationary check: all three s_drive.smooth axes below this value means fully stationary
#define STARTUP_ALIGN_TOL_DEG      5.0f      //Startup alignment tolerance (degrees); allow speed only when wheel error is below this value
//Large redirection threshold: 45° corresponds to counts (45/360*16384); exceeding this is considered tangent rotation/lateral shift
#define REORIENT_JUMP_COUNTS       (45.0f*COUNTS_PER_DEG)
//Steering command smoothing: upstream (ROS/CAN/handle) speed jitter will be reverse-relieved by major steering angle jumps,
//At low speeds, the intensity is most intense (Vx=0.06/Vz=0.3, corresponding to a turning radius of 0.2m, wheels close to ±90 degrees), so limit the width first and then send the reverse solution.
//
//ACC_W is the only strength knob in this smoothing system, with values squeezed by both ends:
//  Adjust down -> controller response becomes sluggish. At 20, full Vz=1.294 rad/s reaches in 65ms; dropping to 5 takes 259ms, noticeable in real car
//  Adjust up -> reduces ability to filter high-frequency noise
//Tests show that this smoothing suppresses steering jitter only (attenuation by only about 1%), and suppresses jitter through the following time throttling and
//deceleration hold gate intent criteria. This smooth positioning serves as a safety net against upstream abnormal noise (such as customer navigation stack output).
#define STEER_ACC_XY               2.0f      //Steering instruction XY line acceleration limit (m/s²), drive start set at 1.0
#define STEER_ACC_W                20.0f     //Steering instruction Z-angle acceleration limit (rad/s2), drive start set at 5.0
//Lower limit issuance gate: only used to avoid invalid issuance with zero change, not responsible for flow limitation (rate limiting is handled by time conservation).
//Too large a threshold causes permanent quantization error; with 2 degrees, steady-state error falls within 2~5 degrees in 20% of cases.
#define STEER_SEND_EPS_COUNTS      (0.5f*COUNTS_PER_DEG)
//Minimum resend interval (ms). The steering driver has a built-in position ring; when the target is received, it will automatically move to its position. Refreshing every 10ms is a waste.
//This is a hard upper bound on CAN load: a position threshold cannot suppress peaks (up to 206 transmissions/second); only time-based throttling provides a definite bound.
#define STEER_SEND_MIN_INTERVAL_MS 25

//deceleration hold gate threshold. Criteria are intent_jump (intention target calculated by unsmoothed instructions vs. current actual wheel angle),
//Decoupling from Steering Smoothing — If a single-cycle hop variable of the smoothed target is used, the gradient target will not reach the threshold,
//Protection will silently fail (tested trigger rate drops from 1.44 times/min to 0.23 times/min).
#define REORIENT_HOLD_TIMEOUT_MS   500       //Maximum redirection deceleration wait (ms); force release at timeout to prevent lockup

/**
 * Wheel Numbering — a unified subscript for the four wheel isomorphisms
 *
 * Physical position when the front of the car faces forward:
 *     B (front left) C (front right)
 *     A (left rear) D (right rear)
 * That is, A/B are the two left wheels, C/D are the right wheels—one steering drive tube is one side, not one shaft.
 *
 * Corresponds one-to-one with the inverse output TargetMotorA~D. All four-wheel arrays in this file
 * (target/speed/progress/lastSent/rpm, etc.) all use this set of subscripts, without writing bare numbers.
 *
 * Hardware binding: A/B is driven by Driver1 (NodeID=1), C/D is driven by Driver2 (NodeID=2),
 * For each drive, channel 1 connects to the front, channel 2 to the latter. Right side (C/D) mirror mounting of the steering motor,
 * Zero position differs from the left side by 180 degrees; add 180 to C/D before converting to motor position.
 */
typedef enum{
	WHEEL_A = 0,
	WHEEL_B,
	WHEEL_C,
	WHEEL_D,
	WHEEL_COUNT
}WheelIndex_t;

/**
 * Steering controller status
 *
 * All four rounds of isomorphism are used as arrays, with subscripts shown in WheelIndex_t.
 * The instance is stored statically and globally unique; when a full reset is needed, the structure is reset to zero.
 */
typedef struct{
	float      target[WHEEL_COUNT];        //Calculated target positions (counts)
	uint16_t   speed[WHEEL_COUNT];         //Steering speeds allocated for simultaneous arrival
	float      lastSent[WHEEL_COUNT];      //The last actual target issued, used to issue cost-cutting criteria
	int32_t    progress[WHEEL_COUNT];      //Remaining deviations (counts) of targets for each track width in the current cycle
	int32_t    lastProgress[WHEEL_COUNT];  //Watchdog is a snapshot, refreshing every 100ms
	int32_t    maxProgress;                //Maximum deviation among four wheels
	float      scale;                      //Speed limit ratio applied to instructions in this cycle (0 = Stop issuance)
	uint8_t    reorientHold;               //Redirection deceleration hold gate
	uint8_t    lowfreCnt;                  //Watchdog frequency counting
	uint8_t    lastEnFlag;                 //Enable edge detection
	TickType_t reorientHoldStart;          //Start time of the deceleration hold gate, used to determine deceleration timeout
	TickType_t lastSendTick;               //The last actual issuance time, used for the minimum interval throttling
}SteerCtrl_t;

/** Drive controller status */
typedef struct{
	RobotControlCMDType_t smooth;            //Actual execution speed after smoothing the speed limit
	float                 rpm[WHEEL_COUNT];  //Four-wheel target speed
}DriveCtrl_t;

//Turn to Down Export (must be declared after SteerCtrl_t definition)
static void Steer_SendDriver(SteerCtrl_t* s,ServoDriverHWType_t* drv,WheelIndex_t i0,WheelIndex_t i1);
static void Steer_Send(SteerCtrl_t* s);

#if DATALOG_ENABLE
//set_MotorPos Cumulative number of calls. The difference between two consecutive samples is the turn-down rate, used to quantify CAN load.
//The issuance has converged to Steer_SendDriver single outlet, so one autoincrement here covers the entire disbursement path.
static uint32_t s_steerSendCnt = 0;
#endif

/**
 * Controller persistence state
 *
 * The main loop duty functions share these items. Only visible in this file, not exposed externally.
 */
static SteerCtrl_t           s_steer;     //Steering controller status
static DriveCtrl_t           s_drive;     //Drive controller status
static RobotParam_t          s_car;       //Four-wheel steering car model (shared with reverse solution/correct solution/projection)
static uint8_t s_steerSendFault = 0; // One fault bit per steering driver.
static RobotParam_t          s_sentCar; // Wheel-speed projection uses successfully sent steering targets.
static RobotParam_t          s_rawCar;    //Dedicated instance for the deceleration hold gate criterion; compute intended targets from unsmoothed commands
static RobotControlCMDType_t s_steerCmd;  //Dedicated steering calculation command after limiting and smoothing the raw command
static RobotControlCMDType_t s_lastCmd;   //The command from the last turn recalculation triggered for change detection
static const RobotControlCMDType_t s_stopCmd = {0,0,0,0};

//IMU assists the cart in a straight line
static RobotControlCMDType_t LinearHelp_Target = { 0 };

//Main loop duty function. RobotControl_task calls in this order one by one, implemented in the latter half of this file.
//Do not reorder: drive smoothing reads steering-calculation results and also releases the deceleration hold gate,
//The steering watchdog operates only when the deceleration hold gate is inactive.
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
 * Log snapshot — copies the controller's internal state all at once to the data observation task
 *
 * s_steer/s_drive etc. are private to this file and cannot be seen in log tasks. Copy the entire file once at the end of each cycle,
 * Prevent the next cycle from overwriting individual fields while the logging task reads them, which would produce a mixed old/new snapshot.
 * Only compiled when DATALOG_ENABLE=1, shipped firmware generates no code.
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

//Control function for four-wheel steering
void RobotControl_task(void* param)
{
	vTaskDelay(2000);

	//Create control command queue
	g_xQueueRobotControl = xQueueCreate(1,sizeof(RobotControlCMDType_t));

	//Obtain the timebase, which can run at a fixed frequency for auxiliary tasks
	TickType_t preTime = xTaskGetTickCount();

	//The control frequency for this task, in Hz
	const uint16_t TaskFreq = ROBOT_CONTROL_FREQ_HZ;

	//Servo motor self-check variables
	uint8_t driverCheck = 0;

	//Idle count
	TickType_t ActiveTick = xTaskGetTickCount();

	//Empty-queue statistics
	uint8_t LostCmdCounts = 0;

	//Queued control command
	RobotControlCMDType_t cmd = { 0 };

	//Initial power-on value. Controller status (s_steer/s_drive/s_car, etc.) is static storage and has been reset by the startup code,
	//Set only two nonzero initial values here: chassis mechanical parameters and default steering speed.
	FourWSCar_param_init(&s_car,RobotHardWareParam.WheelSpacing,RobotHardWareParam.AxleSpacing,0,RobotHardWareParam.WheelPerimeter);
	s_sentCar = s_car;
	for(uint8_t si=0;si<WHEEL_COUNT;si++) s_steer.speed[si]=LSR280_WheelTurnSpeed;

	//Wait for all steering motors to reset
	if( SteeringHoming(HOMING_TIMEOUT_MS)!=0 )
	{
		//Homing timeout: set persistent error flag and disable the chassis; retry through error recovery (long press B)
		SteeringHomeFailed = 1;
	}
	vTaskDelay(200);

	while(1)
	{
		//Obtain control commands: queue read + IMU linear compensation
		Robot_CmdReceive(&cmd,&LostCmdCounts);

		//Voltage update
		Robot_UpdateVoltageEstimate(&cmd);
		
		//Cart enable status check, determining whether to start the chassis based on self-test parameters
		robot_en_check();

		//Steering path: calculate targets -> smooth drive commands (also release the deceleration hold gate) -> run watchdog
		Steer_Solve(&cmd,TaskFreq);
		Drive_Update(&cmd,TaskFreq);
		Steer_Watchdog();

		//Drive path: transmit wheel speeds -> populate three-axis velocity using forward kinematics
		Drive_Output();
		Robot_Odometry();

		//State machine: Active refresh -> Runaway protection -> Parking detection
		Robot_ActiveUpdate(&cmd,&ActiveTick);
		// Command expiry is handled before IMU correction in Robot_CmdReceive.
		Robot_ParkingEnter(&ActiveTick);

		//Low-frequency inspection: drive self-check, error recovery, low battery alert
		Robot_Housekeeping(&driverCheck,TaskFreq,&preTime);

		#if DATALOG_ENABLE
		//Snapshot of the current cycle status for data observation tasks
		Robot_LogSnapshot(&cmd);
		#endif

		/* Delay the specified frequency */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );

	}
}

//Check if the two control commands have changed
uint8_t cmd_is_change(const RobotControlCMDType_t* cmd1,const RobotControlCMDType_t* cmd2)
{
	uint8_t ischange = 0;

	//Added control dead zone to prevent minor jitters from the remote/host repeatedly triggering turn recalculations
	if( fabsf(cmd1->Vx - cmd2->Vx) > CMD_CHANGE_EPS_V  )
		ischange=1;

	if( fabsf(cmd1->Vy - cmd2->Vy) > CMD_CHANGE_EPS_V )
		ischange=1;

	if( fabsf(cmd1->Vz - cmd2->Vz) > CMD_CHANGE_EPS_W )
		ischange=1;

	return ischange;
}

//Calculate the towing speed ratio based on residual steering errors to avoid full speed towing when wheels are misaligned
static float _4ws_SteerScaleByProgress(int max_progress)
{
	float err_deg = (1.0f/COUNTS_PER_DEG) * (float)max_progress;
	if( err_deg <= TURN_FULL_SPEED_DEG ) return 1.0f;
	if( err_deg >= TURN_DRIVE_ALLOW_DEG ) return 0.0f;
	//smoothstep(S-curve): zero slope at both ends, smooths drive re-engagement after a turn
	float t = (TURN_DRIVE_ALLOW_DEG - err_deg) / (TURN_DRIVE_ALLOW_DEG - TURN_FULL_SPEED_DEG);
	return t * t * (3.0f - 2.0f * t);
}

//Four wheels have uniform proportional speed limits, maintaining chassis speed direction and rotation ratio so that single-wheel speed limits do not disrupt them
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

//Determine whether an effective motor control quantity still exists
static uint8_t _4ws_HasMoveCmd(const RobotControlCMDType_t* cmd)
{
	if( fabsf(cmd->Vx) > TURN_MOVE_CMD_EPS || fabsf(cmd->Vy) > TURN_MOVE_CMD_EPS || fabsf(cmd->Vz) > TURN_MOVE_CMD_EPS )
		return 1;
	return 0;
}

//Determine whether the current speed meets the conditions for wide steering protection
static uint8_t _4ws_TurnSpeedReady(void)
{
	if( fabsf(RobotControlParam.feedbackVx) < TURN_SAFE_VX && fabsf(RobotControlParam.feedbackVy) < TURN_SAFE_VY &&
		fabsf(RobotControlParam.feedbackVz) < TURN_SAFE_VZ )
		return 1;
	return 0;
}

/**
 * Send steering targets: low-level output
 *
 * The only place in the entire project where "which drive controls which two wheels" (Driver1->A/B, Driver2->C/D).
 * When changing driver number or adjusting wheel sequence, just change the call parameters here
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
 * Send steering targets for all four wheels and update tracking records
 *
 * Dispatch and posting (lastSent / lastSendTick) must occur in pairs; otherwise, throttling criteria will be inaccurate.
 * Call this function when transmission is needed; do not access these two fields directly elsewhere.
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
 * Implementation of main loop duty functions
 *
 * Each function does only one thing. The call order and its constraints are shown in the declaration block at the beginning of the file.
 * ========================================================================== */

/**
 * Receive instructions — queue reading and IMU linear compensation
 *
 * When the queue is empty, retain cmd from the previous cycle; lostCnt accumulates consecutive empty cycles,
 * Hand over to the safety protection to determine if timeout occurred.
 */
static void Robot_CmdReceive(RobotControlCMDType_t* cmd,uint8_t* lostCnt)
{
	//Read control instructions from the queue
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

	//Gyroscope straight-line compensation
	if( RobotControlParam.ImuAssistedFlag == 1 )
	{
		Linear_IMUHelp(cmd);

		//Straight-line Z-speed compensation
		float final_vz = cmd->Vz + LinearHelp_Target.Vz;
		cmd->Vz = final_vz;
	}
	else
	{
		LinearHelp_Target.Vx=0;LinearHelp_Target.Vy=0;LinearHelp_Target.Vz=0;
	}
}

/**
 * Steering calculation: compute four wheel steering targets from the velocity command and transmit as needed
 *
 * Fixed sequence: Limit smoothing -> Inverse target -> Equal arrival velocity distribution -> deceleration hold gate criterion -> Distribution -> Deviation statistics.
 * Goals are only recalculated when instructions change, but issuance is evaluated every cycle: a single issuance blocked by throttling must have a next opportunity,
 * Otherwise, the error will remain stuck in the dead zone between the threshold and the watchdog.
 */
static void Steer_Solve(const RobotControlCMDType_t* cmd,uint16_t taskFreq)
{
	//Limit and smooth steering input to suppress upstream jitter, independently of all gates
	_4ws_SpeedSmoothLimit(&s_steerCmd,cmd,taskFreq,
		STEER_ACC_XY,STEER_ACC_XY,STEER_ACC_XY,STEER_ACC_XY,STEER_ACC_W,STEER_ACC_W);

	//Inverse kinematics: obtain wheel linear speed in m/s
	//Command to change flag positions
	uint8_t changeFlag = 0;

	//Check if the speed has changed
	changeFlag = cmd_is_change(&s_steerCmd,&s_lastCmd);

	//Historical data is updated only after the dead zone is exceeded, allowing slow-changing control to accumulate and trigger recalculations
	if( changeFlag )
		memcpy(&s_lastCmd,&s_steerCmd,sizeof(RobotControlCMDType_t));

	if( changeFlag )
	{
		//Use unsmoothed raw instructions to calculate an 'intent target' for deceleration hold gate criterion only.
		//Smoothing only changes the execution process and should not alter the judgment of 'whether momentum needs to be discharged first.'
		//Inverse kinematics is a pure function (only reads RealMotor*.radian and hardware); the two instances do not interfere.
		s_rawCar = s_car;
		four_ws_InverseKinematics(cmd->Vx,cmd->Vy,cmd->Vz,&s_rawCar);

		//Intent Target (counts). CD motor mirror installation, zero point differs 180 degrees from AB, so C/D first fills 180 before rotating position
		float rawPos[WHEEL_COUNT];
		rawPos[WHEEL_A] = angle_to_pos(s_rawCar.TargetMotorA.radian*57.2958f);
		rawPos[WHEEL_B] = angle_to_pos(s_rawCar.TargetMotorB.radian*57.2958f);
		rawPos[WHEEL_C] = angle_to_pos(s_rawCar.TargetMotorC.radian*57.2958f+180);
		rawPos[WHEEL_D] = angle_to_pos(s_rawCar.TargetMotorD.radian*57.2958f+180);

		//Calculate the actual target to issue using the smoothed command
		four_ws_InverseKinematics(s_steerCmd.Vx,s_steerCmd.Vy,s_steerCmd.Vz,&s_car);

		//Rotate the angle by the radian and then convert to the position of the steering motor
		s_steer.target[WHEEL_A] = angle_to_pos(s_car.TargetMotorA.radian*57.2958f);
		s_steer.target[WHEEL_B] = angle_to_pos(s_car.TargetMotorB.radian*57.2958f);
		s_steer.target[WHEEL_C] = angle_to_pos(s_car.TargetMotorC.radian*57.2958f+180);
		s_steer.target[WHEEL_D] = angle_to_pos(s_car.TargetMotorD.radian*57.2958f+180);

		//Obtain the actual total path of all motors
		int total_path[WHEEL_COUNT];
		total_path[WHEEL_A] = fabsf(s_steer.target[WHEEL_A]-RobotControlParam.MotorA.ori_pos);
		total_path[WHEEL_B] = fabsf(s_steer.target[WHEEL_B]-RobotControlParam.MotorB.ori_pos);
		total_path[WHEEL_C] = fabsf(s_steer.target[WHEEL_C]-RobotControlParam.MotorC.ori_pos);
		total_path[WHEEL_D] = fabsf(s_steer.target[WHEEL_D]-RobotControlParam.MotorD.ori_pos);

		//Wheel with the largest path to get it
		int max_path = total_path[WHEEL_A];
		if( total_path[WHEEL_B] > max_path ) max_path = total_path[WHEEL_B];
		if( total_path[WHEEL_C] > max_path ) max_path = total_path[WHEEL_C];
		if( total_path[WHEEL_D] > max_path ) max_path = total_path[WHEEL_D];

		//The time taken for the farthest path at maximum speed is the total duration of this turn (S = V * t),
		//Assign speeds for the remaining channels by dividing their travel by this duration, with all four wheels reaching the position simultaneously.
		float tmpT = 0;
		if( max_path > 5 )
		{
			tmpT = (float)max_path/(float)LSR280_WheelTurnSpeed;
		}

		for(uint8_t i=0;i<WHEEL_COUNT;i++)
		{
			//If the stroke is too short (tmpT is 0), do not recalculate speed; just use the previous value
			if( tmpT!=0 ) s_steer.speed[i] = total_path[i]/tmpT;

			if( s_steer.speed[i] < 5 ) s_steer.speed[i] = 5;
			if( s_steer.speed[i] > LSR280_WheelTurnSpeed ) s_steer.speed[i] = LSR280_WheelTurnSpeed;
		}

		//deceleration hold gate criterion is only evaluated when recalculating targets (relying on rawPos calculated for the current cycle)
		if( RobotControlParam.en_flag==1 )
		{
			//The difference between the 'intended target' and the current actual wheel angle = how far the user expects the wheel to be shelved.
			//Using the original unsmoothed command, the magnitude calculated by the same action before and after smoothing is consistent, with no missed judgments due to smoothing.
			float intent_jump = fabsf(rawPos[WHEEL_A]-RobotControlParam.MotorA.ori_pos);
			if( fabsf(rawPos[WHEEL_B]-RobotControlParam.MotorB.ori_pos) > intent_jump ) intent_jump = fabsf(rawPos[WHEEL_B]-RobotControlParam.MotorB.ori_pos);
			if( fabsf(rawPos[WHEEL_C]-RobotControlParam.MotorC.ori_pos) > intent_jump ) intent_jump = fabsf(rawPos[WHEEL_C]-RobotControlParam.MotorC.ori_pos);
			if( fabsf(rawPos[WHEEL_D]-RobotControlParam.MotorD.ori_pos) > intent_jump ) intent_jump = fabsf(rawPos[WHEEL_D]-RobotControlParam.MotorD.ori_pos);

			//Large redirection and still moving the car body: slow down first, wait for the brake, release momentum, then let the wheels pass
			if( intent_jump > REORIENT_JUMP_COUNTS && _4ws_TurnSpeedReady()==0 )
			{
				if( s_steer.reorientHold==0 ){ s_steer.reorientHold=1; s_steer.reorientHoldStart=xTaskGetTickCount(); }
			}
		}
	}

	//Evaluate steering target transmission every cycle, independently of changeFlag.
	//If locked in a changeFlag, once a single issuance is blocked by time savings or thresholds, there will be no second chance
	//(the instruction is stable, changeFlag will no longer be true), the error will stop at the 2~5 degree dead zone:
	//The watchdog resends only above 5 degrees, while transmission is suppressed below the threshold below; the interval between them is not covered.
	if( RobotControlParam.en_flag==1 && s_steer.reorientHold==0 )
	{
		float send_jump = 0.0f;
		for(uint8_t i=0;i<WHEEL_COUNT;i++)
		{
			float d = fabsf(s_steer.target[i]-s_steer.lastSent[i]);
			if( d > send_jump ) send_jump = d;
		}

		//The target changes significantly and the last issuance has been long enough since the last issue. After sending send_jump, reset to zero and stop naturally, without continuously occupying the bus.
		if( (send_jump > STEER_SEND_EPS_COUNTS || s_steerSendFault) &&
		    (xTaskGetTickCount()-s_steer.lastSendTick) >= pdMS_TO_TICKS(STEER_SEND_MIN_INTERVAL_MS) )
		{
			Steer_Send(&s_steer);
		}
	}

	//Raw steering motor value: positive counterclockwise.
	//The remaining deviation is used simultaneously for start-up alignment brakes, continuous speed limiting, and enforcement of the watchdog at three points.
	s_steer.progress[WHEEL_A] = abs((int)s_steer.target[WHEEL_A]-RobotControlParam.MotorA.ori_pos);
	s_steer.progress[WHEEL_B] = abs((int)s_steer.target[WHEEL_B]-RobotControlParam.MotorB.ori_pos);
	s_steer.progress[WHEEL_C] = abs((int)s_steer.target[WHEEL_C]-RobotControlParam.MotorC.ori_pos);
	s_steer.progress[WHEEL_D] = abs((int)s_steer.target[WHEEL_D]-RobotControlParam.MotorD.ori_pos);

	//Find the maximum value
	s_steer.maxProgress = s_steer.progress[WHEEL_A];
	if( s_steer.progress[WHEEL_B] > s_steer.maxProgress ) s_steer.maxProgress = s_steer.progress[WHEEL_B];
	if( s_steer.progress[WHEEL_C] > s_steer.maxProgress ) s_steer.maxProgress = s_steer.progress[WHEEL_C];
	if( s_steer.progress[WHEEL_D] > s_steer.maxProgress ) s_steer.maxProgress = s_steer.progress[WHEEL_D];

	//The steering speed limit ratio actually applied to the instructions in this cycle (0 = stop command issued in this cycle), for data observation purposes
	s_steer.scale = 0.0f;

	//Actively reactivates the current turn target during enable recovery (0->1), no longer relying on watchdog side effects
	if( RobotControlParam.en_flag==1 && s_steer.lastEnFlag==0 && s_steer.reorientHold==0 )
	{
		Steer_Send(&s_steer);
	}
	s_steer.lastEnFlag = RobotControlParam.en_flag;
}

/**
 * Drive smoothing: determine the three-axis velocity actually applied in this cycle
 *
 * Three mutually exclusive paths: emergency brake for deceleration hold gate / Alignment waiting for start / Continuous speed limiting based on wheel angle deviation.
 * The release of the deceleration hold gate is completed within this function, while the Steer_Watchdog guard is conditioned by reorientHold==0,
 * Therefore, the call order must be Steer_Solve -> Drive_Update -> Steer_Watchdog.
 */
static void Drive_Update(const RobotControlCMDType_t* cmd,uint16_t taskFreq)
{
	//Steering logic
	if( s_steer.reorientHold )
	{
		//Redirection gate: when switching at high speed to rotation/lateral motion, first decelerate sharply in a straight line to dissipate momentum, then allow wheel steering to prevent slipping
		if( _4ws_TurnSpeedReady() )
		{
			s_steer.reorientHold = 0;
			if( RobotControlParam.en_flag==1 )
			{
				Steer_Send(&s_steer);
			}
		}
		//Braking hard at full force while waiting to release the previous momentum
		_4ws_SpeedSmoothFastBrake(&s_drive.smooth,&s_stopCmd,taskFreq);
		// A timeout is not evidence that the chassis has slowed down.
		if( s_steer.reorientHold && xTaskGetTickCount()-s_steer.reorientHoldStart > pdMS_TO_TICKS(REORIENT_HOLD_TIMEOUT_MS) )
			s_drive.smooth = s_stopCmd;
	}
	else if( RobotControlParam.en_flag==1 && _4ws_HasMoveCmd(cmd) )
	{
		//Startup alignment gate: after a fully stationary state, wait for wheel alignment (error<tolerance) before applying speed to eliminate startup jitter
		uint8_t was_static = ( fabsf(s_drive.smooth.Vx)<STARTUP_STOP_EPS && fabsf(s_drive.smooth.Vy)<STARTUP_STOP_EPS && fabsf(s_drive.smooth.Vz)<STARTUP_STOP_EPS );
		if( was_static && s_steer.maxProgress > COUNTS_PER_DEG * STARTUP_ALIGN_TOL_DEG )
		{
			//Wheels are not aligned yet; remain stationary until alignment is complete
			_4ws_SpeedSmooth(&s_drive.smooth,&s_stopCmd,taskFreq);
		}
		else
		{
			//Speed is continuously limited according to real-time wheel angle deviation: large deviations (servo not caught up) mean slow, alignment means fast, avoiding repeated braking around and spinning
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
 * Turn to executing the watchdog — Progress is matched every 100ms
 *
 * Occasional lost driver frames can leave a target halfway; detect error increments within a comparison window and resend.
 * Do not intervene while the deceleration hold gate is active; otherwise wheel steering would proceed against the sharp deceleration.
 */
static void Steer_Watchdog(void)
{
	//Check if the steering motor command is invalid
	if( s_steer.reorientHold==0 && RobotControlParam.en_flag==1 && RobotControlParam.ParkingMode==0 && s_steer.maxProgress > COUNTS_PER_DEG * 5.0f ) //Detection when the target is not reached
	{
		s_steer.lowfreCnt++;
		if( s_steer.lowfreCnt == 10 )
		{
			s_steer.lowfreCnt=0;
			//If any steering motor is detected not moving, resend the target command
			if( (s_steer.progress[WHEEL_A] > COUNTS_PER_DEG*5.0f && abs(s_steer.progress[WHEEL_A]-s_steer.lastProgress[WHEEL_A]) < 10) || (s_steer.progress[WHEEL_B] > COUNTS_PER_DEG*5.0f && abs(s_steer.progress[WHEEL_B]-s_steer.lastProgress[WHEEL_B]) < 10) )
			{
				Steer_SendDriver(&s_steer,&Driver1,WHEEL_A,WHEEL_B);
			}
			if((s_steer.progress[WHEEL_C] > COUNTS_PER_DEG*5.0f && abs(s_steer.progress[WHEEL_C]-s_steer.lastProgress[WHEEL_C]) < 10) || (s_steer.progress[WHEEL_D] > COUNTS_PER_DEG*5.0f && abs(s_steer.progress[WHEEL_D]-s_steer.lastProgress[WHEEL_D]) < 10) )
			{
				Steer_SendDriver(&s_steer,&Driver2,WHEEL_C,WHEEL_D);
			}

			//Snapshots are only refreshed each time you compare, ensuring the comparison is the increment of the full 100ms window
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
 * Wheel-speed output: project three-axis velocity onto four wheel speeds and transmit
 *
 * Projection along the already issued steering target, with wheel speed symbols naturally consistent with the turning angle. Overall normalization ensures single-wheel width
 * Preserve the motion ratios between all four wheels. Explicitly transmit 0 speed when disabled; do not rely on the driver stopping by itself.
 */
static void Drive_Output(void)
{
	//Calculates the current speed using smoothing values
	//Project wheel speeds along the transmitted target steering angles; signs and angles remain consistent without a second inverse solution
	four_ws_WheelSpeedProjection(s_drive.smooth.Vx,s_drive.smooth.Vy,s_drive.smooth.Vz,&s_sentCar);

	//Convert wheel speed in m/s to rpm
	s_drive.rpm[WHEEL_A] = linearVel_to_rpm(s_sentCar.TargetMotorA.speed,s_car.hardware.WheelPerimeter);
	s_drive.rpm[WHEEL_B] = linearVel_to_rpm(s_sentCar.TargetMotorB.speed,s_car.hardware.WheelPerimeter);
	s_drive.rpm[WHEEL_C] = linearVel_to_rpm(s_sentCar.TargetMotorC.speed,s_car.hardware.WheelPerimeter);
	s_drive.rpm[WHEEL_D] = linearVel_to_rpm(s_sentCar.TargetMotorD.speed,s_car.hardware.WheelPerimeter);
	//Normalize all wheel speeds together to prevent individual driver limits from distorting the motion ratios
	_4ws_LimitWheelRPM(s_drive.rpm,LSR280_MAX_DRIVE_RPM);

	if( RobotControlParam.en_flag==1 && s_steerSendFault==0 )
	{
		//Execute hub motor commands
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
 * Odometry forward kinematics: derive three-axis velocity from measured wheel angles and speeds
 *
 * The result is written back to RobotControlParam.feedbackV* for the host computer to report and read for IMU linear compensation.
 */
static void Robot_Odometry(void)
{
	//Encoder position wheel angle
	RobotControlParam.MotorA.feedbackRadian = pos_to_angle(RobotControlParam.MotorA.ori_pos);
	RobotControlParam.MotorB.feedbackRadian = pos_to_angle(RobotControlParam.MotorB.ori_pos);
	RobotControlParam.MotorC.feedbackRadian = pos_to_angle(RobotControlParam.MotorC.ori_pos)-180;//CD motor image installation, minus 180 degrees
	RobotControlParam.MotorD.feedbackRadian = pos_to_angle(RobotControlParam.MotorD.ori_pos)-180;

	//Assign actual data to the cart model to calculate the speed of the 3-axis system
	s_car.RealMotorA.radian = RobotControlParam.MotorA.feedbackRadian*0.0174533f;
	s_car.RealMotorB.radian = RobotControlParam.MotorB.feedbackRadian*0.0174533f;
	s_car.RealMotorC.radian = RobotControlParam.MotorC.feedbackRadian*0.0174533f;
	s_car.RealMotorD.radian = RobotControlParam.MotorD.feedbackRadian*0.0174533f;
	s_car.RealMotorA.speed = RobotControlParam.MotorA.feedbackSpeed;
	s_car.RealMotorB.speed = RobotControlParam.MotorB.feedbackSpeed;
	s_car.RealMotorC.speed = RobotControlParam.MotorC.feedbackSpeed;
	s_car.RealMotorD.speed = RobotControlParam.MotorD.feedbackSpeed;

	//Forward kinematics
	four_ws_kinematics(&s_car);

	//Output three-axis velocity
	RobotControlParam.feedbackVx = s_car.feedback_Vx;
	RobotControlParam.feedbackVy = s_car.feedback_Vy;
	RobotControlParam.feedbackVz = s_car.feedback_Vz;
}

/**
 * Active Refresh — Refreshes timestamps upon receiving a motion command and exits the park
 */
static void Robot_ActiveUpdate(const RobotControlCMDType_t* cmd,TickType_t* activeTick)
{
	//Determine whether there is currently an active control quantity
	if( _4ws_HasMoveCmd(cmd) )
	{
		(*activeTick) = xTaskGetTickCount();
		if( RobotControlParam.ParkingMode==1 )
		{
			RobotControlParam.ParkingMode=0;
			//Release parking mode
			Robot_DriverSetParkingMode(0);
		}
	}
}

/**
 * Command-loss protection: after command timeout, use the control source to decide whether to set velocity to zero
 *
 * lostCnt is capped here to prevent counter wraparound from intermittently disabling protection during prolonged command loss.
 */
static void Robot_FailSafe(RobotControlCMDType_t* cmd,uint8_t* lostCnt,uint16_t taskFreq)
{
	//Car loss of control protection settings (queue without data timeout)
	if( (*lostCnt) > taskFreq )
	{
		(*lostCnt)=taskFreq;

		switch( RobotControl_CMDsource )
		{
			//Non-host control types require protection by default
			//This protection would also apply to app control, but cannot be enabled for compatibility with the iOS app, which sends data intermittently.
			case GamePad_CMD:case RCJOY_CMD: case Charger_CMD: case AutoTest_CMD:
				cmd->Vx=0;cmd->Vy=0;cmd->Vz=0;
				break;

			//Host type, protection enabled according to user-set safety levels
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
 * Parking decision: enter power-saving parking after 10 seconds stationary with the chassis level
 *
 * Do not stop on the slope (parking releases the holding moment); instead, refresh the timestamp and continue waiting.
 */
static void Robot_ParkingEnter(TickType_t* activeTick)
{
	//Determine whether to enter parking mode based on whether the control amount is active. If the car stops for 10 seconds and there is no control mode, it will enter parking mode to save power
	if( RobotControlParam.ParkingMode==0 && xTaskGetTickCount() - (*activeTick) > 10000 )
	{
		//Determine whether the site is on a slope based on the IMU angle
		if(fabs(AttitudeVal.pitch) < 2.5f && fabs(AttitudeVal.roll)<2.5f )
		{
			RobotControlParam.ParkingMode=1;
			//Send a hold command
			Robot_DriverSetParkingMode(1);
		}
		else
		{	//The car is on a slope, refreshing the time used for judgment
			(*activeTick) = xTaskGetTickCount();
		}
	}
}

/**
 * Inspection — Drive self-test, error recovery, low battery alert
 *
 * Error recovery may block for hundreds of milliseconds, so reset the periodic baseline here to avoid vTaskDelayUntil
 * Compensate by running a few cycles afterwards
 */
static void Robot_Housekeeping(uint8_t* driverCheck,uint16_t taskFreq,TickType_t* preTime)
{
	pBuzzerInterface_t buzzer = &UserBuzzer;

	//Poll driver self-test parameters at low frequency, including motor error codes and driver errors
	(*driverCheck)++;
	if( (*driverCheck)==10 )
	{
		(*driverCheck)=0;
		Robot_DriverCheckError();
	}

	//Driver fault-recovery request; after a fault, users can set this flag to request an attempt to clear it
	if( RobotControlParam.DriveErrRecovery )
	{
		RobotControlParam.DriveErrRecovery = 0;
		Robot_DriverRecovery();

		//The recovery process may be stuck for a long time (including retrying), and resetting the cycle baseline avoids compensatory continuous running of vTaskDelayUntil
		(*preTime) = xTaskGetTickCount();
	}

	//Low battery buzzer alert for small cars; only models without light strips use buzzer alerts
	static uint32_t LowPowerTipsCnt = 0;
	switch( RobotHardWareParam.CarType )
	{
		case LSR200:case LSR200_OUTDOOR:case REF2:
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

//Multi-mode intelligent control handle button callback functions, all related to the car's functions
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	pBuzzerInterface_t buzzer = &UserBuzzer;

	//LB top-left button: decelerate
	if( keyid == Xbox360KEY_LB )
	{
		RobotControlParam.defalutSpeed -= 50;
		if( RobotControlParam.defalutSpeed<0 ) RobotControlParam.defalutSpeed=0;
		buzzer->AddTask(1,200);
	}

	//RB upper-right button: accelerate
	if( keyid == Xbox360KEY_RB )
	{
		RobotControlParam.defalutSpeed += 50;
		if( RobotControlParam.defalutSpeed>3000 ) RobotControlParam.defalutSpeed=3000;
		buzzer->AddTask(1,200);
	}

	//Right joystick Rjoy: IMU marked zero
	if( keyid == Xbox360KEY_RJoy && event == GamePadKeyEvent_LONGCLICK)
	{
		pIMUInterface_t imu = &UserICM20948;
		imu->UpdateZeroPoint_axis();
		imu->UpdateZeroPoint_attitude();
		buzzer->AddTask(1,200);//Use the buzzer to indicate the operation was successful
	}

	//Y: Auto recharge on/off
	if( keyid == Xbox360KEY_Y && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.ChargeMode = !RobotControlParam.ChargeMode;
		buzzer->AddTask(1,200);
	}

	//B: Clear motor error requests
	if( keyid == Xbox360KEY_B && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.DriveErrRecovery=1;
		buzzer->AddTask(1,200);
	}

	//A: Ultrasonic obstacle avoidance switch. Only switches the RangerAvoidEN flag and beep prompt; the obstacle avoidance decision logic has not yet been implemented
	if( keyid == Xbox360KEY_A && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.RangerAvoidEN = !RobotControlParam.RangerAvoidEN;
		buzzer->AddTask(1,200);
	}

	//X: IMU assists straight switching
	if( keyid==Xbox360KEY_X && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.ImuAssistedFlag = !RobotControlParam.ImuAssistedFlag;
		buzzer->AddTask(1,200);
	}

	//Left analog stick LJoy: Emergency mode
	if( keyid == Xbox360KEY_LJoy && event == GamePadKeyEvent_LONGCLICK )
	{
		RobotControlParam.EmergencyMode = !RobotControlParam.EmergencyMode;
		buzzer->AddTask(1,200);
	}

	//Directional buttons for left and right OLED page turning
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
	//Up key: Venue auto-test, long press to start, click/double-click to stop (beep feedback sent by the test task)
	if( keyid == Xbox360KEY_UP )
	{
		if( event == GamePadKeyEvent_LONGCLICK )
			AutoTest_Start();
		else if( event == GamePadKeyEvent_SINGLECLICK || event == GamePadKeyEvent_DOUBLECLICK )
			AutoTest_Stop();
	}
	#endif

}

//Linear speed m/s to RPM speed
static float linearVel_to_rpm(float linearVel,float wheelper)
{
	return linearVel*60.0f/wheelper;
}

//Steering motor homing: periodically resend the homing command and wait for all motors; return 0 on success, 1 on timeout
static uint8_t SteeringHoming(uint32_t timeout_ms)
{
	TickType_t start = xTaskGetTickCount();
	TickType_t lastSend = start;

	//Issue one zero-back instruction first
	set_MotorPos(&Driver1,0,LSR280_WheelTurnSpeed,0,LSR280_WheelTurnSpeed);
	set_MotorPos(&Driver2,0,LSR280_WheelTurnSpeed,0,LSR280_WheelTurnSpeed);

	while(1)
	{
		vTaskDelay(20);

		//All four wheels return to zero (within tolerance) to be considered complete
		float zeroA = fabsf(RobotControlParam.MotorA.ori_pos);
		float zeroB = fabsf(RobotControlParam.MotorB.ori_pos);
		float zeroC = fabsf(RobotControlParam.MotorC.ori_pos);
		float zeroD = fabsf(RobotControlParam.MotorD.ori_pos);
		if( zeroA <= HOMING_TOL_COUNTS && zeroB <= HOMING_TOL_COUNTS && zeroC <= HOMING_TOL_COUNTS && zeroD <= HOMING_TOL_COUNTS )
			return 0;

		if( xTaskGetTickCount() - start > pdMS_TO_TICKS(timeout_ms) )
			return 1;

		//Periodic retransmission to prevent instructions from being lost due to bus incompleteness during initial power-up
		if( xTaskGetTickCount() - lastSend >= pdMS_TO_TICKS(HOMING_RESEND_MS) )
		{
			lastSend = xTaskGetTickCount();
			set_MotorPos(&Driver1,0,LSR280_WheelTurnSpeed,0,LSR280_WheelTurnSpeed);
			set_MotorPos(&Driver2,0,LSR280_WheelTurnSpeed,0,LSR280_WheelTurnSpeed);
		}
	}
}

//Driver fault-recovery function
static void Robot_DriverRecovery(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		//If the motor has errors, clear them
		if(g_ServoDriveList[i]->errFlag==1)
		{
			ServoDriver_ClearError(g_ServoDriveList[i]);
		}

		//Determine whether to enable the motor based on the state of the enable switch
		if( RobotControlParam.Enkeystate==1 ) ServoDriver_Enable(g_ServoDriveList[i]);
		else ServoDriver_Disable(g_ServoDriveList[i]);

		//Reset rotational speed to zero
		if( g_ServoDriveList[i]->initMode==SpeedMode )
			set_MotorRPM(g_ServoDriveList[i],0,0);
	}

	//After zeroing fails, retry the error recovery action; if successful, clear persistent error flags
	if( SteeringHomeFailed )
	{
		if( SteeringHoming(HOMING_TIMEOUT_MS)==0 )
			SteeringHomeFailed = 0;
	}
}

//Set parking mode
static void Robot_DriverSetParkingMode(uint8_t setflag)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		//Send parking commands only to hub motors (velocity mode); leave steering motors unchanged
		if( g_ServoDriveList[i]->initMode==SpeedMode )
			ServoDriver_ParkingMode(g_ServoDriveList[i],setflag);
	}
}

//Obtain driver error information
static void Robot_DriverCheckError(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_CheckError(g_ServoDriveList[i]);
	}
}

//Disable motor
static void Robot_DriverDisable(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_Disable(g_ServoDriveList[i]);
	}
}

//Enable motor
static void Robot_DriverEnable(void)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		ServoDriver_Enable(g_ServoDriveList[i]);
	}
}

//Motor self-locking stops
//static void Robot_DriverStop(void)
//{
//	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
//	{
//		if( g_ServoDriveList[i]->initMode == SpeedMode )
//			set_MotorRPM(g_ServoDriveList[i],0,0);
//		else if (g_ServoDriveList[i]->initMode == PosMode) // TODO: to be tested
//			set_MotorPosStop(g_ServoDriveList[i]);
//	}
//}

//Updates battery voltage estimates at low frequency based on the state of motion
static void Robot_UpdateVoltageEstimate(const RobotControlCMDType_t* cmd)
{
	const float eps = 0.001f;
	float rawVol = (float)USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL)/4095.0f*3.3f*11;
	float weight = 0.5f;//Default weight
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
	
	//Parking mode has highest priority
	if( RobotControlParam.en_flag == 1 && RobotControlParam.ParkingMode==1 )
	{
		weight = 0.9f;
	}
	
	if( RobotControlParam.en_flag == 1 && cmd != NULL )
	{
		if( (fabsf(cmd->Vx) > eps || fabsf(cmd->Vy) > eps) && (fabsf(cmd->Vz) > eps) )
		{
			weight = 0.3f; //Possible sliding friction in motion scenarios, reducing weight
		}
	}
	
	RobotControlParam.Vol = RobotControlParam.Vol*(1.0f-weight) + rawVol*weight;
}

//Cart enable status check
static void robot_en_check(void)
{
	//The default error code is 0, set according to the following error situations
	RobotControlParam.ErrNum = 0;

	//Software emergency-stop switch state
	if( RobotControlParam.softwareEnflag == 1 ) Set_RobotErrorCode(errCode_SoftWareStop);

	//Steering homing failed; keep the error code set to disable the chassis
	if( SteeringHomeFailed ) Set_RobotErrorCode(errCode_SteeringHome);

	//Read the status of the robot's emergency stop switch and perform corresponding operations when switching buttons
	static uint8_t last_EnKey = 1;
	RobotControlParam.Enkeystate = get_EnKeyState();
	if( RobotControlParam.Enkeystate==0 ) Set_RobotErrorCode(errCode_StopKeyEn);

	//Enable switch is pressed
	if( RobotControlParam.Enkeystate==0 && last_EnKey==1 )
	{
		Robot_DriverDisable();
	}

	//Enable switch pops up
	else if( last_EnKey==0 && RobotControlParam.Enkeystate==1 )
	{
		Robot_DriverEnable();
	}

	last_EnKey = RobotControlParam.Enkeystate;

	//Robot low battery status acquisition
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
			//Insufficient voltage. (If automatic recharge mode is enabled, voltage is ignored)
			if( RobotControlParam.Vol < 20.0f && RobotControlParam.ChargeMode==0 ) Set_RobotErrorCode(errCode_LowPower);
		}
	}
	else
	{
		LowPowerTiming=0U;
		LowPowerStartTick=0U;
		if( RobotControlParam.Vol>21.5f) RobotControlParam.LowPowerFlag=0;
	}

	//Generate different error codes based on the number of drivers
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
 * @brief This function is used to write control queues; all tasks that need to control the cart must write control quantities through this task.
 * @param cmd: The parameter must include the instruction source and specify the three-axis target speed for the cart
 *         woken: if called in an interrupt, this parameter needs to be passed. For non-interrupt calls, just write 0
 * @retval 0 write successful, 1 write failed. Reason for write failure: No specified source of control instruction or currently a higher-priority control is active
 *
 * @details
 * - When users need to add control methods, they must first add an enumeration of control types in the RobotControl_task.h header file and determine their priority.
 *   Then control the cart via the WriteControlQueue write control queue
 */
uint8_t WriteRobotControlQueue(RobotControlCMDType_t* cmd,BaseType_t* woken)
{
	static uint32_t lastTick = 0;
	uint8_t writeflag = 0;

	//Ignore writes if the queue is not created or the control source is unspecified
	if(g_xQueueRobotControl == NULL || cmd->cmdsource == NONE_CMD ||
		cmd->cmdsource >= UnKnownCMD ) return 1;

	//High-priority commands can preempt control directly
	if( cmd->cmdsource <= RobotControl_CMDsource )
	{
		RobotControl_CMDsource = cmd->cmdsource;
		writeflag = 1;
	}
	else
	{
		//Receive low-priority control instructions and check active timestamps
		uint32_t tick = 0;
		if( __get_IPSR()!=0 ) tick = xTaskGetTickCountFromISR();
		else tick = xTaskGetTickCount();

		//If a high-priority command is inactive for more than 2000ms, allow low-priority commands
		if( tick - lastTick >= 2000 )
		{
			RobotControl_CMDsource = cmd->cmdsource;
			writeflag = 1;
		}
	}

	//Allow writing to the queue
	if( writeflag )
	{
		if( __get_IPSR()!=0 ) //Interrupt contextual judgment
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
 * @brief This function is used to calculate the correction factor and to correct the effect of the cart moving in a straight line
 * @param Correction coefficient value, left wheel/right wheel
 * @retval Correction multiplier
 *
 * @details
 * When the car can't go straight, you can adjust the correction factor to improve straight-line performance.
 * 0-100 is an adjustable value, 50 means no correction effect. If the car is more than 50, it will automatically veer to the right,
 * Cars less than 50 meters automatically veer to the left.
 */
float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //Left-wheel correction, corresponding to a correction factor of 50~100 and 1.0~1.2 times
	{
		if( diffparam>50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //Right wheel correction: 50~0 corresponds to a correction factor of 1.0~1.2 times
	{
		if( diffparam<50 )
			return 1.0f + 0.004f*(50-diffparam);
	}

	return 1.0f;//If the condition is not met, the default is 1.
}

static void _4ws_SpeedSmooth(RobotControlCMDType_t* smoothcmd,const RobotControlCMDType_t* cmdtarget,uint16_t RunFreq)
{
	#define ACCEL_START_X 1.0f  //Maximum linear acceleration m/s² during startup (acceleration).
	#define ACCEL_BRAKE_X 1.5f  //Maximum linear acceleration m/s² when stopping (decelerating).
	#define ACCEL_START_Y 1.0f  //Maximum linear acceleration m/s² during startup (acceleration).
	#define ACCEL_BRAKE_Y 1.5f  //Maximum linear acceleration m/s² when stopping (decelerating).
	#define ACCEL_START_W 5.0f  //Acceleration at rotational start angle rad/s²
	#define ACCEL_BRAKE_W 7.0f  //Acceleration of the rotational braking angle

	//Passenger-carrying parameters
//	#define ACCEL_START_X 0.5f //Max linear acceleration when starting (accelerating) m/s2
//	#define ACCEL_BRAKE_X 0.5f //Max linear acceleration when stopping (decelerating) m/s2
//	#define ACCEL_START_Y 0.5f //Max linear acceleration when starting (accelerating) m/s2
//	#define ACCEL_BRAKE_Y 0.5f //Max linear acceleration when stopping (decelerating) m/s2
//	#define ACCEL_START_W 1.5f //Angular acceleration at rotation startup rad/s2
//	#define ACCEL_BRAKE_W 1.5f // Acceleration of the braking angle during rotation

	_4ws_SpeedSmoothLimit(smoothcmd,cmdtarget,RunFreq,
		ACCEL_START_X,ACCEL_BRAKE_X,ACCEL_START_Y,ACCEL_BRAKE_Y,ACCEL_START_W,ACCEL_BRAKE_W);
}

static void _4ws_SpeedSmoothFastBrake(RobotControlCMDType_t* smoothcmd,const RobotControlCMDType_t* cmdtarget,uint16_t RunFreq)
{
	#define FAST_ACCEL_START_X 1.0f   //Normal start acceleration is still used in rapid deceleration mode
	#define FAST_BRAKE_X       5.0f   //Rapid line deceleration m/s² while waiting for steering protection
	#define FAST_ACCEL_START_Y 1.0f   //Normal start acceleration is still used in rapid deceleration mode
	#define FAST_BRAKE_Y       5.0f   //Rapid line deceleration m/s² while waiting for steering protection
	#define FAST_ACCEL_START_W 5.0f   //Normal rotational starting angular acceleration is still used in rapid deceleration mode
	#define FAST_BRAKE_W       15.0f  //Rapid rotational deceleration rad/s2 while waiting for steering protection

	//Activated only during the steering protection waiting phase to avoid affecting normal driving feel
	_4ws_SpeedSmoothLimit(smoothcmd,cmdtarget,RunFreq,
		FAST_ACCEL_START_X,FAST_BRAKE_X,FAST_ACCEL_START_Y,FAST_BRAKE_Y,FAST_ACCEL_START_W,FAST_BRAKE_W);
}

static void _4ws_SpeedSmoothLimit(RobotControlCMDType_t* smoothcmd,const RobotControlCMDType_t* cmdtarget,uint16_t RunFreq,
	float accel_start_x,float accel_brake_x,float accel_start_y,float accel_brake_y,float accel_start_w,float accel_brake_w)
{
	//Calculate speed increments
	float dvx = cmdtarget->Vx - smoothcmd->Vx;
	float dvy = cmdtarget->Vy - smoothcmd->Vy;
	float dvz = cmdtarget->Vz - smoothcmd->Vz;

	//Current control cycle
	const float dt = 1.0f/(float)RunFreq;

	//XY plane velocity limits acceleration as a vector to prevent combined acceleration exceeding limits during oblique motion
	float dxy_len = sqrtf(dvx * dvx + dvy * dvy);
	float current_dot_delta = smoothcmd->Vx * dvx + smoothcmd->Vy * dvy;
	if (dxy_len > 0.00001f)
	{
		//Ellipse synthesis is performed along the direction of change; even if the X/Y limits differ, the oblique acceleration limit remains accurate
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

	//Determining acceleration and deceleration on the Z-axis
	float max_dw;
	if( dvz * smoothcmd->Vz >= 0.0f )
	{
		max_dw = accel_start_w * dt;
	}
	else
	{
		//Reverse command: first press brake to decelerate to 0; if zero crossing occurs within this frame, apply reverse acceleration for the remaining time according to start
		float t_brake = fabsf(smoothcmd->Vz) / accel_brake_w;
		if( t_brake >= dt ) max_dw = accel_brake_w * dt;
		else max_dw = fabsf(smoothcmd->Vz) + accel_start_w * (dt - t_brake);
	}

	//Check if the smooth acceleration limit has been exceeded
	if (fabsf(dvz) > max_dw + 0.00001f)
	{
		dvz = (dvz > 0.0f) ? max_dw : -max_dw;
	}

	//Velocity smoothing
	smoothcmd->Vx += dvx;
	smoothcmd->Vy += dvy;
	smoothcmd->Vz += dvz;

	//Deadzone reset prevents remote stick zero drifting and slight vehicle shaking
//	if (fabsf(smoothcmd->Vx) < 0.008f) smoothcmd->Vx = 0.0f;
//	if (fabsf(smoothcmd->Vy) < 0.008f) smoothcmd->Vy = 0.0f;
//	if (fabsf(smoothcmd->Vz) < 0.005f) smoothcmd->Vz = 0.0f;
}

/**
 * @brief This function performs the IMU data line routing function
 * @param Car target speed value
 * @retval None
 *
 * @details
 *  IMU and car control direction: when rotating counterclockwise, yaw is positive, yaw speed is positive. When Vz is positive, the car rotates counterclockwise
 */
static void Linear_IMUHelp(RobotControlCMDType_t* cmdtarget)
{
	//Used to delay the acquisition of the target angle
	static uint8_t TargetCnt = 0;
	#define DelayCNT 70

	//Target angle
	static float target_angle = 0;

	//PID parameters
	float linear_helpkp = 0.08f;
	float linear_helpkd = 0.05f;

	switch( RobotHardWareParam.CarType )
	{
		//Different models have different PID parameters when adjusting in a straight line, which is related to body weight, body structure, and motor
		case LSR100: linear_helpkp = 0.03f; linear_helpkd = 0.007f; break;
		case LSR300Mini: linear_helpkp = 0.08f; linear_helpkd = 0.01f; break;
		case LSR300: linear_helpkp = 0.06f; linear_helpkd = 0.015f; break;
		case LSR280: linear_helpkp = 0.6f; linear_helpkd = 0.15f; break;
		default:break;
	}

	static uint8_t giveup_flag = 0;

	//Only compensates when Vx has speed; otherwise, no compensation and target angle is updated
	if( (cmdtarget->Vx!=0 || cmdtarget->Vy!=0) && cmdtarget->Vz==0 && giveup_flag==0 )
	{
		//Delay obtaining the target angle
		TargetCnt++;
		if( TargetCnt==DelayCNT )  target_angle = AttitudeVal.yaw;

		//After obtaining the target angle, execute PID control
		if( TargetCnt >= DelayCNT )
		{
			//Lock the counter value to prevent cycling
			TargetCnt=DelayCNT+1;

			//Error calculation
			float error = target_angle-AttitudeVal.yaw;

			//Normalize error angles
			if( error >  180.0f ) error -= 360.0f;
			if( error < -180.0f ) error += 360.0f;

			//When the deviation is too large, abandon adjustments (such as suspension control).
			if( fabs(error)> 30 )
			{
				giveup_flag = 1;
				LinearHelp_Target.Vz=0;
				return;
			}

			//PID adjusts the straight-line effect of the car (only adjusts when the speed exceeds a certain threshold)
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

			//Limit adjustment value
			LinearHelp_Target.Vz = target_limit_float(LinearHelp_Target.Vz,-1.57f,1.57f);
		}

	}
	else if( cmdtarget->Vx==0 && cmdtarget->Vy==0 && cmdtarget->Vz==0 )
	{
		//Refresh target angles after remaining stationary with all velocities at 0 for a while
		TargetCnt++;
		if( TargetCnt>100 )  target_angle = AttitudeVal.yaw,TargetCnt=101;
		LinearHelp_Target.Vx = 0;
		LinearHelp_Target.Vy = 0;
		LinearHelp_Target.Vz = 0;
		giveup_flag=0;
	}
	else
	{
		//In other cases, the refresh of the target angle is delayed to prevent inaccurate refresh during dynamic switching
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

//Reset to enter the bootloader function
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
		//Execute software reset after receiving the reset character "reset" from the host
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			RobotControlCMDType_t cmd = {
				.cmdsource = BootLoader,
				0,0,0
			};

			//Write the highest priority instruction to stop the chassis's movement
			for(uint8_t i=0;i<10;i++)
			{
				WriteRobotControlQueue(&cmd,0);
				vTaskDelay( pdMS_TO_TICKS(10) );
			}

			//Reset system
			NVIC_SystemReset();
		}
	}
}

//Debug mode
void RobotControl_SetDebugLevel(char uart_recv)
{
	//Debug task handles
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

			//Operation prompt sound
			pBuzzerInterface_t tips = &UserBuzzer;
			tips->AddTask(1,200);

			//Send task notifications
			xTaskNotifyGive(g_reportErrTaskHandle);
		}
	}
}
