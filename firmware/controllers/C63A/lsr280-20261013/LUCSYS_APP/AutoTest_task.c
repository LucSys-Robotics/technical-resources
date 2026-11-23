/**
 * @file    AutoTest_task.c
 * @brief 2.4m x 2.4m site automatic endurance test task
 * @date    2026-07-08
 *
 * @details
 * - Publish preset action sequences with the lowest priority via the control queue (WriteRobotControlQueue),
 *   Loop execution: move forward/sideways/backward/diagonal/in-place rotation. Path is "square with all four sides + two diagonal lines X".
 * - Simple closed loop:
 *     Translation segment = track estimation (odometer speed integrated by IMU heading) waypoint navigation + IMU heading hold;
 *     Rotation segment = IMU lap counting;
 *     Time is only used as a bottom-line timeout for each segment; if the timeout is timeout, the segment will be forcibly cut.
 * - The script is arranged according to palindromes, with each side/diagonal moving forward and backward once, systematically drifting first-step reversal.
 * - In supervised scenarios, no virtual fence is set: if the position drifts out of the area, it is manually moved back (testing continues).
 *
 * @note
 * - Start: Long press the up key on the handlebar (record current position as origin, current front direction as +X)
 * - Stop: single/double click the gamepad D-pad up button; automatically stop on emergency stop or disable;
 *         When the joystick moves (high-priority sources continuously occupy the queue), it will automatically stop.
 */
#include "AutoTest_task.h"

//DATALOG_ENABLE=0 (shipping), the entire implementation does not participate in compilation, does not occupy Flash, and will not be created
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

//======== Test parameters (adjusted as needed) ========
#define AT_RUN_FREQ_HZ     50       //Task run frequency
#define AT_EDGE_M          1.6f     //Virtual square side length (leave a 0.4m margin on each side inside the 2.4m test area)
#define AT_SPEED_MPS       0.3f     //Translation cruise speed
#define AT_MIN_SPEED_MPS   0.08f    //Minimum speed as you approach the waypoint
#define AT_DECEL_DIST_M    0.35f    //Remaining distance to start linear deceleration
#define AT_ARRIVE_DIST_M   0.05f    //Determine the radius at the point of arrival
#define AT_ROT_SPEED_RAD   0.4f     //Angular velocity of the rotating segment rad/s
#define AT_ROT_TURNS       2.0f     //Number of turns per rotation segment
#define AT_PAUSE_MS        800      //Pause between segments (waiting for steering alignment while repeatedly testing the steering protection logic)
#define AT_YAW_KP          0.05f    //Translation section heading maintains P-gain (rad/s)/degree
#define AT_YAW_KD          0.02f    //Translation Section Heading Maintain D Gain (diagonal velocity deg/s)
#define AT_YAW_VZ_MAX      0.3f     //Heading correction angular velocity limit rad/s
#define AT_TIMEOUT_SCALE   1.5f     //Segment fallback timeout = nominal duration x this coefficient + base
#define AT_TIMEOUT_BASE_MS 500     //Additional base for segment fallback timeout (ms)
#define AT_WRITE_FAIL_MS   500      //If queue writes continuously fail longer than this duration, assume control has been preempted and stop automatically
#define AT_STOP_HOLD_MS    200      //Duration of zero-speed ending when stopped

//segment type
typedef enum{
	SEG_MOVE = 0,   //Pan to the waypoint
	SEG_ROT,        //Spinning N times in place
}SegType_t;

//Test script single segment: At SEG_MOVE, p1/p2 are the coordinates of the route point (m); When SEG_ROT, p1 is the number of signed turns
typedef struct{
	uint8_t type;
	float p1;
	float p2;
}TestSeg_t;

//Test script: The World series uses the starting point as the origin; at startup, the front direction is +X, and the left side of the hull is +Y
//Four corners: A(0,0) B(E,0) C(E,E) D(0,E)
//Palindrome arrangement: Walk through each side/diagonal once on both sides
#define E_ AT_EDGE_M
static const TestSeg_t TestScript[] = {
	{ SEG_MOVE, E_,  0   },          //S1 A->B Straight ahead
	{ SEG_MOVE, E_,  E_  },          //S2 B->C Move left sideways
	{ SEG_MOVE, 0,   E_  },          //S3 C->D reversed
	{ SEG_MOVE, 0,   0   },          //S4 D->A Move right sideways
	{ SEG_MOVE, E_,  E_  },          //S5 A->C Diagonal shift (front left 45 degrees)
	{ SEG_ROT,  +AT_ROT_TURNS, 0 },  //S6 Spins in place + 2 turns
	{ SEG_MOVE, 0,   0   },          //S7 C->A Diagonal Return (Back Right)
	{ SEG_MOVE, 0,   E_  },          //S8 A->D Left lateral shift
	{ SEG_MOVE, E_,  E_  },          //S9 D->C Straight ahead
	{ SEG_MOVE, E_,  0   },          //S10 C->B right lateral shift
	{ SEG_MOVE, 0,   0   },          //S11 B->A backwards
	{ SEG_MOVE, E_,  0   },          //S12 A->B Straight ahead
	{ SEG_MOVE, 0,   E_  },          //S13 B->D diagonal shift (rear left, another diagonal)
	{ SEG_ROT,  -AT_ROT_TURNS, 0 },  //S14 spin in place - 2 turns
	{ SEG_MOVE, E_,  0   },          //S15 D->B Diagonal Return (Front Right)
	{ SEG_MOVE, 0,   0   },          //S16 B->A backed up and returned to the starting line
};
#undef E_
#define SEG_COUNT  ( sizeof(TestScript)/sizeof(TestScript[0]) )

//Task state machine
typedef enum{
	AT_IDLE = 0,     //Standby, do not write to control queue
	AT_RUN,          //Execute the current segment
	AT_SEG_PAUSE,    //Pause between segments
	AT_STOPPING,     //Release control after zero-speed finishing
}ATState_t;

//Start-stop request (controller key pullback to set position, consumed by this task)
static volatile uint8_t StartReq = 0;
static volatile uint8_t StopReq  = 0;

//Runtime status (for debugging/display)
static volatile ATState_t at_state = AT_IDLE;
static uint16_t at_seg   = 0;    //Current segment index
static uint32_t at_cycle = 0;    //Number of completed cycles

//Track estimation (World system, origin = start point, +X = front direction at start)
static float odo_x = 0, odo_y = 0;

//IMU heading at startup (degrees). Not refreshing throughout the process is intentional: it serves both as the reference direction and heading for the test world
//Maintain the goal. If you re-pick midway, the World system will drift along with the IMU, and the coordinates for track calculation lose their meaning.
static float yaw0  = 0;

//Diagnostic quantity for data observation tasks, see AutoTest_task.h
volatile AutoTestDbg_t g_AutoTestDbg = { 0 };

void AutoTest_DbgWindowReset(void)
{
	g_AutoTestDbg.vzMin   = g_AutoTestDbg.vzLast;
	g_AutoTestDbg.vzMax   = g_AutoTestDbg.vzLast;
	g_AutoTestDbg.flipCnt = 0;
}

//Each control cycle is called once, accumulating the number of Vz envelopes and symbol flips in this window
static void AutoTest_DbgUpdate(float vz)
{
	float prev = g_AutoTestDbg.vzLast;

	if( vz < g_AutoTestDbg.vzMin ) g_AutoTestDbg.vzMin = vz;
	if( vz > g_AutoTestDbg.vzMax ) g_AutoTestDbg.vzMax = vz;

	//A flip is only counted as one flip if neither beat approaches zero, avoiding noise near zero being counted
	if( vz*prev < 0.0f && fabsf(vz) > 0.02f && fabsf(prev) > 0.02f )
	{
		if( g_AutoTestDbg.flipCnt < 255 ) g_AutoTestDbg.flipCnt++;
	}

	g_AutoTestDbg.vzLast = vz;
}

//Normalize angles to (-180,180]
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

//Calculating the bottom-line timeout (ms) for a segment: nominal time consumption x coefficient + base value; time only serves as a bottom-line and does not serve as the main criterion
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

	//segment execution process amount
	TickType_t segStart   = 0;   //Current segment start time
	TickType_t pauseStart = 0;   //Pause at the start of the moment
	TickType_t stopStart  = 0;   //Finish the start moment
	uint32_t   segTimeoutMs = 0; //Current segment fallback timeout
	float      rotAcc  = 0;      //Cumulative angle of the rotation segment (degrees)
	float      lastYaw = 0;      //Previous cycle heading, used for lap counting
	TickType_t writeFailStart = 0; //Time of first queue-write failure; 0 means writes are normal

	//and other systems are stable with IMUs
	vTaskDelay(3000);
	TickType_t preTime = xTaskGetTickCount();

	while(1)
	{
		RobotControlCMDType_t cmd = { 0 };
		cmd.cmdsource = AutoTest_CMD;

		//---- Stop Request----
		if( StopReq )
		{
			StopReq = 0;
			if( at_state==AT_RUN || at_state==AT_SEG_PAUSE )
			{
				at_state = AT_STOPPING;
				stopStart = xTaskGetTickCount();
			}
		}

		//---- Initiate request ----
		if( StartReq )
		{
			StartReq = 0;
			if( at_state == AT_IDLE )
			{
				if( RobotControlParam.en_flag == 1 )
				{
					//Set up the test coordinate system with the current position as the origin and the current front direction at +X
					odo_x = 0; odo_y = 0;
					yaw0 = AttitudeVal.yaw;
					lastYaw = AttitudeVal.yaw;
					at_seg = 0; at_cycle = 0;
					rotAcc = 0;
					writeFailStart = 0;
					segStart = xTaskGetTickCount();
					segTimeoutMs = at_seg_timeout_ms(&TestScript[0]);
					at_state = AT_RUN;
					buzzer->AddTask(1,300); //Startup prompt: 1 sound
				}
				else
				{
					buzzer->AddTask(3,100); //Chassis disabled; refuse startup: 3 short beeps
				}
			}
		}

		//---- Trajectory Calculation (During Test Run) ----
		if( at_state==AT_RUN || at_state==AT_SEG_PAUSE )
		{
			float psi = at_wrap180(AttitudeVal.yaw - yaw0) * 0.0174533f;
			float vx = RobotControlParam.feedbackVx;
			float vy = RobotControlParam.feedbackVy;
			odo_x += ( vx*cosf(psi) - vy*sinf(psi) ) * dt;
			odo_y += ( vx*sinf(psi) + vy*cosf(psi) ) * dt;
		}

		//---- Safety: Immediate test termination in case of emergency stop/disable ----
		if( (at_state==AT_RUN || at_state==AT_SEG_PAUSE) && RobotControlParam.en_flag==0 )
		{
			at_state = AT_IDLE;
			buzzer->AddTask(2,200); //Stop prompt: 2 sounds
		}

		//---- State Machine ----
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
						//Linear deceleration near the route point to prevent overshoot
						float v = AT_SPEED_MPS;
						if( dist < AT_DECEL_DIST_M )
						{
							v = AT_SPEED_MPS * (dist/AT_DECEL_DIST_M);
							if( v < AT_MIN_SPEED_MPS ) v = AT_MIN_SPEED_MPS;
						}

						//The world system speeds to the route point (position closed loop), then rotates according to the heading to the vehicle system
						float vwx = v * dx/dist;
						float vwy = v * dy/dist;
						float psi = at_wrap180(AttitudeVal.yaw - yaw0) * 0.0174533f;
						cmd.Vx =  cosf(psi)*vwx + sinf(psi)*vwy;
						cmd.Vy = -sinf(psi)*vwx + cosf(psi)*vwy;

						//Heading hold (PD, symbols consistent with Linear_IMUHelp): The front of the vehicle locks the starting direction,
						//Ensure the backward segment is truly backward/the transverse segment is truly transverse
						float yaw_err = at_wrap180(yaw0 - AttitudeVal.yaw);
						float vz = AT_YAW_KP*yaw_err - AT_YAW_KD*axis_9Val.gyro.z*57.3f;
						cmd.Vz = at_clampf(vz,-AT_YAW_VZ_MAX,AT_YAW_VZ_MAX);

						//Diagnosis: Record the original output and PD input before the limit to determine which ring the oscillation originates from
						g_AutoTestDbg.yawErr = yaw_err;
						g_AutoTestDbg.vzRaw  = vz;
					}
				}
				else //SEG_ROT: Spins in place, IMU lap counting
				{
					float dyaw = at_wrap180(AttitudeVal.yaw - lastYaw);
					rotAcc += dyaw;

					if( fabsf(rotAcc) >= fabsf(seg->p1)*360.0f )
						arrived = 1;
					else
						cmd.Vz = (seg->p1 >= 0) ? AT_ROT_SPEED_RAD : -AT_ROT_SPEED_RAD;
				}

				//Fallback timeout: force transition to the next segment; 3 short beeps alert the observer
				if( !arrived && xTaskGetTickCount()-segStart > pdMS_TO_TICKS(segTimeoutMs) )
				{
					arrived = 1;
					buzzer->AddTask(3,100);
				}

				if( arrived )
				{
					at_state = AT_SEG_PAUSE;
					pauseStart = xTaskGetTickCount();
					//cmd maintains zero speed and enters a pause
					cmd.Vx = 0; cmd.Vy = 0; cmd.Vz = 0;
				}
				break;
			}

			case AT_SEG_PAUSE:
			{
				//Pause between sections to zero speed, then wait for the steering to align
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
				//Zero-speed finishing send, then release control (after 2 seconds, low-priority sources can take over)
				if( xTaskGetTickCount()-stopStart >= pdMS_TO_TICKS(AT_STOP_HOLD_MS) )
				{
					at_state = AT_IDLE;
					buzzer->AddTask(2,200); //Stop prompt: 2 sounds
				}
				break;
			}

			case AT_IDLE:
			default:
				break;
		}

		//---- Control Command Issuance and Preemptive Detection ----
		if( at_state != AT_IDLE )
		{
			if( WriteRobotControlQueue(&cmd,0) != 0 )
			{
				//Write failed = a higher-priority control source is active (like joystick)
				if( writeFailStart == 0 )
				{
					writeFailStart = xTaskGetTickCount();
				}
				else if( xTaskGetTickCount()-writeFailStart > pdMS_TO_TICKS(AT_WRITE_FAIL_MS) )
				{
					//Control is continuously seized, automatically ending the test
					at_state = AT_IDLE;
					buzzer->AddTask(2,200);
				}
			}
			else
			{
				writeFailStart = 0;
			}
		}

		//Diagnostic volume is updated every cycle, and the window is cleared after reading from the log task
		g_AutoTestDbg.yaw0  = yaw0;
		g_AutoTestDbg.seg   = (uint8_t)at_seg;
		g_AutoTestDbg.state = (uint8_t)at_state;
		AutoTest_DbgUpdate(cmd.Vz);

		//Lap counting reference updated every cycle
		lastYaw = AttitudeVal.yaw;

		vTaskDelayUntil(&preTime, pdMS_TO_TICKS(1000/AT_RUN_FREQ_HZ));
	}
}

#endif //DATALOG_ENABLE
