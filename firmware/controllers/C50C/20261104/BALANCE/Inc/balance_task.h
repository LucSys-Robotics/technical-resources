#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "system.h"

#include "bsp_gamepad.h"

//RTOS task configuration
#define BALANCE_TASK_PRIO		4             //Task priority //Task Priority
#define BALANCE_STK_SIZE 		512           //Task stack size //Task Stack Size
#define BALANCE_TASK_RATE       RATE_100_HZ   //Task frequency

#define PWM_HYSTERESIS			5

//Robot controls relevant parameters
typedef struct{
	u8 ControlMode;       //Robot Control Mode_(Variable description: Variable not directly operated by the user; do not assign values directly, read-only)
	u8 FlagStop;          //Robot enabled flag: 1 means robot disabled, 0 means enabled_ (Variable description: Variable not directly operated by the user, do not assign values directly, read-only)
	u8 command_lostcount; //Robot lost control command count; after a certain period, the robot's self-movement can be stopped
	u8 SoftWare_Stop;     //Robot software emergency stop, set to 1 emergency stop, 0 enable. This is reserved only for use; if needed, set this value to 1
	float Vx;            //Robot X-axis target speed
	float Vy;            //Target speed along the robot's y-axis
	float Vz;            //Robot Z-axis target speed
	float smooth_Vx;     //Robot speed smoothing result value
	float smooth_Vy;     //Robot speed smoothing result value
	float smooth_Vz;     //Robot speed smoothing result value
	
	float smooth_MotorStep;   //Robot motor speed smoothing step value
	float smooth_ServoStep;   //Smoothing step value of the robot's servo speed
	float smooth_Servo;  //Robot speed smoothing result value
	u8 ServoLow_flag;    //Low-speed servo mode, used at power-up and when servo position changes while the chassis is disabled (top-spec Ackermann only).
	
	float rc_speed;      //Robot remote control speed benchmark, default 500 mm/s
	float limt_max_speed;//Maximum allowable speed limit for robots. Unit m/s; standard models default to 3.5 m/s, but in practice, standard robots cannot reach this speed.
	uint32_t LineDiffParam;//Correction factor, adjustable from 0 to 100

}ROBOT_CONTROL_t;

//Robot self-test related variables
typedef struct{
	u8 errorflag; //Self-check error flag
	u8 check_end; //Self-test end flag position
	int check_a;  //Four wheel encoder self-check related variables
	int check_b;
	int check_c;
	int check_d;
	u8 DeepCheck; //Enter depth-detection flag
}ROBOT_SELFCHECK_t;

//Motor state when the robot is disabled.
enum{
	UN_LOCK = 0, //Release motor shaft
	LOCK    = 1  //Motor lock shaft
};

//Variables related to robot parking mode
typedef struct{
	u8 wait_check;  //Wait for the detection of PWM clearing flags
	u16 timecore;   //Timing variables
	u8 start_clear; //Whether to start clearing residual PWM
	u8 clear_state; //PWM residual removal status. Use the highest and lower 4 bits: the lower 4 bits represent 4 motors, the lowest bit represents motor A, and so on. The highest bit indicates whether the removal task has been completed.
}ROBOT_PARKING_t;

//Incremental PI controller structure
typedef struct{
	float Bias;    // Deviation from the target value
	float LastBias;// The previous deviation
	int Output;    // Output
	int kp;        // kp value
	int ki;        // ki value
}PI_CONTROLLER;

//Top-spec Ackermann variables for servo non-self-locking operation
typedef struct{
	uint8_t UnLock;      //Servo non-self-locking mode marker
	uint8_t wait_Calib;  //Wait for calibration
	float UnLock_Pos;   //Position recorded in servo non-self-locking mode
	float UnLock_Target;//Saved target value in non-self-locking mode
	uint16_t UnLock_Output;//Servo PWM value on entering non-self-locking mode; for display only.
}AKM_SERVO_UNLOCK_t;

extern ROBOT_CONTROL_t robot_control;
extern ROBOT_SELFCHECK_t robot_check;
extern AKM_SERVO_UNLOCK_t ServoState;

//Robot control mode settings and reading
enum
{
	_ROS_Control   =  (1<<0), //ROS control
	_PS2_Control   =  (1<<1), //PS2 controls
	_APP_Control   =  (1<<2), //APP control
	_RC_Control    =  (1<<3), //RC transmitter
	_CAN_Control   =  (1<<4), //CAN communication control
	_USART_Control =  (1<<5), //Serial port control
};
//Set the robot control mode
#define Set_Control_Mode(mask)  (robot_control.ControlMode |= (mask), robot_control.ControlMode &= (mask)) 
//Read the robot's control method
#define Get_Control_Mode(mask)  (robot_control.ControlMode & (mask))

//External variables
extern SYS_VAL_t SysVal;
extern ROBOT_CONTROL_t robot_control;
extern PI_CONTROLLER PI_MotorA,PI_MotorB,PI_MotorC,PI_MotorD,PI_Servo;

//External functions
void Balance_task(void *pvParameters); //Task statement
//void Akm_ReadServo_Param(void); // Read Ackermann servo info from Flash
void PI_Controller_Init(PI_CONTROLLER* p,int kp,int ki); //Software initialization function
void ROBOT_CONTROL_t_Init(ROBOT_CONTROL_t* p);
void  Set_Robot_PI_Param(int kp,int ki); //PI controller settings parameters
float rad_to_angle(const float rad);  //Angle and radian rotation
float angle_to_rad(const float angle);//Angle and radian rotation
float Akm_Vz_to_Angle(float Vx,float Vz);//Convert Ackermann's target speed to the left front wheel angle
short get_ServoPWM(short TmpPos);//Estimate the servo's PWM value based on the slide rail data
float target_limit_float(float insert,float low,float high);
void FlashParam_Read(void);

//Internal use function
static void ROBOT_SELFCHECK_t_Init(ROBOT_SELFCHECK_t* p); //Initialize the class

static void PI_Controller_Reset(PI_CONTROLLER *p); //PI control class
static void PI_SetParam(PI_CONTROLLER* p,int kp,int ki);
static int Incremental_MOTOR(PI_CONTROLLER* p,float current,float target);
static int Incremental_Servo(PI_CONTROLLER* p,float current,float target);
static uint8_t PI_Clear_Output(PI_CONTROLLER* p);

static void InverseKinematics_akm(float Vx,float Vz); //Kinematic analysis class
static void InverseKinematics_diff(float Vx,float Vz);
static void InverseKinematics_mec(float Vx,float Vy,float Vz);
static void InverseKinematics_4wd(float Vx,float Vz);
static void InverseKinematics_omni(float Vx,float Vy,float Vz);

static void Drive_Motor(float T_Vx,float T_Vy,float T_Vz); //Control class
static void ResponseControl(void);
static void UnResponseControl(uint8_t mode);
static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo);
static u8 Turn_Off(void);
static void Get_APPcmd(void);
static void Remote_Control(void);
static void PS2_control(void);
static void robot_mode_check(void);
static void Get_Robot_FeedBack(void);
static void Servo_UnLock_Check(uint8_t car_stopflag);

static void Robot_ParkingCheck(void); //Status detection class
static void UserKey_Scan(u16 rate);
static void Charger_DevCheck(void);

//static uint8_t Akm_SaveServo_Param(uint8_t *flag); //helper function
static float Vel_SmoothControl(float now_speed,float targetSpeed,float step);

static int target_limit_int(int insert,int low,int high);
static int Slide_Mean_Filter(int data);
static uint8_t Deep_SelfCheck( u16 RATE );
uint8_t ValChangeCheck(const uint16_t rate,const short checkval,const uint8_t changeEva);
static uint8_t FlashParam_Save(uint8_t *flag);

#endif  

