#ifndef __STEPPER_MOTOR_ARM_H
#define __STEPPER_MOTOR_ARM_H			  	 
#include "sys.h"
#include "system.h"

#define STEPPER_TASK_PRIO		4     //Task priority //Task Priority
#define STEPPER_STK_SIZE 		256   //Task stack size //Task Stack Size
#define RAD2DEG    57.2957795f
#define Link_B     0.1400f //Length of pole B
#define Link_C     0.1600f //Length of rod C
#define AngleA_Min -1.9f
#define AngleA_Max  1.9f 

#define AngleB_Min -0.1700f //Minimum angle B of joints (relative to the X-axis), counterclockwise is positive
#define AngleB_Max  1.5708f //Maximum angle of joint B (relative to the X-axis), clockwise is negative

#define AngleC_Min -1.1790f //Minimum angle of joint C (relative to the X-axis), counterclockwise is positive
#define AngleC_Max  0.7273f //Maximum angle of joint C (relative to the X-axis), clockwise is negative

#define SERVO_PWM_MIN    500   // Minimum pulse width, corresponding to full opening
#define SERVO_PWM_MAX   2500   // Maximum vein width, corresponding to complete closure
#define GRIPPER_LEVEL_MIN  40 //Minimum opening angle
#define GRIPPER_LEVEL_MAX  82 //Maximum opening angle; excessive opening may cause servo overheating. It is not recommended to keep it at 85-100 for long periods, as it may cause servo burnout
#define GRIPPER_LEVEL_INIT 50 //Initial angle
#define SERVO_LEVEL_MAX   100   // Opening angle range: 0~100, 0 fully opened, 100 fully closed; actual recommended use is 40-82

#define ANGLE_CHANGE_THRESHOLD 0.05f
#define MAX_SPEED  90     // Maximum speed (your motor speed range: 0-90)
#define MIN_SPEED  5      // Minimum speed (avoid speeds too low to turn the motor; recommended 5-10)

#define STEPS_PER_DEG (10.0f*13.755f) //Tenfold transmission * reduction ratio

typedef enum {
    DISTANCE_LESS_OR_EQUAL,   // Distance less than or equal to the radius
    DISTANCE_GREATER_OR_EQUAL // Distance greater than or equal to radius
} DistanceComparison;

typedef enum {
    Forward_Kinematic,   //Positive kinematics
    Inverse_Kinematic    //Inverse kinematics
} Kinematic_Mode;


typedef enum {
    ROS_Mode,            
	  APP_Mode,
    PS2_Mode,
	  Uart1_Mode
} Control_Mode;


typedef struct {
    float center_x;
    float center_y;
    float radius;
    float theta_start;
    float theta_end;
    uint8_t distance_mode;
} ArmWorkspaceArc_t;


typedef struct {
	  Control_Mode control_mode;
	  Kinematic_Mode kinematic_mode;
	  float joint_max[3];
	  float joint_min[3];
	  float joint_rad[3];
	  float joint_deg[3];
	  float last_joint_deg[3];
    uint8_t joint_flag[3];
	  uint8_t joint_speed[3];
	  uint8_t gripper;
    float ee_x;
    float ee_y;
} Arm_t;
extern Arm_t A280;	

void Gripper_SetLevel(uint8_t level);
void UpdateMotorTargets_FromJoints(Arm_t *arm,uint8_t idx);
void Arm2DOF_Init(Arm_t *arm);
void CAN_UpdateAndSend(void);
void Arm2DOF_Forward(const Arm_t *arm,float joint_b, float joint_c,float *out_x, float *out_y) ;
uint8_t isPointInsideArcs(float x,float y);
uint8_t Arm2DOF_SetTargetEndEffector(Arm_t *arm,float x, float y);
uint8_t Arm2DOF_SetTargetJoint(Arm_t *arm,float joint_b, float joint_c);
uint8_t Arm2DOF_CheckJointLimits(const Arm_t *arm,float joint_b, float joint_c);
uint8_t Arm2DOF_Inverse(const Arm_t *arm,float x, float y,float *out_b, float *out_c);							



#endif  
