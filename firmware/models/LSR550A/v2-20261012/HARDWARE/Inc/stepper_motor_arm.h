#ifndef __STEPPER_MOTOR_ARM_H
#define __STEPPER_MOTOR_ARM_H			  	 
#include "sys.h"
#include "system.h"

#define STEPPER_TASK_PRIO		4     //Task priority //任务优先级
#define STEPPER_STK_SIZE 		256   //Task stack size //任务堆栈大小
#define RAD2DEG    57.2957795f
#define Link_B     0.1400f //杆B长度
#define Link_C     0.1600f //杆C长度
#define AngleA_Min -1.9f
#define AngleA_Max  1.9f 

#define AngleB_Min -0.1700f //关节B角度最小值(相对X轴)，逆时针为正
#define AngleB_Max  1.5708f //关节B角度最大值(相对X轴)，顺时针为负

#define AngleC_Min -1.1790f //关节C角度最小值(相对X轴)，逆时针为正
#define AngleC_Max  0.7273f //关节C角度最大值(相对X轴)，顺时针为负

#define SERVO_PWM_MIN    500   // 最小脉宽，对应完全张开
#define SERVO_PWM_MAX   2500   // 最大脉宽，对应完全闭合
#define GRIPPER_LEVEL_MIN  40 //最小张开角度
#define GRIPPER_LEVEL_MAX  82 //最大张开角度，过大可能导致舵机发热严重，不建议长期处于85-100，可能导致舵机烧坏
#define GRIPPER_LEVEL_INIT 50 //初始角度
#define SERVO_LEVEL_MAX   100   // 开合角度范围 0~100，0完全张开，100完全闭合，实际使用建议在40-82

#define ANGLE_CHANGE_THRESHOLD 0.05f
#define MAX_SPEED  90     // 最大速度（你的电机速度范围：0-90）
#define MIN_SPEED  5      // 最小速度（避免速度过低导致电机不转，建议 5-10）

#define STEPS_PER_DEG (10.0f*13.755f) //十倍发送*减速比

typedef enum {
    DISTANCE_LESS_OR_EQUAL,   // 距离小于或等于半径
    DISTANCE_GREATER_OR_EQUAL // 距离大于或等于半径
} DistanceComparison;

typedef enum {
    Forward_Kinematic,   //正运动学
    Inverse_Kinematic    //逆运动学
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
