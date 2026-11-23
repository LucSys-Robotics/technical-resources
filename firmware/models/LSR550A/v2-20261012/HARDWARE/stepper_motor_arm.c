#include "stepper_motor_arm.h"

Arm_t A280;


static const ArmWorkspaceArc_t g_workspace_arcs[] = {
    { 0.0f,   0.14f,  0.16f,  -1.3f,  0.7f,  DISTANCE_GREATER_OR_EQUAL },
    { 0.0f,   0.0f,   0.0447f,-0.65f,-0.25f, DISTANCE_GREATER_OR_EQUAL },
    { 0.0f,   0.0f,   0.264f, -0.65f, 1.1f,  DISTANCE_LESS_OR_EQUAL    },
    { 0.0f,  -0.16f,  0.14f, -0.150f, 1.3f,  DISTANCE_GREATER_OR_EQUAL },
    { 0.138f,-0.0237f,0.16f, -1.57f, -1.0f,  DISTANCE_LESS_OR_EQUAL    },
};



void Gripper_SetLevel(uint8_t level)
{
    if (level > GRIPPER_LEVEL_MAX) level = GRIPPER_LEVEL_MAX;
    if (level < GRIPPER_LEVEL_MIN) level = GRIPPER_LEVEL_MIN;
    A280.gripper = level;
    uint16_t pulse = SERVO_PWM_MIN
                   + (uint16_t)((SERVO_PWM_MAX - SERVO_PWM_MIN) * level / SERVO_LEVEL_MAX);

    TIM9->CCR2 = pulse;
}


void Arm2DOF_Init(Arm_t *arm) {
	  arm->control_mode=ROS_Mode;
	  arm->kinematic_mode=Inverse_Kinematic;
	
    arm->joint_min[0]  = AngleA_Min;
    arm->joint_max[0]  = AngleA_Max;
	
    arm->joint_min[1]  = AngleB_Min;
    arm->joint_max[1]  = AngleB_Max;
	
    arm->joint_min[2]  = AngleC_Min + PI/2.0f;
    arm->joint_max[2]  = AngleC_Max + PI/2.0f;
	
	  arm->last_joint_deg[0]=0.0f;
    arm->last_joint_deg[1]=0.0f; 
		arm->last_joint_deg[2]=0.0f; 
    
		arm->joint_speed[0]=0;
	  arm->joint_speed[1]=0;
	  arm->joint_speed[2]=0;
	  
	
	  arm->ee_x=0.062;
	  arm->ee_y=-0.008;
	  
	  arm->joint_rad[0]=0.0f;
		arm->joint_rad[1]=1.570796f;
		arm->joint_rad[2]=0.391797f;
		arm->gripper = GRIPPER_LEVEL_INIT;	  
}



void UpdateMotorTargets_FromJoints(Arm_t *arm,uint8_t idx){
	float new_deg_A, new_deg_B, new_deg_C;
	new_deg_A = arm->joint_rad[0] * RAD2DEG;
	float b2=1.5707f-arm->joint_rad[1];
	float c2=arm->joint_rad[2]-0.391796f-b2;
	new_deg_B = -b2 * RAD2DEG;
  new_deg_C = -c2 * RAD2DEG;
	if (idx == 1 || idx == 123) {
        if (fabsf(new_deg_A - arm->last_joint_deg[0]) > ANGLE_CHANGE_THRESHOLD) {
            arm->joint_deg[0] = new_deg_A;
            arm->joint_flag[0] = 1; 
        } else {
            arm->joint_flag[0] = 0; 
        }
    }

    if (idx == 23 || idx == 123) {
        // Check the B axis
        if (fabsf(new_deg_B - arm->last_joint_deg[1]) > ANGLE_CHANGE_THRESHOLD) {
            arm->joint_deg[1] = new_deg_B;
            arm->joint_flag[1] = 1;
        } else {
            arm->joint_flag[1] = 0;
        }

        // Check the C-axis
        if (fabsf(new_deg_C - arm->last_joint_deg[2]) > ANGLE_CHANGE_THRESHOLD) {
            arm->joint_deg[2] = new_deg_C;
            arm->joint_flag[2] = 1;
        } else {
            arm->joint_flag[2] = 0;
        }
    }

}


uint8_t isPointInsideArcs(float x,float y){
    uint8_t arc_count=(y>0.0f)?4:5;
    for(uint8_t i=0;i<arc_count;++i){
        float dx = x - g_workspace_arcs[i].center_x;
				float	dy = y - g_workspace_arcs[i].center_y; 
				float d2 = dx*dx + dy*dy;
			  float r2 = g_workspace_arcs[i].radius * g_workspace_arcs[i].radius;
        uint8_t within;
        if     (g_workspace_arcs[i].distance_mode==DISTANCE_LESS_OR_EQUAL)    within=(d2<=r2);
        else if(g_workspace_arcs[i].distance_mode==DISTANCE_GREATER_OR_EQUAL) within=(d2>=r2);
        else within=0; 
		    if(!within) return 0;
		}
    return 1;
}


uint8_t Arm2DOF_SetTargetEndEffector(Arm_t *arm,
                                     float x, float y) {
    if (!isPointInsideArcs(x, y)) return 0;
    float b, c;
    if (!Arm2DOF_Inverse(arm, x, y, &b, &c)) return 0;
    arm->joint_rad[1] = b; arm->joint_rad[2] = c;
    arm->ee_x = x; arm->ee_y = y;
    return 1;
}
 
uint8_t Arm2DOF_SetTargetJoint(Arm_t *arm,
                               float joint_b, float joint_c) {
		float x, y; 
    if (!Arm2DOF_CheckJointLimits(arm, joint_b, joint_c)) return 0;
		Arm2DOF_Forward(arm, joint_b, joint_c, &x, &y);
    if (!isPointInsideArcs(x, y)) return 0;
    arm->joint_rad[1] = joint_b; arm->joint_rad[2] = joint_c;
    arm->ee_x = x; arm->ee_y = y;
    return 1;
}


void Arm2DOF_Forward(const Arm_t *arm,
                     float joint_b, float joint_c,
                     float *out_x, float *out_y) {
    float xb = Link_B * cosf(joint_b);
    float yb = Link_B * sinf(joint_b);
    float xc = xb + Link_C * cosf(joint_c - PI + joint_b);
    float yc = yb + Link_C * sinf(joint_c - PI + joint_b);
    *out_x = xc; *out_y = yc;
}

uint8_t Arm2DOF_Inverse(const Arm_t *arm,
                        float x, float y,
                        float *out_b, float *out_c) {
    float r3 = sqrtf(x*x + y*y);
    float b1 = acosf((Link_B*Link_B + r3*r3
                     - Link_C*Link_C)
                     / (2.0f*Link_B*r3));
    float b2 = atan2f(y, x);
    float b  = b1 + b2;
    float c  = acosf((Link_B*Link_B
                     + Link_C*Link_C - r3*r3)
                     / (2.0f*Link_B*Link_C));
    if (!Arm2DOF_CheckJointLimits(arm, b, c)) return 0;
    *out_b = b; *out_c = c;
    return 1;
}

uint8_t Arm2DOF_CheckJointLimits(const Arm_t *arm,
                                 float joint_b, float joint_c) {
    if (joint_b < arm->joint_min[1] || joint_b > arm->joint_max[1]) return 0;
    if (joint_c < arm->joint_min[2] || joint_c > arm->joint_max[2]) return 0;
    if (joint_b + joint_c < PI/2.0f) return 0;
    return 1;
}



void CAN_UpdateAndSend(void){
  float angle_diff[3]={0.0f,0.0f,0.0f};
  float max_angle_diff=0.0f;
  u8  i;
  u8  motor_id[3]={0x01,0x02,0x03};
  u8  active_axis_count = 0; // Records several axes that need to be moved
	
  // 1) Only calculate angle difference & maximum difference for axes with joint_flag
  for(i=0;i<3;i++){
    if(A280.joint_flag[i]){
      angle_diff[i]=fabsf(A280.joint_deg[i]-A280.last_joint_deg[i]);
      if(angle_diff[i]>max_angle_diff) max_angle_diff=angle_diff[i];
			active_axis_count++;
    }else{
      angle_diff[i]=0.0f;
    }
  } 
	if(active_axis_count == 0) return;
	
	
	  // 2) Calculate synchronization speed based on the maximum difference
  if(max_angle_diff>0.01f){
    for(i=0;i<3;i++){
      if(A280.joint_flag[i]){
        float spd=angle_diff[i]/max_angle_diff*MAX_SPEED;
				
        if(spd<MIN_SPEED ) spd=MIN_SPEED;
        A280.joint_speed[i]=spd;
      }
    }
  }else{
	    return;
  }
  // 3) Update the last_joint_deg and send + packets to each axis that needs to be moved
	  for(i=0;i<3;i++){
    if(!A280.joint_flag[i]) continue;
    A280.last_joint_deg[i]=A280.joint_deg[i];

    u8  buf[8];
    u8  dir;


		float steps_f = A280.joint_deg[i] * STEPS_PER_DEG;
		u16   steps   = (u16)fabsf(steps_f);	

    u16 spd  =(u16)A280.joint_speed[i];

    // Direction: Axis A Positive = 0 Negative = 1; Axis B/C Positive = 1 Negative = 0
    if(i==0) dir=(A280.joint_deg[i]>0.0f)?0:1;
    else     dir=(A280.joint_deg[i]>0.0f)?1:0;

    buf[0]=2;         // Ctrl_Mode
    buf[1]=dir;
    buf[2]=32;        // Microstep
    buf[3]=steps>>8;  buf[4]=steps;
    buf[5]=spd>>8;    buf[6]=spd;
    buf[7]=0;


    CAN1_Send_Num(motor_id[i],buf);
    A280.joint_flag[i]=0;
  }
}


