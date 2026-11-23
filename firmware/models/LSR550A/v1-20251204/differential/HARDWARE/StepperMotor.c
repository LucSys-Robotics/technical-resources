#include "StepperMotor.h" // 包含步进电机相关头文件

Arm_t A280; // 声明一个全局机械臂结构体变量

// 机械臂工作空间限制定义 (通过一系列圆弧或圆来定义限制范围)
static const ArmWorkspaceArc_t g_workspace_arcs[] = {
    // { center_x, center_y, radius, start_angle, end_angle, distance_mode }
    { 0.0f, 0.14f, 0.16f, -1.3f, 0.7f, DISTANCE_GREATER_OR_EQUAL }, // 限制1：距离 >= 半径
    { 0.0f, 0.0f, 0.0447f, -0.65f, -0.25f, DISTANCE_GREATER_OR_EQUAL }, // 限制2：距离 >= 半径
    { 0.0f, 0.0f, 0.264f, -0.65f, 1.1f, DISTANCE_LESS_OR_EQUAL },     // 限制3：距离 <= 半径
    { 0.0f, -0.16f, 0.14f, -0.150f, 1.3f, DISTANCE_GREATER_OR_EQUAL }, // 限制4：距离 >= 半径
    { 0.138f, -0.0237f, 0.16f, -1.57f, -1.0f, DISTANCE_LESS_OR_EQUAL }, // 限制5：距离 <= 半径
};

/**
 * @brief 设置机械爪的开合程度。
 * @param level 机械爪的目标等级 (0 到 SERVO_LEVEL_MAX)。
 */
void Gripper_SetLevel(uint8_t level)
{
    // 限制 level 的范围在 [0, SERVO_LEVEL_MAX]
    if (level > SERVO_LEVEL_MAX) level = SERVO_LEVEL_MAX;
    if (level < 0) level = 0;

    // 根据 level 线性计算 PWM 脉冲宽度
    uint16_t pulse = SERVO_PWM_MIN
                     + (uint16_t)((SERVO_PWM_MAX - SERVO_PWM_MIN) * level / SERVO_LEVEL_MAX);

    // 设置舵机 PWM 占空比 (假设使用 TIM9 的 CCR2 通道)
    TIM9->CCR2 = pulse;
}

/**
 * @brief 初始化 Arm_t 机械臂结构体。
 * @param arm 指向要初始化的机械臂结构体指针。
 */
void Arm2DOF_Init(Arm_t *arm) {
    // 初始化控制模式和运动学模式
    arm->control_mode = ROS_Mode;
    arm->kinematic_mode = Inverse_Kinematic;

    // 设置关节 1 (A 轴) 的角度限制
    arm->joint_min[0] = AngleA_Min;
    arm->joint_max[0] = AngleA_Max;

    // 设置关节 2 (B 轴) 的角度限制
    arm->joint_min[1] = AngleB_Min;
    arm->joint_max[1] = AngleB_Max;

    // 设置关节 3 (C 轴) 的角度限制 (并进行角度偏移校准)
    arm->joint_min[2] = AngleC_Min + PI / 2.0f;
    arm->joint_max[2] = AngleC_Max + PI / 2.0f;

    // 初始化上一次发送的目标角度为 0
    arm->last_joint_deg[0] = 0.0f;
    arm->last_joint_deg[1] = 0.0f;
    arm->last_joint_deg[2] = 0.0f;

    // 初始化关节速度
    arm->joint_speed[0] = 0;
    arm->joint_speed[1] = 0;
    arm->joint_speed[2] = 0;

    // 初始化末端执行器 (End Effector) 坐标
    arm->ee_x = 0.062;
    arm->ee_y = -0.008;

    // 初始化关节弧度值 (机械臂的初始/home 位置)
    arm->joint_rad[0] = 0.0f;          // 关节 A (底座)
    arm->joint_rad[1] = 1.570796f;     // 关节 B (与初始位置 PI/2 对应)
    arm->joint_rad[2] = 0.391797f;     // 关节 C
    arm->gripper = 0;                  // 机械爪初始状态
}

/**
 * @brief 将关节弧度目标值转换为步进电机所需角度目标值，并设置运动标志。
 * @param arm 指向机械臂结构体指针。
 * @param idx 用于指定检查哪些关节 (1:A, 23:BC, 123:ABC)。
 */
void UpdateMotorTargets_FromJoints(Arm_t *arm, uint8_t idx) {
    float new_deg_A, new_deg_B, new_deg_C;

    // 1. 转换 A 轴 (底座) 弧度到角度
    new_deg_A = arm->joint_rad[0] * RAD2DEG;

    // 2. 转换 B 轴和 C 轴弧度到步进电机角度
    // 注意：这里的转换涉及机械臂的结构校准
    float b2 = 1.5707f - arm->joint_rad[1]; // B 轴校准
    float c2 = arm->joint_rad[2] - 0.391796f - b2; // C 轴校准
    new_deg_B = -b2 * RAD2DEG;
    new_deg_C = -c2 * RAD2DEG;

    // 检查 A 轴是否需要移动 (idx=1 或 123)
    if (idx == 1 || idx == 123) {
        // 如果新角度与上一次的角度差大于阈值
        if (fabsf(new_deg_A - arm->last_joint_deg[0]) > ANGLE_CHANGE_THRESHOLD) {
            arm->joint_deg[0] = new_deg_A; // 更新目标角度 (度)
            arm->joint_flag[0] = 1;      // 设置移动标志
        } else {
            arm->joint_flag[0] = 0;
        }
    }

    // 检查 B 轴和 C 轴是否需要移动 (idx=23 或 123)
    if (idx == 23 || idx == 123) {
        // 检查 B 轴
        if (fabsf(new_deg_B - arm->last_joint_deg[1]) > ANGLE_CHANGE_THRESHOLD) {
            arm->joint_deg[1] = new_deg_B;
            arm->joint_flag[1] = 1;
        } else {
            arm->joint_flag[1] = 0;
        }

        // 检查 C 轴
        if (fabsf(new_deg_C - arm->last_joint_deg[2]) > ANGLE_CHANGE_THRESHOLD) {
            arm->joint_deg[2] = new_deg_C;
            arm->joint_flag[2] = 1;
        } else {
            arm->joint_flag[2] = 0;
        }
    }
}

/**
 * @brief 检查给定坐标 (x, y) 是否在机械臂工作空间定义的所有圆弧限制内。
 * @param x 末端执行器 X 坐标。
 * @param y 末端执行器 Y 坐标。
 * @return 1 (在工作空间内) 或 0 (超出工作空间)。
 */
uint8_t isPointInsideArcs(float x, float y) {
    // 如果 y > 0，只检查前 4 个限制；否则检查全部 5 个限制。
    uint8_t arc_count = (y > 0.0f) ? 4 : 5;
    for (uint8_t i = 0; i < arc_count; ++i) {
        float dx = x - g_workspace_arcs[i].center_x;
        float dy = y - g_workspace_arcs[i].center_y;
        float d2 = dx * dx + dy * dy; // 距离平方
        float r2 = g_workspace_arcs[i].radius * g_workspace_arcs[i].radius; // 半径平方

        uint8_t within;
        // 根据 distance_mode 检查点是否满足圆弧限制条件
        if (g_workspace_arcs[i].distance_mode == DISTANCE_LESS_OR_EQUAL) {
            within = (d2 <= r2);
        } else if (g_workspace_arcs[i].distance_mode == DISTANCE_GREATER_OR_EQUAL) {
            within = (d2 >= r2);
        } else {
            within = 0;
        }

        // 只要有一个限制不满足，立即返回 0
        if (!within) return 0;
    }
    return 1; // 所有限制都满足
}

/**
 * @brief 设置末端执行器的目标坐标 (X, Y)，并尝试进行逆运动学解算。
 * @param arm 机械臂结构体指针。
 * @param x 目标 X 坐标。
 * @param y 目标 Y 坐标。
 * @return 1 (设置成功) 或 0 (超出工作空间或逆解失败)。
 */
uint8_t Arm2DOF_SetTargetEndEffector(Arm_t *arm, float x, float y) {
    if (!isPointInsideArcs(x, y)) return 0; // 检查是否在工作空间内

    float b, c;
    // 尝试逆运动学解算关节 B 和 C 的弧度
    if (!Arm2DOF_Inverse(arm, x, y, &b, &c)) return 0;

    // 更新目标关节弧度值和末端执行器坐标
    arm->joint_rad[1] = b;
    arm->joint_rad[2] = c;
    arm->ee_x = x;
    arm->ee_y = y;

    return 1;
}

/**
 * @brief 设置关节 B 和 C 的目标弧度值，并进行运动学和工作空间检查。
 * @param arm 机械臂结构体指针。
 * @param joint_b 目标关节 B 弧度。
 * @param joint_c 目标关节 C 弧度。
 * @return 1 (设置成功) 或 0 (超出关节限制或工作空间)。
 */
uint8_t Arm2DOF_SetTargetJoint(Arm_t *arm, float joint_b, float joint_c) {
    float x, y;

    if (!Arm2DOF_CheckJointLimits(arm, joint_b, joint_c)) return 0; // 检查关节限制
    
    // 正运动学解算，获取该关节角度对应的末端执行器坐标 (x, y)
    Arm2DOF_Forward(arm, joint_b, joint_c, &x, &y);

    if (!isPointInsideArcs(x, y)) return 0; // 检查坐标是否在工作空间内

    // 更新目标关节弧度值和末端执行器坐标
    arm->joint_rad[1] = joint_b;
    arm->joint_rad[2] = joint_c;
    arm->ee_x = x;
    arm->ee_y = y;

    return 1;
}

/**
 * @brief 机械臂正运动学解算：根据关节弧度计算末端执行器坐标。
 * @param arm 机械臂结构体指针。
 * @param joint_b 关节 B 弧度。
 * @param joint_c 关节 C 弧度。
 * @param out_x 输出的 X 坐标指针。
 * @param out_y 输出的 Y 坐标指针。
 */
void Arm2DOF_Forward(const Arm_t *arm,
                     float joint_b, float joint_c,
                     float *out_x, float *out_y) {
    // 关节 B 坐标
    float xb = Link_B * cosf(joint_b);
    float yb = Link_B * sinf(joint_b);
    
    // 关节 C 坐标 (注意 C 轴的连杆角度是 joint_c - PI + joint_b)
    float xc = xb + Link_C * cosf(joint_c - PI + joint_b);
    float yc = yb + Link_C * sinf(joint_c - PI + joint_b);

    *out_x = xc;
    *out_y = yc;
}

/**
 * @brief 机械臂逆运动学解算：根据末端执行器坐标计算关节弧度。
 * @param arm 机械臂结构体指针。
 * @param x 目标 X 坐标。
 * @param y 目标 Y 坐标。
 * @param out_b 输出的关节 B 弧度指针。
 * @param out_c 输出的关节 C 弧度指针。
 * @return 1 (解算成功) 或 0 (解算失败或超出关节限制)。
 */
uint8_t Arm2DOF_Inverse(const Arm_t *arm,
                        float x, float y,
                        float *out_b, float *out_c) {
    float r3 = sqrtf(x * x + y * y); // 末端执行器到原点的距离

    // 根据余弦定理计算关节 B 对应角度 (b1)
    // b1 = acos( (L_B^2 + r3^2 - L_C^2) / (2 * L_B * r3) )
    float b1 = acosf((Link_B * Link_B + r3 * r3 - Link_C * Link_C)
                     / (2.0f * Link_B * r3));
                     
    // 末端执行器相对于原点的角度 (b2)
    float b2 = atan2f(y, x);
    
    // 关节 B 的最终弧度
    float b = b1 + b2;

    // 根据余弦定理计算关节 C 对应角度 (c)
    // c = acos( (L_B^2 + L_C^2 - r3^2) / (2 * L_B * L_C) )
    float c = acosf((Link_B * Link_B + Link_C * Link_C - r3 * r3)
                     / (2.0f * Link_B * Link_C));

    if (!Arm2DOF_CheckJointLimits(arm, b, c)) return 0; // 检查解算出的关节是否在限制内

    *out_b = b;
    *out_c = c;
    return 1;
}

/**
 * @brief 检查关节 B 和 C 的目标弧度值是否在限制范围内。
 * @param arm 机械臂结构体指针。
 * @param joint_b 目标关节 B 弧度。
 * @param joint_c 目标关节 C 弧度。
 * @return 1 (在限制内) 或 0 (超出限制)。
 */
uint8_t Arm2DOF_CheckJointLimits(const Arm_t *arm,
                                 float joint_b, float joint_c) {
    // 检查 B 轴限制
    if (joint_b < arm->joint_min[1] || joint_b > arm->joint_max[1]) return 0;
    
    // 检查 C 轴限制
    if (joint_c < arm->joint_min[2] || joint_c > arm->joint_max[2]) return 0;
    
    // 检查 B 轴和 C 轴的联动限制 (防止碰撞或奇异点，角度和大于 PI/2)
    if (joint_b + joint_c < PI / 2.0f) return 0;

    return 1;
}

/**
 * @brief 计算各轴速度，更新目标角度，并通过 CAN 总线发送指令给步进电机。
 */
void CAN_UpdateAndSend(void) {
    float angle_diff[3] = {0.0f, 0.0f, 0.0f}; // 角度差
    float max_angle_diff = 0.0f;             // 最大角度差
    u8 i;
    u8 motor_id[3] = {0x01, 0x02, 0x03};     // 步进电机 CAN ID
    u8 active_axis_count = 0;                // 记录需要移动的轴数

    // 1) 仅计算有 joint_flag 标志的轴的角度差 & 最大差
    for (i = 0; i < 3; i++) {
        if (A280.joint_flag[i]) {
            // 计算当前目标角度与上次发送角度的绝对差值
            angle_diff[i] = fabsf(A280.joint_deg[i] - A280.last_joint_deg[i]);
            if (angle_diff[i] > max_angle_diff) max_angle_diff = angle_diff[i];
            active_axis_count++;
        } else {
            angle_diff[i] = 0.0f;
        }
    }
    if (active_axis_count == 0) return; // 没有轴需要移动，直接返回

    // 2) 根据最大差值，计算各轴的同步速度
    if (max_angle_diff > 0.01f) {
        for (i = 0; i < 3; i++) {
            if (A280.joint_flag[i]) {
                // 按比例计算速度，实现多轴同步运动 (角速度同步)
                float spd = angle_diff[i] / max_angle_diff * MAX_SPEED;

                // 速度不能低于最小速度
                if (spd < MIN_SPEED) spd = MIN_SPEED;
                A280.joint_speed[i] = spd;
            }
        }
    } else {
        return; // 角度变化太小，忽略
    }
    
    // 3) 更新 last_joint_deg，并对每个需要动的轴封包+发送 CAN 指令
    for (i = 0; i < 3; i++) {
        if (!A280.joint_flag[i]) continue; // 跳过不需要移动的轴

        A280.last_joint_deg[i] = A280.joint_deg[i]; // 更新上次发送的角度

        u8 buf[8];
        u8 dir;

        // 计算所需步数：角度 * 每度步数
        float steps_f = A280.joint_deg[i] * STEPS_PER_DEG;
        u16 steps = (u16)fabsf(steps_f);

        u16 spd = (u16)A280.joint_speed[i]; // 获取计算好的同步速度

        // 确定方向：A 轴和 B/C 轴的方向定义可能相反
        // A轴：正角=0, 负角=1
        if (i == 0) dir = (A280.joint_deg[i] > 0.0f) ? 0 : 1;
        // B/C轴：正角=1, 负角=0
        else dir = (A280.joint_deg[i] > 0.0f) ? 1 : 0;

        // 构造 CAN 数据包 (假设为特定的步进电机控制协议)
        buf[0] = 2;        // Ctrl_Mode: 2 (位置/速度控制模式)
        buf[1] = dir;      // Direction
        buf[2] = 32;       // Microstep: 32 细分
        buf[3] = steps >> 8;
        buf[4] = steps;    // Target Steps (高8位, 低8位)
        buf[5] = spd >> 8;
        buf[6] = spd;      // Speed (高8位, 低8位)
        buf[7] = 0;

        // 通过 CAN 发送数据
        CAN1_Send_Num(motor_id[i], buf);
        A280.joint_flag[i] = 0; // 清除移动标志
    }
}