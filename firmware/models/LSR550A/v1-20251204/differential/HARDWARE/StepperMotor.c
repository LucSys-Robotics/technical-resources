#include "StepperMotor.h" // Contains header files related to stepper motors

Arm_t A280; // Declare a global robotic arm structure variable

// Definition of robotic arm workspace constraints (defined by a series of arcs or circles)
static const ArmWorkspaceArc_t g_workspace_arcs[] = {
    // { center_x, center_y, radius, start_angle, end_angle, distance_mode }
    { 0.0f, 0.14f, 0.16f, -1.3f, 0.7f, DISTANCE_GREATER_OR_EQUAL }, // Constraint 1: distance >= radius
    { 0.0f, 0.0f, 0.0447f, -0.65f, -0.25f, DISTANCE_GREATER_OR_EQUAL }, // Constraint 2: distance >= radius
    { 0.0f, 0.0f, 0.264f, -0.65f, 1.1f, DISTANCE_LESS_OR_EQUAL },     // Constraint 3: distance <= radius
    { 0.0f, -0.16f, 0.14f, -0.150f, 1.3f, DISTANCE_GREATER_OR_EQUAL }, // Constraint 4: distance >= radius
    { 0.138f, -0.0237f, 0.16f, -1.57f, -1.0f, DISTANCE_LESS_OR_EQUAL }, // Constraint 5: distance <= radius
};

/**
 * @brief Set the degree of opening and closing of the mechanical jaw.
 * @param level Target level of the robotic gripper (0 to SERVO_LEVEL_MAX).
 */
void Gripper_SetLevel(uint8_t level)
{
    // Limit level to [0, SERVO_LEVEL_MAX]
    if (level > SERVO_LEVEL_MAX) level = SERVO_LEVEL_MAX;
    if (level < 0) level = 0;

    // Calculate PWM pulse width linearly based on level
    uint16_t pulse = SERVO_PWM_MIN
                     + (uint16_t)((SERVO_PWM_MAX - SERVO_PWM_MIN) * level / SERVO_LEVEL_MAX);

    // Set the servo PWM duty cycle (assuming the use of TIM9's CCR2 channel)
    TIM9->CCR2 = pulse;
}

/**
 * @brief Initialize the Arm_t robotic arm structure.
 * @param arm Pointer to the robotic arm structure to be initialized.
 */
void Arm2DOF_Init(Arm_t *arm) {
    // Initialize control mode and kinematic mode
    arm->control_mode = ROS_Mode;
    arm->kinematic_mode = Inverse_Kinematic;

    // Set the angle limit for joint 1 (A axis).
    arm->joint_min[0] = AngleA_Min;
    arm->joint_max[0] = AngleA_Max;

    // Set the angle limit for joint 2 (B axis).
    arm->joint_min[1] = AngleB_Min;
    arm->joint_max[1] = AngleB_Max;

    // Set the angle limit for joint 3 (C axis) (and perform angle offset calibration)
    arm->joint_min[2] = AngleC_Min + PI / 2.0f;
    arm->joint_max[2] = AngleC_Max + PI / 2.0f;

    // Initialize the last sent target angle to 0
    arm->last_joint_deg[0] = 0.0f;
    arm->last_joint_deg[1] = 0.0f;
    arm->last_joint_deg[2] = 0.0f;

    // Initialize joint speed
    arm->joint_speed[0] = 0;
    arm->joint_speed[1] = 0;
    arm->joint_speed[2] = 0;

    // Initialize the End Effector coordinates
    arm->ee_x = 0.062;
    arm->ee_y = -0.008;

    // Initialize joint curvature value (initial/home position of the robotic arm)
    arm->joint_rad[0] = 0.0f;          // Joint A (base)
    arm->joint_rad[1] = 1.570796f;     // Joint B (corresponding to initial position PI/2)
    arm->joint_rad[2] = 0.391797f;     // Joint C
    arm->gripper = 0;                  // Initial state of the mechanical claw
}

/**
 * @brief Convert the joint arc target value to the angle target required by the stepper motor, and set the motion marker.
 * @param arm Pointer to the robotic arm structure.
 * @param idx Used to specify which joints to check (1:A, 23:BC, 123:ABC).
 */
void UpdateMotorTargets_FromJoints(Arm_t *arm, uint8_t idx) {
    float new_deg_A, new_deg_B, new_deg_C;

    // 1. Convert A-axis (base) from radians to degrees
    new_deg_A = arm->joint_rad[0] * RAD2DEG;

    // 2. Convert the B-axis and C-axis radian to the stepper motor angle
    // Note: This conversion involves structural calibration of the robotic arm
    float b2 = 1.5707f - arm->joint_rad[1]; // B-axis calibration
    float c2 = arm->joint_rad[2] - 0.391796f - b2; // C-axis calibration
    new_deg_B = -b2 * RAD2DEG;
    new_deg_C = -c2 * RAD2DEG;

    // Check if the A axis needs to be moved (idx=1 or 123)
    if (idx == 1 || idx == 123) {
        // If the new angle differs from the previous angle by more than the threshold
        if (fabsf(new_deg_A - arm->last_joint_deg[0]) > ANGLE_CHANGE_THRESHOLD) {
            arm->joint_deg[0] = new_deg_A; // Update target angle (degrees)
            arm->joint_flag[0] = 1;      // Set move flags
        } else {
            arm->joint_flag[0] = 0;
        }
    }

    // Check whether the B and C axes need to be moved (idx=23 or 123)
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

/**
 * @brief Check whether the given coordinates (x, y) fall within all arc limits defined in the robotic arm's workspace.
 * @param x End effector X coordinate.
 * @param y End effector Y coordinate.
 * @return 1 (within workspace) or 0 (outside workspace).
 */
uint8_t isPointInsideArcs(float x, float y) {
    // If y > 0, only the first 4 limits are checked; otherwise, all 5 limits are checked.
    uint8_t arc_count = (y > 0.0f) ? 4 : 5;
    for (uint8_t i = 0; i < arc_count; ++i) {
        float dx = x - g_workspace_arcs[i].center_x;
        float dy = y - g_workspace_arcs[i].center_y;
        float d2 = dx * dx + dy * dy; // Distance squared
        float r2 = g_workspace_arcs[i].radius * g_workspace_arcs[i].radius; // Radius squared

        uint8_t within;
        // Check whether the point meets the arc constraint condition based on the distance_mode
        if (g_workspace_arcs[i].distance_mode == DISTANCE_LESS_OR_EQUAL) {
            within = (d2 <= r2);
        } else if (g_workspace_arcs[i].distance_mode == DISTANCE_GREATER_OR_EQUAL) {
            within = (d2 >= r2);
        } else {
            within = 0;
        }

        // Returns 0 immediately if one limit is not met
        if (!within) return 0;
    }
    return 1; // All restrictions are met
}

/**
 * @brief Set the target coordinates (X, Y) of the end effector and attempt inverse kinematic solutions.
 * @param arm Robotic arm structure pointer.
 * @param x Target X coordinate.
 * @param y Target Y coordinate.
 * @return 1 (setup successful) or 0 (workspace out or reverse solution failed).
 */
uint8_t Arm2DOF_SetTargetEndEffector(Arm_t *arm, float x, float y) {
    if (!isPointInsideArcs(x, y)) return 0; // Check if you are inside the workspace

    float b, c;
    // Try to solve the radians of joints B and C using reverse kinematics
    if (!Arm2DOF_Inverse(arm, x, y, &b, &c)) return 0;

    // Update the target joint radian value and end effector coordinates
    arm->joint_rad[1] = b;
    arm->joint_rad[2] = c;
    arm->ee_x = x;
    arm->ee_y = y;

    return 1;
}

/**
 * @brief Set target radius values for joints B and C, and conduct kinematic and workspace checks.
 * @param arm Robotic arm structure pointer.
 * @param joint_b Target joint B in radians.
 * @param joint_c Target joint C in radians.
 * @return 1 (setup successful) or 0 (joint limit or workspace exceeded).
 */
uint8_t Arm2DOF_SetTargetJoint(Arm_t *arm, float joint_b, float joint_c) {
    float x, y;

    if (!Arm2DOF_CheckJointLimits(arm, joint_b, joint_c)) return 0; // Check joint limitations
    
    // Positive kinematic calculation to obtain the end effector coordinates (x, y) corresponding to the joint angle
    Arm2DOF_Forward(arm, joint_b, joint_c, &x, &y);

    if (!isPointInsideArcs(x, y)) return 0; // Check if the coordinates are within the workspace

    // Update the target joint radian value and end effector coordinates
    arm->joint_rad[1] = joint_b;
    arm->joint_rad[2] = joint_c;
    arm->ee_x = x;
    arm->ee_y = y;

    return 1;
}

/**
 * @brief Forward Kinematics Solution for Robotic Arms: Calculate the end-effector coordinates based on joint curvature.
 * @param arm Robotic arm structure pointer.
 * @param joint_b Joint B in radians.
 * @param joint_c Joint C in radians.
 * @param out_x Pointer to output X coordinate.
 * @param out_y Pointer to output Y coordinate.
 */
void Arm2DOF_Forward(const Arm_t *arm,
                     float joint_b, float joint_c,
                     float *out_x, float *out_y) {
    // Joint B coordinate
    float xb = Link_B * cosf(joint_b);
    float yb = Link_B * sinf(joint_b);
    
    // Joint C-coordinate (note that the C-axis linkage angle is joint_c - PI + joint_b)
    float xc = xb + Link_C * cosf(joint_c - PI + joint_b);
    float yc = yb + Link_C * sinf(joint_c - PI + joint_b);

    *out_x = xc;
    *out_y = yc;
}

/**
 * @brief Inverse Kinematic Solution for the Robotic Arm: Calculate joint curvature based on the end-effector coordinates.
 * @param arm Robotic arm structure pointer.
 * @param x Target X coordinate.
 * @param y Target Y coordinate.
 * @param out_b Pointer to output joint B radians.
 * @param out_c Pointer to output joint C radians.
 * @return 1 (solution successful) or 0 (solution failed or joint limit exceeded).
 */
uint8_t Arm2DOF_Inverse(const Arm_t *arm,
                        float x, float y,
                        float *out_b, float *out_c) {
    float r3 = sqrtf(x * x + y * y); // Distance from the end effector to the origin

    // Calculate the angle corresponding to joint B (b1) based on the Law of Cosines.
    // b1 = acos( (L_B^2 + r3^2 - L_C^2) / (2 * L_B * r3) )
    float b1 = acosf((Link_B * Link_B + r3 * r3 - Link_C * Link_C)
                     / (2.0f * Link_B * r3));
                     
    // Angle of the end effector relative to the origin (B2)
    float b2 = atan2f(y, x);
    
    // The final curvature of joint B
    float b = b1 + b2;

    // Calculate the angle corresponding to joint C according to the Law of Cosines (c)
    // c = acos( (L_B^2 + L_C^2 - r3^2) / (2 * L_B * L_C) )
    float c = acosf((Link_B * Link_B + Link_C * Link_C - r3 * r3)
                     / (2.0f * Link_B * Link_C));

    if (!Arm2DOF_CheckJointLimits(arm, b, c)) return 0; // Check whether the calculated joints are within the limit

    *out_b = b;
    *out_c = c;
    return 1;
}

/**
 * @brief Check whether the target radian values for joints B and C are within the limit.
 * @param arm Robotic arm structure pointer.
 * @param joint_b Target joint B in radians.
 * @param joint_c Target joint C in radians.
 * @return 1 (within limits) or 0 (exceeding limits).
 */
uint8_t Arm2DOF_CheckJointLimits(const Arm_t *arm,
                                 float joint_b, float joint_c) {
    // Check the B-axis limits
    if (joint_b < arm->joint_min[1] || joint_b > arm->joint_max[1]) return 0;
    
    // Check the C-axis limits
    if (joint_c < arm->joint_min[2] || joint_c > arm->joint_max[2]) return 0;
    
    // Check the linkage limits of the B and C axes (to prevent collisions or singularities, angles, and values greater than PI/2)
    if (joint_b + joint_c < PI / 2.0f) return 0;

    return 1;
}

/**
 * @brief Calculates the speed of each axis, updates the target angle, and sends commands to the stepper motor via the CAN bus.
 */
void CAN_UpdateAndSend(void) {
    float angle_diff[3] = {0.0f, 0.0f, 0.0f}; // Poor angle
    float max_angle_diff = 0.0f;             // Maximum angle difference
    u8 i;
    u8 motor_id[3] = {0x01, 0x02, 0x03};     // Stepper motor CAN ID
    u8 active_axis_count = 0;                // Record the number of axes that need to be moved

    // 1) Only calculate the angle difference & maximum difference of axes with the joint_flag
    for (i = 0; i < 3; i++) {
        if (A280.joint_flag[i]) {
            // Calculate the absolute difference between the current target angle and the last sent angle
            angle_diff[i] = fabsf(A280.joint_deg[i] - A280.last_joint_deg[i]);
            if (angle_diff[i] > max_angle_diff) max_angle_diff = angle_diff[i];
            active_axis_count++;
        } else {
            angle_diff[i] = 0.0f;
        }
    }
    if (active_axis_count == 0) return; // No axis to move, just return directly

    // 2) Calculate the synchronization speed of each axis based on the maximum difference
    if (max_angle_diff > 0.01f) {
        for (i = 0; i < 3; i++) {
            if (A280.joint_flag[i]) {
                // Calculate speed proportionally to achieve multi-axis synchronous motion (angular velocity synchronization)
                float spd = angle_diff[i] / max_angle_diff * MAX_SPEED;

                // Speed must not fall below the minimum speed
                if (spd < MIN_SPEED) spd = MIN_SPEED;
                A280.joint_speed[i] = spd;
            }
        }
    } else {
        return; // Angle changes are too small and ignored
    }
    
    // 3) Update the last_joint_deg and send CAN instructions + to each axis packet that needs to be moved
    for (i = 0; i < 3; i++) {
        if (!A280.joint_flag[i]) continue; // Skip axes that do not need to be moved

        A280.last_joint_deg[i] = A280.joint_deg[i]; // Update the last sent angle

        u8 buf[8];
        u8 dir;

        // Calculate the required number of steps: angle * steps per degree
        float steps_f = A280.joint_deg[i] * STEPS_PER_DEG;
        u16 steps = (u16)fabsf(steps_f);

        u16 spd = (u16)A280.joint_speed[i]; // Get the calculated synchronization speed

        // Determining direction: The directions of the A and B/C axes may be defined oppositely
        // A-axis: positive angle = 0, negative angle = 1
        if (i == 0) dir = (A280.joint_deg[i] > 0.0f) ? 0 : 1;
        // B/C axis: positive angle = 1, negative angle = 0
        else dir = (A280.joint_deg[i] > 0.0f) ? 1 : 0;

        // Construct CAN packets (assuming a specific stepper motor control protocol)
        buf[0] = 2;        // Ctrl_Mode: 2 (position/speed control mode)
        buf[1] = dir;      // Direction
        buf[2] = 32;       // Microstep: 32 subdivisions
        buf[3] = steps >> 8;
        buf[4] = steps;    // Target Steps (8 bits high, 8 bits lower)
        buf[5] = spd >> 8;
        buf[6] = spd;      // Speed (high 8 bits, low 8 bits)
        buf[7] = 0;

        // Send data through CAN
        CAN1_Send_Num(motor_id[i], buf);
        A280.joint_flag[i] = 0; // Clear the move flag
    }
}