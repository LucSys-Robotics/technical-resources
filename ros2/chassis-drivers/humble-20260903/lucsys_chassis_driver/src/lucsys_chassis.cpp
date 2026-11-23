// Copyright 2026 LucSys Robotics - integration and maintenance changes only.
//
// LucSys Robotics distributes this source under supplier authorization.
// LucSys Robotics is responsible for integration and maintenance only.
// This notice does not grant an additional open-source license.
// Third-party components retain their respective licenses and notices.

#include "lucsys_chassis_driver/lucsys_chassis.h"
#include <cstdint>

/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
Functions: Main function, ROS initialization, creates Robot_control objects via turn_on_robot classes and automatically calls constructor initialization
***************************************/
int main(int argc, char ** argv)
{
  // ROS initializes and sets the node name //ROS init and set node name
  rclcpp::init(argc, argv);
  // Instantiate an object //Instantiate an object
  turn_on_robot Robot_Control;
  // Loop through data collection and publish the topic
  Robot_Control.Control();
  return 0;
}

// Generic BCC checksum: XOR the first length bytes of the specified buffer
uint8_t Calculate_BCC(const uint8_t * data, std::size_t length)
{
  uint8_t bcc = 0;
  for (std::size_t i = 0; i < length; ++i) {
    bcc ^= data[i];
  }
  return bcc;
}

/**************************************
Date: January 28, 2021
Function: Data conversion function
Function: Data conversion function
***************************************/
int16_t turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  int16_t transition_16;
  transition_16 = 0;
  transition_16 |= Data_High << 8;
  transition_16 |= Data_Low;
  return transition_16;
}
float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  float data_return;
  int16_t transition_16;
  transition_16 = 0;
  // Get the high 8 bits of data // Get the high 8 bits of data
  transition_16 |= Data_High << 8;
  // Get the lowest 8 bits of data // Get the lowest 8 bits of data
  transition_16 |= Data_Low;
  // Speed unit converted from mm/s to m/s // Speed unit changed from mm/s to m/s
  data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001;
  return data_return;
}

// Set the color of the light strip
void turn_on_robot::Set_LightRgb_Callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  // Check if the data length is 4
  if (msg->data.size() != 4) {
    RCLCPP_ERROR(
      this->get_logger(), "Data length error. It must be 4. Got %zu elements.", msg->data.size());
    return;
  }

  // Parse data: Index 0 is bool (0 or 1), index 1-3 is uint8
  uint8_t en_flag = (msg->data[0] == 1);
  uint8_t r = msg->data[1];
  uint8_t g = msg->data[2];
  uint8_t b = msg->data[3];

  Send_Data.tx[0] = FRAME_HEADER;
  // 04 represents the function of this frame to set the light strip
  Send_Data.tx[1] = 04;
  // Set the enable bit
  Send_Data.tx[2] = en_flag;
  Send_Data.tx[3] = r;
  Send_Data.tx[4] = g;
  Send_Data.tx[5] = b;
  Send_Data.tx[6] = 0;
  Send_Data.tx[7] = 0;
  Send_Data.tx[8] = 0;
  // BCC check byte // BCC check bit
  Send_Data.tx[9] = Calculate_BCC(Send_Data.tx, SEND_DATA_SIZE - 2);
  // frame tail 0x7D // Frame tail 0X7D
  Send_Data.tx[10] = FRAME_TAIL;
  try {
    // Sends data to the downloader via serial port //Send data to the downloader via serial port
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  } catch (serial::IOException & e) {
    // If sending data fails, an error message is printed
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
  }
}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
Function: Speed topic subscription callback function Callback, which controls lower-level computers via serial port based on subscribed instructions
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr ori_twist)
{
  // intermediate variable
  int16_t transition;

  // frame head 0x7B // Frame head 0X7B
  Send_Data.tx[0] = FRAME_HEADER;
  // set aside // Reserve a space
  Send_Data.tx[1] = AutoRecharge;
  // set aside // Reserve a space
  Send_Data.tx[2] = SecurityPLY;

  // The target velocity of the X-axis of the robot
  // Target linear velocity of the robot's x-axis
  transition = 0;
  // Floating-point numbers are amplified by a thousand times, simplifying transmission
  transition = ori_twist->linear.x * 1000;
  // Retrieve the lower 8 bits of the data
  Send_Data.tx[4] = transition;
  // Retrieve the data from the upper 8 bits
  Send_Data.tx[3] = transition >> 8;

  // The target velocity of the Y-axis of the robot
  // Target linear velocity along the robot's y-axis
  transition = 0;
  transition = ori_twist->linear.y * 1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition >> 8;

  // The target angular velocity of the robot's Z axis
  // Target angular velocity along the robot's z-axis
  transition = 0;
  transition = ori_twist->angular.z * 1000;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition >> 8;

  // BCC check byte // BCC check bit
  Send_Data.tx[9] = Calculate_BCC(Send_Data.tx, SEND_DATA_SIZE - 2);
  // frame tail 0x7D // Frame tail 0X7D
  Send_Data.tx[10] = FRAME_TAIL;
  try {
    // Sends data to the downloader via serial port //Send data to the downloader via serial port
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  } catch (serial::IOException & e) {
    // If sending data fails, an error message is printed
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
  }
}

/**
 * @brief Robotic arm control command callback function
 *
 * @param arm_cmd_msg Message containing robotic arm control data, data[0-2] are the three joint angles, data[3] is the gripper status
 *
 * This function receives robotic arm control commands and detects changes in joint angle and gripper status,
 * After converting control data into serial communication protocol format, it is sent via serial to the STM32 subcomputer.
 * Data is only sent when control data changes significantly (angle change > 0.01 degrees or clamp state change).
 */
void turn_on_robot::arm_cmd_Callback(const std_msgs::msg::Float32MultiArray arm_cmd_msg)
{
  // printf("joint_states_Callback \n");
  if (fabs(last_A - arm_cmd_msg.data[0]) > 0.01 ||
    fabs(last_B - arm_cmd_msg.data[1]) > 0.01 ||
    fabs(last_C - arm_cmd_msg.data[2]) > 0.01 ||
    last_grap != arm_cmd_msg.data[3])
  {
    last_A = arm_cmd_msg.data[0];
    last_B = arm_cmd_msg.data[1];
    last_C = arm_cmd_msg.data[2];
    last_grap = arm_cmd_msg.data[3];
  } else {return;}
  Send_Data.tx[0] = 0xAA;
  Send_Data.tx[1] = ((int16_t)(arm_cmd_msg.data[0] * 1000)) >> 8;
  Send_Data.tx[2] = (int16_t)(arm_cmd_msg.data[0] * 1000);

  Send_Data.tx[3] = ((int16_t)(arm_cmd_msg.data[1] * 1000)) >> 8;
  Send_Data.tx[4] = (int16_t)(arm_cmd_msg.data[1] * 1000);

  Send_Data.tx[5] = ((int16_t)(arm_cmd_msg.data[2] * 1000)) >> 8;
  Send_Data.tx[6] = (int16_t)(arm_cmd_msg.data[2] * 1000);

  Send_Data.tx[7] = (uint8_t)arm_cmd_msg.data[3];
  Send_Data.tx[8] = Calculate_BCC(Send_Data.tx, 8);
  Send_Data.tx[9] = 0xBB;

  try {
    // Send data to the serial port
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    // ROS_INFO_STREAM("New control command"); //Show that a new control command has been received
  } catch (serial::IOException & e) {
    // If sending data fails, an error message is printed
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
  }
}

/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
Function: Publish IMU data topics
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  // Instantiate IMU topic data //Instantiate IMU topic data
  sensor_msgs::msg::Imu Imu_Data_Pub;
  Imu_Data_Pub.header.stamp = this->now();
  // IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack
  Imu_Data_Pub.header.frame_id = gyro_frame_id;
  // IMU corresponds to TF coordinates; this setting
  // is required to use robot_pose_ekf function
  // package
  // A quaternion represents a three-axis attitude
  Imu_Data_Pub.orientation.x = onboard_imu.orientation.x;
  Imu_Data_Pub.orientation.y = onboard_imu.orientation.y;
  Imu_Data_Pub.orientation.z = onboard_imu.orientation.z;
  Imu_Data_Pub.orientation.w = onboard_imu.orientation.w;
  // Three-axis attitude covariance matrix
  Imu_Data_Pub.orientation_covariance[0] = 1e6;
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  // tues
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  // Triaxial angular velocity // Triaxial angular velocity
  Imu_Data_Pub.angular_velocity.x = onboard_imu.angular_velocity.x;
  Imu_Data_Pub.angular_velocity.y = onboard_imu.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = onboard_imu.angular_velocity.z;
  // Triaxial angular velocity covariance matrix // Triaxial angular velocity covariance matrix
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  // tues
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  // Triaxial acceleration // Triaxial linear acceleration
  Imu_Data_Pub.linear_acceleration.x = onboard_imu.linear_acceleration.x;
  Imu_Data_Pub.linear_acceleration.y = onboard_imu.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = onboard_imu.linear_acceleration.z;
  // Pub IMU topic //Publish IMU topic
  imu_publisher->publish(Imu_Data_Pub);
}
/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
Features: Post odometer topics including position, attitude, three-axis velocity, angular velocity around triaxes, TF parent-child coordinates, and covariance matrix
***************************************/
void turn_on_robot::Publish_Odom()
{
  // Convert the Z-axis rotation Angle into a quaternion for expression
  // Convert the Z-axis rotation angle into quaternions for expression
  tf2::Quaternion q;
  q.setRPY(0, 0, Robot_Pos.Z);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  // Instantiate the odometer topic data
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  // Odometer TF parent coordinates
  odom.header.frame_id = odom_frame_id;
  // Position // Position
  odom.pose.pose.position.x = Robot_Pos.X;
  odom.pose.pose.position.y = Robot_Pos.Y;
  odom.pose.pose.position.z = Robot_Pos.Z;
  // Posture, Quaternion converted by Z-axis rotation // Posture, a quaternion converted by Z-axis
  // rotation
  odom.pose.pose.orientation = odom_quat;

  // Odometer TF subcoordinates
  odom.child_frame_id = robot_frame_id;
  // Speed in the X direction // X direction speed
  odom.twist.twist.linear.x = Robot_Vel.X;
  // Speed in the Y direction // Y direction speed
  odom.twist.twist.linear.y = Robot_Vel.Y;
  // Angular velocity around the Z axis // Angular velocity around the Z axis
  odom.twist.twist.angular.z = Robot_Vel.Z;

  // There are two types of this matrix, which are used when the robot is at rest and when it is
  // moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature
  // pack
  // Two matrices are used for stationary and moving states; the two officially provided extended
  // Kalman filter matrices are for robot_pose_ekf
  if (Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0) {
    // If the velocity is zero, it means that the error of the encoder will be relatively small,
    // and the data of the encoder will be considered more reliable
    // If velocity is zero, it means the encoder's error is smaller, and the encoder data is
    // considered more reliable
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
    memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
  } else {
    // If the velocity of the trolley is non-zero, considering the sliding error that may be
    // brought by the encoder in motion, the data of IMU is considered to be more reliable
    // If the vehicle's velocity is non-zero, considering possible slippage from the encoder during
    // movement, the IMU data is considered more reliable
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
    memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
  }
  // Pub odometer topic //Publish odometer topic
  odom_publisher->publish(odom);
}
/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
Function: Publish voltage-related information
***************************************/
void turn_on_robot::Publish_Voltage()
{
  // Define the data type of the power supply voltage publishing topic // Define the data type of
  // the power supply voltage publishing topic
  std_msgs::msg::Float32 voltage_msgs;
  static float Count_Voltage_Pub = 0;
  if (Count_Voltage_Pub++ > 10) {
    Count_Voltage_Pub = 0;
    // The voltage supplied by the power supply // Voltage from the power supply
    voltage_msgs.data = Power_voltage;
    // Post the power supply voltage topic unit: V, volt // Post the power supply voltage topic
    // unit: V, volt
    voltage_publisher->publish(voltage_msgs);
  }
}

// //////// Recharge Release and Callback ////////
/**************************************
Date: January 17, 2022
Function: Pub the topic whether the robot finds the infrared signal (charging station)
Feature: Posts whether the robot has found infrared signals (charging stations).
***************************************/
void turn_on_robot::Publish_RED()
{
  std_msgs::msg::UInt8 msg;
  msg.data = Red;
  RED_publisher->publish(msg);
}
/**************************************
Date: January 14, 2022
Function: Publish a topic about whether the robot is charging
Feature: Post topics about whether the robot is charging
***************************************/
void turn_on_robot::Publish_Charging()
{
  static bool last_charging;
  std_msgs::msg::Bool msg;
  msg.data = Charging;
  Charging_publisher->publish(msg);
  if (last_charging == false && Charging == true) {
    std::cout << GREEN << "Robot is charging." << std::endl << RESET;
  }
  if (last_charging == true && Charging == false) {
    std::cout << RED << "Robot charging has disconnected." << std::endl << RESET;
  }
  last_charging = Charging;

  // Publish the low-level recharge setup status
  msg.data = charge_set_state;
  ChargingMode_publisher->publish(msg);
}
/**************************************
Date: January 28, 2021
Function: Publish charging current information
Function: Publishes charging current information
***************************************/
void turn_on_robot::Publish_ChargingCurrent()
{
  std_msgs::msg::Float32 msg;
  msg.data = Charging_Current;
  Charging_current_publisher->publish(msg);
}

/**************************************
Date:
Function:
Function: Publishes information related to ultrasonic measurement distances
***************************************/
void turn_on_robot::Publish_distance()
{
  // Ultrasonic sensor data
  sensor_msgs::msg::Range range_msg;
  // Use the node clock to obtain the timestamp
  range_msg.header.stamp = this->now();
  // Data type is ultrasonic
  range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  // Field of view, approximately 30°
  range_msg.field_of_view = 0.52;
  // 2cm blind spot
  range_msg.min_range = 0.02;
  // 5 meters maximum distance
  range_msg.max_range = 5.3;

  // Publish rangerA
  range_msg.header.frame_id = "ultrasonic_data_A";
  range_msg.range = distanceA;
  range_a_publisher_->publish(range_msg);

  // Publish rangerB
  range_msg.header.frame_id = "ultrasonic_data_B";
  range_msg.range = distanceB;
  range_b_publisher_->publish(range_msg);

  // Publish rangerC
  range_msg.header.frame_id = "ultrasonic_data_C";
  range_msg.range = distanceC;
  range_c_publisher_->publish(range_msg);

  // Publish rangerD
  range_msg.header.frame_id = "ultrasonic_data_D";
  range_msg.range = distanceD;
  range_d_publisher_->publish(range_msg);

  // Publish rangerE
  range_msg.header.frame_id = "ultrasonic_data_E";
  range_msg.range = distanceE;
  range_e_publisher_->publish(range_msg);

  // Publish rangerF
  range_msg.header.frame_id = "ultrasonic_data_F";
  range_msg.range = distanceF;
  range_f_publisher_->publish(range_msg);
}

/**************************************
Date: January 14, 2022
Function: Subscription robot recharge flag bit topic, used to tell the lower machine speed command is normal command or recharge command
Function: Subscribe to the topic of whether the bot recharges the flag, used to tell the subordinate computer whether the speed command is normal or recharge
***************************************/
void turn_on_robot::Recharge_Flag_Callback(const std_msgs::msg::Int8::SharedPtr Recharge_Flag)
{
  // Set the automatic recharge flag
  AutoRecharge = Recharge_Flag->data;

  // New version agreement
  // frame head 0x7B // Frame head 0X7B
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 1;

  if (AutoRecharge != 0) {Send_Data.tx[2] = 0xA1;} else {Send_Data.tx[2] = 0xA0;}

  Send_Data.tx[3] = 0;Send_Data.tx[4] = 0;
  Send_Data.tx[5] = 0;Send_Data.tx[6] = 0;
  Send_Data.tx[7] = 0;Send_Data.tx[8] = 0;
  // BCC check byte // BCC check bit
  Send_Data.tx[9] = Calculate_BCC(Send_Data.tx, SEND_DATA_SIZE - 2);
  // frame tail 0x7D // Frame tail 0X7D
  Send_Data.tx[10] = FRAME_TAIL;
  try {
    // Sends data to the downloader via serial port //Send data to the downloader via serial port
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  } catch (...) {
    // If sending data fails, an error message is printed
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
  }

  // Legacy Agreement
  if (AutoRecharge == 0) {
    // frame head 0x7B // Frame head 0X7B
    Send_Data.tx[0] = FRAME_HEADER;
    // set aside // Reserve a space
    Send_Data.tx[1] = AutoRecharge;
    // set aside // Reserve a space
    Send_Data.tx[2] = 0;
    Send_Data.tx[3] = 0;Send_Data.tx[4] = 0;
    Send_Data.tx[5] = 0;Send_Data.tx[6] = 0;
    Send_Data.tx[7] = 0;Send_Data.tx[8] = 0;
    // BCC check byte // BCC check bit
    Send_Data.tx[9] = Calculate_BCC(Send_Data.tx, SEND_DATA_SIZE - 2);
    // frame tail 0x7D // Frame tail 0X7D
    Send_Data.tx[10] = FRAME_TAIL;
    try {
      // Sends data to the downloader via serial port //Send data to the downloader via serial port
      Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    } catch (...) {
      // If sending data fails, an error message is printed
      RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
    }
  }
}

// Flag for chassis safe opening or closing
void turn_on_robot::Security_Callback(const std_msgs::msg::Int8 & Security_Flag)
{
  SecurityPLY = Security_Flag.data;

  // New version agreement
  // frame head 0x7B // Frame head 0X7B
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;

  if (SecurityPLY != 0) {Send_Data.tx[2] = 0xB1;} else {Send_Data.tx[2] = 0xB0;}

  Send_Data.tx[3] = 0;Send_Data.tx[4] = 0;
  Send_Data.tx[5] = 0;Send_Data.tx[6] = 0;
  Send_Data.tx[7] = 0;Send_Data.tx[8] = 0;
  // BCC check byte // BCC check bit Send_Data.tx[10]=FRAME_TAIL; frame tail 0x7D // Frame tail
  // 0X7D
  Send_Data.tx[9] = Calculate_BCC(Send_Data.tx, SEND_DATA_SIZE - 2);
  try {
    // Sends data to the downloader via serial port //Send data to the downloader via serial port
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  } catch (...) {
    // If sending data fails, an error message is printed
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
  }

  // Legacy Agreement
  // frame head 0x7B // Frame head 0X7B
  Send_Data.tx[0] = FRAME_HEADER;
  // set aside // Reserve a space
  Send_Data.tx[1] = 0;
  // set aside // Reserve a space
  Send_Data.tx[2] = SecurityPLY;
  Send_Data.tx[3] = 0;Send_Data.tx[4] = 0;
  Send_Data.tx[5] = 0;Send_Data.tx[6] = 0;
  Send_Data.tx[7] = 0;Send_Data.tx[8] = 0;
  // BCC check byte // BCC check bit
  Send_Data.tx[9] = Calculate_BCC(Send_Data.tx, SEND_DATA_SIZE - 2);
  // frame tail 0x7D // Frame tail 0X7D
  Send_Data.tx[10] = FRAME_TAIL;
  try {
    // Sends data to the downloader via serial port //Send data to the downloader via serial port
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  } catch (...) {
    // If sending data fails, an error message is printed
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
  }
}

/**************************************
Date: November 18, 2021
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
Function: Read and verify data sent from the lower computer frame by frame via serial port, then convert the data into international units
***************************************/
bool turn_on_robot::Get_Sensor_Data()
{
  // Intermediate variable //Intermediate variable
  int16_t transition_16 = 0;
  // Temporary variable to save lower machine data //Temporary variable to store lower machine data
  uint8_t check = 0, check2 = 0, check3 = 0, error = 1, error2 = 1, error3 = 1, Receive_Data_Pr[1];
  // Static variable for counting // Static variable for counting
  static int count, count2, count3;
  static uint8_t Last_Receive;
  if (Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr)) != 1) {
    // No received byte: do not parse uninitialized data.
    return false;
    // //Read the data sent by the lower computer through the serial port //Read data sent from
    // lower
    // computer via serial
  }

  // Fill the array with serial data //Fill the array with serial data
  Receive_Data.rx[count] = Receive_Data_Pr[0];
  Receive_AutoCharge_Data.rx[count3] = Receive_Data_Pr[0];
  Distance_Data.rx[count2] = Receive_Data_Pr[0];

  // The first part of the data is the frame header 0X7B // The first byte of the data is the frame
  // header 0X7B
  Receive_Data.Frame_Header = Receive_Data.rx[0];
  // The last bit of data is frame tail 0X7D // The last byte of the data is the frame tail 0X7D
  Receive_Data.Frame_Tail = Receive_Data.rx[23];

  // Basic packet reception (7F 7B or 7D 7B)
  if ( ((Last_Receive == AutoCharge_TAIL || Last_Receive == FRAME_TAIL ) &&
    Receive_Data_Pr[0] == FRAME_HEADER) ||
    count > 0)
  {
    count++;
  } else {
    count = 0;
  }

  // Automatic recharge packet reception (FC 7C or 7D 7C)
  if ( ((Last_Receive == Distance_TAIL || Last_Receive == FRAME_TAIL ) &&
    Receive_Data_Pr[0] == AutoCharge_HEADER) ||
    count3 > 0)
  {
    count3++;
  } else {
    count3 = 0;
  }

  // Ultrasonic data reception (7D FA)
  if ( (Last_Receive == FRAME_TAIL && Receive_Data_Pr[0] == Distance_HEADER) ||
    count2 > 0)
  {
    count2++;
  } else {
    count2 = 0;
  }

  // Save the data received this time
  Last_Receive = Receive_Data_Pr[0];

  // Automatic recharge data processing
  if (count3 == AutoCharge_DATA_SIZE) {
    count3 = 0;
    // Confirm the frame end
    if (Receive_AutoCharge_Data.rx[AutoCharge_DATA_SIZE - 1] == AutoCharge_TAIL) {
      // Check bit calculation
      check3 = Calculate_BCC(Receive_AutoCharge_Data.rx, AutoCharge_DATA_SIZE - 2);
      // Validation is correct
      if (check3 == Receive_AutoCharge_Data.rx[AutoCharge_DATA_SIZE - 2]) {
        error3 = 0;
      }
      // Validation is correct and start assignment
      if (error3 == 0) {
        transition_16 = 0;
        transition_16 |= Receive_AutoCharge_Data.rx[1] << 8;
        transition_16 |= Receive_AutoCharge_Data.rx[2];
        // Charging current
        Charging_Current = transition_16 / 1000 + (transition_16 % 1000) * 0.001;

        // Infrared acceptance status
        Red = Receive_AutoCharge_Data.rx[3];
        // Car charging status
        Charging = Receive_AutoCharge_Data.rx[4];
        charge_set_state = Receive_AutoCharge_Data.rx[5];
        // Data successfully received flags
        check_AutoCharge_data = true;
      }
    }
  }

  // Ultrasound data
  if (count2 == Distance_DATA_size) {
    count2 = 0;
    if (Distance_Data.rx[Distance_DATA_size - 1] == Distance_TAIL) {
      check2 = Calculate_BCC(Distance_Data.rx, 17);
      if (check2 == Distance_Data.rx[Distance_DATA_size - 2]) {
        error2 = 0;
      }
      if (error2 == 0) {
        distanceA = ((int16_t)((Distance_Data.rx[1] << 8) | (Distance_Data.rx[2] ))) / 1000.0f;
        distanceB = ((int16_t)((Distance_Data.rx[3] << 8) | (Distance_Data.rx[4] ))) / 1000.0f;
        distanceC = ((int16_t)((Distance_Data.rx[5] << 8) | (Distance_Data.rx[6] ))) / 1000.0f;
        distanceD = ((int16_t)((Distance_Data.rx[7] << 8) | (Distance_Data.rx[8] ))) / 1000.0f;
        distanceE = ((int16_t)((Distance_Data.rx[9] << 8) | (Distance_Data.rx[10]))) / 1000.0f;
        distanceF = ((int16_t)((Distance_Data.rx[11] << 8) | (Distance_Data.rx[12]))) / 1000.0f;
        distanceG = 0;
        distanceH = 0;
        // Self-check data
        // temp_selfcheck =
        // (uint32_t)(Distance_Data.rx[13]<<24|Distance_Data.rx[14]<<16|Distance_Data.rx[15]<<8|Dis
        // tance_Data.rx[16]);
        // distance.G = ((int16_t)((Distance_Data.rx[13]<<8)|(Distance_Data.rx[14])))/1000.0f;
        // distance.H = ((int16_t)((Distance_Data.rx[15]<<8)|(Distance_Data.rx[16])))/1000.0f;
        distance_flag = true;
      }
    }
  }

  // Verify the length of the packet // Verify the length of the packet
  if (count == 24) {
    // Prepare for the serial port data to be refill into the array //Get ready to refill data from
    // the serial port into the array
    count = 0;
    // Verify the frame tail of the packet // Verify the frame tail of the packet
    if (Receive_Data.Frame_Tail == FRAME_TAIL) {
      // BCC check // BCC verification
      check = Calculate_BCC(Receive_Data.rx, RECEIVE_DATA_SIZE - 2);

      if (check == Receive_Data.rx[22]) {
        // XOR bit check successful // XOR bit check successful
        error = 0;
      }
      if (error == 0) {
        // set aside // Reserve a space
        Receive_Data.Flag_Stop = Receive_Data.rx[1];
        // Get the speed of the moving chassis in the X direction // Get the speed of the moving
        // chassis in the X direction
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);

        // Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the
        // omnidirectional mobile robot chassis
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
        // Obtain the Y-direction speed
        // of the moving chassis; Y speed
        // is only valid for
        // omnidirectional mobile robot
        // chassis
        // Get the speed of the moving chassis in the Z direction // Get the speed of the moving
        // chassis in the Z direction
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);

        // Parse raw data from the onboard IMU without depending on a specific sensor model.
        // Parses raw data from the onboard IMU, without relying on specific sensor models.
        // Get the X-axis acceleration of the IMU // Get the X-axis acceleration of the IMU
        onboard_imu_raw_data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);
        // Get the Y-axis acceleration of the IMU // Get the Y-axis acceleration of the IMU
        onboard_imu_raw_data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);
        // Get the Z-axis acceleration of the IMU // Get the Z-axis acceleration of the IMU
        onboard_imu_raw_data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);
        // Get the X-axis angular velocity of the IMU // Get the X-axis angular velocity of the IMU
        onboard_imu_raw_data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);
        // Get the Y-axis angular velocity of the IMU // Get the Y-axis angular velocity of the IMU
        onboard_imu_raw_data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);
        // Get the Z-axis angular velocity of the IMU // Get the Z-axis angular velocity of the IMU
        onboard_imu_raw_data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);
        // Linear acceleration unit conversion is related to the range of IMU initialization of
        // STM32, where the range is ±2g=19.6m/s^2
        // Linear acceleration unit conversion is related to the range at IMU initialization on
        // STM32, where ± range is 2g = 19.6m/s^2
        onboard_imu.linear_acceleration.x = onboard_imu_raw_data.accele_x_data / ACCEl_RATIO;
        onboard_imu.linear_acceleration.y = onboard_imu_raw_data.accele_y_data / ACCEl_RATIO;
        onboard_imu.linear_acceleration.z = onboard_imu_raw_data.accele_z_data / ACCEl_RATIO;
        // The gyroscope unit conversion is related to the range of STM32's IMU when initialized.
        // Here, the range of IMU's gyroscope is ±500°/s
        // Because the robot generally has a slow Z-axis speed, reducing the range can improve the
        // accuracy
        // Gyroscope unit conversion depends on the range set during STM32 IMU initialization; here
        // the range is +/-500 degrees/s
        // Since robots generally have low Z-axis speeds, reducing the range can improve accuracy
        onboard_imu.angular_velocity.x = onboard_imu_raw_data.gyros_x_data * GYROSCOPE_RATIO;
        onboard_imu.angular_velocity.y = onboard_imu_raw_data.gyros_y_data * GYROSCOPE_RATIO;
        onboard_imu.angular_velocity.z = onboard_imu_raw_data.gyros_z_data * GYROSCOPE_RATIO;

        // Get the battery voltage
        // Obtain battery voltage
        transition_16 = 0;
        transition_16 |= Receive_Data.rx[20] << 8;
        transition_16 |= Receive_Data.rx[21];
        // Unit conversion millivolt (mv) - >volt (v) // Unit conversion millivolt (mv) - > volt
        // (v)
        Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001;

        return true;
      }
    }
  }
  return false;
}
/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
Function: Loop to retrieve subordinate computer data and publish topics
***************************************/
void turn_on_robot::Control()
{
  _Last_Time = this->now();
  while (rclcpp::ok()) {
    try {
      _Now = this->now();
      // Retrieves time interval, which is used to integrate velocity to obtain displacement
      // (mileage)
      Sampling_Time = (_Now - _Last_Time).seconds();
      // Get the time interval to get displacement (mileage) by integrating speed
      // The serial port reads and verifies the data sent by the lower computer, and then the data
      // is converted to international units
      if (true == Get_Sensor_Data()) {
        // Read and validate controller data through the serial
        // port, then convert it to SI units

        // Odometer error correction
        Robot_Vel.X = Robot_Vel.X * odom_x_scale;
        Robot_Vel.Y = Robot_Vel.Y * odom_y_scale;
        if (Robot_Vel.Z >= 0) {
          Robot_Vel.Z = Robot_Vel.Z * odom_z_scale_positive;
        } else {
          Robot_Vel.Z = Robot_Vel.Z * odom_z_scale_negative;
        }

        Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) *
          // Calculate the displacement in the X direction, unit: m // Calculate the displacement
          // in the X direction, unit: m
          Sampling_Time;
        Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) *
          // Calculate the displacement in the Y direction, unit: m // Calculate the displacement
          // in the Y direction, unit: m
          Sampling_Time;
        // The angular displacement about the Z-axis, in rad // Angular displacement around Z-axis,
        // in rad
        Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

        // Calculate the three-axis attitude from the IMU with the angular velocity around the
        // three-axis and the three-axis acceleration
        // Calculate three-axis attitude from IMU three-axis angular velocity and acceleration
        Quaternion_Solution(
          onboard_imu.angular_velocity.x, onboard_imu.angular_velocity.y,
          onboard_imu.angular_velocity.z, \
          onboard_imu.linear_acceleration.x, onboard_imu.linear_acceleration.y,
          onboard_imu.linear_acceleration.z);

        // Pub the speedometer topic //Publish speedometer topic
        Publish_Odom();
        // Pub the IMU topic //Publish IMU topic
        Publish_ImuSensor();
        // Pub the topic of power supply voltage //Publish topic of power supply voltage
        Publish_Voltage();

        // Record the time and use it to calculate the time interval // Record time for calculating
        // the interval
        _Last_Time = _Now;
      }

      // Automatically recharge data topics
      if (check_AutoCharge_data) {
        // Pub a topic about whether the robot is charging //Publish a topic about whether the
        // robot is charging
        Publish_Charging();
        // Pub the topic whether the robot finds the infrared signal (charging station) //Publish
        // topic about whether the robot detects the infrared signal (charging station)
        Publish_RED();
        // Pub the charging current topic //Publish charging current topic
        Publish_ChargingCurrent();
        check_AutoCharge_data = false;
      }

      // Ultrasound data
      if (distance_flag == true) {
        // Publish ultrasonic data
        Publish_distance();
        distance_flag = false;
      }
      // The loop waits for the callback function // Loop waiting for the callback function
      rclcpp::spin_some(this->get_node_base_interface());
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(this->get_logger(), "unexpectedly failed whith %s", e.what());
    }
  }
}
/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
Function: Constructor, executed only once, used for initialization
***************************************/
turn_on_robot::turn_on_robot()
: rclcpp::Node("lucsys_robot")
{
  // Initialize member variables
  check_AutoCharge_data = false;
  charge_set_state = false;
  last_A = 0.0;
  last_B = 0.0;
  last_C = 0.0;
  last_grap = 0.0;
  distance_flag = false;
  SecurityPLY = false;
  distanceA = 5.1f;
  distanceB = 5.1f;
  distanceC = 5.1f;
  distanceD = 5.1f;
  distanceE = 5.1f;
  distanceF = 5.1f;
  distanceG = 5.1f;
  distanceH = 5.1f;
  Sampling_Time = 0;
  Power_voltage = 0;
  AutoRecharge = 0;
  Charging = false;
  Red = 0;
  Charging_Current = 0.0f;
  // Clear the data
  // Clear data
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&onboard_imu_raw_data, 0, sizeof(onboard_imu_raw_data));

  // ros::NodeHandle private_nh("~"); Create a node handle // Create a node handle
  // The private_nh.param() entry parameter corresponds to the initial value of the name of the
  // parameter variable on the parameter server
  // private_nh.param() entry parameters correspond to: name on the parameter server, parameter
  // variable name, initial value

  this->declare_parameter<int>("serial_baud_rate", 115200);

  this->declare_parameter<std::string>("usart_port_name", "/dev/lucsys_controller");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  this->declare_parameter<std::string>("car_mode", "");
  this->declare_parameter<double>("odom_x_scale", 1.0);
  this->declare_parameter<double>("odom_y_scale", 1.0);
  this->declare_parameter<double>("odom_z_scale_positive", 1.0);
  this->declare_parameter<double>("odom_z_scale_negative", 1.0);

  // Communicate baud rate 115200 to the lower machine // Communicate with the lower machine at
  // 115200 baud rate
  this->get_parameter("serial_baud_rate", serial_baud_rate);
  // Fixed serial port number //Fixed serial port number
  this->get_parameter("usart_port_name", usart_port_name);
  // The odometer topic corresponds to the parent TF coordinate // Odometer topic corresponds to
  // parent TF coordinates
  this->get_parameter("odom_frame_id", odom_frame_id);
  // The odometer topic corresponds to sub-TF coordinates // Odometer topic corresponds to child TF
  // coordinates
  this->get_parameter("robot_frame_id", robot_frame_id);
  // IMU topics correspond to TF coordinates
  this->get_parameter("gyro_frame_id", gyro_frame_id);
  this->get_parameter("car_mode", car_mode);
  this->get_parameter("odom_x_scale", odom_x_scale);
  this->get_parameter("odom_y_scale", odom_y_scale);
  this->get_parameter("odom_z_scale_positive", odom_z_scale_positive);
  this->get_parameter("odom_z_scale_negative", odom_z_scale_negative);

  // Convert car_mode to lowercase to facilitate later model mode determination
  std::transform(
    car_mode.begin(), car_mode.end(), car_mode.begin(),
    [](unsigned char c) {return std::tolower(c);});

  // Create the odometer topic publisher // Create odometer topic publisher
  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 2);
  // Create an IMU topic publisher // Create IMU topic publisher
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2);
  // Create a battery-voltage topic publisher // Create battery voltage topic publisher
  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);

  // Recharge the publisher
  Charging_publisher = create_publisher<std_msgs::msg::Bool>("robot_charging_flag", 10);
  Charging_current_publisher =
    create_publisher<std_msgs::msg::Float32>("robot_charging_current", 10);
  RED_publisher = create_publisher<std_msgs::msg::UInt8>("robot_red_flag", 10);
  ChargingMode_publisher = create_publisher<std_msgs::msg::Bool>("robot_charging_mode", 1);

  // Ultrasound publisher
  range_a_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_A", 10);
  range_b_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_B", 10);
  range_c_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_C", 10);
  range_d_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_D", 10);
  range_e_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_E", 10);
  range_f_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_F", 10);

  // Recharge subscribers
  Recharge_Flag_Sub = create_subscription<std_msgs::msg::Int8>(
    "robot_recharge_flag", 10,
    std::bind(&turn_on_robot::Recharge_Flag_Callback, this, std::placeholders::_1));
  // Provision of recharge services
  // SetCharge_Service=this-
  // >create_service<turtlesim::srv::Spawn>("/set_charge",std::bind(&turn_on_robot::Set_Charge_Call
  // back,this,std::placeholders::_1
  // ,std::placeholders::_2));
  arm_cmd_Sub = create_subscription<std_msgs::msg::Float32MultiArray>(
    "arm_cmd", 10, std::bind(&turn_on_robot::arm_cmd_Callback, this, std::placeholders::_1));
  SetRgb_Sub = create_subscription<std_msgs::msg::UInt8MultiArray>(
    "set_rgb_color", 5, std::bind(
      &turn_on_robot::Set_LightRgb_Callback, this,
      std::placeholders::_1));

  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, std::placeholders::_1));

  // Chassis Security Protection: Close topic subscriptions
  Security_Sub =
    create_subscription<std_msgs::msg::Int8>(
    "chassis_security", 2,
    std::bind(&turn_on_robot::Security_Callback, this, std::placeholders::_1));

  // Prompt message //Prompt message
  RCLCPP_INFO(this->get_logger(), "LucSys chassis data ready");

  try {
    // Attempts to initialize and open the serial port // Attempts to initialize and open the
    // serial port
    // Select the serial port number to enable //Select the serial port number to enable
    Stm32_Serial.setPort(usart_port_name);
    // Set the baud rate // Set the baud rate
    Stm32_Serial.setBaudrate(serial_baud_rate);
    // Timeout // Timeout waiting
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    // Open the serial port
    Stm32_Serial.open();
    Stm32_Serial.flushInput();
  } catch (serial::IOException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      // If opening the serial port fails, an error message is printed
      "LucSys chassis cannot open serial port,Please check the serial port cable! ");
  }
  if (Stm32_Serial.isOpen()) {
    // Serial port opened successfully //Serial port opened successfully
    RCLCPP_INFO(this->get_logger(), "LucSys chassis serial port opened");
  }
}
/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
Function: Destructor, executed only once; the system calls this function when the object ends its lifecycle
***************************************/
turn_on_robot::~turn_on_robot()
{
  // Sends the stop motion command to the lower machine before the turn_on_robot object ends
  // Send a stop motion command to the subcomputer before the object finishes turn_on_robot
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  // Target velocity of the robot's X-axis // Target velocity of the robot's X-axis
  Send_Data.tx[4] = 0;
  Send_Data.tx[3] = 0;

  // Target velocity of the robot's Y-axis // Target velocity of the robot's Y-axis
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;

  // Target angular velocity of the robot's Z-axis // Target velocity of the robot's Z-axis
  Send_Data.tx[8] = 0;
  Send_Data.tx[7] = 0;
  // BCC check byte // BCC check bit
  Send_Data.tx[9] = Calculate_BCC(Send_Data.tx, SEND_DATA_SIZE - 2);
  Send_Data.tx[10] = FRAME_TAIL;
  try {
    // Send data to the serial port //Send data to the serial port
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  } catch (serial::IOException & e) {
    // If sending data fails, an error message is printed
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port");
  }

  // Sends the stop motion command to the lower machine before the turn_on_robot object ends
  // Send a stop motion command to the subcomputer before the object finishes turn_on_robot
  Send_Data.tx[0] = 0xAA;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  Send_Data.tx[3] = ((int16_t)(1.5707 * 1000)) >> 8;
  Send_Data.tx[4] = (int16_t)(1.5707 * 1000);

  Send_Data.tx[5] = ((int16_t)(0.3917 * 1000)) >> 8;
  Send_Data.tx[6] = (int16_t)(0.3917 * 1000);

  // Target angular velocity of the robot's Z-axis // Target velocity of the robot's Z-axis
  Send_Data.tx[7] = 0;
  // BCC check byte // BCC check bit
  Send_Data.tx[8] = Calculate_BCC(Send_Data.tx, 8);
  Send_Data.tx[9] = 0xBB;

  try {
    // Send data to the serial port //Send data to the serial port
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  } catch (serial::IOException & e) {
    // If sending data fails, an error message is printed
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port");
  }

  // Close the serial port // Close the serial port
  Stm32_Serial.close();
  // Prompt message //Prompt message
  RCLCPP_INFO(this->get_logger(), "Shutting down");
}
