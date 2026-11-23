// Copyright 2026 LucSys Robotics - integration and maintenance changes only.
//
// LucSys Robotics distributes this source under supplier authorization.
// LucSys Robotics is responsible for integration and maintenance only.
// This notice does not grant an additional open-source license.
// Third-party components retain their respective licenses and notices.

#ifndef LUCSYS_CHASSIS_DRIVER__LUCSYS_CHASSIS_H_
#define LUCSYS_CHASSIS_DRIVER__LUCSYS_CHASSIS_H_

#include <tf2_ros/transform_broadcaster.h>
#include <serial/serial.h>
#include <fcntl.h>
#include <math.h>
#include <rcl/types.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdint>
#include <iostream>
#include <string>
#include "lucsys_chassis_driver/Quaternion_Solution.h"
#include "lucsys_robot_msg/msg/data.hpp"
#include "lucsys_robot_msg/msg/supersonic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <turtlesim/srv/spawn.hpp>

// New additions related to recharge

// Ultrasound data

using std::string;

#define RESET   string("\033[0m")
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"

// Macro definition
// Macro definition
// Send data check flag bits //Send data check flag bits
#define SEND_DATA_CHECK   1
// Receive data to check flag bits // Receive data to check flag bits
#define READ_DATA_CHECK   0
// Frame head // Frame head
#define FRAME_HEADER      0X7B
// Frame tail // Frame tail
#define FRAME_TAIL        0X7D
// The length of the data sent by the lower computer // Length of data sent by the lower computer
#define RECEIVE_DATA_SIZE 24
// The length of data sent by ROS to the lower machine // Length of data sent from ROS to the lower
// computer
#define SEND_DATA_SIZE    11
// PI // Pi
#define PI                                3.1415926f
// Ultrasonic measurement of distance-related variables
#define Distance_DATA_size 19
// Frame_header // Frame header
#define Distance_HEADER    0XFA
// Frame_tail // End of frame
#define Distance_TAIL      0XFC

// Automatic recharge related to recharge
// Frame_header // Automatically recharge the data frame head
#define AutoCharge_HEADER      0X7C
// Frame_tail // Automatically recharge the end of the data frame
#define AutoCharge_TAIL        0X7F
// The length of the automatic recharge data sent from the lower computer
#define AutoCharge_DATA_SIZE    8

// Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is
// ±32768
// The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
// Related to the range set by the IMU gyroscope, range ± 500°, corresponding data range ± 32768
// Convert raw gyroscope data to radians (rad): 1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
// Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is
// ±32768
// Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84
// Related to the range set by the IMU accelerometer, which ± 2g, corresponding to a data range
// ±32768
// The original accelerometer data conversion bit m/s^2 unit, 32768/2g = 32768/19.6=1671.84
#define ACCEl_RATIO       1671.84f

// extern sensor_msgs::msg::Imu Mpu6050; // External variables, IMU topic data

// Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
// Covariance matrix, used for odometer topic data, used for robt_pose_ekf function packages
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
  0, 1e-3, 0, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e3};

const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
  0, 1e-3, 1e-9, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e-9};

const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
  0, 1e-3, 0, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e3};

const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
  0, 1e-3, 1e-9, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e-9};

// Data structure for speed and position
// Velocity/position data structure
typedef struct
{
  float X;
  float Y;
  float Z;
} Vel_Pos_Data;

// IMU data structure
// IMU data structure
typedef struct
{
  int16_t accele_x_data;
  int16_t accele_y_data;
  int16_t accele_z_data;
  int16_t gyros_x_data;
  int16_t gyros_y_data;
  int16_t gyros_z_data;
} IMU_DATA;

// The structure of the ROS to send data to the down machine
// Structure for ROS to send data to the lower computer
typedef struct
{
  uint8_t tx[SEND_DATA_SIZE];
  float X_speed;
  float Y_speed;
  float Z_speed;
  unsigned char Frame_Tail;
} SEND_DATA;
// Data structures related to automatic recharge sent by the subcomputer
typedef struct
{
  // 8 bytes
  uint8_t rx[AutoCharge_DATA_SIZE];
  // Frame header
  unsigned char Frame_Header;
  // Frame end
  unsigned char Frame_Tail;
} RECEIVE_AutoCharge_DATA;
// Ultrasound-related data structures sent by the subordinate computer
// Ultrasonic data structures sent by the subcomputer to ROS
typedef struct
{
  uint8_t rx[Distance_DATA_size];
  unsigned char Frame_Header;
  unsigned char Frame_Tail;
} DISTANCE_DATA;

typedef struct
{
  float A;
  float B;
  float C;
  float D;
  float E;
  float F;
  float G;
  float H;
} Supersonic_data;

// The structure in which the lower computer sends data to the ROS
// Structure that sends data from the subcomputer to ROS
typedef struct
{
  uint8_t rx[RECEIVE_DATA_SIZE];
  uint8_t Flag_Stop;
  unsigned char Frame_Header;
  float X_speed;
  float Y_speed;
  float Z_speed;
  float Power_Voltage;
  unsigned char Frame_Tail;
} RECEIVE_DATA;

// The robot chassis class uses constructors to initialize data, publish topics, etc
// Robot chassis class, using constructors to initialize data and publish topics, etc
class turn_on_robot: public rclcpp::Node
{
public:
  // Constructor // Constructor
  turn_on_robot();
  // Destructor // Destructor
  ~turn_on_robot();
  // Loop control code
  void Control();
  // Declare a serial object // Declare a serial port object
  serial::Serial Stm32_Serial;

private:
  // ros::NodeHandle n; Create a ROS node handle // Create a ROS node handle
  // Time dependent, used for integration to find displacement (mileage) // Time-dependent, used
  // for integration to find displacement (mileage)
  rclcpp::Time _Now, _Last_Time;
  // Sampling time, used for integration to find displacement (mileage) // Sampling time, used for
  // integration to find displacement (mileage)
  float Sampling_Time;
  rclcpp::Subscription < std_msgs::msg::Float32MultiArray > ::SharedPtr arm_cmd_Sub;

  // Initialize the topic subscriber
  rclcpp::Subscription < geometry_msgs::msg::Twist > ::SharedPtr Cmd_Vel_Sub;

  rclcpp::Subscription < std_msgs::msg::Bool > ::SharedPtr AvoidFlag_Sub;

  // Initialize the topic publisher
  rclcpp::Publisher < nav_msgs::msg::Odometry > ::SharedPtr odom_publisher;
  rclcpp::Publisher < std_msgs::msg::Float32 > ::SharedPtr voltage_publisher;
  rclcpp::Publisher < sensor_msgs::msg::Imu > ::SharedPtr imu_publisher;
  rclcpp::Publisher < lucsys_robot_msg::msg::Supersonic > ::SharedPtr distance_publisher;

  // Ultrasound publisher
  rclcpp::Publisher < sensor_msgs::msg::Range > ::SharedPtr range_a_publisher_;
  rclcpp::Publisher < sensor_msgs::msg::Range > ::SharedPtr range_b_publisher_;
  rclcpp::Publisher < sensor_msgs::msg::Range > ::SharedPtr range_c_publisher_;
  rclcpp::Publisher < sensor_msgs::msg::Range > ::SharedPtr range_d_publisher_;
  rclcpp::Publisher < sensor_msgs::msg::Range > ::SharedPtr range_e_publisher_;
  rclcpp::Publisher < sensor_msgs::msg::Range > ::SharedPtr range_f_publisher_;

  // Recharge relevant publishers
  rclcpp::Publisher < std_msgs::msg::Bool > ::SharedPtr Charging_publisher;
  rclcpp::Publisher < std_msgs::msg::Float32 > ::SharedPtr Charging_current_publisher;
  rclcpp::Publisher < std_msgs::msg::UInt8 > ::SharedPtr RED_publisher;
  rclcpp::Publisher < std_msgs::msg::Bool > ::SharedPtr ChargingMode_publisher;

  // Recharge relevant subscribers
  rclcpp::Subscription < geometry_msgs::msg::Twist > ::SharedPtr Red_Vel_Sub;
  rclcpp::Subscription < std_msgs::msg::Int8 > ::SharedPtr Recharge_Flag_Sub;
  // Recharge-related services
  rclcpp::Service < turtlesim::srv::Spawn > ::SharedPtr SetCharge_Service;
  // Chassis safety level settings
  rclcpp::Subscription < std_msgs::msg::Int8 > ::SharedPtr Security_Sub;
  // Light strip settings
  rclcpp::Subscription < std_msgs::msg::UInt8MultiArray > ::SharedPtr SetRgb_Sub;
  // The speed topic subscribes to the callback function
  // Velocity-topic subscription callback
  void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
  void arm_cmd_Callback(const std_msgs::msg::Float32MultiArray arm_joint);
  // Recharge-related callback functions
  void Recharge_Flag_Callback(const std_msgs::msg::Int8::SharedPtr Recharge_Flag);
  // Chassis safety level callback function
  void Security_Callback(const std_msgs::msg::Int8 & Security_Flag);
  void Set_LightRgb_Callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  // Recharge related publishing functions
  void Publish_Charging();
  void Publish_ChargingCurrent();
  void Publish_RED();

  // Pub the speedometer topic //Publish speedometer topic
  void Publish_Odom();
  // Pub the IMU sensor topic //Publish the IMU sensor topic
  void Publish_ImuSensor();
  // Pub the power supply voltage topic //Publish power supply voltage topic
  void Publish_Voltage();
  // Publish ultrasonic distance
  void Publish_distance();
  // Read the sports chassis speed, IMU, and power supply voltage data from the serial port
  // (ttyUSB).
  // Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
  bool  Get_Sensor_Data();
  // IMU data conversion read // IMU data converted read
  int16_t IMU_Trans(uint8_t Data_High, uint8_t Data_Low);
  // Odometer data converted for reading
  float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);

  string car_mode;
  // Define the related variables // Define the related variables
  string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id;
  string cmd_vel;
  bool ranger_avoid_flag;
  // Serial communication baud rate //Serial communication baud rate
  int serial_baud_rate;
  // The data structure received by the serial port // Data structure received by the serial port
  RECEIVE_DATA Receive_Data;
  // The data structure sent by the serial port // Data structure sent by the serial port
  SEND_DATA Send_Data;
  // Ultrasound data
  DISTANCE_DATA Distance_Data;
  // Serial port receives automatic recharge data structures
  RECEIVE_AutoCharge_DATA Receive_AutoCharge_Data;
  // Ultrasonic distance objects
  Supersonic_data distance;
  // The position of the robot // Robot's position
  Vel_Pos_Data Robot_Pos;
  // The robot's speed // Robot's speed
  Vel_Pos_Data Robot_Vel;
  // Onboard IMU raw data
  IMU_DATA onboard_imu_raw_data;

  int8_t AutoRecharge = 0;
  // Power supply voltage // Power supply voltage
  float Power_voltage;
  // Whether the robot is charging the flag bit // Is the robot charging the flag bit?
  bool Charging = 0;
  // Charging_Current // Charging current
  float Charging_Current = 0;
  // Whether the robot finds the marker bit of infrared signal (charging pile) // Whether the robot
  // detects the marker bit of the infrared signal (charging pile)
  uint8_t Red = 0;
  // Odometer correction parameters
  float odom_x_scale, odom_y_scale, odom_z_scale_positive, odom_z_scale_negative;
  bool check_AutoCharge_data;
  bool charge_set_state;
  double last_A, last_B, last_C, last_grap;

  // Ultrasound-related landmarks
  bool distance_flag;

  // Chassis safety level setting bit
  bool SecurityPLY;

  // Ultrasonic sensor data
  float distanceA;
  float distanceB;
  float distanceC;
  float distanceD;
  float distanceE;
  float distanceF;
  float distanceG;
  float distanceH;
};
#endif  // LUCSYS_CHASSIS_DRIVER__LUCSYS_CHASSIS_H_
