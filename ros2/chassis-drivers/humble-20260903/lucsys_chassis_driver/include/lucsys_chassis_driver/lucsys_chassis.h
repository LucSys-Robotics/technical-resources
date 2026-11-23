
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h> 
#include <stdlib.h>    
#include <unistd.h> 

#include "rclcpp/rclcpp.hpp"
#include <rcl/types.h>
#include <sys/stat.h>
#include <fcntl.h>          
#include <stdbool.h>
      
#include <sys/types.h>

#include <serial/serial.h>


#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "lucsys_robot_msg/msg/data.hpp" 
#include "lucsys_robot_msg/msg/supersonic.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "lucsys_chassis_driver/Quaternion_Solution.h"
#include "lucsys_robot_msg/msg/data.hpp" 
//New additions related to recharge
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <turtlesim/srv/spawn.hpp>

//Ultrasound data
#include <sensor_msgs/msg/range.hpp>

using namespace std;

#define RESET   string("\033[0m")
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"

//Macro definition
//Macro definition
#define SEND_DATA_CHECK   1          //Send data check flag bits //Send data check flag bits
#define READ_DATA_CHECK   0          //Receive data to check flag bits // Receive data to check flag bits
#define FRAME_HEADER      0X7B       //Frame head // Frame head
#define FRAME_TAIL        0X7D       //Frame tail // Frame tail
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer // Length of data sent by the lower computer
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine // Length of data sent from ROS to the lower computer
#define PI 				  3.1415926f //PI // Pi
// Ultrasonic measurement of distance-related variables
#define Distance_DATA_size 19
#define Distance_HEADER    0XFA //Frame_header // Frame header
#define Distance_TAIL      0XFC //Frame_tail // End of frame

//Automatic recharge related to recharge
#define AutoCharge_HEADER      0X7C //Frame_header // Automatically recharge the data frame head
#define AutoCharge_TAIL        0X7F //Frame_tail // Automatically recharge the end of the data frame
#define AutoCharge_DATA_SIZE    8   //The length of the automatic recharge data sent from the lower computer

//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//Related to the range set by the IMU gyroscope, range ± 500°, corresponding data range ± 32768
//Convert raw gyroscope data to radians (rad): 1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//Related to the range set by the IMU accelerometer, which ± 2g, corresponding to a data range ±32768
//The original accelerometer data conversion bit m/s^2 unit, 32768/2g = 32768/19.6=1671.84
#define ACCEl_RATIO 	  1671.84f  	

//extern sensor_msgs::msg::Imu Mpu6050; // External variables, IMU topic data

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//Covariance matrix, used for odometer topic data, used for robt_pose_ekf function packages
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

//Data structure for speed and position
//Velocity/position data structure
typedef struct 
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

//IMU data structure
//IMU data structure
typedef struct 
{
	short accele_x_data; 
	short accele_y_data; 	
	short accele_z_data; 
    short gyros_x_data; 
	short gyros_y_data; 	
	short gyros_z_data; 

}IMU_DATA;

//The structure of the ROS to send data to the down machine
//Structure for ROS to send data to the lower computer
typedef struct   
{
	    uint8_t tx[SEND_DATA_SIZE];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail; 
}SEND_DATA;
//Data structures related to automatic recharge sent by the subcomputer
typedef struct      
{
	    uint8_t rx[AutoCharge_DATA_SIZE];  //8 bytes
		unsigned char Frame_Header;     //Frame header
		unsigned char Frame_Tail;		//Frame end
}RECEIVE_AutoCharge_DATA;
//Ultrasound-related data structures sent by the subordinate computer
//Ultrasonic data structures sent by the subcomputer to ROS
typedef struct      
{
	    uint8_t rx[Distance_DATA_size];
		unsigned char Frame_Header;
		unsigned char Frame_Tail;
}DISTANCE_DATA;

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
}Supersonic_data;

//The structure in which the lower computer sends data to the ROS
//Structure that sends data from the subcomputer to ROS
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
}RECEIVE_DATA;

//The robot chassis class uses constructors to initialize data, publish topics, etc
//Robot chassis class, using constructors to initialize data and publish topics, etc
class turn_on_robot : public rclcpp::Node
{
	public:
		turn_on_robot();  //Constructor // Constructor
		~turn_on_robot(); //Destructor // Destructor
		void Control();   //Loop control code
		serial::Serial Stm32_Serial; //Declare a serial object // Declare a serial port object
	private:
		//ros::NodeHandle n; Create a ROS node handle // Create a ROS node handle
		rclcpp::Time _Now, _Last_Time;  //Time dependent, used for integration to find displacement (mileage) // Time-dependent, used for integration to find displacement (mileage)
		float Sampling_Time;         //Sampling time, used for integration to find displacement (mileage) // Sampling time, used for integration to find displacement (mileage)
		rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_cmd_Sub;

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;//Initialize the topic subscriber
   
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr AvoidFlag_Sub;

        //Initialize the topic publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;       
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;        
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
        rclcpp::Publisher<lucsys_robot_msg::msg::Supersonic>::SharedPtr distance_publisher;         

		//Ultrasound publisher
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_a_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_b_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_c_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_d_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_e_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_f_publisher_;

		//Recharge relevant publishers
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Charging_publisher;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr Charging_current_publisher;
		rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr RED_publisher;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ChargingMode_publisher;
		
		//Recharge relevant subscribers
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Red_Vel_Sub;
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr Recharge_Flag_Sub;
		//Recharge-related services
		rclcpp::Service<turtlesim::srv::Spawn>::SharedPtr SetCharge_Service;
		//Chassis safety level settings
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr Security_Sub;
		//Light strip settings
		rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr SetRgb_Sub;
		//The speed topic subscribes to the callback function
		//Velocity-topic subscription callback
        void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
        void arm_cmd_Callback(const std_msgs::msg::Float32MultiArray arm_joint);
		//Recharge-related callback functions
		void Recharge_Flag_Callback(const std_msgs::msg::Int8::SharedPtr Recharge_Flag); 
		//Chassis safety level callback function
		void Security_Callback(const std_msgs::msg::Int8 &Security_Flag);
    	void Set_LightRgb_Callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
		//Recharge related publishing functions
		void Publish_Charging();       
		void Publish_ChargingCurrent();
		void Publish_RED();

		void Publish_Odom();      //Pub the speedometer topic //Publish speedometer topic
		void Publish_ImuSensor(); //Pub the IMU sensor topic //Publish the IMU sensor topic
		void Publish_Voltage();   //Pub the power supply voltage topic //Publish power supply voltage topic
		void Publish_distance();//Publish ultrasonic distance
        //Read the sports chassis speed, IMU, and power supply voltage data from the serial port (ttyUSB).
        //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
        bool  Get_Sensor_Data();   
        short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);  //IMU data conversion read // IMU data converted read
		float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //Odometer data converted for reading

		string car_mode;
        string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id; //Define the related variables // Define the related variables
		string cmd_vel;
		bool ranger_avoid_flag;
        int serial_baud_rate;      //Serial communication baud rate //Serial communication baud rate
        RECEIVE_DATA Receive_Data; //The data structure received by the serial port // Data structure received by the serial port
        SEND_DATA Send_Data;       //The data structure sent by the serial port // Data structure sent by the serial port
        DISTANCE_DATA Distance_Data; //Ultrasound data
        RECEIVE_AutoCharge_DATA Receive_AutoCharge_Data;  //Serial port receives automatic recharge data structures
        Supersonic_data distance;  //Ultrasonic distance objects
        Vel_Pos_Data Robot_Pos;    //The position of the robot // Robot's position
        Vel_Pos_Data Robot_Vel;    //The robot's speed // Robot's speed
        IMU_DATA  onboard_imu_raw_data; //Onboard IMU raw data
		
		int8_t AutoRecharge=0;
        float Power_voltage;       //Power supply voltage // Power supply voltage
        bool Charging=0;           //Whether the robot is charging the flag bit // Is the robot charging the flag bit?
        float Charging_Current=0;  //Charging_Current // Charging current
        uint8_t Red=0;                //Whether the robot finds the marker bit of infrared signal (charging pile) // Whether the robot detects the marker bit of the infrared signal (charging pile)
        float odom_x_scale,odom_y_scale,odom_z_scale_positive,odom_z_scale_negative; //Odometer correction parameters
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
#endif
