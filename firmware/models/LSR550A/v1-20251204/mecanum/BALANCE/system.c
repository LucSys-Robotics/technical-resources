/* LucSys Robotics prepares and maintains this distribution. */

#include "system.h"

//Robot software fails to flag bits
//Robot software-disable flag
u8 Flag_Stop=0, Last_Flag_Stop=1;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADC value segmented variable, depending on the number of car models; currently there are 6 car models
int Divisor_Mode;

// Robot type variable
//Robot model variable
//0=Mec_Car£¬1=Omni_Car£¬2=Akm_Car£¬3=Diff_Car£¬4=FourWheel_Car£¬5=Tank_Car
u8 Car_Mode=0; 

//Servo control PWM value, Ackerman car special
//Servo control PWM value, exclusively for Ackermann trolleys
int Servo;  

//Default speed of remote control car, unit: mm/s
//Default remote-control chassis speed, unit: mm/s
float RC_Velocity=500; 

//Vehicle three-axis target moving speed, unit: m/s
//Three-axis target motion speed of the cart, unit: m/s
float Move_X, Move_Y, Move_Z;   

//PID parameters of Speed control
//Speed-control PID parameters
float Velocity_KP=300,Velocity_KI=300;  

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//Smooth control of intermediate variables, dedicated to omnidirectional moving carts
Smooth_Control smooth_control;  

//The parameter structure of the motor
//Parameter structure of the motor
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

/************ Car Model Related Variables **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//Encoder accuracy
float Encoder_precision; 
//Wheel circumference, unit: m
//Wheel circumference, unit: m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//Active wheel track width, unit: m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//Wheelbase of the front and rear axles of the chassis, unit: m
float Wheel_axlespacing; 
//All-directional wheel turning radius, unit: m
//Turning radius of the omnidirectional wheel, unit: m
float Omni_turn_radiaus; 
/************ Car Model Related Variables **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1 communication control flag bit.
//These 5 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2 controller, Bluetooth app, RC transmitter, CAN communication, serial port 1 communication control flags. These five flag bits default to 0, representing serial port 3 control mode
u8 PS2_ON_Flag=0, APP_ON_Flag=0, Remote_ON_Flag=0, CAN_ON_Flag=0, Usart_ON_Flag=0; 

//Bluetooth remote control associated flag bits
//Bluetooth remote control-related signs
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//Flags for sending parameters to the Bluetooth app
u8 PID_Send; 
//Robotic arm parameters
float Moveit_Angle1=0,Moveit_Angle2=0,Moveit_Angle3=0,Moveit_Angle4=0,Moveit_Angle5=0,Moveit_Angle6=0; //Degrees of freedom The target angle of the robotic arm
u8 motor1_flag,motor2_flag,motor3_flag,return_zero_flag=0;
float Angle_poor[3] = {0,0,0};
float Angle1_poor,Angle2_poor,Angle3_poor,Last_angleA=0,Last_angleB=0,Last_angleC=0,Speed1,Speed2,Speed3;
int Servo1=0;
short Moveit_Angle1_init=0;
float Position1=1500,Velocity1=0;
//The PS2 gamepad controls related variables
//PS2 controller controls related variables
float PS2_LX,PS2_LY,PS2_RX,PS2_RY; 
u8 PS2_KEY;
u8 Get_Charging_HardWare=0;//Whether there is an auto-recharge equipment
u8 charger_check=0;//Real-time monitoring of the online status of charging equipment

void systemInit(void)
{       
	//Interrupt priority group setting
	//Interrupt priority grouping settings
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//Delay function initialization
	//Delay function initialization
	delay_init(168);			                                            
	
	//Initialize the hardware interface connected to the LED lamp
	//Initialize the hardware interface connected to the LED lights
	LED_Init();                     
	    
  //Initialize the hardware interface connected to the buzzer	
  //Initialize the hardware interface connected to the buzzer
	Buzzer_Init();  
	
	//Initialize the hardware interface connected to the enable switch
	//Initialize the hardware interface connected to the enable switch
	Enable_Pin();

  //Initialize the hardware interface connected to the OLED display
  //Initialize the hardware interface connected to the OLED display
	OLED_Init();     
	
	//Initialize the hardware interface connected to the user's key
	//Initialize the hardware interface connected to the user key
	KEY_Init();	
	
	//Serial port 1 initialization, communication baud rate 115200, 
	//can be used to communicate with ROS terminal
	//Serial port 1 initialized, communication baud rate 115200, usable for communication with ROS terminals
	uart1_init(115200);	  
	
	//Serial port 4 initialization, communication baud rate 9600, 
	//used to communicate with Bluetooth APP terminal
	//Serial port 4 initialized, communication baud rate 9600, used for communication with Bluetooth app terminals
	uart4_init(9600);  
	
	//Serial port 3 is initialized and the baud rate is 115200. 
	//Serial port 3 is the default port used to communicate with ROS terminal
	//Serial port 3 initialized, communication baud rate 115200, serial port 3 is the default serial port used for communication with the ROS side
	uart3_init(115200);

  //Initialize the CAN communication interface
  //Initialize the CAN communication interface
	CAN1_Mode_Init(1,3,3,6,0);      
	
	//ADC pin initialization, used to read the battery voltage and potentiometer gear, 
	//potentiometer gear determines the car after the boot of the car model
	//ADC pin initialization is used to read battery voltage and potentiometer gear. The potentiometer gear determines the model of the car after startup
 	Adc_Init();  
  //According to the tap position of the potentiometer, determine which type of car needs to be matched, 
  //and then initialize the corresponding parameters	
  //Based on the potentiometer gear, determine which model of car needs to be adapted, then initialize the corresponding parameters
	Robot_Select();                 
	             
	
	//Encoder A is initialized to read the real time speed of motor A
	//Encoder A initializes to read the real-time speed of motor A
	Encoder_Init_TIM2(); 
	//Encoder B is initialized to read the real time speed of motor B
  //Encoder B initializes to read the real-time speed of motor B
	Encoder_Init_TIM3(); 
  //Encoder C is initialized to read the real time speed of motor C  
  //Encoder C initialization, used to read the real-time speed of motor C
	Encoder_Init_TIM4();
	//Encoder D is initialized to read the real time speed of motor D
  //Encoder D initialization, used to read the real-time speed of motor D
	Encoder_Init_TIM5();  
	
	//Initialize the motor direction control pin, 
	//which is used to control the motor forward and reverse rotation
	//Initialize motor direction control pins to control forward and reverse motor rotation
	LucSys_Motor_Init();  

  //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
  //Initialize motor speed control and control motor speed, PWM frequency 10KHZ
	LucSys_PWM_Init(16799,0);  //Advanced timer TIM8 frequency is 168M; full PWM is 16799; frequency=168M/((16799+1)*(0+1))=10k

    //Initialize the robotic arm gripper
	TIM9_Init(9999,168-1);
  //IIC initialization for MPU6050
  //IIC Initialization, used for MPU6050
	I2C_GPIOInit();

  //MPU6050 is initialized to read the vehicle's three-axis attitude, 
	//three-axis angular velocity and three-axis acceleration information
  //MPU6050 initialization, used to read the cart's three-axis angular velocity and three-axis acceleration information
	MPU6050_initialize();
	
	//Initialize the model remote control interface		
	//Initialize the RC transmitter interface
	TIM1_Cap_Init(9999,168-1);  //Advanced timer TIM1 frequency is 168M
	//Initialize the hardware interface to the PS2 controller
	//Initialize the hardware interface connected to the PS2 controller
	PS2_Init();
	//PS2 gamepad configuration is initialized and configured in analog mode
  //PS2 controller configuration initialized to analog mode
	PS2_SetInit();		 

	//Sample battery voltage once as the initial reference for voltage smoothing
	base_vol = Get_battery_volt();
	
	//Check if auto-recharge equipment exists
	delay_ms(100);
	Find_Charging_HardWare();
	
}

void Find_Charging_HardWare(void)
{
	u8 tmpdata[8]={1,1,1,1,1,1,1,1};
	CAN1_Send_EXTid_Num(0x12345678,tmpdata);
}
