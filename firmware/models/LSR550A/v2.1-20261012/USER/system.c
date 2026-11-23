/* LucSys Robotics prepares and maintains this distribution. */

#include "system.h"

//System-related variables
SYS_VAL_t SysVal;

void systemInit(void)
{
	//================= General Hardware Initialization Section =================//
	//================= General Hardware Initialization Section =================//
    //Interrupt priority group setting
    //Interrupt priority grouping settings
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //Delay function initialization
    //Delay function initialization
    delay_init(168);
	
	//Prioritize initializing IIC and IMU C50C board distinguishes between new and old versions based on IMU model
	//Prioritize initialization of IIC and IMU. The C50C board distinguishes new and old versions by IMU model
    //IIC initialization
    //IIC Initialization
    I2C_GPIOInit();
	
	//Initialize system-related software parameters
	SYS_VAL_t_Init(&SysVal);
	
    //Serial port 1 initialization, communication baud rate 115200,
    //can be used to communicate with ROS terminal
    //Serial port 1 initialized, communication baud rate 115200, usable for communication with ROS terminals
    UART1_Init(115200);
	
	//If the IMU is MPU6050, it is the older version of the C50C
	if( MPU6050_DEFAULT_ADDRESS == MPU6050_getDeviceID() )
	{
		SysVal.HardWare_Ver = V1_0;
		
		//Initialize the hardware of the older version of the C50C
		V1_0_LED_Init();
		V1_0_CAN1_Mode_Init(1,3,3,6,0);
		V1_0_LucSys_PWM_Init(16799,0);
		
		//Initialize the hardware interface to the PS2 controller
		//Initialize the hardware interface connected to the PS2 controller
		PS2_Init();
		
		//Initialize PS2 software parameters
		PS2_Key_Param_Init();
		
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//MPU6050 initialization, used to read the cart's three-axis angular velocity and three-axis acceleration information
		MPU6050_initialize();
	}
	
	//If the IMU model is ICM20948, it is the new C50C
	else if( REG_VAL_WIA == ICM20948_getDeviceID() )//Read ICM20948 ID
	{
		//Initialize version interface
		Version_GPIO_Init();
		
		uint8_t ver=0;
		ver |= (VersionBit2)<<2;
		ver |= (VersionBit1)<<1;
		ver |= (VersionBit0)<<0;
		
		switch( ver )
		{
			case 0:
				SysVal.HardWare_Ver = V1_1;
				break;
			case 1:
				SysVal.HardWare_Ver = V1_2;
				UART2_Init(115200); //Supports 485 interface after version 1.2
				D50A_Init();
				break;
		}

	
		//Initialize the hardware interface connected to the LED lamp
		//Initialize the hardware interface connected to the LED lights
		RGB_LightStrip_Init();
		
		//Initialize the CAN communication interface
		//Initialize the CAN communication interface
		CAN1_Mode_Init(1,3,3,6,0);
		
		//Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
		//Initialize motor speed control and control motor speed, PWM frequency 10KHZ
		LucSys_PWM_Init(16799,0);  //Advanced timer TIM8 frequency is 168M; full PWM is 16799; frequency=168M/((16799+1)*(0+1))=10k
	
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//ICM20948 Initialization, used to read the car's three-axis angular velocity and three-axis acceleration information
		invMSInit();
		
		//USB PS2 initialization
		MX_USB_HOST_Init();//Create USB controller task
	}
	else //Unrecognized gyroscope, reset system
	{
		NVIC_SystemReset();
	}
	
    //Initialize the hardware interface connected to the buzzer
    //Initialize the hardware interface connected to the buzzer
    Buzzer_Init();
    
    //Initialize the hardware interface connected to the enable switch
    //Initialize the hardware interface connected to the enable switch
    EnableKey_Init();

    //Initialize the hardware interface connected to the user's key
    //Initialize the hardware interface connected to the user key
    KEY_Init();
	
    //Initialize the hardware interface connected to the OLED display
    //Initialize the hardware interface connected to the OLED display
    OLED_Init();

    //Serial port 4 initialization, communication baud rate 9600,
    //used to communicate with Bluetooth APP terminal
    //Serial port 4 initialized, communication baud rate 9600, used for communication with Bluetooth app terminals
	#if VECT_TAB_OFFSET == 0x10000 //Choose the Bluetooth baud rate based on whether wireless burning is enabled
		UART4_Init(230400);
	#elif VECT_TAB_OFFSET == 0
		UART4_Init(9600);
	#endif
	
    //Serial port 3 is initialized and the baud rate is 115200.
    //Serial port 3 is the default port used to communicate with ROS terminal
    //Serial port 3 initialized, communication baud rate 115200, serial port 3 is the default serial port used for communication with the ROS side
    UART3_Init(115200);

    //Initialize the model remote control interface
    //Initialize the RC transmitter interface
    Remoter_Init();
	
    //Encoder A is initialized to read the real time speed of motor A
    //Encoder A initializes to read the real-time speed of motor A
    EncoderA_Init();
    //Encoder B is initialized to read the real time speed of motor B
    //Encoder B initializes to read the real-time speed of motor B
    EncoderB_Init();
	
    //ADC pin initialization, used to read the battery voltage and potentiometer gear,
    //potentiometer gear determines the car after the boot of the car model
    //ADC pin initialization is used to read battery voltage and potentiometer gear. The potentiometer gear determines the model of the car after startup
    ADC1_Init();
	
	//Ackermann chassis uses ADC2, not encoders C/D; other chassis types do the opposite
	#if defined AKM_CAR
		ADC2_Init();
	#else
		//Encoder C is initialized to read the real time speed of motor C  
		//Encoder C initialization, used to read the real-time speed of motor C
		EncoderC_Init();
		//Encoder D is initialized to read the real time speed of motor D
		//Encoder D initialization, used to read the real-time speed of motor D
		EncoderD_Init();  
	#endif
	
	//================= Software Parameter Initialization Section =================//
	
	//Determine the robot model and initialize the robot's mechanical parameters and PID parameters.
	Robot_Select(); 
	
	//Initialization of robot control-related variables, including remote control speed references, maximum speed limits, speed smoothing coefficients, and other content.
	ROBOT_CONTROL_t_Init(&robot_control); 
	
	//Initialize 4 PI controllers
	PI_Controller_Init(&PI_MotorA,robot.V_KP,robot.V_KI);
	PI_Controller_Init(&PI_MotorB,robot.V_KP,robot.V_KI);
	PI_Controller_Init(&PI_MotorC,robot.V_KP,robot.V_KI);
	PI_Controller_Init(&PI_MotorD,robot.V_KP,robot.V_KI);
	
	//Automatic recharge equipment software parameter initialization
	auto_recharge_reset();
	
	//Gripper PWM
	TIM9_Init(9999,168-1);
	Arm2DOF_Init(&A280);

	//OLED software parameter initialization
	OLED_Param_Init(&oled);
	
	//Initialize the app software parameters
	APPKey_Param_Init(&appkey);
	
	//Initialize parameters of the RC transmitter software
	Remoter_Param_Init(&remoter);
	
	//Servo parameter initialization
	Akm_ServoParam_Init(&Akm_Servo);
	
	//Read servo data from Flash; if no data exists, default initialization values are used
	FlashParam_Read();    
	
	//Initialize servo for Ackermann chassis
	#if defined AKM_CAR
	
	//Initialize high-spec Ackermann servo
	Servo_Senior_Init(10000-1,168-1,Akm_Servo.Mid);
	robot.SERVO.Output = Akm_Servo.Mid;
	
	//Initialize top-spec Ackermann servo
	if( robot.type >= 2 && robot.type!= 9 )
	{
		//Wait for DMA to collect data
		delay_ms(200);
		
		//Read a set of rail data to estimate the servo's position. Then initialize the estimated position as the PWM value to avoid sudden and rapid servo repositioning.
		short TmpPWM = get_ServoPWM( get_DMA_SlideRes() );
		
		//Initialize top-spec Ackermann servo with an offset to avoid rapid reset
		Servo_Top_Init(10000-1,84-1, TmpPWM );
		
		//Servo PI controller initialization. Note: The servo PID parameters cannot be modified.
		PI_Controller_Init(&PI_Servo,0,0);
		
		//Set the servo PI control reference value and add deviation values to prevent servo jittering when PI control is first entered
		PI_Servo.Output =  TmpPWM;
		
		//Servo speed smoothing value
		robot_control.smooth_Servo = TmpPWM;
		
		//Set the low-speed servo mode to allow the servo to return to position slowly
		robot_control.ServoLow_flag = 1;
	}
	

	#endif
	
	//All hardware and software devices are initialized, and the buzzer prompts you to enter RTOS
	Buzzer_AddTask(1,100);//One beep for 1000ms
}


