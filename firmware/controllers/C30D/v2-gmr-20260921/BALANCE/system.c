/* LucSys Robotics distribution maintenance.
 * Third-party license terms remain applicable.
 * Version: V5.0
 * Modification date: 2022-05-05
 * Version: V5.0
 * Update£º2022-05-05
 */

#include "system.h"

//Robot software fails to flag bits
//Robot software-disable flag
u8 Flag_Stop=0;

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
float Axle_spacing;
//All-directional wheel turning radius, unit: m
//Turning radius of the omnidirectional wheel, unit: m
float Omni_turn_radiaus;
/************ Car Model Related Variables **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1, serial port 5 communication control flag bit.
//These 6 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2 controller, Bluetooth app, RC transmitter, CAN communication, serial port 1, serial port 5 communication control flags. These six flag bits default to 0, representing serial port 3 control mode
u8 PS2_ON_Flag=0, APP_ON_Flag=0, Remote_ON_Flag=0, CAN_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag;

//Bluetooth remote control associated flag bits
//Bluetooth remote control-related signs
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag;

//Sends the parameter's flag bit to the Bluetooth APP
//Flags for sending parameters to the Bluetooth app
u8 PID_Send;

//The PS2 gamepad controls related variables
//PS2 controller controls related variables
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;

//Self-check the relevant flag variables
//Self-check related flag variables
int Check=0;

//Check the result code
//Self-test result code
long int ErrorCode=0;

u8 Get_Charging_HardWare=0;//Whether there is an auto-recharge equipment
u8 charger_check=0;//Real-time monitoring of the online status of charging equipment

//System-related variables
SYS_VAL_t SysVal;

//Potentiometer records values
int POT_val;

//Motor full rotation value
int Full_rotation = 16799;

void systemInit(void)
{
	//Interrupt priority group setti  ng
	//Interrupt priority grouping settings
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	//Delay function initialization
	//Delay function initialization
    delay_init(168);
	//IIC initialization for IMU
    //IIC Initialization, used for IMU
    I2C_GPIOInit();
	//Initialize system-related software parameters
	SYS_VAL_t_Init(&SysVal);
	
    //Serial port 1 initialization, communication baud rate 115200,
    //can be used to communicate with ROS terminal
    //Serial port 1 initialized, communication baud rate 115200, usable for communication with ROS terminals
    uart1_init(115200);

    //Serial port 2 initialization, communication baud rate 9600,
    //used to communicate with Bluetooth APP terminal
    //Serial port 2 initialized, communication baud rate 9600, used for communication with Bluetooth app terminals
    uart2_init(9600);

    //Serial port 3 is initialized and the baud rate is 115200.
    //Serial port 3 is the default port used to communicate with ROS terminal
    //Serial port 3 initialized, communication baud rate 115200, serial port 3 is the default serial port used for communication with the ROS side
    uart3_init(115200);
//	uart3_init(230400);
	
    //Serial port 5 initialization, communication baud rate 115200,
    //can be used to communicate with ROS terminal
    //Serial port 5 initialized, communication baud rate 115200, usable for communication with ROS terminal
    uart5_init(115200);
	
	//If the IMU is MPU6050, it is an older version of the C30D
	if( MPU6050_DEFAULT_ADDRESS == MPU6050_getDeviceID() )
	{
		SysVal.HardWare_Ver = V1_0;
		//Initialize the hardware interface to the PS2 controller
		//Initialize the hardware interface connected to the PS2 controller
		PS2_Init();
		//PS2 gamepad configuration is initialized and configured in analog mode
		//PS2 controller configuration initialized to analog mode
		PS2_SetInit();		 
		//Initialize the hardware interface connected to the LED lamp
		//Initialize the hardware interface connected to the LED lights
		V1_0_LED_Init(); 
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//MPU6050 initialization, used to read the cart's three-axis angular velocity and three-axis acceleration information
		MPU6050_initialize();
	}
	//If the IMU model is ICM20948, it is the new C30D
	else if( REG_VAL_WIA == ICM20948_getDeviceID() )//Read ICM20948 ID
	{
		SysVal.HardWare_Ver = V1_1;
		
		//USBHOS initialization
		MX_USB_HOST_Init();
		
		//Initialize the hardware interface connected to the LED lamp
		//Initialize the hardware interface connected to the LED lights
		V1_1_LED_Init();
		//MPU6050 is initialized to read the vehicle's three-axis attitude,
		//three-axis angular velocity and three-axis acceleration information
		//ICM20948 Initialization, used to read the car's three-axis angular velocity and three-axis acceleration information
		invMSInit();
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
    Enable_Pin();

    //Initialize the hardware interface connected to the OLED display
    //Initialize the hardware interface connected to the OLED display
    OLED_Init();

    //Initialize the hardware interface connected to the user's key
    //Initialize the hardware interface connected to the user key
    KEY_Init();
		if(KEY==0)			Check=1;
		else if(KEY==1)	Check=0;

    //ADC pin initialization, used to read the battery voltage and potentiometer gear,
    //potentiometer gear determines the car after the boot of the car model
    //ADC pin initialization is used to read battery voltage and potentiometer gear. The potentiometer gear determines the model of the car after startup
    Adc_Init();
    Adc_POWER_Init();


	//Initialize the CAN communication interface
	//Initialize the CAN communication interface
	//Baud rate calculation method: 42M/[(parameter 1 + parameter 2 + parameter 3) * parameter 4]
	//Example: 42M / [(1+7+6)*3] = 1M (only related to the first 4 parameters)
    CAN1_Mode_Init(1,7,6,3,0);

    //According to the tap position of the potentiometer, determine which type of car needs to be matched,
    //and then initialize the corresponding parameters
    //Based on the potentiometer gear, determine which model of car needs to be adapted, then initialize the corresponding parameters
    Robot_Select();

    //Encoder A is initialized to read the real time speed of motor C
    //Encoder A initializes to read the real-time speed of motor C
    Encoder_Init_TIM2();
    //Encoder B is initialized to read the real time speed of motor D
    //Encoder B initializes to read the real-time speed of motor D
    Encoder_Init_TIM3();
    //Encoder C is initialized to read the real time speed of motor B
    //Encoder C initializes to read the real-time speed of motor B
    Encoder_Init_TIM4();
    //Encoder D is initialized to read the real time speed of motor A
    //Encoder D initializes to read the real-time speed of motor A
    Encoder_Init_TIM5();

    //Timer 12 serves as the PWM interface for the servo
    TIM12_SERVO_Init(9999,84-1);  //APB1 clock frequency is 84M, frequency = 84M / ((9999+1)*(83+1))) = 100Hz

    //Standard small cars default timer 8 as the model aircraft interface
    // TIM8_SERVO_Init(9999,168-1); The APB2 clock frequency is 168M, frequency = 168M / ((9999+1)*(167+1))) = 100Hz
    //Initialize the model remote control interface
    //Initialize the RC transmitter interface
    TIM8_Cap_Init(9999,168-1);  //Advanced timer TIM8 clock frequency is 168M

    //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
    //Initialize motor speed control and control motor speed, PWM frequency 10KHZ
    //APB2 clock frequency is 168M, full PWM is 16799, frequency = 168M / ((16799+1)*(0+1)) = 10k
    TIM1_PWM_Init(16799,0);
    TIM9_PWM_Init(16799,0);
    TIM10_PWM_Init(16799,0);
    TIM11_PWM_Init(16799,0);

    //Sample battery voltage once as the initial reference for voltage smoothing
    base_vol = Get_battery_volt();

    //Check if auto-recharge equipment exists
    delay_ms(100);
    Find_Charging_HardWare();
	
	//Read Flash data
	FlashParam_Read();
}

void Find_Charging_HardWare(void)
{
    u8 tmpdata[8]= {1,1,1,1,1,1,1,1};
    CAN1_Send_EXTid_Num(0x12345678,tmpdata);
}

