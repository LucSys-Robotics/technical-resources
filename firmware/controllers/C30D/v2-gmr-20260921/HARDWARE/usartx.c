#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
SEND_AutoCharge_DATA Send_AutoCharge_Data;

// External IP address store
u8 Received_IP[4] = {0, 0, 0, 0};          // Store the final confirmed IP address
// IP retrieves function-related variables
u8 Last_IP[4] = {0, 0, 0, 0};              // Stores the last received IP (for continuity verification)
u8 IP_Frame_Valid = 0;                     // IP frame valid flag
u16 IP_Valid_Count = 0;                    // Consecutive identical-IP counter
u8 Debug_Flag=0;
/**************************************************************************
Function: Usartx3, Usartx1,Usartx5 and CAN send data task 
Input   : none
Output  : none
Function: Serial Ports 3, 1, 5, and CAN send data task
Entry parameters: None
Return value: none
**************************************************************************/
void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 20hz
			//This task runs at a frequency of 20Hz
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));
			if(Check==0)
			{
				//Assign the data to be sent
				//Assign values to the data to be sent
				data_transition(); 
				USART1_SEND();     //Serial port 1 sends data //Serial port 1 sends data
				USART3_SEND();     //Serial port 3 (ROS) sends data //Serial port 3 (ROS) sends data
				USART5_SEND();		 //Serial port 5 sends data //Serial port 5 sends data
				CAN_SEND();        //CAN send data //CAN Send data
			}
		}
}
/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
Function: Assign values to the data sent by the serial port
Entry parameters: None
Return value: none
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header // Frame header
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail // End of frame
	
	//According to different vehicle types, different kinematics algorithms were selected to carry out the forward kinematics solution, 
	//and the three-axis velocity was obtained from each wheel velocity
	//Select different kinematic algorithms for different vehicle models to perform kinematic correct solutions, calculating the three-axis speeds from the speeds of each wheel
	switch(Car_Mode)
	{	
		case Mec_Car:case Mec_Car_V550:
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000;
	    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder-MOTOR_D.Encoder)/4)*1000; 
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4/(Axle_spacing+Wheel_spacing))*1000;         
		  break; 
		
    case Omni_Car:      
			Send_Data.Sensor_Str.X_speed = ((MOTOR_C.Encoder-MOTOR_B.Encoder)/2/X_PARAMETER)*1000; 
	    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder*2-MOTOR_B.Encoder-MOTOR_C.Encoder)/3)*1000; 
	    Send_Data.Sensor_Str.Z_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder)/3/Omni_turn_radiaus)*1000;      
		  break; 
    
		case Akm_Car:  
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/Wheel_spacing)*1000;
		  break; 
		
		case Diff_Car: 
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/Wheel_spacing)*1000;
			break; 
		
		case FourWheel_Car:case FourWheel_Car_V550:
      Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000; 
	    Send_Data.Sensor_Str.Y_speed = 0;
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_B.Encoder-MOTOR_A.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/2/(Axle_spacing+Wheel_spacing))*1000;
		 break; 
		
		case Tank_Car:   
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/(Wheel_spacing)*1000);
			break; 
	}
	
	//The acceleration of the triaxial acceleration // Triaxial accelerometer acceleration
	Send_Data.Sensor_Str.Accelerometer.X_data= imu.accel.y; //The accelerometer Y-axis is converted to the ROS coordinate X-axis // Accelerometer Y-axis converted to ROS coordinate X-axis
	Send_Data.Sensor_Str.Accelerometer.Y_data=-imu.accel.x; //The accelerometer X-axis is converted to the ROS coordinate Y-axis // Accelerometer X-axis converted to ROS coordinate Y-axis
	Send_Data.Sensor_Str.Accelerometer.Z_data= imu.accel.z; //The accelerometer Z-axis is converted to the ROS coordinate Z-axis // Accelerometer Z-axis converted to ROS coordinate Z-axis
	
	//The angular velocity of the triaxial gyro //Triaxial gyro angular velocity
	Send_Data.Sensor_Str.Gyroscope.X_data= imu.gyro.y; //The Y-axis is converted to the ROS coordinate X-axis // Gyroscope Y-axis converted to ROS coordinate X-axis
	Send_Data.Sensor_Str.Gyroscope.Y_data=-imu.gyro.x; //The X-axis is converted to the ROS coordinate Y-axis // Gyroscope X-axis converted to ROS coordinate Y-axis
	if(Flag_Stop==0) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //If motor control is enabled, normally send Z-axis angular velocity
		Send_Data.Sensor_Str.Gyroscope.Z_data=imu.gyro.z;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //If the robot is stationary (motor control disabled), then the sent Z-axis angular velocity is 0
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;        
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//Battery voltage (here, the floating-point number is amplified by 1,000 times for transmission, and correspondingly, the receiver will reduce the data by 1,000 times after receiving it)
	Send_Data.Sensor_Str.Power_Voltage = Voltage*1000; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade // Frame header
  Send_Data.buffer[1]=Flag_Stop; //Chassis software-disable flag
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//The cart's three-axis speed is split into two 8-bit data points for each axle and then sent
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU accelerometer three-axis acceleration, each axis is split into two 8-bit data before sending
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU angular velocity meter for three axes, with each axis split into two 8-bit data points for transmission
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//Battery voltage, split into two 8-bit data transmissions
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //Data parity calculations, Mode 1 is data parity
	Send_Data.buffer[22]=Check_Sum(22,1); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail // End of frame

	///////////////////////Automatic recharge of related variable assignment/////////////////////
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Header = AutoCharge_HEADER;   //Frame header assignment 0x7C
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail = AutoCharge_TAIL;		//Frame tail assignment 0x7F
	Send_AutoCharge_Data.AutoCharge_Str.Charging_Current = (short)Charging_Current;//Charging current assignment
	
	Send_AutoCharge_Data.AutoCharge_Str.RED = RED_STATE; //Assign values to infrared marker bits
	Send_AutoCharge_Data.AutoCharge_Str.Charging = Charging; //Whether a value is assigned at the charging marker

	Send_AutoCharge_Data.buffer[0] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Header;		//Frame header 0x7C
	Send_AutoCharge_Data.buffer[1] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current>>8;//High 8 bits of charging current
	Send_AutoCharge_Data.buffer[2] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current;	//Low 8 bits of charging current
	Send_AutoCharge_Data.buffer[3] = Send_AutoCharge_Data.AutoCharge_Str.RED;				//Whether infrared marker bits have been received
	Send_AutoCharge_Data.buffer[4] = Send_AutoCharge_Data.AutoCharge_Str.Charging;			//Whether it is at the charging marker
	Send_AutoCharge_Data.buffer[5] = Allow_Recharge;										//Whether automatic recharge is enabled
	Send_AutoCharge_Data.buffer[6] = Check_Sum_AutoCharge(6,1);								//Check bit
	Send_AutoCharge_Data.buffer[7] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail;		//Frame end 0x7F
	///////////////////////Automatic recharge of related variable assignment/////////////////////
	
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
Function: Send data via Serial Port 1
Entry parameters: None
Return value: none
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
	if(Get_Charging_HardWare==1)
	{
		//When recharge equipment exists, send auto-recharge variables to the upper layer
		for(i=0; i<8; i++)
		{
			usart1_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}	
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
Function: Send data via Serial Port 3
Entry parameters: None
Return value: none
**************************************************************************/
void USART3_SEND(void)
{
	unsigned char i = 0;
	// Send the original data
	for(i = 0; i < 24; i++)
	{
		usart3_send(Send_Data.buffer[i]);
	}	
	if(Get_Charging_HardWare == 1)
	{
		// When recharge equipment exists, send auto-recharge variables to the upper layer
		for(i = 0; i < 8; i++)
		{
			usart3_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}
}
void USART3_Return(void)
{
	for(int i=0; i<check_parameter.Usart3_rec_count; i++)
	{
		usart3_send(check_parameter.Usart3_rec_buffer[i]);
	}
	usart3_send('\r');
	usart3_send('\n');
}
/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
Function: Send data via Serial Port 5
Entry parameters: None
Return value: none
**************************************************************************/
void USART5_SEND(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		usart5_send(Send_Data.buffer[i]);
	}	 
	if(Get_Charging_HardWare==1)
	{
		//When recharge equipment exists, send auto-recharge variables to the upper layer
		for(i=0; i<8; i++)
		{
			usart5_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}
}
/**************************************************************************
Function: CAN sends data
Input   : none
Output  : none
Function: CAN to send data
Entry parameters: None
Return value: none
**************************************************************************/
void CAN_SEND(void) 
{
	u8 CAN_SENT[8],i;
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
	
	//////////////////Automatic recharge-related data transmission //////////////////
	if(Get_Charging_HardWare) CAN_Send_AutoRecharge();
}
/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
Function: Initialize Serial Port 1
Entry parameters: None
Return value: none
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Output mode
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //Push-pull output
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //High speed: 50MHz
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //Pull up
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //Initialize
	
  //UsartNVIC configuration // UsartNVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //Preempt priority
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority // Subpriority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//Initialize the VIC register according to the specified parameters
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings initialization settings
	USART_InitStructure.USART_BaudRate = bound; //Port rate // Serial port baud rate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Word length is 8-bit data format // Word length is 8-bit data format
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit // A stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //No parity bits
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control // No hardware data flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //Sending and receiving mode
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1
}
/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
Function: Initialize Serial Port 2
Entry parameters: None
Return value: none
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Output mode
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //Push-pull output
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //High speed: 50MHz
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //Pull up
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //Initialize
	
	//UsartNVIC configuration // UsartNVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //Preempt priority
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority // Subpriority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//Initialize the VIC register according to the specified parameters
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART Initialization Settings initialization settings
	USART_InitStructure.USART_BaudRate = bound; //Port rate // Serial port baud rate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Word length is 8-bit data format // Word length is 8-bit data format
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit // A stop
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //No parity bits
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control // No hardware data flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //Sending and receiving mode
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2
}
/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
Function: Initialize Serial Port 3
Entry parameters: None
Return value: none
**************************************************************************/
void uart3_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Output mode
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //Push-pull output
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //High speed: 50MHz
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //Pull up
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //Initialize
	
  //UsartNVIC configuration // UsartNVIC configuration
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //Preempt priority
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //Preempt priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//Initialize the VIC register according to the specified parameters
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings initialization settings
	USART_InitStructure.USART_BaudRate = bound; //Port rate // Serial port baud rate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Word length is 8-bit data format // Word length is 8-bit data format
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit // A stop
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //No parity bits
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control // No hardware data flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //Sending and receiving mode
  USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts
  USART_Cmd(USART3, ENABLE);                     //Enable serial port 3
}
/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
Function: Initialize Serial Port 5
Entry parameters: None
Return value: none
**************************************************************************/
void uart5_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//PC12 TX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock
		//PD2 RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Output mode
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //Push-pull output
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //High speed: 50MHz
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //Pull up
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //Initialize

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Output mode
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //Push-pull output
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //High speed: 50MHz
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //Pull up
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //Initialize
	
  //UsartNVIC configuration // UsartNVIC configuration
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //Preempt priority
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //Preempt priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//Initialize the VIC register according to the specified parameters
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings initialization settings
	USART_InitStructure.USART_BaudRate = bound; //Port rate // Serial port baud rate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Word length is 8-bit data format // Word length is 8-bit data format
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit // A stop
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //No parity bits
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control // No hardware data flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //Sending and receiving mode
  USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts
  USART_Cmd(UART5, ENABLE);                     //Enable serial port 5
}
/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
Function: Serial Port 1 receive interrupt
Entry parameters: None
Return value: none
**************************************************************************/
int USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received // Check if data has been received
	{
		u8 Usart_Receive;
		static u8 Count;
		static u8 rxbuf[11];
		int check=0,error=1,i;
		
		Usart_Receive = USART_ReceiveData(USART1); //Read the data //Read data
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //Do not process data 10 seconds before powering on
			return 0;	
			
		//Fill the array with serial data
		//Serial port data is populated into an array
    rxbuf[Count]=Usart_Receive;  
		
		//Ensure that the first data in the array is FRAME_HEADER
		//Ensure the first data in the array is FRAME_HEADER
    if(Usart_Receive == FRAME_HEADER||Count>0) 
			Count++; 
		else 
			Count=0;
		
		if (Count == 11) //Verify the length of the packet // Verify the length of the packet
		{   
				Count=0; //Prepare for the serial port data to be refill into the array //Get ready to refill data from the serial port into the array
				if(rxbuf[10] == FRAME_TAIL) //Verify the frame tail of the packet // Verify the frame tail of the packet
				{			
					for(i=0; i<9; i++)
					{
						//XOR bit check, used to detect data error
						//XOR bit check, used to detect data errors
						check=rxbuf[i]^check; 
					}
					if(check==rxbuf[9]) 
						//XOR bit check successful
					  //XOR bit verification succeeded
					  error=0; 
					
					if(error==0)	 
				  {		
						float Vz;
            if(Usart1_ON_Flag==0)
						{	
							//Serial port 1 controls flag position 1, other flag position 0
							//Set serial-port-1 control flag to 1 and all other flags to 0
							//Usart_ON_Flag=1;
							Usart1_ON_Flag=1;
							APP_ON_Flag=0;
							PS2_ON_Flag=0;
							Remote_ON_Flag=0;
							CAN_ON_Flag=0;
							
						}		
						command_lost_count=0;
						//Calculate the 3-axis target velocity from the serial data, which is divided into 8-bit high and 8-bit low units mm/s
						//Calculate the three-axis target velocity from serial port data, divided into 8 high and low 8 bits, with units in mm/s
						Move_X=XYZ_Target_Speed_transition(rxbuf[3],rxbuf[4]);
						Move_Y=XYZ_Target_Speed_transition(rxbuf[5],rxbuf[6]);
						Vz    =XYZ_Target_Speed_transition(rxbuf[7],rxbuf[8]);
						if(Car_Mode==Akm_Car)
						{
							Move_Z=Vz_to_Akm_Angle(Move_X, Vz);
						}
						else
						{
							Move_Z=XYZ_Target_Speed_transition(rxbuf[7],rxbuf[8]);
						}		
					}
			  }
		 }
	}
		return 0;	
}

/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
Function: Serial Port 2 receive interrupt
Entry parameters: None
Return value: none
**************************************************************************/
//BT04-A Bluetooth AT instruction feedback filtering function. Includes connection feedback and disconnect feedback
static uint8_t ATCommandFeedBack_BT04A(uint8_t recv)
{
	#define DEBUG_BT04ACommand 0
	
	uint8_t isFilter = 0;//Whether the character is allowed to pass, 1 filtered, 0 not processed
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;
	
	const char* BT04AConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\n+CONNECTED\r\n";
	const char* BT04ADisConnect = "+DISC:SUCCESS\r\n+READY\r\n+PAIRABLE\r\n";
	enum{
		BT04A_NORMAL=  0,
		BT04A_CONNECTSTART,
		BT04A_DISCONNECTSTART,
	};
	
	static uint8_t statemachine = BT04A_NORMAL;
	
	switch( statemachine )
	{
		case BT04A_NORMAL:
			if( recv=='C'&&lastrecv=='+' )
			{
				statemachine = BT04A_CONNECTSTART;//Receive feature values and start matching
				isFilter = 1;//Filter characters
				filterIndex = 2; //Indexing starts on the 2nd
			}
			else if( recv=='D'&&lastrecv=='+' )
			{
				statemachine = BT04A_DISCONNECTSTART;//Receive feature values and start matching
				isFilter = 1;//Filter characters
				filterIndex = 2; //Indexing starts on the 2nd
			}
			break;
		case BT04A_CONNECTSTART:
			if( BT04AConnect[filterIndex] == recv )  //Start filtering the connection characters and match them byte-byte-byte
			{
				isFilter = 1;//Match connection fields; filter if matched is complete
				
				#if 1== DEBUG_BT04ACommand 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else if( (filterIndex>=13&&filterIndex<=29) && \
       				  ((recv>='0'&&recv<='9')||(recv>='a'&&recv<='z')) )//Enter MAC-address matching stage; match fields 0~9 and a~z
			{
				isFilter = 1;//MAC address filtering
				#if 1== DEBUG_BT04ACommand 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				//Neither condition applies; allow the character through and exit filtering mode
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand 
				printf("No:get:%c,but:%c\r\n",recv,BT04AConnect[filterIndex]);
				#endif
			}
			
			//index until the filter list is complete
			filterIndex++;
			if( filterIndex == strlen(BT04AConnect) )
			{
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand 
				printf("filter con done!\r\n");
				#endif
			}
			break;
		case BT04A_DISCONNECTSTART:
			if( BT04ADisConnect[filterIndex] == recv )  //Start filtering the connection characters and match them byte-byte-byte
			{
				isFilter = 1;//Match connection fields; filter if matched is complete
				#if 1== DEBUG_BT04ACommand 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand
				printf("No:get:%c,but:%c\r\n",recv,BT04ADisConnect[filterIndex]);
				#endif
			}				
			
			//index until the filter list is complete
			filterIndex++;
			if( filterIndex == strlen(BT04ADisConnect) ) 
			{
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand
				printf("filter dis done!\r\n");
				#endif
			}
			break;
	}
	lastrecv = recv;
	
	return isFilter;
}


//JDY-33 Bluetooth AT command set filtering
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv)
{
	#define DEBUG_JDY33Command 0
	
	uint8_t isFilter = 0;//Whether the character is allowed to pass, 1 filter, 0 allowed
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;
	
	const char* JDY33_SPPConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\nCONNECTED\r\r\n";
	const char* JDY33_BLEConnect = "CONNECTED\r\r\n";
	const char* JDY33_DisConnect = "+DISC:SUCCESS\r\r\n\0"; //Note: 0 will not be counted for word length; you need to manually add +1 to the character length
	enum{
		JDY33_NORMAL=  0,
		JDY33_SPPCONNECTSTART,
		JDY33_BLECONNECTSTART,
		JDY33_DISCONNECTSTART,
	};
	
	static uint8_t statemachine = JDY33_NORMAL;
	
	switch( statemachine )
	{
		case JDY33_NORMAL:
			if( recv=='C'&&lastrecv=='+' )
			{
				statemachine = JDY33_SPPCONNECTSTART;//Receive feature values and start matching
				isFilter = 1;//Filter characters
				filterIndex = 2; //Indexing starts on the 2nd
			}
			else if( recv=='O'&&lastrecv=='C' )
			{
				statemachine = JDY33_BLECONNECTSTART;//Receive feature values and start matching
				isFilter = 1;//Filter characters
				filterIndex = 2; //Indexing starts on the 2nd
			}
			else if( recv=='D'&&lastrecv=='+' )
			{
				statemachine = JDY33_DISCONNECTSTART;//Receive feature values and start matching
				isFilter = 1;//Filter characters
				filterIndex = 2; //Indexing starts on the 2nd
			}
			else if( recv=='C'&&lastrecv!='C' )//Check state entry before checking ambiguity
			{
				//Ambiguity regarding C: "+C" and "CO" cannot be passed "? C is also not allowed to pass through," to ensure that no control commands exist during connection. If you need to control the cart, you need continuous C
				isFilter = 1;//Prohibited from passing
			}
			break;
		case JDY33_SPPCONNECTSTART:
			if( JDY33_SPPConnect[filterIndex] == recv )  //Start filtering the connection characters and match them byte-byte-byte
			{
				isFilter = 1;//Match connection fields; filter if matched is complete
				
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else if( (filterIndex>=13&&filterIndex<=29) && \
       				  ((recv>='0'&&recv<='9')||(recv>='A'&&recv<='Z')) )//Enter MAC-address matching stage; match fields 0~9 and a~z
			{
				isFilter = 1;//MAC address filtering
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				//Neither condition applies; allow the character through and exit filtering mode
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP->No:get:%c,but:%c\r\n",recv,JDY33_SPPConnect[filterIndex]);
				#endif
			}
			
			//index until the filter list is complete
			filterIndex++;
			if( filterIndex == strlen(JDY33_SPPConnect) )
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP filter con done!\r\n");
				#endif
			}
			break;
		case JDY33_BLECONNECTSTART:
			if( JDY33_BLEConnect[filterIndex] == recv )  //Start filtering the connection characters and match them byte-byte-byte
			{
				isFilter = 1;//Match connection fields; filter if matched is complete
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("BLE->No:get:%c,but:%c\r\n",recv,JDY33_BLEConnect[filterIndex]);
				#endif
			}				
			
			//index until the filter list is complete
			filterIndex++;
			if( filterIndex == strlen(JDY33_BLEConnect) ) 
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("ble filter dis done!\r\n");
				#endif
			}
			break;
		case JDY33_DISCONNECTSTART:
			if( JDY33_DisConnect[filterIndex] == recv )  //Start filtering the connection characters and match them byte-byte-byte
			{
				isFilter = 1;//Match connection fields; filter if matched is complete
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("dis->No:get:%c,but:%c\r\n",recv,JDY33_DisConnect[filterIndex]);
				#endif
			}				
			
			//index until the filter list is complete
			filterIndex++;
			if( filterIndex == strlen(JDY33_DisConnect)+1 ) //+1 is the supplementary null character '0'
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("filter dis done!\r\n");
				#endif
			}
			break;
	}
	lastrecv = recv;
	
	return isFilter;
}

//Debug mode
void RobotControl_SetDebugLevel(char uart_recv)
{
	
	static char res_buf[4];
	static char res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='L'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==4)
	{
		res_count = 0;
		
		if( res_buf[0]=='L'&&res_buf[1]=='O'&&res_buf[2]=='G')
		{
			switch( res_buf[3] )
			{
				case '3': Debug_Flag= 1; break;
				default: Debug_Flag= 0; break;
			}
			//Operation prompt sound
		}
	}
}
uint8_t FlashWriteFlag = 0;

int USART2_IRQHandler(void)
{	
	int Usart_Receive;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //Check if data is received // Check if data has been received
	{	      
		static u8 Flag_PID,i,j,Receive[50],Last_Usart_Receive;
		static float Data;
				
		Usart_Receive=USART2->DR; //Read the data //Read data
		
		//Filter Bluetooth AT-command responses
		uint8_t ATFilter = 0;
		ATFilter += ATCommandFeedBack_JDY33(Usart_Receive);
		ATFilter += ATCommandFeedBack_BT04A(Usart_Receive);
		if( ATFilter > 0 ) return 0; //Filter any Bluetooth filter information exists
		
		_System_Reset_(Usart_Receive);
		RobotControl_SetDebugLevel(Usart_Receive);
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //Do not process data 10 seconds before powering on
		  return 0;	
		
		if(Check==0)
		{
		if(Usart_Receive==0x41&&Last_Usart_Receive==0x41&&APP_ON_Flag==0)
			//10 seconds after startup, press the forward button of APP to enter APP control mode
		  //The APP controls the flag position 1 and the other flag position 0
			//After turning on the device for 10 seconds, press the APP's forward button to enter app control mode
		  //Set app-control flag to 1 and all other flags to 0
			PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=1,CAN_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0;
    Last_Usart_Receive=Usart_Receive;			
	  
		if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //Enter the app steering-control screen
			Turn_Flag=1;  
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//Enter the app direction-control screen
			Turn_Flag=0;	
		
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APP joystick control interface commands
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APP steering control interface commands
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation // Right rotation
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation // rotate left
			else                         Flag_Left=0,Flag_Right=0;
			if     (Usart_Receive==0x41||Usart_Receive==0x45) Flag_Direction=Usart_Receive-0x40;
			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s // Accelerate the keys, +100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons, -100mm/s // Slow down button, -100mm/s
	  
	 // The following is the communication with the APP debugging interface
	 //The following is communication with the APP debugging interface
	 if(Usart_Receive==0x7B) Flag_PID=1;   //Start bit of the APP parameter command // Start bit of APP parameter instruction
	 if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stop bit //APP parameter instruction stop bit
	
	if( Usart_Receive=='b' ) Allow_Recharge = !Allow_Recharge;
		
	 if(Flag_PID==1) //Collect data // Collect data
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2) //Analyze the data // Analyze data
	 {
			if(Receive[3]==0x50) 	 PID_Send=1;
			else if( Receive[3]==0x57 ) FlashWriteFlag = 1;
			else  if(Receive[1]!=0x23) 
			{								
				for(j=i;j>=4;j--)
				{
					Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				switch(Receive[1])
				 {
					 case 0x30:  RC_Velocity=Data;break;
					 case 0x31:  Velocity_KP=Data;break;
					 case 0x32:  Velocity_KI=Data;break;
					 case 0x33:  LineDiffParam=Data;break;
					 case 0x34:  break;
					 case 0x35:  break;
					 case 0x36:  break;
					 case 0x37:  break;
					 case 0x38:  break; 	
				 }
      }		
      //Relevant flag position is cleared			
      //Reset relevant marker positions
			Flag_PID=0;
			i=0;
			j=0;
			Data=0;
			memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero
	 }
   if(RC_Velocity<0)   RC_Velocity=0; 
 }
		else if(Check==1)
		{
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				check_parameter.Usart2_check_dir = Usart_Receive-0x40;
			}
			else 
				check_parameter.Usart2_check_dir = 9;
		}
  }
  return 0;	
}

// ========== Serial port 3 receives interrupt handling ==========
int USART3_IRQHandler(void)
{	
	static u8 Count = 0;
	u8 Usart_Receive;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
		Usart_Receive = USART_ReceiveData(USART3);
		
		if(SysVal.Time_count < CONTROL_DELAY)
			return 0;	
		
		if(Check == 0)
		{
			// ========== Handle ROS control frame ==========
			//Fill the array with serial data
			//Serial port data is populated into an array
			Receive_Data.buffer[Count] = Usart_Receive;
			// Ensure that the first data in the array is FRAME_HEADER
			//Ensure the first data in the array is FRAME_HEADER
			if(Usart_Receive == FRAME_HEADER || Count > 0) 
				Count++; 
			else 
				Count = 0;
			
			if(Count == 11)//Verify the length of the packet // Verify the length of the packet
			{   
				Count = 0; //Prepare for the serial port data to be refill into the array //Get ready to refill data from the serial port into the array
				
				if(Receive_Data.buffer[10] == FRAME_TAIL)//Verify the frame tail of the packet // Verify the frame tail of the packet
				{
					//Data exclusionary or bit check calculation, mode 0 is sent data check
					//Data XOR bit verification calculation, mode 0 is the data sending validation
					if(Receive_Data.buffer[9] == Check_Sum(9, 0))	 
					{	
						float Vz;		
						if(Receive_Data.buffer[1] == 0)
						{			
							Allow_Recharge = 0;//Turn off automatic recharge to receive normal data
							//All modes flag position 0, USART3 control mode
							//Set all mode flags to 0 for Usart3 control mode
							PS2_ON_Flag = 0;
							Remote_ON_Flag = 0;
							APP_ON_Flag = 0;
							CAN_ON_Flag = 0;
							Usart1_ON_Flag = 0;
							Usart5_ON_Flag = 0;
							command_lost_count = 0;
							//Calculate the target speed of three axis from serial data, unit m/s
							//Calculate the three-axis target velocity from serial port data, unit m/s
							Move_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3], Receive_Data.buffer[4]);
							Move_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5], Receive_Data.buffer[6]);
							Vz = XYZ_Target_Speed_transition(Receive_Data.buffer[7], Receive_Data.buffer[8]);
							
							if(Car_Mode == Akm_Car)
							{
								Move_Z = Vz_to_Akm_Angle(Move_X, Vz);
							}
							else
							{
								Move_Z = Vz;
							}		
						}
						else if(Receive_Data.buffer[1] == 1 || Receive_Data.buffer[1] == 2)
						{
							Allow_Recharge = 1;//Enable automatic recharge
							if(Receive_Data.buffer[1] == 1 && RED_STATE == 0) 
								nav_walk = 1;//Enable automatic recharging, with the host computer controlling the robot
							//Calculate the target speed of three axis from serial data, unit m/s
							//Calculate the three-axis target velocity from serial port data, unit m/s
							Recharge_UP_Move_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3], Receive_Data.buffer[4]);
							Recharge_UP_Move_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5], Receive_Data.buffer[6]);
							Vz = XYZ_Target_Speed_transition(Receive_Data.buffer[7], Receive_Data.buffer[8]);	
							
							if(Car_Mode == Akm_Car)
							{
								Recharge_UP_Move_Z = Vz_to_Akm_Angle(Recharge_UP_Move_X, Vz);
							}
							else
								Recharge_UP_Move_Z = Vz;
						}
						else if(Receive_Data.buffer[1] == 3)
						{
							//Set the speed of the infrared interconnection, unit m/s
							//Set the speed of infrared docking, unit m/s
							Red_Docker_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3], Receive_Data.buffer[4]);
							Red_Docker_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5], Receive_Data.buffer[6]);
							Red_Docker_Z = XYZ_Target_Speed_transition(Receive_Data.buffer[7], Receive_Data.buffer[8]);	
						}
										 // ================== IP Frame Processing (New) ================== //
						else if(Receive_Data.buffer[1] == 0xFF)  // IP frame identification
						{
							uint8_t temp_ip[4];
							
							// Extract received IPs (index 3-6)
							temp_ip[0] = Receive_Data.buffer[3];
							temp_ip[1] = Receive_Data.buffer[4];
							temp_ip[2] = Receive_Data.buffer[5];
							temp_ip[3] = Receive_Data.buffer[6];
							
							
							// Check if this IP is the same as the last one
							if(temp_ip[0] == Last_IP[0] &&
							temp_ip[1] == Last_IP[1] &&
							temp_ip[2] == Last_IP[2] &&
							temp_ip[3] == Last_IP[3])
							{
								// IP is continuously the same, with increasing counts
								IP_Valid_Count++;
								
								// Receive the same IP multiple times consecutively, confirming it is correct
								if(IP_Valid_Count >= IP_CONFIRM_COUNT)
								{
									// Write the confirmed IP to the final result
									Received_IP[0] = temp_ip[0];
									Received_IP[1] = temp_ip[1];
									Received_IP[2] = temp_ip[2];
									Received_IP[3] = temp_ip[3];
									IP_Frame_Valid = 1;          // Tag IP is valid
								}
							}
							else
							{
								// IP changes, reset counter, save the new IP to Last_IP
								IP_Valid_Count = 1;  // Count from 1
								Last_IP[0] = temp_ip[0];
								Last_IP[1] = temp_ip[1];
								Last_IP[2] = temp_ip[2];
								Last_IP[3] = temp_ip[3];
								
							}
						}
					}
				}
			}
		}
		else if(Check==1)
		{
			check_parameter.Usart3_rec_buffer[check_parameter.Usart3_rec_count] = Usart_Receive;
			if(Usart_Receive=='#')		
			{
				check_parameter.Usart3_rec_buffer[check_parameter.Usart3_rec_count]='\0';
				check_parameter.Usart3_check_flag = 1;
			}
			check_parameter.Usart3_rec_count++;
		}
	} 
  return 0;	
}

/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
Function: Serial Port 5 receive interrupt
Entry parameters: None
Return value: none
**************************************************************************/
int UART5_IRQHandler(void)
{	
	static u8 Count=0;
	u8 Usart_Receive;

	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //Check if data is received // Check if data has been received
	{
		Usart_Receive = USART_ReceiveData(UART5);//Read the data //Read data
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //Do not process data 10 seconds before powering on
		  return 0;	
		
		//Fill the array with serial data
		//Serial port data is populated into an array
    Receive_Data.buffer[Count]=Usart_Receive;
		
		// Ensure that the first data in the array is FRAME_HEADER
		//Ensure the first data in the array is FRAME_HEADER
		if(Usart_Receive == FRAME_HEADER||Count>0) 
			Count++; 
		else 
			Count=0;
		
		if (Count == 11) //Verify the length of the packet // Verify the length of the packet
		{   
				Count=0; //Prepare for the serial port data to be refill into the array //Get ready to refill data from the serial port into the array
				if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet // Verify the frame tail of the packet
				{
					//Data exclusionary or bit check calculation, mode 0 is sent data check
					//Data XOR bit verification calculation, mode 0 is the data sending validation
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 
				  {	
						float Vz;						
						//All modes flag position 0, USART3 control mode
            //Set all mode flags to 0 for Usart5 control mode
						PS2_ON_Flag=0;
						Remote_ON_Flag=0;
						APP_ON_Flag=0;
						CAN_ON_Flag=0;
						Usart5_ON_Flag=0;
						command_lost_count=0;
						//Calculate the target speed of three axis from serial data, unit m/s
						//Calculate the three-axis target velocity from serial port data, unit m/s
						Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
						Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						Vz    =XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
						if(Car_Mode==Akm_Car)
						{
							Move_Z=Vz_to_Akm_Angle(Move_X, Vz);
						}
						else
						{
							Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
						}				  }
			}
		}
	} 
  return 0;	
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
Function: Converts the target forward speed Vx and target angular velocity Vz sent from the host computer into the right front wheel rotation angle of the Ackermann car
Entry parameters: target forward velocity Vx, target angular velocity Vz, units: m/s, rad/s
Return value: Ackermann chassis front-right wheel steering angle, unit: rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//Ackermann chassis requires a minimum turning radius
	//If the target speed requires a turning radius less than the minimum turning radius,
	//This can greatly increase the friction force of the chassis, seriously affecting control effectiveness
	Min_Turn_Radius=MINI_AKM_MIN_TURN_RADIUS;
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//If the target speed requires a turning radius less than the minimum turning radius
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//Reduce target angular velocity relative to forward speed to increase turning radius to the minimum allowed radius
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);	
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Axle_spacing/(R+0.5f*Wheel_spacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
Function: Integrates the high 8-bit and low 8-bit data sent from the host computer into a single short data, then performs unit restore and conversion
Entry parameters: 8 bits high, 8 bits lower
Return value: robot target X/Y/Z-axis velocities
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//Intermediate variable for data transformation
	short transition; 
	
	//Combine the high 8 bits and low 8 bits into a single 16-bit short data
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s - >m/s // Unit conversion, mm/s - >m/s
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
Function: Send data via Serial Port 1
Entry parameters: Data to send
Return value: none
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
Function: Send data via Serial Port 2
Entry parameters: Data to send
Return value: none
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
Function: Send data via Serial Port 3
Entry parameters: Data to send
Return value: none
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}

/**************************************************************************
Function: Serial port 5 sends data
Input   : The data to send
Output  : none
Function: Send data via Serial Port 5
Entry parameters: Data to send
Return value: none
**************************************************************************/
void usart5_send(u8 data)
{
	UART5->DR = data;
	while((UART5->SR&0x40)==0);	
}
/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
Function: Calculates the data validation results to be sent or received
Entry parameters: Count_Number: Verify the first few digits; Mode: 0 - Verify received data, 1 - Verify sent data
Return value: checksum result
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//Validate the data to be sent
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//Validate the received data
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}

//Automatic recharge sends byte-specific parity functions
u8 Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//Validate the data to be sent
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_AutoCharge_Data.buffer[k];
	}
	
	return check_sum;	
}


//Bluetooth AT command packet capture to prevent instructions from interfering with the robot's normal Bluetooth communication
u8 AT_Command_Capture(u8 uart_recv)
{
	/*
	Characters sent during Bluetooth connection, 00:11:22:33:44:55 are the MAC addresses of Bluetooth
	+CONNECTING<<00:11:22:33:44:55\r\n
	+CONNECTED\r\n
	A total of 44 characters
	
	Characters sent when Bluetooth is disconnected
	+DISC:SUCCESS\r\n
	+READY\r\n
	+PAIRABLE\r\n
	A total of 34 characters
	\r -> 0x0D
	\n -> 0x0A
	*/
	
	static u8 pointer = 0; //Pointer recorder when Bluetooth accepts
	static u8 bt_line = 0; //indicates which row it is currently on
	static u8 disconnect = 0;
	static u8 connect = 0;
	
	//Disconnect
	static char* BlueTooth_Disconnect[3]={"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};
	
	//Start the connection
	static char* BlueTooth_Connect[2]={"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


	//Special identifiers, start alert (use -1)
	if(uart_recv=='+') 
	{
		bt_line++,pointer=0; //Receiving a '+' means the number of lines has been switched
		disconnect++,connect++;
		return 1;//Capture packets, prohibit control
	}

	if(bt_line!=0) 
	{	
		pointer++;

		//Start tracking whether the data matches the disconnected feature; if it does, block all; if not, unmask
		if(uart_recv == BlueTooth_Disconnect[bt_line-1][pointer])
		{
			disconnect++;
			if(disconnect==34) disconnect=0,connect=0,bt_line=0,pointer=0;
			return 1;//Capture packets, prohibit control
		}			

		//Track connection signature: the (bt_line==1&&connect>=13) segment is the Bluetooth MAC address; skip it because each address differs
		else if(uart_recv == BlueTooth_Connect[bt_line-1][pointer] || (bt_line==1&&connect>=13) )
		{		
			connect++;
			if(connect==44) connect=0,disconnect=0,bt_line=0,pointer=0;		
			return 1;//Capture packets, prohibit control
		}	

		//Stop capturing after receiving other commands during packet capture
		else
		{
			disconnect = 0;
			connect = 0;
			bt_line = 0;		
			pointer = 0;
			return 0;//Data is not prohibited; control is allowed
		}			
	}
	
	return 0;//Data is not prohibited; control is allowed
}

//Software reset into the BootLoader region
void _System_Reset_(u8 uart_recv)
{
	static u8 res_buf[5];
	static u8 res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//Upon receiving the reset character "reset" requested by the host computer, perform a software reset
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			NVIC_SystemReset();//Perform a software reset; execute BootLoader after reset
		}
	}
}




