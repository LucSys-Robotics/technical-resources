#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
SEND_AutoCharge_DATA Send_AutoCharge_Data;
extern int Time_count;
u8 usart_sent[21];
u8 usart_sent_current[21];//Send current
u8 	fun_flag;
float Step_SPEED=90;
float last_Moveit_Angle1,last_Moveit_Angle2,last_Moveit_Angle3;
SemaphoreHandle_t xMutex;
/**************************************************************************
Function: Usartx3, Usartx1 and CAN send data task 
Input   : none
Output  : none
Function: Serial Ports 3, 1, and CAN send data task
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
      data_transition(); 
			USART3_SEND(); 
			USART1_SEND();
			if(A280.joint_flag[0] || A280.joint_flag[1] || A280.joint_flag[2])
			{
				CAN_UpdateAndSend();
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
	#if Mec
	Motion_analysis_transformation(MOTOR_A.Encoder,MOTOR_B.Encoder,MOTOR_C.Encoder,MOTOR_D.Encoder); 
	#elif Omni
	Motion_analysis_transformation(MOTOR_A.Encoder,MOTOR_B.Encoder,MOTOR_C.Encoder); 
	#elif Diff
	Motion_analysis_transformation(MOTOR_A.Encoder,MOTOR_B.Encoder);
	#endif
	
	//The acceleration of the triaxial acceleration // Triaxial accelerometer acceleration
	Send_Data.Sensor_Str.Accelerometer.X_data= accel[1]; //The accelerometer Y-axis is converted to the ROS coordinate X-axis // Accelerometer Y-axis converted to ROS coordinate X-axis
	Send_Data.Sensor_Str.Accelerometer.Y_data=-accel[0]; //The accelerometer X-axis is converted to the ROS coordinate Y-axis // Accelerometer X-axis converted to ROS coordinate Y-axis
	Send_Data.Sensor_Str.Accelerometer.Z_data= accel[2]; //The accelerometer Z-axis is converted to the ROS coordinate Z-axis // Accelerometer Z-axis converted to ROS coordinate Z-axis
	
	//The angular velocity of the triaxial gyro //Triaxial gyro angular velocity
	Send_Data.Sensor_Str.Gyroscope.X_data= gyro[1]; //The Y-axis is converted to the ROS coordinate X-axis // Gyroscope Y-axis converted to ROS coordinate X-axis
	Send_Data.Sensor_Str.Gyroscope.Y_data=-gyro[0]; //The X-axis is converted to the ROS coordinate Y-axis // Gyroscope X-axis converted to ROS coordinate Y-axis
	if(Flag_Stop==0) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //If motor control is enabled, normally send Z-axis angular velocity
		Send_Data.Sensor_Str.Gyroscope.Z_data=gyro[2];  
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
	for(i=0; i<24; i++)
	{
		usart3_send(Send_Data.buffer[i]);
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
Function: Serial port 4 initialization
Input   : none
Output  : none
Function: Initialize Serial Port 4
Entry parameters: None
Return value: none
**************************************************************************/
void uart4_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //Enable the Usart clock
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11 ,GPIO_AF_UART4);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Output mode
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //Push-pull output
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //High speed: 50MHz
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //Pull up
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //Initialize
	
	//UsartNVIC configuration // UsartNVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
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
	USART_Init(UART4, &USART_InitStructure);      //Initialize serial port 2
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts
	USART_Cmd(UART4, ENABLE);                     //Enable serial port 2
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
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	 //Enable the gpio clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Output mode
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //Push-pull output
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //High speed: 50MHz
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //Pull up
	GPIO_Init(GPIOB, &GPIO_InitStructure);  		          //Initialize
	
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
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
Function: Serial Port 1 receive interrupt
Entry parameters: None
Return value: none
**************************************************************************/
int USART1_IRQHandler(void)
{	
	static u8 Count=0,Count_moveit;
	u8 Usart_Receive;
	static char rxbuf_moveit[11];
		int check=0,error=1,i;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received // Check if data has been received
	{
		Usart_Receive = USART_ReceiveData(USART1);//Read the data //Read data
		if(Time_count<CONTROL_DELAY)
			// Data is not processed until 25 seconds after startup
		  //Do not process data 25 seconds before powering on
			return 0;	//Do not interrupt early on
		
		//Fill the array with serial data
		//Serial port data is populated into an array
       Receive_Data.buffer[Count]=Usart_Receive;
	   rxbuf_moveit[Count_moveit]=Usart_Receive;
		//Ensure that the first data in the array is FRAME_HEADER
		//Ensure the first data in the array is FRAME_HEADER
		if(Usart_Receive == FRAME_HEADER||Count>0) 
			Count++; 
		else if(Usart_Receive == FRAME_HEADER_MOVEIT||Count_moveit>0)
			Count_moveit++;
		else 
			Count=0,Count_moveit=0;
		
		if(Count == 11) //Verify the length of the packet // Verify the length of the packet
		{   
				Count=0; //Prepare for the serial port data to be refill into the array //Get ready to refill data from the serial port into the array
				if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet // Verify the frame tail of the packet
				{
					//Data exclusionary or bit check calculation, mode 0 is sent data check
					//Data XOR bit verification calculation, mode 0 is the data sending validation
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))
				  {		
						//Serial port 1 controls flag position 1, other flag position 0
						//Set serial-port-1 control flag to 1 and all other flags to 0
						PS2_ON_Flag=0;
						Remote_ON_Flag=0;
						APP_ON_Flag=0;
						CAN_ON_Flag=0;
						Usart_ON_Flag=1;
						command_lost_count=0;//Reset the serial port and CAN control command loss counts
					  
						//Calculate the target speed of three axis from serial data, unit m/s
						//Calculate the three-axis target velocity from serial port data, unit m/s
						Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
						Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
				  }
			}
		}
		if (Count_moveit == 10) //Verify the length of the packet // Verify the length of the packet
		{   
			   Count_moveit=0;
		       if(rxbuf_moveit[9] == FRAME_TAIL_MOVEIT) //Verify the checksum at the end of the data packet
				{ 
					for(i=0; i<8; i++)
					{
						check=rxbuf_moveit[i]^check; //XOR, used to detect data errors
					}
					if(check==rxbuf_moveit[8]) error=0; //Inspection successful
					
					if(error==0)	 //Data parity calculations
				  {		
					    
						A280.control_mode=ROS_Mode;
			            
						       
						A280.joint_rad[0]=((short)((rxbuf_moveit[1]<<8)+(rxbuf_moveit[2])))*0.001f;   //Take the target angle of the first motor, divided into 8 high and low 8 bits, units (radians)
						A280.joint_rad[1]=((short)((rxbuf_moveit[3]<<8)+(rxbuf_moveit[4])))*0.001f;   //Take the target angle of the second motor, divided into 8 high and low 8 bits, unit (radians)
						A280.joint_rad[2]=((short)((rxbuf_moveit[5]<<8)+(rxbuf_moveit[6])))*0.001f;   //Take the target angle of the third motor, divided into 8 high and low 8 bits, unit (radians)
						A280.gripper=rxbuf_moveit[7];   
						UpdateMotorTargets_FromJoints(&A280,123);
						Gripper_SetLevel(A280.gripper);
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
Function: Serial Port 4 receive interrupt
Entry parameters: None
Return value: none
**************************************************************************/
int UART4_IRQHandler(void)
{	
	  int Usart_Receive;
	  u8 Last_Usart_Receive;
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //Check if data is received // Check if data has been received
    {
        static u8 Flag_PID,i,j,Receive[50];
        static float Data;

        Usart_Receive=UART4->DR; //Read the data //Read data

        if(Deviation_Count<CONTROL_DELAY)
            // Data is not processed until 10 seconds after startup
            //Do not process data 10 seconds before powering on
            return 0;

//		if(Usart_Receive==0x41&&Last_Usart_Receive==0x41&&APP_ON_Flag==0)
//			//10 seconds after startup, press the forward button of APP to enter APP control mode
//		  //The APP controls the flag position 1 and the other flag position 0
//			//After turning on the device for 10 seconds, press the APP's forward button to enter app control mode
//		  //Set app-control flag to 1 and all other flags to 0
//		  Arm_Mode_APP=0,
//			PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;

        //Switch APP control car mode
       //Switch APP control car mode
         if(Usart_Receive=='a')
        {
            APP_ON_Flag=1;
           A280.control_mode=ROS_Mode;
        }
        //Switch forward kinematics mode
        else if(Usart_Receive=='b')
        {
            APP_ON_Flag=0;
            A280.control_mode=APP_Mode;
					  A280.kinematic_mode=Forward_Kinematic;
        }
        //Switch inverse kinematics mode
        else if(Usart_Receive=='c')
        {
            APP_ON_Flag=0;
					  A280.control_mode=APP_Mode;
            A280.kinematic_mode=Inverse_Kinematic;
        }

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
				
					if(Usart_Receive>='a' && Usart_Receive<='o')
				{
					fun_flag=Usart_Receive-0x60;
				}
					
				
        if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s // Accelerate the keys, +100mm/s
        else if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons, -100mm/s // Slow down button, -100mm/s

        // The following is the communication with the APP debugging interface
        //The following is communication with the APP debugging interface
        if(Usart_Receive==0x7B) Flag_PID=1;   //Start bit of the APP parameter command // Start bit of APP parameter instruction
        else if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stop bit //APP parameter instruction stop bit

        if(Flag_PID==1) //Collect data // Collect data
        {
            Receive[i]=Usart_Receive;
            i++;
        }
        if(Flag_PID==2) //Analyze the data // Analyze data
        {
            if(Receive[3]==0x50) 	 PID_Send=1;
            else  if(Receive[1]!=0x23)
            {
                for(j=i; j>=4; j--)
                {
                    Data+=(Receive[j-1]-48)*pow(10,i-j);
                }
                switch(Receive[1])
                {
                case 0x30:
                    RC_Velocity=Data;
                    break;
                case 0x31:
                    Velocity_KP=Data;
                    break;
                case 0x32:
                    Velocity_KI=Data;
                    break;
//                case 0x33:
//                    Target_Angle_A=(Data-4000)/1000;
//                    break;
//                case 0x34:
//                    Target_Angle_B=(Data-4000)/1000;
//                    break;
//                case 0x35:
//                    Target_Angle_C=(Data-4000)/1000;
//                    break;
//                case 0x36:
//                    Target_Angle_D=(Data-4000)/1000;
//                    break;
//                case 0x37:
//                    ARM_Target_X=Data/1000-0.4f;
//                    break;
//                case 0x38:
//                    ARM_Target_Y=Data/1000-0.4f;
//                    break;
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
    return 0;
}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
Function: Serial Port 3 receive interrupt
Entry parameters: None
Return value: none
**************************************************************************/
int USART3_IRQHandler(void)
{	
	static u8 Count=0,Count_moveit;
	u8 Usart_Receive;
	static char rxbuf_moveit[11];
	int check=0,error=1,i;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received // Check if data has been received
	{
		Usart_Receive = USART_ReceiveData(USART3);//Read the data //Read data
		if(Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //Do not process data 10 seconds before powering on
		  return 0;	
		
		//Fill the array with serial data
		//Serial port data is populated into an array
		 Receive_Data.buffer[Count]=Usart_Receive;
	   rxbuf_moveit[Count_moveit]=Usart_Receive;
		//Ensure that the first data in the array is FRAME_HEADER
		//Ensure the first data in the array is FRAME_HEADER
		if(Usart_Receive == FRAME_HEADER||Count>0) 
			Count++; 
		else if(Usart_Receive == FRAME_HEADER_MOVEIT||Count_moveit>0)
			Count_moveit++;
		else 
			Count=0,Count_moveit=0;
		
		
		if (Count == 11) //Verify the length of the packet // Verify the length of the packet
		{   
			Count=0; //Prepare for the serial port data to be refill into the array //Get ready to refill data from the serial port into the array
			if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet // Verify the frame tail of the packet
			{
				//Data exclusionary or bit check calculation, mode 0 is sent data check
				//Data XOR bit verification calculation, mode 0 is the data sending validation
				if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 
			   {		
				  command_lost_count=0; //CAN/serial port control command loss count cleared to zero
					if(Receive_Data.buffer[1]==0)
					{
						
						//All modes flag position 0, USART3 control mode
						//Set all mode flags to 0 for Usart3 control mode
						PS2_ON_Flag=0;
						Remote_ON_Flag=0;
						APP_ON_Flag=0;
						CAN_ON_Flag=0;
						Usart_ON_Flag=0;
										
						//Calculate the target speed of three axis from serial data, unit m/s
						//Calculate the three-axis target velocity from serial port data, unit m/s
						Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
						Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);		
					}
			  }
			}
		}
		
		if (Count_moveit == 10) //Verify the length of the packet // Verify the length of the packet
		{   
			   Count_moveit=0;
		       if(rxbuf_moveit[9] == FRAME_TAIL_MOVEIT) //Verify the checksum at the end of the data packet
				{ 
					 
					for(i=0; i<8; i++)
					{
						check=rxbuf_moveit[i]^check; //XOR, used to detect data errors
					}
					if(check==rxbuf_moveit[8]) error=0; //Inspection successful
					
					if(error==0)	 //Data parity calculations
				  {		
					    
							A280.control_mode=ROS_Mode;
			            
						       
						A280.joint_rad[0]=((short)((rxbuf_moveit[1]<<8)+(rxbuf_moveit[2])))*0.001f;   //Take the target angle of the first motor, divided into 8 high and low 8 bits, units (radians)
						A280.joint_rad[1]=((short)((rxbuf_moveit[3]<<8)+(rxbuf_moveit[4])))*0.001f;   //Take the target angle of the second motor, divided into 8 high and low 8 bits, unit (radians)
						A280.joint_rad[2]=((short)((rxbuf_moveit[5]<<8)+(rxbuf_moveit[6])))*0.001f;   //Take the target angle of the third motor, divided into 8 high and low 8 bits, unit (radians)
						A280.gripper=rxbuf_moveit[7];   
						UpdateMotorTargets_FromJoints(&A280,123);
						Gripper_SetLevel(A280.gripper);
				  }
			  }
		}
	} 
return 0;	
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
Function: The current XYZ three axis velocity of the robot is calculated from the data of the encoder
Input   : Data from various encoders
Output  : none
Function: Calculates the robot's current XYZ three-axis speed based on encoder data
Input parameters: data from each encoder
Return value: none
**************************************************************************/
#if Mec
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C,float Encoder_D)
{
	Send_Data.Sensor_Str.X_speed = ((Encoder_A+Encoder_B+Encoder_C+Encoder_D)/4)*1000; 
	Send_Data.Sensor_Str.Y_speed = ((Encoder_A-Encoder_B+Encoder_C-Encoder_D)/4)*1000; 
	Send_Data.Sensor_Str.Z_speed = ((-Encoder_A-Encoder_B+Encoder_C+Encoder_D)/4/(Wheel_axlespacing+Wheel_spacing))*1000;
}
#elif Omni
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C)
{
	Send_Data.Sensor_Str.X_speed = ((Encoder_C-Encoder_B)/2/X_PARAMETER)*1000;
	Send_Data.Sensor_Str.Y_speed = ((Encoder_A*2-Encoder_B-Encoder_C)/3)*1000; 
	Send_Data.Sensor_Str.Z_speed = ((Encoder_A+Encoder_B+Encoder_C)/3/Omni_turn_radiaus)*1000;
}
#elif Diff
void Motion_analysis_transformation(float Encoder_A,float Encoder_B)
{
		Send_Data.Sensor_Str.X_speed = ((Encoder_A+Encoder_B)/2)*1000; //The speed of the cart's x-axis
		Send_Data.Sensor_Str.Y_speed = 0;
		Send_Data.Sensor_Str.Z_speed = ((Encoder_B-Encoder_A)/Wheel_spacing)*1000;//chassis Z-axis speed
}
#endif
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
void usart4_send(u8 data)
{
	UART4->DR = data;
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



