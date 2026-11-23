#include "uartx_callback.h"
#include "data_task.h"

//Structure for receiving data
RECEIVE_DATA Receive_Data;

/**************************************************************************¡¢
Functionality: This function receives serial commands to control the movement of a robot.
Input Parameters:
Which serial port (e.g.,USART1,USART3,etc.)
The received data from the serial port
Return Value: None
Author: LUCSYS
Function: Receives serial port commands to control robot movements
Entry parameters: which serial port receives data
Return value: none
Author: LUCSYS
**************************************************************************/
static void UartxControll_Callback(USART_TypeDef* USARTx,uint8_t Usart_Receive)
{
	static uint16_t Count = 0;
	static uint16_t Count_moveit = 0;
	static USART_TypeDef* Last_Uartx;
	static char rxbuf_moveit[11];
	int check=0,error=1,i;
	//Functions are prohibited from multiple serial ports sharing simultaneously; only one interface can control the robot at a time.
	if( USARTx != Last_Uartx && Count!=0 )
	{
		Count = 0;
	}
	Last_Uartx = USARTx;
	
	//Fill the array with serial data
	//Serial port data is populated into an array
	Receive_Data.buffer[Count]=Usart_Receive;
	rxbuf_moveit[Count_moveit]=Usart_Receive;
	// Ensure that the first data in the array is FRAME_HEADER
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
			if(Receive_Data.buffer[9] ==Check_BCC(Receive_Data.buffer,9))
			{	
				//Reset the serial port and CAN control command loss counts
				robot_control.command_lostcount = 0;

				if(Receive_Data.buffer[1]==0)
				{
					//Determine the control method based on the different serial ports
					       if(USARTx==USART1) Set_Control_Mode(_USART_Control);
					else if( USARTx==USART3 ) Set_Control_Mode(_ROS_Control);
					
					charger.AllowRecharge = 0;
					//Calculate the target speed of three axis from serial data, unit m/s
					//Calculate the three-axis target velocity from serial port data, unit m/s
					robot_control.Vx = XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
					robot_control.Vy = XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
					robot_control.Vz = XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
					
					#if defined AKM_CAR 
					//Convert the Z-axis speed to the Ackerman left front wheel steering angle \
					for an Ackerman-type vehicle, and then input the calculated angle into the \
					inverse kinematics to control the vehicle.	
					//Ackermann chassis converts Z-axis velocity to front-left wheel angle, then passes it to inverse kinematics for chassis control.
					robot_control.Vz = Akm_Vz_to_Angle( robot_control.Vx , robot_control.Vz );
					#endif
				}
				else if( Receive_Data.buffer[1]==1 || Receive_Data.buffer[1]==2 )
				{
					//Enable automatic recharge
					charger.AllowRecharge = 1;
					
					//Ackermann robot determines navigation control from a flag
					#if defined AKM_CAR
						if(Receive_Data.buffer[1]==2) charger.NavWalk=1; 
						else charger.NavWalk=0;
					
					//Other robots determine whether to use navigation control via infrared
					#else
						if(Receive_Data.buffer[1]==1 && charger.RED_STATE==0) charger.NavWalk = 1; 
					#endif
					
					//Calculate the target speed of three axis from serial data, unit m/s
					//Calculate the three-axis target velocity from serial port data, unit m/s
					charger.Up_MoveX = XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
					charger.Up_MoveY = XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
					charger.Up_MoveZ = XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
					
					#if defined AKM_CAR 
					//Convert the Z-axis speed to the Ackerman letf front wheel steering angle \
					for an Ackerman-type vehicle, and then input the calculated angle into the \
					inverse kinematics to control the vehicle.	
					//Ackermann chassis converts Z-axis velocity to front-left wheel angle, then passes it to inverse kinematics for chassis control.
					charger.Up_MoveZ = Akm_Vz_to_Angle( charger.Up_MoveX , charger.Up_MoveZ );
					#endif

				}
				else if( Receive_Data.buffer[1]==3 )
				{
					//Set the speed of the infrared interconnection, unit m/s
					//Set the speed of infrared docking, unit m/s
					charger.Dock_MoveX = XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
					charger.Dock_MoveY = XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
					charger.Dock_MoveZ = XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
				}
			}
		}
	}
		if (Count_moveit == 10) //Verify the length of the packet // Verify the length of the packet
		{   
			   Count_moveit=0;
		     if(rxbuf_moveit[9] == FRAME_TAIL_MOVEIT) //Verify the checksum at the end of the data packet
				{ 
					 
					for(u8 i=0; i<8; i++)
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
    u8 Usart_Receive;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received // Check if data has been received
    {
        Usart_Receive = USART_ReceiveData(USART1);//Read the data //Read data
		
		// Data is not processed until 25 seconds after startup
		//Do not process data CONTROL_DELAY seconds before startup
        if(SysVal.Time_count<CONTROL_DELAY)
            return 0;	//Do not interrupt early on

		//Processing serial control data for robots
        UartxControll_Callback(USART1,Usart_Receive);
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
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //Check if data is received // Check if data has been received
    {
        static u8 Flag_PID,i,j,Receive[50],Last_Usart_Receive;
        static float Data;

        Usart_Receive=UART4->DR; //Read the data //Read data
		
		//Allow Bluetooth to reset the system and enter the BootLoader program
        _System_Reset_(Usart_Receive);

		// Data is not processed until 10 seconds after startup
		//Do not process data 10 seconds before powering on
        if(SysVal.Time_count<CONTROL_DELAY)
            return 0;
		
		//Capture AT commands when APP connects and disconnects to prevent interference
        if(AT_Command_Capture(Usart_Receive)) return 1;

		//10 seconds after startup, press the forward button of APP to enter APP control mode
		//The APP controls the flag position 1 and the other flag position 0
		//After turning on the device for 10 seconds, press the APP's forward button to enter app control mode
//        if(Usart_Receive==0x41&&Last_Usart_Receive==0x41&&(Get_Control_Mode(_APP_Control))==0)
//           Set_Control_Mode(_APP_Control);

//        Last_Usart_Receive=Usart_Receive;
				        //Switch APP control car mode
        if(Usart_Receive=='a')
        {
           Set_Control_Mode(_APP_Control);
           A280.control_mode=ROS_Mode;
        }
        //Switch forward kinematics mode
        else if(Usart_Receive=='b')
        { 
            Set_Control_Mode(_ROS_Control);
            A280.control_mode=APP_Mode;
					  A280.kinematic_mode=Forward_Kinematic;
        }
        //Switch inverse kinematics mode
        else if(Usart_Receive=='c')
        {
            Set_Control_Mode(_ROS_Control);
            A280.control_mode=APP_Mode;
            A280.kinematic_mode=Inverse_Kinematic;
        }
				
			
        //Enter the APP steering control interface
        //Enter the app steering-control screen
        if(Usart_Receive==0x4B)
            appkey.TurnPage = 1;
        else if(Usart_Receive==0x49||Usart_Receive==0x4A)
            appkey.TurnPage = 0;

        if( 0 == appkey.TurnPage )
        {
            //App rocker control interface command
            //APP joystick control interface commands
            if(Usart_Receive>=0x41&&Usart_Receive<=0x48)
            {
                appkey.DirectionFlag=Usart_Receive-0x40;
            }
            else	if(Usart_Receive<=8)
            {
                appkey.DirectionFlag=Usart_Receive;
            }
            else  appkey.DirectionFlag=0;
        }
        else if( 1 == appkey.TurnPage )
        {
            //APP steering control interface command
            //APP steering control interface commands
            if     (Usart_Receive==0x43) appkey.TurnFlag = 2; //Right rotation // Right rotation
            else if(Usart_Receive==0x47) appkey.TurnFlag = 1; //Left rotation // rotate left
            else                         appkey.TurnFlag = 0;

            if     (Usart_Receive==0x41||Usart_Receive==0x45) appkey.DirectionFlag=Usart_Receive-0x40;
            else  appkey.DirectionFlag=0;
        }

        if(Usart_Receive==0x58)  robot_control.rc_speed+=100; //Accelerate the keys, +100mm/s // Accelerate the keys, +100mm/s
        if(Usart_Receive==0x59)  robot_control.rc_speed-=100; //Slow down buttons, -100mm/s // Slow down button, -100mm/s

        // The following is the communication with the APP debugging interface
        //The following is communication with the APP debugging interface
        if(Usart_Receive==0x7B) Flag_PID=1;   //Start bit of the APP parameter command // Start bit of APP parameter instruction
        if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stop bit //APP parameter instruction stop bit
		
        if( Usart_Receive=='b' ) charger.AllowRecharge = !charger.AllowRecharge;
        else if(Usart_Receive=='m'  )
        {
            oled.page++;
            if(oled.page>oled.MAX_PAGE) oled.page=1;
        }

        if(Flag_PID==1) //Collect data // Collect data
        {
            Receive[i]=Usart_Receive;
            i++;
        }
        if(Flag_PID==2) //Analyze the data // Analyze data
        {
            if(Receive[3]==0x50) 	 appkey.ParamSendflag = 1; //Send data to the app display
            else if( Receive[3]==0x57 ) appkey.ParamSaveFlag = 1; //Save parameters to STM32 Flash
            else  if(Receive[1]!=0x23)
            {
                for(j=i; j>=4; j--)
                {
                    Data+=(Receive[j-1]-48)*pow(10,i-j);
                }
                switch(Receive[1])
                {
					case 0x30: robot_control.rc_speed = Data ; break; //Modify the robot's remote control speed
					//PID parameter settings
					case 0x31:
						Set_Robot_PI_Param(Data,-1);/* Use -1 to indicate the corresponding parameter is not set */
						break;
					case 0x32:
						Set_Robot_PI_Param(-1,Data);/* Use -1 to indicate the corresponding parameter is not set */
						break;
					
					case 0x33: robot_control.smooth_MotorStep = Data/1000 ; break; 
					case 0x34: robot_control.smooth_ServoStep = Data ;  break;         
					case 0x35: Akm_Servo.Max=Data;  break;        
					case 0x36: Akm_Servo.Min=Data; break;                
					case 0x37: Akm_Servo.Mid=Data; break;                
					case 0x38: robot_control.LineDiffParam = Data; break;       
                }
            }
            else if( Receive[1]==0x23 ) //Click "Send All Data" on the app to handle the process
            {
                float num;
                u8 dataIndex=0;
                float dataArray[9];

                if( i<=50 ) //Data is within acceptable range
                {
                    Receive[i]='}'; //Supplement frame ends

                    for(u8 kk=0; Receive[kk]!='}'; kk++)
                    {
                        if( Receive[kk]>='0' && Receive[kk]<='9' )
                        {
                            num = num*10 + ( Receive[kk] - '0' );
                        }
                        else if( Receive[kk]==':' )
                        {
                            dataArray[dataIndex++] = num;
                            num = 0;
                        }

                    }
                    //Process the last piece of data
                    dataArray[dataIndex] = num;

                    //Data assignment
                    robot_control.rc_speed = dataArray[0];

                    //kp¡¢ki
                    Set_Robot_PI_Param(dataArray[1],dataArray[2]);
					
                    //Velocity smoothing coefficient
                    robot_control.smooth_MotorStep = dataArray[3]/1000;
                    robot_control.smooth_ServoStep = dataArray[4];

                    //Ackermann steering data
                    Akm_Servo.Max = dataArray[5];
                    Akm_Servo.Min = dataArray[6];
                    Akm_Servo.Mid = dataArray[7];
					
					//Correction factor
					robot_control.LineDiffParam = dataArray[8];
					
                    //Actively send modified data to the app for display
                    appkey.ParamSendflag=1;
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
		
		//Set maximum and minimum speed limits on the remote control
		robot_control.rc_speed = target_limit_float(robot_control.rc_speed,0,robot_control.limt_max_speed*1000);
		
		//Limit correction parameter to 0~100
		robot_control.LineDiffParam = robot_control.LineDiffParam > 100 ? 100 : robot_control.LineDiffParam;
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
    u8 Usart_Receive;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received // Check if data has been received
    {
        Usart_Receive = USART_ReceiveData(USART3);//Read the data //Read data

		// Data is not processed until 10 seconds after startup
		//Do not process data 10 seconds before powering on
        if(SysVal.Time_count<CONTROL_DELAY)
            return 0;

		//Processing serial control data for robots
        UartxControll_Callback(USART3,Usart_Receive);
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
static float XYZ_Target_Speed_transition(u8 High,u8 Low)
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


//Bluetooth AT command packet capture to prevent instructions from interfering with the robot's normal Bluetooth communication
static u8 AT_Command_Capture(u8 uart_recv)
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
    static char* BlueTooth_Disconnect[3]= {"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};

    //Start the connection
    static char* BlueTooth_Connect[2]= {"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


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
static void _System_Reset_(u8 uart_recv)
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

