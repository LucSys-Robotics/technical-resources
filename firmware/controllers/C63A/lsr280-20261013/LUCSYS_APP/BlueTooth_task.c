/**
 * @file    BlueTooth_task.c
 * @brief Bluetooth APP Data Analysis.
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - The file includes a filter for AT feedback commands of the Bluetooth module to prevent interference from AT commands during connection and disconnection.
 * - The main task is BlueToothControlTask. After receiving instructions on the serial port, a queue will be written and the scheduler will wake this up
 *   tasks for data processing.
 * 
 *
 * @note
 * 
 * 
 */
#include "BlueTooth_task.h"

//HAL Lib Include File
#include "usart.h"

//C Include File
#include <string.h>
#include <math.h>
#include <stdio.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_ServoDrive.h"
#include "bsp_flash.h"
#include "bsp_buzzer.h"
#include "bsp_icm20948.h"


//APP Include File
#include "RobotControl_task.h"
#include "show_task.h"

enum{
	AppKeyNone = 0,
	AppKeyFront = 1,
	AppKeyFR=2,
	AppKeyRight=3,
	AppKeyBR=4,
	AppKeyBack=5,
	AppKeyBL = 6,
	AppKeyLeft=7,
	AppKeyFL = 8,
	APPKeyUp = 0x18,
	APPKeyDown = 0x19,
	APPKeyStop = 0x1A,
};


//Bluetooth AT command packet capture
static uint8_t ATCommandFeedBack_BT04A(uint8_t recv);
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv);

//LUCSYS mobile app
LUCSYS_APPKey_t lucsysApp = { 0 };

//Bluetooth mission
void BlueToothControlTask(void* param)
{
	uint8_t recv = 0;     //Used to receive queue data
	uint8_t lastrecv = 0;
	uint8_t app_page = 1; //Number of pages on the mobile app control page. 0: Gravity page 1: Joystick page 2: Button page
	uint8_t controlKey = 0; //Control value
	
	//Auxiliary parameters of the app reference page
	static uint8_t paramFlag=0,param_i=0,param_j=0,paramReceive[50]={0};
	float paramData=0;
	
	//Write control queue
	extern QueueHandle_t g_xQueueBlueTooth;
	
	//APP control unlocking
	uint8_t AppUnlock = 0;
	
	pBuzzerInterface_t tips = &UserBuzzer;
	while( 1 )
	{
		if( pdPASS == xQueueReceive(g_xQueueBlueTooth,&recv,portMAX_DELAY) ) 
		{
			uint8_t writeflag = 1;
			
			_System_Reset_FromAPP_RTOS(recv);
			
			//APP control unlocking
			if( lastrecv == 'A' && recv=='A' && AppUnlock==0)
				AppUnlock=1;
			
			RobotControlCMDType_t cmd = {
				.cmdsource = APP_CMD,//Annotation control source, APP control
				0,0,0
			};
			
			//Filter Bluetooth AT-command responses
			uint8_t ATFilter = 0;
			ATFilter += ATCommandFeedBack_JDY33(recv);
			ATFilter += ATCommandFeedBack_BT04A(recv);
			if( ATFilter > 0 ) continue; //Filter any Bluetooth filter information exists
				
			//Debug level settings
			RobotControl_SetDebugLevel(recv);
			
			/* Switches the app page with key buttons */
			if( recv == 'K' ) app_page = 2;      //Button page
			else if( recv == 'J' ) app_page = 1; //Joystick page
			else if( recv == 'I' ) app_page = 0; //Gravity-ball screen
			
			//Screen
			lucsysApp.page = app_page;
			
			//Read control information
			uint8_t Tmpcontrolkey = recv-0x40;
			
			//For controlled keys, a second confirmation is required. Only after receiving consecutive keys, control is enabled
			if( Tmpcontrolkey >= AppKeyFront && Tmpcontrolkey <= AppKeyFL )
			{
				if( lastrecv-0x40 == Tmpcontrolkey ) controlKey = Tmpcontrolkey;
			}
			else controlKey = Tmpcontrolkey;
			
			//Save the last received data
			lastrecv = recv;
			
			//Direction key
			lucsysApp.dirkey = controlKey;
			
			float base_vz = PI/4.0f;
				
			switch( controlKey )
			{
				case AppKeyFront:  
					cmd.Vx = RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = 0; 
				break;
				
				case AppKeyFR:     
					cmd.Vx = RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = -base_vz;
				break;
				
				case AppKeyRight:  
					//cmd.Vx = 0; cmd.Vy=0; cmd.Vz = -base_vz;
					if( app_page==2 )
					{
						cmd.Vx = 0; cmd.Vy=0; cmd.Vz = -base_vz;
					}
					else
					{
						cmd.Vx = 0; cmd.Vy=-RobotControlParam.defalutSpeed/1000.0f; cmd.Vz = 0;
					}
						
				break; 
				
				case AppKeyBR:     
					cmd.Vx = -RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = +base_vz;
				break;
				
				case AppKeyBack:  
					cmd.Vx = -RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = 0;
				break;
				
				case AppKeyBL:     
					cmd.Vx = -RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = -base_vz;
				break;
				
				case AppKeyLeft:   
					//cmd.Vx = 0; cmd.Vy=0; cmd.Vz = +base_vz;
					if(app_page==2)
					{
						cmd.Vx = 0; cmd.Vy=0; cmd.Vz = +base_vz;
					}
					else
					{
						cmd.Vx = 0; cmd.Vy=RobotControlParam.defalutSpeed/1000.0f; cmd.Vz = 0;
					}
					
				break;
				
				case AppKeyFL:     
					cmd.Vx = RobotControlParam.defalutSpeed/1000.0f; cmd.Vy=0; cmd.Vz = +base_vz;
				break;
				
				case APPKeyUp:   
					writeflag=0;
					RobotControlParam.defalutSpeed+=100;
					if( RobotControlParam.defalutSpeed>3000 ) RobotControlParam.defalutSpeed=3000;
				break;
				
				case APPKeyDown:
					writeflag=0;					
					RobotControlParam.defalutSpeed-=100;
					if( RobotControlParam.defalutSpeed<=0 ) RobotControlParam.defalutSpeed=0;
				break;
				
				case APPKeyStop:   
					cmd.Vx = 0; cmd.Vy=0; cmd.Vz = 0;
				break;
				
				default: writeflag = 0;  break;
			}
			
			//Write to control queue, control the car
			if( writeflag && AppUnlock )
			{
				if(WriteRobotControlQueue(&cmd,0))
				{
					AppUnlock = 0;//If the write fails, the app will be locked for control
				}
			}
				
			////////////// APP Custom Key Area /////////////////
			if( recv == 'a' ) 
			{
				//IMU zero calibration
				pIMUInterface_t imu = &UserICM20948;
				imu->UpdateZeroPoint_axis();
				imu->UpdateZeroPoint_attitude();
				
				// Set absolute zero
				set_MotorPos_Zero(&Driver1);
				set_MotorPos_Zero(&Driver2);
				
//				TurnA_Zero = RobotControlParam.MotorA.ori_pos+TurnA_Zero;
//				TurnB_Zero = RobotControlParam.MotorB.ori_pos+TurnB_Zero;
//				TurnC_Zero = RobotControlParam.MotorC.ori_pos+TurnC_Zero;
//				TurnD_Zero = RobotControlParam.MotorD.ori_pos+TurnD_Zero;
				
				tips->AddTask(1,200);//Use the buzzer to indicate the operation was successful
			}
			else if( recv == 'b' ) 
			{
				//Automatic recharge function on/off
				RobotControlParam.ChargeMode = !RobotControlParam.ChargeMode;
				tips->AddTask(1,200);
			}
			else if( recv == 'c' )
			{
				//Error requesting clear drive
				RobotControlParam.DriveErrRecovery=1;
				tips->AddTask(1,200);
			}
			else if( recv == 'd' )
			{
				//Enter and exit emergency mode. Press 3 to enter, buzzer sounds prolongedly. Press exit 1, buzzer beeps briefly
				static uint8_t entryCnt = 0;
				if( RobotControlParam.EmergencyMode==1 )
				{
					entryCnt=0;
					RobotControlParam.EmergencyMode=0;
					tips->AddTask(1,200);
				}
				else
				{
					entryCnt++;
					if( entryCnt==3 ) 
					{	//Force all motors to be activated
						for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
						{
							ServoDriver_Enable(g_ServoDriveList[i]);
						}
						RobotControlParam.EmergencyMode=1,tips->AddTask(1,1000);
					}
				}
			}
			else if( recv == 'e' )
			{
				//Turn the ultrasonic obstacle avoidance function on/off
				RobotControlParam.RangerAvoidEN = !RobotControlParam.RangerAvoidEN;
				tips->AddTask(1,200);
			}
			else if( recv == 'f' )
			{	//Turn the IMU assisted straight line function on/off
				RobotControlParam.ImuAssistedFlag = !RobotControlParam.ImuAssistedFlag;
				tips->AddTask(1,200);
			}
			else if( recv == 'g' )
			{	
				oled_page_down();
				tips->AddTask(1,200);
			}
			else if( recv == 'h' )
			{	
				oled_page_up();
				tips->AddTask(1,200);
			}
			//////////////      END          /////////////////
			
			//APP Parameter Page Data Format: {?:?}
			if(recv==0x7B) paramFlag=1;        //Start bit of the APP parameter command // Start bit of APP parameter instruction
			else if(recv==0x7D) paramFlag=2;   //The APP parameter instruction stop bit //APP parameter instruction stop bit
			
			if(paramFlag==1) //Collect data // Collect data
			{
				paramReceive[(param_i%50)]=recv;
				param_i++;
			}
			else if(paramFlag==2) //Analyze the data // Analyze data
			{
				if(paramReceive[3]==0x50)  // {Q:P} Get device parameters
				{
					lucsysApp.reportparam = 1;//Get the parameter request
				}
					
				else if( paramReceive[3]==0x57 ) // {Q:W} sets power-off save parameters
				{
					lucsysApp.saveflash = 1;//Save the Flash parameter request
					//Save the drive's parameters
					ServoDriver_SaveEEPROM(&Driver1);
					ServoDriver_SaveEEPROM(&Driver2);
					ServoDriver_SaveEEPROM(&Driver3);
					
					//Ensure the car stops before writing to flash
					cmd.cmdsource = BootLoader;
					cmd.Vx=0,cmd.Vy=0,cmd.Vz=0;
					WriteRobotControlQueue(&cmd,0);
					vTaskDelay(200);
					
					//Saving parameters for the initial speed of the cart
					taskENTER_CRITICAL(); //Enter critical section; suspend all tasks and interrupts
					uint32_t writeFlashbuf[2] = { RobotControlParam.defalutSpeed,RobotControlParam.LineDiffParam };
					int8_t res = User_Flash_SaveParam((uint32_t*)writeFlashbuf,2); //Write data
					taskEXIT_CRITICAL(); //Write complete, exit critical section
					
					if( res==0 ) tips->AddTask(1,1000);
				}					
				
				else  if(paramReceive[1]!=0x23)  // {0:xxx} {1:xxx} {2:xxx} single-channel value settings
				{
					for(param_j=param_i; param_j>=4; param_j--)
					{
						paramData+=(paramReceive[param_j-1]-48)*pow(10,param_i-param_j);
					}
					switch(paramReceive[1])
					{
						case 0x30: RobotControlParam.defalutSpeed = paramData; tips->AddTask(1,200);break;
						case 0x31: RobotControlParam.LineDiffParam = paramData;tips->AddTask(1,200);break;
						case 0x32: ServoDrive_SetSpeedkp(paramData); tips->AddTask(1,200);break;
						case 0x33: ServoDrive_SetSpeedki(paramData); tips->AddTask(1,200);break;
						case 0x34: ServoDrive_SetCurkp(paramData);   tips->AddTask(1,200);break;
						case 0x35: ServoDrive_SetCurki(paramData);   tips->AddTask(1,200);break;
						case 0x36: break;
						case 0x37: break;
						case 0x38: break;
					}
				}
				else if( paramReceive[1]==0x23 ) //Click "Send All Data" in the app Processing method // {#xxx:xxx:xxx... xxx}
				{
					float num=0;
					uint8_t dataIndex=0;
					float dataArray[9]={0};

					if( param_i<=50 ) //Data is within acceptable range
					{
						paramReceive[param_i]='}'; //Supplement frame ends

						for(uint8_t kk=0; paramReceive[kk]!='}'; kk++)
						{
							if( paramReceive[kk]>='0' && paramReceive[kk]<='9' )
							{
								num = num*10 + ( paramReceive[kk] - '0' );
							}
							else if( paramReceive[kk]==':' )
							{
								dataArray[dataIndex++] = num;
								num = 0;
							}

						}
						//Process the last piece of data
						dataArray[dataIndex] = num;
						
						//Data acquisition
						RobotControlParam.defalutSpeed = dataArray[0];
						RobotControlParam.LineDiffParam = dataArray[1];
						ServoDrive_SetSpeedkp(dataArray[2]);
						ServoDrive_SetSpeedki(dataArray[3]);
						ServoDrive_SetCurkp(dataArray[4]);
						ServoDrive_SetCurki(dataArray[5]);
						
						//The buzzer indicates the operation was successful
						tips->AddTask(1,200);
					}
				}
				
				//Default speed adjustment limit
				if( RobotControlParam.defalutSpeed<0 ) RobotControlParam.defalutSpeed=0;
				if( RobotControlParam.defalutSpeed>3000 ) RobotControlParam.defalutSpeed=3000;
				
				//Limit on the chassis's correction factor
				if( RobotControlParam.LineDiffParam > 100 ) RobotControlParam.LineDiffParam = 100;
				
				//Relevant flag position is cleared
				//Reset relevant marker positions
				paramFlag=0;param_i=0;param_j=0;paramData=0;
				memset(paramReceive, 0, sizeof(uint8_t)*50); //Clear the array to zero
			}	
		}
	}
}


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

