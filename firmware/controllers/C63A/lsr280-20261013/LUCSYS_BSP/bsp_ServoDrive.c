#include "bsp_ServoDrive.h"

//C Include File
#include <stdio.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_can.h"

//APP Include File
#include "robot_select_init.h"

extern QueueHandle_t g_xQueueCANopenCallback;

//Set the debug level
#define DRIVER_DEBUG_LEVEL 0

//Debug log output methods
#if (DRIVER_DEBUG_LEVEL > 0U)
#define  Driver_UsrLog(...)   do { \
                            printf(__VA_ARGS__); \
} while (0)
#else
#define Driver_UsrLog(...) do {} while (0)
#endif

//Driver definitions; each driver controls 2 hub motors
ServoDriverHWType_t Driver1 = {
	.NodeID = 1,
	.initMode = SpeedMode,
	.comm_qua = 100,
	.comm_cnt = 0,
	.comm_Errcnt = 0,
	.time_tick = 0,
	.onlineFlag = 0,
	.errFlag = 0    
};

ServoDriverHWType_t Driver2 = {
	.NodeID = 2,
	.initMode = SpeedMode,
	.comm_qua = 100,
	.comm_cnt = 0,
	.comm_Errcnt = 0,
	.time_tick = 0,
	.onlineFlag = 0,
	.errFlag = 0  
};

ServoDriverHWType_t Driver3 = {
	.NodeID = 3,
	.initMode = SpeedMode,
	.comm_qua = 100,
	.comm_cnt = 0,
	.comm_Errcnt = 0,
	.time_tick = 0,
	.onlineFlag = 0,
	.errFlag = 0  
};

ServoDriverHWType_t Driver4 = {
	.NodeID = 4,
	.initMode = SpeedMode,
	.comm_qua = 100,
	.comm_cnt = 0,
	.comm_Errcnt = 0,
	.time_tick = 0,
	.onlineFlag = 0,
	.errFlag = 0  
};


//List of available drivers
ServoDriverHWType_t* g_ServoDriveList[] = {&Driver1,&Driver2,&Driver3,&Driver4};

//All PID parameters of servo motors
ServoDriverPIDType_t DrivePid = { 0 };

//Reset the drive's status data
static void DriverInfoReset(ServoDriverHWType_t* driver)
{
	driver->comm_cnt=0;
	driver->comm_Errcnt=0;
	driver->comm_qua=0;
	driver->errFlag=0;
	driver->LeftMotorErr=0;
	driver->onlineFlag=0;
	driver->RightMotorErr=0;
	driver->time_tick=0;
	driver->motorEnflag=0;
}

//Compare whether the content of CAN communications meets expectations
static uint8_t compare_canopen_msg(CANmsgType_t* a,CANmsgType_t* b)
{
	if( a->id == b->id )
	{
		for(uint8_t i=0;i<8;i++)
		{
			if( a->buffer[i]!=b->buffer[i] ) return 0;
		}
		
		return 1;
	}
	
	return 0;
}

//Retrieve the content of CANopen-RSDO server-side write data
static void CANopen_getSendList(uint8_t id,uint16_t addr,uint8_t index,uint8_t byteNum,uint32_t writedata,CANmsgType_t* sendlist,CANmsgType_t* fblist)
{
	if( id<1 || id > 127 ) return;
	
	sendlist->id = 0x600 + id;
	uint8_t i=0;
	
	//Set command
	if( byteNum==1 ) sendlist->buffer[0]=0x2F;
	else if( byteNum==2 ) sendlist->buffer[0]=0x2B;
	else if( byteNum==3 ) sendlist->buffer[0]=0x27;
	else sendlist->buffer[0]=0x23;
	
	//Register address, with the lower bits coming first
	sendlist->buffer[1]=addr;
	sendlist->buffer[2]=addr>>8;
	
	//Index value
	sendlist->buffer[3]=index;
	
    //The data to be written is sorted from the lowest to the highest level
    for(i=0;i<4;i++)
    {
        sendlist->buffer[4+i] = ( writedata>>(8*i) )&0xff;
    }
	
	fblist->id = 0x580 + id;
	fblist->buffer[0]=0x60;
	fblist->buffer[1]=sendlist->buffer[1];
	fblist->buffer[2]=sendlist->buffer[2];
	fblist->buffer[3]=sendlist->buffer[3];
	fblist->buffer[4]=0;
	fblist->buffer[5]=0;
	fblist->buffer[6]=0;
	fblist->buffer[7]=0;
}

//Retrieve the content when reading data from the CANopen-RSDO server
static void CANopen_getReadList(uint8_t id,uint16_t addr,uint8_t index,uint8_t byteNum,CANmsgType_t* sendlist)
{
	if( id<1 || id > 127 ) return;
	
	sendlist->id = 0x600 + id;
	uint8_t i=0;
	
	//Read command
	sendlist->buffer[0]=0x40;

	//Register address, with the lower bits coming first
	sendlist->buffer[1]=addr;
	sendlist->buffer[2]=addr>>8;
	
	//Index value
	sendlist->buffer[3]=index;
	
    //No value at the end when reading
    for(i=0;i<4;i++)
    {
        sendlist->buffer[4+i] = 0;
    }
}

//Read data in a FreeRTOS environment
int8_t CANopenRSDO_Read_RTOS(ServoDriverHWType_t* driver,uint16_t addr,uint8_t index,uint8_t byteNum,uint32_t* readdata,uint8_t retrytime)
{
	int8_t res = -1;
	
	//Specify the CAN device to use
	pCANInterface_t candev = &UserCAN2Dev;
	
	//Obtain the data sending list
	CANmsgType_t send,fbreal;
	CANopen_getReadList(driver->NodeID,addr,index,byteNum,&send);
	
	//Send data and wait for processing feedback
	uint8_t times = 1;
	if( retrytime!=0 ) times = retrytime;
	while( times-- )
	{
		driver->comm_cnt++;//Count the number of communications
		
		BaseType_t queue_state;
		
		//Send data
		candev->sendStd(send.id,send.buffer,8);
		
		//Read feedback
		queue_state = xQueueReceive(g_xQueueCANopenCallback,&fbreal,pdMS_TO_TICKS(5));
		
		if( queue_state==pdTRUE )
		{
//			printf("id=%X\r\n",fbreal.id);
//			for(uint8_t i=0;i<8;i++)
//				printf("%02X\t",fbreal.buffer[i]);
//			printf("\r\n");
			
			//Compare feedback results with expected outcomes
			if( fbreal.id == driver->NodeID+0x580 && (fbreal.buffer[0]==0x4F || fbreal.buffer[0]==0x4B || fbreal.buffer[0]==0x47 ||fbreal.buffer[0]==0x43) )
			{
				*readdata = fbreal.buffer[7]<<24 | fbreal.buffer[6]<<16 | fbreal.buffer[5]<<8 | fbreal.buffer[4];
				res = 0;
				break;		
			}
			else
			{
//				printf("read:err\r\n");
				driver->comm_Errcnt++;
				res = -2;//Feedback content error
			}
		}
		else 
		{
			res = -1 , driver->comm_Errcnt++;//Timeout
		}
	}

	//Communication quality statistics: use this value only for long-term averages; do not judge bus quality by the minimum value.
	//  1. It is a ratio, and the denominator comm_cnt changes with how many SDOs are issued in this cycle. Reducing the distribution will make the denominator smaller,
	//    The ratio actually looks worse. Based on this, it was mistakenly judged that "CAN was flooded with set_MotorPos" and raised the threshold to lower the threshold,
	//    After cutting two-thirds of the actual sent, the value actually decreases, proving that attribution is incorrect.
	//  2. The counter is hard-reset every 10 seconds, causing monotonous degradation within the window, and when switching, it suddenly jumps back to 100, with no physical meaning in the waveform.
	//  3. Retry counted in the denominator: one SDO requires 3 retries to succeed, recorded as 3 communications and 2 failures, so the business is actually successful.
	//Actual test reference: LSR280 loaded for 10 minutes, average 98.6%, 98.4% time >=95%, below 70% only 1 frame/7811 frames.
	float err = ((float)driver->comm_Errcnt/(float)driver->comm_cnt)*100;
	driver->comm_qua = 100.0f-err;
	if( xTaskGetTickCount()-driver->time_tick > 10000 )
	{
		driver->comm_Errcnt=0;
		driver->comm_cnt=0;
		driver->time_tick=xTaskGetTickCount();
	}
	
	return res;
}

//Write data in a FreeRTOS environment
int8_t CANopenRSDO_Write_RTOS(ServoDriverHWType_t* driver,uint16_t addr,uint8_t index,uint8_t byteNum,uint32_t writedata,uint8_t retrytime)
{
	int8_t res = -1;
	
	//Specify the CAN device to use
	pCANInterface_t candev = &UserCAN2Dev;
	
	//Obtain the data sending list
	CANmsgType_t send,fbwait,fbreal;
	CANopen_getSendList(driver->NodeID,addr,index,byteNum,writedata,&send,&fbwait);
	
	//Send data and wait for processing feedback
	uint8_t times = 1;
	if( retrytime!=0 ) times = retrytime;
	while( times-- )
	{
		driver->comm_cnt++;//Count the number of communications
		
		BaseType_t queue_state;
		
		//Send data
		candev->sendStd(send.id,send.buffer,8);
		
		//Read feedback
		queue_state = xQueueReceive(g_xQueueCANopenCallback,&fbreal,pdMS_TO_TICKS(5));
		
		//Compare feedback results with expected outcomes
		uint8_t compare_state = compare_canopen_msg(&fbwait,&fbreal);
		
//		printf("id=%X\r\n",fbreal.id);
//		for(uint8_t i=0;i<8;i++)
//			printf("%02X\t",fbreal.buffer[i]);
//		printf("\r\n");
		
		//The actual feedback was as expected, and this communication was successful
		if( queue_state==pdTRUE )
		{
			if( compare_state == 1 )
			{
				res=0;
				break;
			}
			else 
			{
//				printf("write:err\r\n");
				res = -2,driver->comm_Errcnt++;//Feedback content error
			}
		}
		else 
		{
//			printf("write:timeout\r\n");
			res = -1,driver->comm_Errcnt++;//Timeout
		}
	}
	
	//Communication quality statistics: use this value only for long-term averages; do not judge bus quality by the minimum value.
	//  1. It is a ratio, and the denominator comm_cnt changes with how many SDOs are issued in this cycle. Reducing the distribution will make the denominator smaller,
	//    The ratio actually looks worse. Based on this, it was mistakenly judged that "CAN was flooded with set_MotorPos" and raised the threshold to lower the threshold,
	//    After cutting two-thirds of the actual sent, the value actually decreases, proving that attribution is incorrect.
	//  2. The counter is hard-reset every 10 seconds, causing monotonous degradation within the window, and when switching, it suddenly jumps back to 100, with no physical meaning in the waveform.
	//  3. Retry counted in the denominator: one SDO requires 3 retries to succeed, recorded as 3 communications and 2 failures, so the business is actually successful.
	//Actual test reference: LSR280 loaded for 10 minutes, average 98.6%, 98.4% time >=95%, below 70% only 1 frame/7811 frames.
	float err = ((float)driver->comm_Errcnt/(float)driver->comm_cnt)*100;
	driver->comm_qua = 100.0f-err;
	if( xTaskGetTickCount()-driver->time_tick > 10000 )
	{
		driver->comm_Errcnt=0;
		driver->comm_cnt=0;
		driver->time_tick=xTaskGetTickCount();
	}
	
	return res;
}

//Encoder parameter settings
int8_t set_encoder_accuracy(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;
	
	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x200E,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200E,1,2,param,10);
	
	Driver_UsrLog("Lencoder:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x200E,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200E,2,2,param,10);
	
	Driver_UsrLog("Rencoder:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_HallOffset(ServoDriverHWType_t* driver,short param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;
	
	//Clamp
	if(param<-360) param=-360;
	if(param> 360) param=360;
	
	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2011,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2011,1,2,param,10);
	
	Driver_UsrLog("L_Hall:%d\r\n",(short)tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2011,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2011,2,2,param,10);
	
	Driver_UsrLog("R_Hall:%d\r\n",(short)tmpRead);
	
	return comm_state;
}

int8_t set_poles(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param<4) param=4;
	if(param>64) param=64;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x200C,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200C,1,2,param,10);
	
	Driver_UsrLog("L_poles:%d\r\n",(short)tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x200C,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200C,2,2,param,10);
	
	Driver_UsrLog("R_poles:%d\r\n",(short)tmpRead);
	
	return comm_state;
}


int8_t set_rated_current(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>150) param=150;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2014,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2014,1,2,param,10);
	
	Driver_UsrLog("L_cur:%d\r\n",(short)tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2014,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2014,2,2,param,10);
	
	Driver_UsrLog("R_cur:%d\r\n",(short)tmpRead);
	
	return comm_state;
}


int8_t set_max_current(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>300) param=300;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2015,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2015,1,2,param,10);
	
	Driver_UsrLog("L_MAXcur:%d\r\n",(short)tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2015,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2015,2,2,param,10);
	
	Driver_UsrLog("R_MAXcur:%d\r\n",(short)tmpRead);
	
	return comm_state;
}

int8_t set_max_rpm(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param<1)    param=1;
	if(param>3000) param=3000;

	comm_state = CANopenRSDO_Read_RTOS(driver,0x2008,0,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2008,0,2,param,10);
	
	Driver_UsrLog("MAXRpm:%d\r\n",(short)tmpRead);
	
	return comm_state;
}

int8_t set_overload_factor(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>300) param=300;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2012,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2012,1,2,param,10);
	
	Driver_UsrLog("L_OverLoad:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2012,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2012,2,2,param,10);
	
	Driver_UsrLog("R_OverLoad:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_overload_time(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>6553) param=6553;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2016,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2016,1,2,param,10);
	
	Driver_UsrLog("L_OverLoadTime:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2016,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2016,2,2,param,10);
	
	Driver_UsrLog("R_OverLoadTime:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_encoder_ErrorAlarm(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>6553) param=6553;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2017,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2017,1,2,param,10);
	
	Driver_UsrLog("L_Encoderdiff:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2017,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2017,2,2,param,10);
	
	Driver_UsrLog("R_Encoderdiff:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_maxTemp(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>1200) param=1200;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2013,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2013,1,2,param,10);
	
	Driver_UsrLog("L_MaxTemp:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2013,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2013,2,2,param,10);
	
	Driver_UsrLog("R_MaxTemp:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_pos_kp(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2020,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2020,1,2,param,10);
	
	Driver_UsrLog("L_PosKp:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2020,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2020,2,2,param,10);
	
	Driver_UsrLog("R_PosKp:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_pos_kf(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2021,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2021,1,2,param,10);
	
	Driver_UsrLog("L_PosKf:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2021,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2021,2,2,param,10);
	
	Driver_UsrLog("R_PosKf:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t get_speed_kp(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Read PID parameters, referencing only the data from the single-sided wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201D,1,2,&tmpRead,10);
	
	//TODO: Recovery
	if( driver->initMode == SpeedMode )
		DrivePid.SpeedKp = tmpRead;
	
	//TODO:Del adjusts the steering motor PID parameters
//	if( driver->initMode == PosMode )
//		DrivePid.SpeedKp = tmpRead;

	Driver_UsrLog("speedKp:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t ServoDrive_SetSpeedkp(uint16_t param)
{
	int8_t comm_state = 0;

	//Clamp
	if(param>30000) param=30000;

	//Use the configured driver count for the chassis (previously hardcoded to 3; LSR280 has 4 drivers, so Driver4 could not obtain PID parameters)
	
	//TODO: Recovery
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		if( g_ServoDriveList[i]->onlineFlag==1 && g_ServoDriveList[i]->initMode == SpeedMode ) //Temporarily disable the steering motor's parameter adjustment
		{
			//Set parameters
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201D,1,2,param,10);
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201D,2,2,param,10);
		}
	}

//	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
//	{
//		if( g_ServoDriveList[i]->onlineFlag==1 && g_ServoDriveList[i]->initMode == PosMode) // Temporarily disable parameter adjustment for the steering motor
//		{
//			//Set parameters
//			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201D,1,2,param,10);
//			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201D,2,2,param,10);
//		}
//	}
	
	//Only update parameters when all driver writes succeed
	if( comm_state==0 ) DrivePid.SpeedKp = param;
	
	return comm_state;
}

int8_t get_speed_ki(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Read PID parameters, referencing only the data from the single-sided wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201E,1,2,&tmpRead,10);
	
	//TODO: Recovery
	if( driver->initMode == SpeedMode )
		DrivePid.SpeedKi = tmpRead;
	
	//TODO:Del adjusts the steering motor PID parameters
//	if( driver->initMode == PosMode )
//		DrivePid.SpeedKi = tmpRead;
	
	Driver_UsrLog("speedKi:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t ServoDrive_SetSpeedki(uint16_t param)
{
	int8_t comm_state = 0;

	//Clamp
	if(param>30000) param=30000;

	//Use the configured driver count for the chassis (previously hardcoded to 3; LSR280 has 4 drivers, so Driver4 could not obtain PID parameters)

	//TODO: Recovery
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		if( g_ServoDriveList[i]->onlineFlag==1 && g_ServoDriveList[i]->initMode == SpeedMode ) //Temporarily disable the steering motor's parameter adjustment
		{
			//Set parameters
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201E,1,2,param,10);
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201E,2,2,param,10);
		}
	}
	
	//TODO: Del adjusts the steering motor
//	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
//	{
//		if( g_ServoDriveList[i]->onlineFlag==1 && g_ServoDriveList[i]->initMode == PosMode) // Temporarily disable parameter adjustment for the steering motor
//		{
//			//Set parameters
//			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201E,1,2,param,10);
//			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201E,2,2,param,10);
//		}
//	}

	//Only update parameters when all driver writes succeed
	if( comm_state==0 ) DrivePid.SpeedKi = param;
	
	return comm_state;
}


int8_t set_speed_kf(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201F,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201F,1,2,param,10);
	
	Driver_UsrLog("L_speedKf:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201F,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201F,2,2,param,10);
	
	Driver_UsrLog("R_speedKf:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_speed_smooth(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2018,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2018,1,2,param,10);
	
	Driver_UsrLog("L_speed_smooth:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2018,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x2018,2,2,param,10);
	
	Driver_UsrLog("R_speed_smooth:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_smooth_kf(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201B,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201B,1,2,param,10);
	
	Driver_UsrLog("L_smoothKf:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201B,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201B,2,2,param,10);
	
	Driver_UsrLog("R_smoothKf:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_tor_smooth(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201C,1,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201C,1,2,param,10);
	
	Driver_UsrLog("L_torKf:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201C,2,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x201C,2,2,param,10);
	
	Driver_UsrLog("R_torKf:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t get_curloop_kp(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Read PID parameters, referencing only the data from the single-sided wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2019,1,2,&tmpRead,10);
	
	//TODO: Recovery
//	if( driver->initMode == SpeedMode )
//		DrivePid.CurKp = tmpRead;
	
	//TODO:Del adjusts the steering motor PID parameters
	if( driver->initMode == PosMode )
		DrivePid.CurKp = tmpRead;
	
	Driver_UsrLog("CurKp:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t ServoDrive_SetCurkp(uint16_t param)
{
	int8_t comm_state = 0;

	//Clamp
	if(param>30000) param=30000;

	//Use the configured driver count for the chassis (previously hardcoded to 3; LSR280 has 4 drivers, so Driver4 could not obtain PID parameters)

	//TODO: Recovery
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		if( g_ServoDriveList[i]->onlineFlag==1 && g_ServoDriveList[i]->initMode == SpeedMode ) //Temporarily disable the steering motor's parameter adjustment
		{
			//Set parameters
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x2019,1,2,param,10);
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x2019,2,2,param,10);
		}
	}

//	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
//	{
//		if( g_ServoDriveList[i]->onlineFlag==1 && g_ServoDriveList[i]->initMode == PosMode) // Temporarily disable parameter adjustment for the steering motor
//		{
//			//Set parameters
//			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x2019,1,2,param,10);
//			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x2019,2,2,param,10);
//		}
//	}

	//Only update parameters when all driver writes succeed
	if( comm_state==0 ) DrivePid.CurKp = param;

	
	return comm_state;
}


int8_t get_curloop_ki(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Clamp
	if(param>30000) param=30000;

	//Read PID parameters, referencing only the data from the single-sided wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x201A,1,2,&tmpRead,10);
	
	//TODO: Recovery
//	if( driver->initMode == SpeedMode )
//		DrivePid.CurKi = tmpRead;
	
	//TODO:Del adjusts the steering motor PID parameters
	if( driver->initMode == PosMode )
		DrivePid.CurKi = tmpRead;
	
	Driver_UsrLog("CurKi:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t ServoDrive_SetCurki(uint16_t param)
{
	int8_t comm_state = 0;

	//Clamp
	if(param>30000) param=30000;

	//Use the configured driver count for the chassis (previously hardcoded to 3; LSR280 has 4 drivers, so Driver4 could not obtain PID parameters)
	
	//TODO: Recovery
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		if( g_ServoDriveList[i]->onlineFlag==1 && g_ServoDriveList[i]->initMode == SpeedMode ) //Temporarily disable the steering motor's parameter adjustment
		{
			//Set parameters
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201A,1,2,param,10);
			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201A,2,2,param,10);
		}
	}

//	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
//	{
//		if( g_ServoDriveList[i]->onlineFlag==1 && g_ServoDriveList[i]->initMode == PosMode) // Temporarily disable parameter adjustment for the steering motor
//		{
//			//Set parameters
//			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201A,1,2,param,10);
//			comm_state += CANopenRSDO_Write_RTOS(g_ServoDriveList[i],0x201A,2,2,param,10);
//		}
//	}

	//Only update parameters when all driver writes succeed
	if( comm_state==0 ) DrivePid.CurKi = param;
	
	return comm_state;
}

int8_t set_SynCon_state(ServoDriverHWType_t* driver,uint8_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	if(param!=0) param=1;

	comm_state = CANopenRSDO_Read_RTOS(driver,0x200F,0,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x200F,0,2,param,10);
	
	Driver_UsrLog("SynMode:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_acctime(ServoDriverHWType_t* driver,uint32_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	if(param>32767) param=32767;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6083,1,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6083,1,4,param,10);
	
	Driver_UsrLog("L_AccTime:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6083,2,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6083,2,4,param,10);
	
	Driver_UsrLog("R_AccTime:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_dectime(ServoDriverHWType_t* driver,uint32_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	if(param>32767) param=32767;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6084,1,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6084,1,4,param,10);
	
	Driver_UsrLog("L_DecTime:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6084,2,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6084,2,4,param,10);
	
	Driver_UsrLog("R_DecTime:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_stoptime(ServoDriverHWType_t* driver,uint32_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	if(param>32767) param=32767;

	//Revolver
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6085,1,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6085,1,4,param,10);
	
	Driver_UsrLog("L_StopTime:%d\r\n",tmpRead);
	
	//Right wheel
	comm_state = CANopenRSDO_Read_RTOS(driver,0x6085,2,4,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x6085,2,4,param,10);
	
	Driver_UsrLog("R_StopTime:%d\r\n",tmpRead);
	
	return comm_state;
}

int8_t set_hearbeatTime(ServoDriverHWType_t* driver,uint16_t param)
{
	int8_t comm_state = 0;
	uint32_t tmpRead = 0;

	//Each node is set with regular intervals in heartbeat duration
	param = param + driver->NodeID*10;

	comm_state = CANopenRSDO_Read_RTOS(driver,0x1017,0,2,&tmpRead,10);
	if( tmpRead!=param )
		comm_state = CANopenRSDO_Write_RTOS(driver,0x1017,0,2,param,10);
	
	Driver_UsrLog("HearTime:%d\r\n",tmpRead);
	
	return comm_state;
}

//Set the motor control mode
/*
	1: Position mode
	3: Speed mode
	4: Torque mode
*/
int8_t set_ControlMode(ServoDriverHWType_t* driver,uint8_t param)
{
	int8_t comm_state = 0;
	if(param>4) param=0;
	
	comm_state = CANopenRSDO_Write_RTOS(driver,0x6060,0,1,param,10);
	
	return comm_state;
}

int8_t set_MotorRPM(ServoDriverHWType_t* driver,short Lm,short Rm)
{
	if( driver->onlineFlag==0 ) return 1;
	
	int8_t comm_state = 0;

	uint32_t TmpWrite = (uint32_t)Rm<<16|(uint16_t)Lm;
	
	comm_state = CANopenRSDO_Write_RTOS(driver,0x60FF,3,4,TmpWrite,5);
	
	return comm_state;
}


//Absolute target setting
int8_t set_MotorPos(ServoDriverHWType_t* driver,short Lm,uint32_t Lrpm,short Rm,uint32_t Rrpm)
{
	if( driver->onlineFlag==0 ) return 1;
	
	int8_t comm_state = 0;

	//Pause the current movement
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,0x02,10);
	vTaskDelay(2);//Must delay waiting for the drive's state word; otherwise, replication is unstable
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,0x0F,10);
	
	//Set left and right target values
	comm_state += CANopenRSDO_Write_RTOS(driver,0x607A,1,4,Lm,5);
	comm_state += CANopenRSDO_Write_RTOS(driver,0x607A,2,4,Rm,5);
	
	//Set the left and right motor speeds
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6081,1,4,Lrpm,5);
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6081,2,4,Rrpm,5);
	
	//Enable absolute position control
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,0x0F,5);
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,0x1F,5);
	return comm_state;
}

int8_t set_MotorPos_Relative(ServoDriverHWType_t* driver,short Lm,uint32_t Lrpm,short Rm,uint32_t Rrpm)
{
	if( driver->onlineFlag==0 ) return 1;
	
	int8_t comm_state = 0;
	
	//Set left and right target values
	comm_state += CANopenRSDO_Write_RTOS(driver,0x607A,1,4,Lm,5);
	comm_state += CANopenRSDO_Write_RTOS(driver,0x607A,2,4,Rm,5);
	
	//Set the left and right motor speeds
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6081,1,4,Lrpm,5);
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6081,2,4,Rrpm,5);
	
	//Enable absolute position control
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,0x4F,5);
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,0x5F,5);
	
	return comm_state;
}

int8_t set_MotorPos_Zero(ServoDriverHWType_t* driver)
{
	if( driver->onlineFlag==0 ) return 1;
	
	int8_t comm_state = 0;
	
	comm_state += CANopenRSDO_Write_RTOS(driver,0x2006,0,2,3,5);

	return comm_state;
}

int8_t set_MotorPosStop(ServoDriverHWType_t* driver) 
{
	if( driver->onlineFlag==0 ) return 1;
	
	int8_t comm_state = 0;
	
	//Set the left and right motor speeds to 0
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6084,1,4,0,5);
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6084,2,4,0,5);
	
	return comm_state;
}

int8_t ServoDriver_Enable(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;
	
	//Enable step: Write 06, 07, 0F to the control word
	uint32_t param = 0x06;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	param = 0x07;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	param = 0x0F;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	if( driver->initMode == SpeedMode )
	{
		//Reset rotational speed to zero
		comm_state += set_MotorRPM(driver,0,0);
	}
	else if( driver->initMode == PosMode )
	{
		comm_state += set_MotorPos(driver,0,T280_WheelTurnSpeed,0,T280_WheelTurnSpeed);
	}


	return comm_state;
}

int8_t ServoDriver_Disable(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;
	
	uint32_t param = 0;
	comm_state = CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	return comm_state;
}

//Release motor
int8_t ServoDriver_ReleaseMotor(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;
	
	//The motor is allowed to be released only when it is in the enabled state and the driver has no errors
	if( driver->motorEnflag==1 && driver->errFlag==0 )
	{
		comm_state += ServoDriver_Disable(driver);
		comm_state += ServoDriver_Enable(driver);
	}

	return comm_state;
}

//Set parking mode
int8_t ServoDriver_ParkingMode(ServoDriverHWType_t* driver,uint8_t set)
{
	int8_t comm_state = 0;
	
	if(set==1)
	{
		//Release motor
		comm_state += ServoDriver_ReleaseMotor(driver);
	}

	//Set parking mode (velocity mode only)
	if( driver->initMode==SpeedMode )
		comm_state += CANopenRSDO_Write_RTOS(driver,0x2026,4,2,set,5);
	
	return comm_state;
}


//Clear driver errors
int8_t ServoDriver_ClearError(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;

	//bit7 reset failure from 0->1
	uint32_t param = 0x00;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	param = 0x80;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x6040,0,2,param,10);
	
	return comm_state;
}

//TPDO0 mapping configuration
int8_t ServoDriver_MappingTPDO0_Speed(ServoDriverHWType_t* driver,uint32_t COBID)
{
	int8_t comm_state = 0;

	uint32_t param = 0;
	
	//Clear the mapping on TPDO0
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,0,1,param,10);
	
	//Mapping 1, content is 0x606C address, index 01, byte count 0x20(32)
	param = 0x606C0120;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,1,4,param,10);
	
	//Mapping 2, content 0x606C address, index 02, byte count 0x20(32)
	param = 0x606C0220;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,2,4,param,10);
	
	//Set the frame ID for TPDO reporting
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1800,1,4,COBID,10);
	
	//Set up 254 events; data changes are reported immediately
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1800,2,1,254,10);
	
	//Set a ban time; if the data interval between two data does not exceed this time, it will not trigger
	param = 100;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1800,3,2,param,10);
	
	//Enable two TPDO0 mappings
	param = 2;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,0,1,param,10);
	
	return comm_state;
}

//TPDO0 mapping configuration
int8_t ServoDriver_MappingTPDO0_Pos(ServoDriverHWType_t* driver,uint32_t COBID)
{
	int8_t comm_state = 0;

	uint32_t param = 0;
	
	//Clear the mapping on TPDO0
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,0,1,param,10);
	
	//Mapping 1, content is 0x606C address, index 01, byte count 0x20(32)
	param = 0x60640120;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,1,4,param,10);
	
	//Mapping 2, content 0x606C address, index 02, byte count 0x20(32)
	param = 0x60640220;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,2,4,param,10);
	
	//Set the frame ID for TPDO reporting
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1800,1,4,COBID,10);
	
	//Set up 254 events; data changes are reported immediately
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1800,2,1,254,10);
	
	//Set a ban time; if the data interval between two data does not exceed this time, it will not trigger
	param = 100;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1800,3,2,param,10);
	
	//Enable two TPDO0 mappings
	param = 2;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A00,0,1,param,10);
	
	return comm_state;
}

// TPDO1: two motor currents followed by two motor temperatures.
// Each mapped value is int16_t, so the complete PDO occupies 8 bytes.
static int8_t ServoDriver_MappingTPDO1_CurrentTemperature(ServoDriverHWType_t* driver,uint32_t COBID)
{
	int8_t comm_state = 0;
	uint32_t param = 0;

	// Disable TPDO1 mapping before changing its entries.
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A01,0,1,param,10);

	param = 0x60770110; // Left motor current, int16_t
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A01,1,4,param,10);

	param = 0x60770210; // Right motor current, int16_t
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A01,2,4,param,10);

	param = 0x20320110; // Left motor temperature, int16_t, 0.1 degC
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A01,3,4,param,10);

	param = 0x20320210; // Right motor temperature, int16_t, 0.1 degC
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A01,4,4,param,10);

	comm_state += CANopenRSDO_Write_RTOS(driver,0x1801,1,4,COBID,10);
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1801,2,1,255,10);

	// 2000 * 100 us = 200 ms minimum interval between TPDO1 frames.
	param = 2000;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1801,3,2,param,10);

	param = 4;
	comm_state += CANopenRSDO_Write_RTOS(driver,0x1A01,0,1,param,10);

	return comm_state;
}

//Save parameters
int8_t ServoDriver_SaveEEPROM(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;

	uint32_t param = 1;

	if( driver->onlineFlag==1 )
		comm_state += CANopenRSDO_Write_RTOS(driver,0x2010,0,2,param,10);
	
	return comm_state;
}

//Get driver self-test error information
int8_t ServoDriver_CheckError(ServoDriverHWType_t* driver)
{
	int8_t comm_state = 0;
	uint32_t readtmp;
	uint16_t LeftMotor;
	uint16_t RightMotor;
	
	//Self-test is performed only when the driver is online
	if( driver->onlineFlag == 1 )
	{
		comm_state = CANopenRSDO_Read_RTOS(driver,0x6041,0,4,&readtmp,10);
		if( comm_state==0 )
		{
			RightMotor = readtmp>>16;
			LeftMotor = readtmp;
			
			//Check the motor's enabled or released status
			if( (LeftMotor&0x07)==7 && (RightMotor&0x07)==7 )
			{
				driver->motorEnflag = 1;
			}
			else driver->motorEnflag = 0;

			//Check the error codes on the motor
			if( 1 == ((RightMotor>>7)&0x01) || 1==((LeftMotor>>7)&0x01) ) 
			{
				driver->errFlag = 1;
				CANopenRSDO_Read_RTOS(driver,0x603F,0,4,&readtmp,10);
				driver->LeftMotorErr = readtmp>>16;
				driver->RightMotorErr = readtmp;
			}
			else 
			{
				driver->errFlag = 0;
				driver->LeftMotorErr = 0;
				driver->RightMotorErr = 0;
			}
		}
	}
	
	//Driver offline; reset its information
	else
	{
		DriverInfoReset(driver);
	}
	
	return comm_state;

}

//Based on the feedback NodeID, activate the corresponding online flag for heartbeat detection
void ServoDriver_OnLineCheck(uint8_t nodeid)
{
	for(uint8_t i=0;i<RobotHardWareParam.driveCounts;i++)
	{
		if( nodeid == g_ServoDriveList[i]->NodeID )
		{
			g_ServoDriveList[i]->online_tick++;
			if( g_ServoDriveList[i]->onlineFlag==0 )
			{
				g_ServoDriveList[i]->onlineFlag = 1;
				g_ServoDriveList[i]->comm_Errcnt=0;
				g_ServoDriveList[i]->comm_qua=100;
			}
		}		
	}
}

//Servo motor initialization
uint8_t ServoDriver_Init(ServoDriverHWType_t* driver)
{
	int8_t comm_state;
	uint32_t readdata = 0;
	
	//Enter pre-operational state, stop mapping and configure the driver (disable previously configured mappings to avoid interfering with initialization)
	pCANInterface_t candev = &UserCAN2Dev;
	uint8_t PreOp[8]= {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	candev->sendStd(0x00,PreOp,8);
	
	//Read the drive software version number and check if the driver is online
	comm_state = CANopenRSDO_Read_RTOS(driver,0x2031,0,2,&readdata,10);
	if( comm_state==0 )
	{
		driver->SoftWareVer = readdata;
		driver->onlineFlag = 1;
	}
	else 
	{
		driver->onlineFlag=0;
		return 0;
	}
	
	//Initialize driver parameters
	#if 1
	Driver_UsrLog("======= driver param init %d ========\r\n",driver->NodeID);
	
	//Secondary confirmation of pre-operation
	candev->sendStd(0x00,PreOp,8);
	
	//Motor parameters, which perform different initializations depending on the wheel model
	if( driver->motorType == SingleAxis_8inch )
	{
		comm_state += set_encoder_accuracy(driver,4096); //Set the number of encoder lines
		comm_state += set_HallOffset(driver,0);          //Set the angle offset
		comm_state += set_poles(driver,15);              //Set the number of motor pole pairs
	}
	else if( driver->motorType  == DoubleAxis_8inch )
	{
		comm_state += set_encoder_accuracy(driver,1024);
		comm_state += set_HallOffset(driver,0);         
		comm_state += set_poles(driver,15);             
	}
	else if( driver->motorType  == SingleAxis_5inch )
	{
		comm_state += set_encoder_accuracy(driver,4096);
		comm_state += set_HallOffset(driver,240);       
		comm_state += set_poles(driver,10);             
	}
	else if( driver->motorType  == DoubleAxis_5inch )
	{
		comm_state += set_encoder_accuracy(driver,1024);
		comm_state += set_HallOffset(driver,240);       
		comm_state += set_poles(driver,10);             
	}
	else if( driver->motorType == SteeringMotor )
	{
		//Hollow motor
		comm_state += set_encoder_accuracy(driver,0);
		comm_state += set_HallOffset(driver,270);       
		comm_state += set_poles(driver,19);   
	}
	
	comm_state += set_rated_current(driver,150);     //Set the rated current
	comm_state += set_max_current(driver,300);       //Set the maximum current
	comm_state += set_max_rpm(driver,1000);          //Set the rated speed
	comm_state += set_overload_factor(driver,300);   //Overload coefficient
	comm_state += set_overload_time(driver,6553);    //Overload time
	comm_state += set_encoder_ErrorAlarm(driver,6553);//Deviation alarm value
	comm_state += set_maxTemp(driver,1200);           //Temperature protection threshold
	
	//PID Parameter Configuration start /////////////
	
	//User_Speed_Kp¡¢User_Speed_Ki¡¢User_Cur_Kp¡¢User_Cur_Ki
	//Check whether user-defined parameters have been stored; if not, execute configuration
	//8-inch single-axle wheels, drive version 25473, speed loop kp_420_ki_200, current loop kp_220_ki_180
	
	//PID Parameter Configuration END ///////////////
	
	
	//Position ring parameters
//	comm_state += set_pos_kp(driver,200);
//	comm_state += set_pos_kf(driver,200);
	
	//Speed-loop parameters
	comm_state += get_speed_kp(driver,420);
	comm_state += get_speed_ki(driver,200);
//	comm_state += set_speed_kf(driver,800);
	
	//Output smoothing coefficient
//	comm_state += set_speed_smooth(driver,50);
//	comm_state += set_smooth_kf(driver,100);
//	comm_state += set_tor_smooth(driver,100);
	
	//Current loop parameters
	comm_state += get_curloop_kp(driver,300);
	comm_state += get_curloop_ki(driver,200);
	
	comm_state += set_SynCon_state(driver,1);//Set synchronization controls
	comm_state += set_acctime(driver,5);    //Set acceleration time
	comm_state += set_dectime(driver,5);    //Set the deceleration time
	comm_state += set_stoptime(driver,10);   //Set emergency-stop deceleration time
	
	//Initialization in velocity mode
	if( driver->initMode == SpeedMode )
	{
		comm_state += set_ControlMode(driver,3); //Set the speed control mode
		comm_state += set_MotorRPM(driver,0,0);  //Set the speed to 0
		comm_state += ServoDriver_MappingTPDO0_Speed(driver,(0x190+driver->NodeID-1));//TPDO0 maps speed
		//TPDO1 (current/temperature) only maps to hub motors. The hollow motor used for steering does not have current or temperature data collection hardware,
		//Applying this mapping to PosMode drive 1/2 will only cause all SDOs to time out (about 0.45 seconds boot delay per drive).
		//and no data can be obtained. Confirmed as hardware limitations, do not add more.
		if( RobotHardWareParam.CarType == T280 )
			comm_state += ServoDriver_MappingTPDO1_CurrentTemperature(driver,(0x290+driver->NodeID-1));
		comm_state += ServoDriver_Enable(driver);            //Enable motor
	}
	
	//Initialization in position mode
	else if( driver->initMode == PosMode )
	{
		comm_state += set_ControlMode(driver,1); //Set position control
		comm_state += ServoDriver_Enable(driver);            //Enable motor
		comm_state += ServoDriver_MappingTPDO0_Pos(driver,(0x190+driver->NodeID-1));//TPDO0 mapping position
		comm_state += set_MotorPos(driver,0,T280_WheelTurnSpeed,0,T280_WheelTurnSpeed);  //Set position to 0
	}
	
	comm_state += set_hearbeatTime(driver,5000);         //Set the heartbeat time
	comm_state += ServoDriver_SaveEEPROM(driver);//Save the above parameters to EEPROM
	
	//Configuration complete; start NMT and enable mapping for all TPDOs and RPDOs
	uint8_t start[8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	candev->sendStd(0x00,start,8);
	
	Driver_UsrLog("comm_state:%d\r\n",comm_state);
	Driver_UsrLog("Qua:%.3f \r\n",driver->comm_qua);
	#else //Communication test; dummy data
	comm_state += set_encoder_accuracy(driver,10); //Set the number of encoder lines
	comm_state += set_HallOffset(driver,11);          //Set the angle offset
	comm_state += set_poles(driver,12);              //Set the number of motor pole pairs
	comm_state += set_rated_current(driver,13);     //Set the rated current
	comm_state += set_max_current(driver,14);       //Set the maximum current
	comm_state += set_max_rpm(driver,15);          //Set the rated speed
	comm_state += set_overload_factor(driver,16);   //Overload coefficient
	comm_state += set_overload_time(driver,17);    //Overload time
	comm_state += set_encoder_ErrorAlarm(driver,18);//Deviation alarm value
	comm_state += set_maxTemp(driver,19);           //Temperature protection threshold
	
	//Position ring
	comm_state += set_pos_kp(driver,20);
	comm_state += set_pos_kf(driver,21);
	
	//Speed loop
	comm_state += set_speed_kp(driver,22);
	comm_state += set_speed_ki(driver,23);
	comm_state += set_speed_kf(driver,24);
	
	//Output smoothing coefficient
	comm_state += set_speed_smooth(driver,25);
	comm_state += set_smooth_kf(driver,26);
	comm_state += set_tor_smooth(driver,27);
	
	//Current Loop
	comm_state += set_curloop_kp(driver,28);
	comm_state += set_curloop_ki(driver,29);
	#endif
	Driver_UsrLog("======= driver param init End ========\r\n\r\n");
	return comm_state;
}


//Servo-driven heartbeat detection to determine whether the drive is online
void HeartbeatTask(void* param)
{
	uint8_t driveCnt = sizeof(g_ServoDriveList)/sizeof(g_ServoDriveList[0]);
	while(1)
	{
		//Heartbeat message settings online_tick increment
		for(uint8_t i=0;i<driveCnt;i++)
		{
			g_ServoDriveList[i]->online_tick=0;
		}
		vTaskDelay(5000);
		
		//If there is no heartbeat message, the driver has been disconnected
		for(uint8_t i=0;i<driveCnt;i++)
		{
			if( g_ServoDriveList[i]->online_tick==0 )
				g_ServoDriveList[i]->onlineFlag=0;
		}
	}
}



