#include "HubDriverCan.h"

int BrushlessEnable=1;
/**************************************************************************
Function: 
Input   : none
Output  : none
Function: Hub speed mode initialization: 30ms
Entry parameters: None
Return value: none
**************************************************************************/
int acc_time=30; //Tests show acceleration and deceleration times as low as 3ms; if lower, the motor will stop responding
void hub_CAN_velocity_init(void)
{
	//CAN_setBaudrate(1000);
	CAN_Asyn_ctrl_set(0);
  CAN_V_mode_set();
	CAN_acc_time(acc_time);
//	CAN_acc_time_left(0);
//	CAN_dec_time_left(0);
//	CAN_acc_time_right(0);
//	CAN_dec_time_right(0);
	
  CAN_Enable();
}

void hub_CAN_position_init(void)
{
  CAN_P_mode_set();
	CAN_acc_time(acc_time);
  CAN_Enable();
  CAN_SetMaxRpm_left_Right(60, 60);
}

void hub_CAN_torque_init(void)
{
	CAN_Asyn_ctrl_set(1);
  CAN_T_mode_set();
	CAN_Left_Torque_mAs(100);
	CAN_Right_Torque_mAs(100);
  CAN_Enable();
}

void CAN_setBaudrate(int Baudrate)
{
	u8 baudrate[8]= {0x2f, 0x0b, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	switch(Baudrate)
  {
	 case 1000: baudrate[4]=0; break;
	 case 500:  baudrate[4]=1; break;
	 case 250:  baudrate[4]=2; break;
	 case 125:  baudrate[4]=3; break;
	 case 100:  baudrate[4]=4; break;
	 case 50:   baudrate[4]=5; break;
	 case 25:   baudrate[4]=6; break;
  }
	
	CAN1_Send_Num(0x601,baudrate);
	delay_xms(500);
	CAN_Write_EEPROM();
	delay_xms(500);
}

void CAN_Write_EEPROM(void)
{
	u8 write_EEPROM[8]= {0x2f, 0x10, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
	
	CAN1_Send_Num(0x601,write_EEPROM);
}

void CAN_Stop(void)
{
	u8 Stop[8]= {0x2b, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	CAN1_Send_Num(0x601,Stop);
}

void CAN_Enable(void)
{
	u8 enable1[8]= {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
	u8 enable2[8]= {0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
	u8 enable3[8]= {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
	
	delay_xms(20);
	CAN1_Send_Num(0x601,enable1);
	delay_xms(20);
  CAN1_Send_Num(0x601,enable2);
	delay_xms(20);
	CAN1_Send_Num(0x601,enable3);
}

void CAN_Asyn_ctrl_set(int asyn)
{
	u8 asyn_ctrl_set[8]= {0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	if(asyn==0)asyn_ctrl_set[4]=0x01;
	CAN1_Send_Num(0x601,asyn_ctrl_set);
}

void CAN_V_mode_set(void)
{
	u8 v_mode_set[8]= {0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
	
	CAN1_Send_Num(0x601,v_mode_set);
}

void CAN_P_mode_set(void)
{
	u8 p_mode_set[8]= {0x2f, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
	
	delay_ms(20);
	CAN1_Send_Num(0x601,p_mode_set);
}

void CAN_T_mode_set(void)
{
	u8 t_mode_set[8]= {0x2f, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00};
	
	delay_ms(20);
	CAN1_Send_Num(0x601,t_mode_set);
}

void CAN_acc_time(int time)
{
	u8 left_acc_time [8]= {0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	u8 right_acc_time[8]= {0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};
	u8 left_dec_time [8]= {0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};
	u8 right_dec_time[8]= {0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	
	if(time>0)
		left_acc_time [5]=time>>8,
	  left_acc_time [4]=time;
	  right_acc_time[5]=time>>8,
	  right_acc_time[4]=time;
		left_dec_time [5]=time>>8,
	  left_dec_time [4]=time;
		right_dec_time[5]=time>>8,
	  right_dec_time[4]=time;
	
	delay_xms(20);
	CAN1_Send_Num(0x601,left_acc_time);
	delay_xms(20);
	CAN1_Send_Num(0x601,right_acc_time);
	delay_xms(20);
	CAN1_Send_Num(0x601,left_dec_time);
	delay_xms(20);
	CAN1_Send_Num(0x601,right_dec_time);
}

void CAN_acc_time_left(int time)
{
	u8 left_acc_time [8]= {0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	
	if(time<0) time=0;
	if(time>0)
		left_acc_time [5]=time>>8,
	  left_acc_time [4]=time;

	delay_xms(20);
	CAN1_Send_Num(0x601,left_acc_time);
}

void CAN_dec_time_left(int time)
{
	u8 left_dec_time [8]= {0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};
	
	if(time<0) time=0;
	if(time>0)
		left_dec_time [5]=time>>8,
	  left_dec_time [4]=time;

	delay_xms(20);
	CAN1_Send_Num(0x601,left_dec_time);
}

void CAN_acc_time_right(int time)
{
	u8 right_acc_time[8]= {0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};
	
	if(time<0) time=0;
	if(time>0)
	  right_acc_time[5]=time>>8,
	  right_acc_time[4]=time;
	
	delay_xms(20);
	CAN1_Send_Num(0x601,right_acc_time);
}

void CAN_dec_time_right(int time)
{
	u8 right_dec_time[8]= {0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	
	if(time<0) time=0;
	if(time>0)
		right_dec_time[5]=time>>8,
	  right_dec_time[4]=time;

	delay_xms(20);
	CAN1_Send_Num(0x601,right_dec_time);
}

void CAN_SetMaxRpm_left_Right(int Left, int Right)
{
  u8 set_left_max[8]=  {0x23, 0x81, 0x60, 0x01, 0x3C, 0x00, 0x00, 0x00};
  u8 set_right_max[8]= {0x23, 0x81, 0x60, 0x02, 0x3C, 0x00, 0x00, 0x00};
	
	delay_ms(30);
		if(Left>0)
	  set_left_max [5]=Left>>8,
	  set_left_max [4]=Left;
	if(Right>0)
	  set_right_max[5]=Right>>8,
	  set_right_max[4]=Right;
	
	CAN1_Send_Num(0x601,set_left_max);
	delay_ms(30);
	CAN1_Send_Num(0x601,set_right_max);
}

/**************************************************************************
Function: 
Input   : none
Output  : none
Function: Set the torque slope of the left motor, default 500mA/s
Inlet parameters: mAs: torque slope, unit: mA/s
Return value: none
**************************************************************************/
void CAN_Left_Torque_mAs(int mAs)
{
  u8 left_torque[8]= {0x23, 0x87, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};

	if(mAs>0)
		left_torque[5]=mAs>>8,
	  left_torque[4]=mAs;
	
  delay_ms(30);	
	CAN1_Send_Num(0x601,left_torque);	
}
/**************************************************************************
Function: 
Input   : none
Output  : none
Function: Set the right motor torque slope, default 500mA/s
Inlet parameters: mAs: torque slope, unit: mA/s
Return value: none
**************************************************************************/
void CAN_Right_Torque_mAs(int mAs)
{
  u8 right_torque[8]= {0x23, 0x87, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};

	if(mAs>0)
		right_torque[5]=mAs>>8,
	  right_torque[4]=mAs;
	
  delay_ms(30);	
	CAN1_Send_Num(0x601,right_torque);	
}

/**************************************************************************
Function: 
Input   : none
Output  : none
Function: Set hub speed
Entry parameters: ¡À20480
Return value: none
**************************************************************************/
void hub_CAN_Rpm(int R_L, int rpm)
{
  u8 set_rpm[8]= {0x23, 0xFF, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	
	//Set the left motor or the right motor
	if     (R_L==0) set_rpm[3]=0x01; //Left motor
	else if(R_L==1) set_rpm[3]=0x02; //Right motor
	
	//Set the speed and direction
	if(rpm>=0)
	{
		set_rpm[4]=rpm;
		set_rpm[5]=rpm>>8;
	}
	else
	{
		set_rpm[4]=(0xffff+rpm+1);
		set_rpm[5]=(0xffff+rpm+1)>>8;
	}
	
  CAN1_Send_Num(0x601,set_rpm);
}

//void hub_CAN_Syn_Rpm(int rpm)
//{
//  u8 set_rpm[8]= {0x23, 0xFF, 0x60, 0x03, 0x64, 0x00, 0x64, 0x00};
//	
//	//Set the speed and direction
//	if(rpm>=0)
//	{
//		set_rpm[4]=rpm;
//		set_rpm[5]=rpm>>8;
//		set_rpm[6]=rpm;
//		set_rpm[7]=rpm>>8;
//	}
//	else
//	{
//		set_rpm[4]=(0xffff+rpm+1);
//		set_rpm[5]=(0xffff+rpm+1)>>8;
//		set_rpm[6]=(0xffff+rpm+1);
//		set_rpm[7]=(0xffff+rpm+1)>>8;
//	}
//	
//  CAN1_Send_Num(0x601,set_rpm);
//}
void hub_CAN_Syn_Rpm(int left_rpm, int right_rpm)
{
  u8 set_rpm[8]= {0x23, 0xFF, 0x60, 0x03, 0x64, 0x00, 0x64, 0x00};
	
	//Set the speed and direction
	if(left_rpm>=0)
	{
		set_rpm[4]=left_rpm;
		set_rpm[5]=left_rpm>>8;
	}
	else
	{
		set_rpm[4]=(0xffff+left_rpm+1);
		set_rpm[5]=(0xffff+left_rpm+1)>>8;
	}
	if(right_rpm>=0)
	{
		set_rpm[6]=right_rpm;
		set_rpm[7]=right_rpm>>8;
	}
	else
	{
		set_rpm[6]=(0xffff+right_rpm+1);
		set_rpm[7]=(0xffff+right_rpm+1)>>8;
	}	
  CAN1_Send_Num(0x601,set_rpm);
}
/**************************************************************************
Function: 
Input   : none
Output  : none
Function: Set the hub position
Entry parameters: None
Return value: none
**************************************************************************/
void hub_CAN_Rel_Position(int R_L, long int position)
{
	u8 set_position[8]= {0x23, 0x7a, 0x60, 0x01, 0x00, 0x7d, 0x00, 0x00};
  u8 set_start1[8]=   {0x2b, 0x40, 0x60, 0x00, 0x4f, 0x00, 0x00, 0x00};
  u8 set_start2[8]=   {0x2b, 0x40, 0x60, 0x00, 0x5f, 0x00, 0x00, 0x00};
	
	//Set the left motor or the right motor
	if     (R_L==0) set_position[3]=0x01; //Left motor
	else if(R_L==1) set_position[3]=0x02; //Right motor
	
	if(position>=0)
	{
		set_position[7] = position>>24;
		set_position[6] = position>>16;
		set_position[5] = position>>8;
		set_position[4] = position;
	}
	else
	{
		set_position[7] =(0xffffffff+position+1)>>24;
		set_position[6] =(0xffffffff+position+1)>>16;
		set_position[5] =(0xffffffff+position+1)>>8;
		set_position[4] =(0xffffffff+position+1);
	}
	
	CAN1_Send_Num(0x601,set_position);
	CAN1_Send_Num(0x601,set_start1);
	CAN1_Send_Num(0x601,set_start2);
}

void hub_CAN_Abs_Position(int R_L, long int position)
{
	u8 set_position[8]= {0x23, 0x7a, 0x60, 0x01, 0x00, 0x7d, 0x00, 0x00};
  u8 set_start1[8]=   {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
  u8 set_start2[8]=   {0x2b, 0x40, 0x60, 0x00, 0x1f, 0x00, 0x00, 0x00};
	
	//Set the left motor or the right motor
	if     (R_L==0) set_position[3]=0x01; //Left motor
	else if(R_L==1) set_position[3]=0x02; //Right motor
	
	if(position>=0)
	{
		set_position[7] = position>>24;
		set_position[6] = position>>16;
		set_position[5] = position>>8;
		set_position[4] = position;
	}
	else
	{
		set_position[7] =(0xffffffff+position+1)>>24;
		set_position[6] =(0xffffffff+position+1)>>16;
		set_position[5] =(0xffffffff+position+1)>>8;
		set_position[4] =(0xffffffff+position+1);
	}
	
	CAN1_Send_Num(0x601,set_position);
	CAN1_Send_Num(0x601,set_start1);
	CAN1_Send_Num(0x601,set_start2);
}

/**************************************************************************
Function: 
Input   : none
Output  : none
Function: Set hub torque
Input parameters: ¡À2000mA
Return value: none
**************************************************************************/
void hub_CAN_Torque(int R_L, int torque)
{
  u8 set_torque[8]= {0x2b, 0x71, 0x60, 0x01, 0xe8, 0x03, 0x00, 0x00};
	
	//Set the left motor or the right motor
	if     (R_L==0) set_torque[3]=0x01; //Left motor
	else if(R_L==1) set_torque[3]=0x02; //Right motor
	
	//Set the speed and direction
	if(torque>=0)
	{
		set_torque[7] = torque>>24;
		set_torque[6] = torque>>16;
		set_torque[5] = torque>>8;
		set_torque[4] = torque;
	}
	else
	{
		set_torque[7] =(0xffffffff+torque+1)>>24;
		set_torque[6] =(0xffffffff+torque+1)>>16;
		set_torque[5] =(0xffffffff+torque+1)>>8;
		set_torque[4]= (0xffffffff+torque+1);
	}

	CAN1_Send_Num(0x601,set_torque);
}
/**************************************************************************
Function: 
Input   : none
Output  : none
Function: Set hub synchronous torque
Input parameters: ¡À2000mA
Return value: none
**************************************************************************/
void hub_CAN_Syn_Torque(int torque)
{
	u8 set_torque[8]= {0x23, 0x71, 0x60, 0x03, 0xe8, 0x03, 0x00, 0x00};
	
	if(torque>=0)
	{
		set_torque[7] = torque>>8;
		set_torque[6] = torque;
		set_torque[5] = torque>>8;
		set_torque[4] = torque;
	}
	else
	{
		set_torque[7] =(0xffff+torque+1)>>8;
		set_torque[6] =(0xffff+torque+1);
		set_torque[5] =(0xffff+torque+1)>>8;
		set_torque[4]= (0xffff+torque+1);
	}

	CAN1_Send_Num(0x601,set_torque);
}

/**************************************************************************
Function: 
Input   : none
Output  : none
Function: enable reading the real-time encoder
Input parameters: ¡À2000mA
Return value: none
**************************************************************************/
void hub_CAN_Encoder_init(void)
{
	u8 Clean_TPD00 [8]= {0x2f, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	u8 Mapping_01_V  [8]= {0x23, 0x00, 0x1a, 0x01, 0x20, 0x01, 0x6C, 0x60}; //Position: 64 60, torque: 71 60, temperature: 32 20
	u8 Mapping_02_V  [8]= {0x23, 0x00, 0x1a, 0x02, 0x20, 0x02, 0x6C, 0x60}; //Position: 64 60, torque: 71 60, temperature: 32 20
	
//	u8 Mapping_01_P [8]= {0x23, 0x00, 0x1a, 0x01, 0x20, 0x01, 0x64, 0x60}; //position: 64 60, torque: 71 60, temperature: 32 20
//	u8 Mapping_02_P  [8]= {0x23, 0x00, 0x1a, 0x02, 0x20, 0x02, 0x64, 0x60};
//	
//	u8 Mapping_01_T [8]= {0x23, 0x00, 0x1a, 0x01, 0x20, 0x01, 0x71, 0x60}; //position: 64 60, torque: 71 60, temperature: 32 20
//	u8 Mapping_02_T  [8]= {0x23, 0x00, 0x1a, 0x02, 0x20, 0x02, 0x71, 0x60};
	
	u8 Set_ID_181  [8]= {0x23, 0x00, 0x18, 0x01, 0x81, 0x01, 0x00, 0x00};
	u8 Set_event   [8]= {0x2f, 0x00, 0x18, 0x02, 0xfe, 0x00, 0x00, 0x00};
	//u8 Set_hibTime [8]= {0x2b, 0x00, 0x18, 0x03, 0xf4, 0x01, 0x00, 0x00}; // Interval time: 50ms = (0x01*256 + 0xf4)/10
	u8 Set_hibTime [8]= {0x2b, 0x00, 0x18, 0x03, 0xc8, 0x00, 0x00, 0x00}; //Interval: 20ms=(0x01*256+0xc8)/10
	u8 StartMapping[8]= {0x2f, 0x00, 0x1a, 0x00, 0x02, 0x00, 0x00, 0x00};
	u8 SaveEEPROM  [8]= {0x2B, 0x09, 0x20, 0x00, 0x02, 0x00, 0x00, 0x00};
	u8 Enter       [8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	

	CAN1_Send_Num(0x601,Clean_TPD00);
	
  delay_xms(20);
	CAN1_Send_Num(0x601,Mapping_01_V);
	CAN1_Send_Num(0x601,Mapping_02_V);

	
//	delay_ms(5);
//	CAN1_Send_Num(0x601,Mapping_01_P);
//	delay_ms(5);
//	CAN1_Send_Num(0x601,Mapping_02_P);
//	delay_ms(5);
	
//	delay_ms(5);
//	CAN1_Send_Num(0x601,Mapping_01_T);
//	delay_ms(5);
//	CAN1_Send_Num(0x601,Mapping_02_T);
//	delay_ms(5);
	
	delay_xms(20);
	CAN1_Send_Num(0x601,Set_ID_181);

	CAN1_Send_Num(0x601,Set_event);

	CAN1_Send_Num(0x601,Set_hibTime);

	CAN1_Send_Num(0x601,StartMapping);

	CAN1_Send_Num(0x601,SaveEEPROM);

	CAN1_Send_Num(0x00,Enter);
}

