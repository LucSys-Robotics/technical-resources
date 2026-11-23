#include "can_callback.h"

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;   
	
	u8 temp_rxbuf[8];
	
	//Read data from CAN1 FIFO0 mailbox
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	//Use the data readout buffer
	memcpy(temp_rxbuf,RxMessage.Data,8);
	
	//Standard Frame ID Data Processing
	if( RxMessage.IDE==CAN_Id_Standard )
	{
		switch( RxMessage.StdId ) //Frame ID number
		{
			case 0x181: //Frame ID 0x181 is a chassis control command
			{
				//Set CAN control mode
				if( Get_Control_Mode(_CAN_Control)==0 ) Set_Control_Mode(_CAN_Control);
				robot_control.command_lostcount = 0;//command timeout refresh
				//Accept the target speed of the chassis
				robot_control.Vx = ((float)((short)((temp_rxbuf[0]<<8)|(temp_rxbuf[1]))))/1000.0f;
				robot_control.Vy = ((float)((short)((temp_rxbuf[2]<<8)|(temp_rxbuf[3]))))/1000.0f;
				robot_control.Vz = ((float)((short)((temp_rxbuf[4]<<8)|(temp_rxbuf[5]))))/1000.0f;
				break;
			}
			
			case 0x182: //Commands for automatic recharge devices
			{		
				charger.OutLine_Check = 0;//Recharge status refreshes online
				#if defined AKM_CAR
					Handle_AKM_AutoRecharge(temp_rxbuf); //Ackermann automatic docking logic function
				#else
					Handle_Normal_AutoRecharge(temp_rxbuf); //Other trolleys automatically recharge processing logic functions
				#endif
			}
			
			default:
				break;
		}
	}
	
	//Extended frame ID data processing
	else if( RxMessage.IDE==CAN_Id_Extended )
	{
		switch( RxMessage.ExtId ) //Frame ID number
		{
			case 0x12345678://Automatic recharge feature code verification, used to check whether the chassis has automatic recharge equipment
			{
				u8 check=0;
				for(u8 i=0;i<8;i++) 
				{
					check += temp_rxbuf[i];
				}
				if( check==8 ) SysVal.HardWare_charger = 1;
				break;
			}
			
			case 0x001:
			{
				break;
			}
				
			default:
				break;
		}
	}
	
}
