#include "can.h"
#include "system.h"
/**************************************************************************
Function: CAN1 initialization
Input   : tsjw£ºResynchronize the jump time unit, Scope: 1 ~ 3;
 			    tbs2£ºTime unit of time period 2, range :1~8;
 			    tbs1£ºTime unit of time period 1, range :1~16;
 			    brp £ºBaud rate divider, range :1 to 1024;(We're actually going to add 1, which is 1 to 1024) tq=(brp)*tpclk1
 			    mode£º0, normal mode;1. Loop mode;
Output  : 0- Initialization successful;Other - initialization failed
Note: none of the entry parameters (except mode) can be 0
Function: CAN1 initialization
Entry parameters: tsjw: Resynchronize jump time unit, range: 1~3;
 			    tbs2: time units for time period 2, range: 1~8;
 			    tbs1: time units for time period 1, range: 1~16;
 			    brp: baud rate crossover, range: 1~1024; (Actually add 1, i.e., 1~1024) tq = (brp) * tpclk1
 			    mode: 0, normal mode; 1, loop mode;
Return value: 0-initialization succeeded; other-initialization failed
Note: Entry parameters (except mode) cannot be set to 0
Baud rate = Fpclk1 / ((tbs1 + tbs2 + tsjw) * brp), Fpclk1 is 42M

**************************************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	u16 i=0;
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
	tsjw-=1; //Subtract 1 before setting // Subtract 1 before setting
	tbs2-=1;
	tbs1-=1;
	brp-=1;

	//Enable the related clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//Enable the PORTB clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//Enable CAN1 clock
	//Initialize GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//Reuse function
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
	GPIO_Init(GPIOD, &GPIO_InitStructure);//Initialize PB8, PB9
//	//Pin multiplexing mapping configuration
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //GPIOB8 is multiplexed to CAN1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //GPIOB9 is multiplexed to CAN1

	CAN1->MCR=0x0000;	//Exit sleep mode (set all bits to 0 at the same time) // Exit sleep mode (set all bits to 0 simultaneously)
	CAN1->MCR|=1<<0;	//Request CAN1 to enter initialization mode // Request CAN1 to enter initialization mode
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100)return 2; //Failed to enter initialization mode //Failed to enter initialization mode
	}
	//Non-time triggered communication mode
	//Non-time-triggered communication mode
	CAN1->MCR|=0<<7;
  //Software automatic offline management	
	//Automatic software offline management
	CAN1->MCR|=0<<6;	
	//Sleep mode is awakened by software (clear CAN1- >;
  //Sleep mode is activated via software (clearing the SLEEP bit of CAN1->MCR)
	CAN1->MCR|=0<<5;
	//Disallow automatic message transmission
  //Disable automatic transmission of messages
	CAN1->MCR|=1<<4;
	//Messages are not locked, the new overwrites the old
  //Messages are not locked; new ones overwrite old ones
	CAN1->MCR|=0<<3;
	//The priority is determined by the message identifier	
  //Priority is determined by the message identifier
	CAN1->MCR|=0<<2;
  //Clear the original Settings	
  //Clear the original settings
	CAN1->BTR=0x00000000; 
	//Mode set to 0, normal mode;1. Loop mode;
	//Mode settings 0, normal mode; 1, loop mode;
	CAN1->BTR|=mode<<30;
  //Resynchronization jump width (TSJW) is TSJW +1 time unit	
	//Resynchronization jump width (Tsjw) is tsjw+1 time quanta
	CAN1->BTR|=tsjw<<24; 
  //Tbs2= Tbs2 +1 time unit	
	//Tbs2=tbs2+1 time unit
	CAN1->BTR|=tbs2<<20; 
	//Tbs1= Tbs1 +1 time unit	
  //Tbs1=tbs1+1 time unit
	CAN1->BTR|=tbs1<<16;
	//Frequency division coefficient (Fdiv) is brp +1, boulder rate: Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
  //Frequency division factor (Fdiv) is brp+1, Baud rate: Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN1->BTR|=brp<<0;  
  //Request CAN1 to exit initialization mode	
  //Request CAN1 to exit initialization mode
	CAN1->MCR&=~(1<<0);	  
	while((CAN1->MSR&1<<0)==1)
	{
		i++;
		if(i>0XFFF0)return 3; //Failed to exit initialization mode //Failed to exit initialization mode
	}
	/*** Filter initialization || Filter initialization ***/
	//Filter groups work in initialization mode
	//Filter bank operates in initialization mode
	CAN1->FMR|=1<<0;	
	//Filter 0 is not active
  //Filter 0 disabled
	CAN1->FA1R&=~(1<<0);	
	//The filter bit width is 32 bits
  //Filter width is 32 bits
	CAN1->FS1R|=1<<0; 
	//Filter 0 works in identifier masking bit mode
  //Filter 0 operates in identifier mask mode
	CAN1->FM1R|=0<<0;	
	//Filter 0 is associated with FIFO0
  //Filter 0 assigned to FIFO0
	CAN1->FFA1R|=0<<0;	
  //32 bit ID	
	//32-bit ID
	CAN1->sFilterRegister[0].FR1=0X00000000;
	//32-bit MASK
	//32-bit MASK
	CAN1->sFilterRegister[0].FR2=0X00000000;
	//Activation filter 0
	//Activate Filter 0
	CAN1->FA1R|=1<<0;	
	//Filter group enters normal mode
  //Filter bank enters normal mode
	CAN1->FMR&=0<<0;			

#if CAN1_RX0_INT_ENABLE
  //Enable interrupt reception
	
	//FIFO0 message register interrupt enable
	CAN1->IER|=1<<1;			
	
	//Configure CAN to receive interrupts
	//Configure CAN receive interrupt
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  //Preemption priority	
	//Preemption priority
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Son priority
	//Sub-priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  //IRQ channel enablement
	//IRQ channel enabled
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initializes the VIC register according to the specified parameters 
	//Initialize the VIC register according to the specified parameters
	NVIC_Init(&NVIC_InitStructure);	

  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
#endif
	return 0;
}   
/**************************************************************************
Function: CAN sends data
Input   : id:Standard ID(11 bits)/ Extended ID(11 bits +18 bits)
			    ide:0, standard frame;1, extension frames
			    rtr:0, data frame;1, remote frame
			    len:Length of data to be sent (fixed at 8 bytes, valid data is 6 bytes in time-triggered mode) 
			    *dat:Pointer to the data
Output  : 0~3, mailbox number. 0xFF, no valid mailbox
Function: CAN to send data
Entry parameters: id: Standard ID (11 bits) / Extended ID (11 bits + 18 bits)
			    ide:0, standard frame; 1, extended frame
			    rtr:0, data frame; 1, remote frame
			    len: Length of data to be sent (fixed at 8 bytes; effective data in time-triggered mode is 6 bytes)
			    *dat: Data pointer.
Return value: 0~3-mailbox number; 0XFF-no valid mailbox
**************************************************************************/
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
	u8 mbox;	  
	if(CAN1->TSR&(1<<26))mbox=0;		  //Mailbox 0 is empty // Mailbox 0 is empty
	else if(CAN1->TSR&(1<<27))mbox=1;	//Mailbox 1 is empty // Mailbox 1 is empty
	else if(CAN1->TSR&(1<<28))mbox=2;	//Mailbox 2 is empty // Mailbox 2 is empty
	else return 0XFF;					        //No empty mailbox, cannot send // No empty mailbox, cannot send
	
	CAN1->sTxMailBox[mbox].TIR=0; //Clear the previous Settings // Clear previous settings
	if(ide==0) //The standard frame // Standard frame
	{
		id&=0x7ff; //Take the low 11-bit STDID // Take the low 11-bit STDID
		id<<=21;		  
	}else	//Extend the frame // Extend frame
	{
		id&=0X1FFFFFFF; //Take a low 32-bit extid // Take a low 32-bit extid
		id<<=3;									   
	}
	CAN1->sTxMailBox[mbox].TIR|=id;		 
	CAN1->sTxMailBox[mbox].TIR|=ide<<2;	  
	CAN1->sTxMailBox[mbox].TIR|=rtr<<1;
	len&=0X0F; //Get lower 4 bits // Get lower 4 bits
	CAN1->sTxMailBox[mbox].TDTR&=~(0X0000000F);
	CAN1->sTxMailBox[mbox].TDTR|=len;	//Set the DLC // Set up the DLC
	//The data to be sent is stored in the mailbox
	//Store data to be sent in the mailbox
	CAN1->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
								((u32)dat[6]<<16)|
 								((u32)dat[5]<<8)|
								((u32)dat[4]));
	CAN1->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
								((u32)dat[2]<<16)|
 								((u32)dat[1]<<8)|
								((u32)dat[0]));
	CAN1->sTxMailBox[mbox].TIR|=1<<0; //Request to send mailbox data // Request to send mailbox data
	return mbox;
}
/**************************************************************************
Function: Get the send status
Input   : Mbox: mailbox number
Output  : 0, hang;0X05, send failed;0X07, successful transmission
Function: Obtain the sending status
Entry parameter: mbox: mailbox number
Return value: 0-pending; 0X05-transmission failed; 0X07-transmission succeeded
**************************************************************************/
u8 CAN1_Tx_Staus(u8 mbox)
{	
	u8 sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN1->TSR&(1<<0);			   //RQCP0
			sta |= CAN1->TSR&(1<<1);			   //TXOK0
			sta |=((CAN1->TSR&(1<<26))>>24); //TME0
			break;
		case 1: 
			sta |= CAN1->TSR&(1<<8)>>8;		   //RQCP1
			sta |= CAN1->TSR&(1<<9)>>8;		   //TXOK1
			sta |=((CAN1->TSR&(1<<27))>>25); //TME1	   
			break;
		case 2: 
			sta |= CAN1->TSR&(1<<16)>>16;	   //RQCP2
			sta |= CAN1->TSR&(1<<17)>>16;	   //TXOK2
			sta |=((CAN1->TSR&(1<<28))>>26); //TME2
			break;
		default:
			sta=0X05; //Wrong email number, failed // Wrong email number, failed
		break;
	}
	return sta;
} 
/**************************************************************************
Function: Returns the number of packets received in FIFO0/FIFO1
Input   : Fifox: FIFO number (0, 1)
Output  : Number of packets in FIFO0/FIFO1
Function: Gets the number of messages received in FIFO0/FIFO1
Entry parameters: fifox: FIFO number (0, 1)
Return value: number of messages in FIFO0/FIFO1
**************************************************************************/
u8 CAN1_Msg_Pend(u8 fifox)
{
	if(fifox==0)return CAN1->RF0R&0x03; 
	else if(fifox==1)return CAN1->RF1R&0x03; 
	else return 0;
}
/**************************************************************************
Function: Receive data
Input   : fifox£ºEmail
		    	id:Standard ID(11 bits)/ Extended ID(11 bits +18 bits)
			    ide:0, standard frame;1, extension frames 
			    rtr:0, data frame;1, remote frame
			    len:Length of data received (fixed at 8 bytes, valid at 6 bytes in time-triggered mode)
			    dat:Data cache
Output  : none
Function: Receive data
Entry parameter: fifox: Email number
		    	ID: Standard ID (11-bit) / Extended ID (11-bit + 18-bit)
			    ide:0, standard frame; 1, extended frame
			    rtr:0, data frame; 1, remote frame
			    len: Received data length (fixed at 8 bytes; in time-trigger mode, valid data is 6 bytes)
			    dat: Data cache
Return value: none
**************************************************************************/
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
	*ide=CAN1->sFIFOMailBox[fifox].RIR&0x04; //Gets the value of the identifier selection bit // Get the value of the identifier selection bit
 	if(*ide==0) //Standard identifier // Standard identifier
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>21;
	}else	     //Extended identifier // Extended identifier
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr=CAN1->sFIFOMailBox[fifox].RIR&0x02;	//Gets the remote send request value // Get the remote send request value
	*len=CAN1->sFIFOMailBox[fifox].RDTR&0x0F; //Get the DLC // Get the DLC
 	//*fmi=(CAN1->sFIFOMailBox[FIFONumber]. RDTR>>8) &0xFF; // Get the FMI // Get the FMI
	//Receive data // Receive data
	dat[0]=CAN1->sFIFOMailBox[fifox].RDLR&0XFF;
	dat[1]=(CAN1->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
	dat[2]=(CAN1->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
	dat[3]=(CAN1->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
	dat[4]=CAN1->sFIFOMailBox[fifox].RDHR&0XFF;
	dat[5]=(CAN1->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
	dat[6]=(CAN1->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
	dat[7]=(CAN1->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
  if(fifox==0)CAN1->RF0R|=0X20;      //Free the FIFO0 mailbox // Free the FIFO0 mailbox
	else if(fifox==1)CAN1->RF1R|=0X20; //Free the FIFO1 mailbox // Free the FIFO1 mailbox
}
/**************************************************************************
Function: CAN receives interrupt service function, conditional compilation
Input   : none
Output  : none
Function: CAN receives interrupt service functions and compiles conditionally
Entry parameters: None
Return value: none
**************************************************************************/
#if CAN1_RX0_INT_ENABLE	//Enable RX0 interrupt
void CAN1_RX0_IRQHandler(void)
{
	u32 id;
	u8 ide,rtr,len;     

	u8 temp_rxbuf[8];

 	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,temp_rxbuf);
	
	//Recharge equipment signature code
	if(id==0x12345678)
	{
		u8 check=0;
		for(u8 i=0;i<8;i++) 
		{
			check += temp_rxbuf[i];
		}
		if( check==8 ) Get_Charging_HardWare=1;
	}
	
	else if(id==0x181)
	{
		float Vz;
		CAN_ON_Flag=1, PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0;
		command_lost_count=0;
		//Calculate the three-axis target velocity, unit: m/s
		//Calculate the three-axis target velocity, unit: m/s
		Move_X=((float)((short)((temp_rxbuf[0]<<8)+(temp_rxbuf[1]))))/1000; 
		Move_Y=((float)((short)((temp_rxbuf[2]<<8)+(temp_rxbuf[3]))))/1000;
		Vz    =((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		if(Car_Mode==Akm_Car)
		{
			Move_Z=Vz_to_Akm_Angle(Move_X, Vz);
		}
		else
		{
			Move_Z=((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		}
	}
	
	//Automatic recharge related to recharge
	else if(id==0x182)
	{
		charger_check=0;
		
		//Automatic recharge status adjusts PID parameters to speed up motor response
		if(Allow_Recharge) Velocity_KP=600,Velocity_KI=600;
		else Velocity_KP=300,Velocity_KI=300;
		
		//Calculate the three-axis target velocity, unit: mm/s
		//Calculate the three-axis target velocity, unit: mm/s
		Recharge_Red_Move_X=((float)((short)((temp_rxbuf[0]<<8)+(temp_rxbuf[1]))))/1000; 
		Recharge_Red_Move_Y=((float)((short)((temp_rxbuf[2]<<8)+(temp_rxbuf[3]))))/1000;
		Recharge_Red_Move_Z=((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		
		Charging=temp_rxbuf[6]&1;		//Charging status indicator
		L_A = (temp_rxbuf[6]>>5)&0x01;
		L_B = (temp_rxbuf[6]>>4)&0x01;
		R_B = (temp_rxbuf[6]>>3)&0x01;
		R_A = (temp_rxbuf[6]>>2)&0x01;
		RED_STATE = L_A + L_B + R_B + R_A;
		
		if(RED_STATE==0) Recharge_Red_Move_X = Recharge_Red_Move_Y = Recharge_Red_Move_Z = 0;
		if( Charging==1) Recharge_Red_Move_X = Recharge_Red_Move_Y = Recharge_Red_Move_Z = 0;
		
		//Charging current conversion
		if(temp_rxbuf[7]>128)Charging_Current=-(256-temp_rxbuf[7])*30;
		else Charging_Current=(temp_rxbuf[7]*30);
	}
}
#endif

/**************************************************************************
Function: CAN1 sends a set of data (fixed format :ID 0X601, standard frame, data frame)
Input   : msg:Pointer to the data
    			len:Data length (up to 8)
Output  : 0, success, others, failure;
Function: CAN1 sends a set of data (fixed format: ID is 0X601, standard frame, data frame)
Entry parameter: msg: Data pointer
    			len: Data length (maximum 8)
Return value: 0-success, other-failure;
**************************************************************************/
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(0X601,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending // Waiting for sending to finish
	if(i>=0XFFF)return 1; //Send failure //Send failure
	return 0;	//Send a success //Send success
}
/**************************************************************************
Function: The CAN1 port receives data queries
Input   : Buf: The data cache
Output  : 0, number of data received, other, length of data received
Function: CAN1 port receives data queries
Entry parameter: buf: data cache
Return value: 0-no data received; otherwise, received data length
**************************************************************************/
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
	u32 id;
	u8 ide,rtr,len; 
	if(CAN1_Msg_Pend(0)==0)return 0;			   //No data received, exit directly // No data received, exit directly
  	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,buf); //Read the data //Read data
    if(id!=0x12||ide!=0||rtr!=0)len=0;		 //Receive error // Receive error
	return len;	
}
/**************************************************************************
Function: Can1 sends a set of data tests
Input   : msg:Pointer to the data
			    len:Data length (up to 8)
Output  : 0, success, 1, failure
Function: CAN1 sends a set of data for testing
Entry parameter: msg: Data pointer
			    len: Data length (maximum 8)
Return value: 0-success, 1-failure
**************************************************************************/
u8 CAN1_Send_MsgTEST(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
    mbox=CAN1_Tx_Msg(0X701,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending // Waiting for sending to finish
	if(i>=0XFFF)return 1;	//Send failure //Send failure
	return 0;	//Send a success //Send success
}
/**************************************************************************
Function: Sends an array to the given ID
Input   : id£ºID no.
			    msg£ºThe transmitted data pointer
Output  : 0, success, 1, failure
Function: Send a command to an array given to a given ID
Entry parameter: ID: ID number
			    msg: The data pointer being conveyed
Return value: 0-success, 1-failure
**************************************************************************/
u8 CAN1_Send_Num(u32 id,u8* msg)
{
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(id,0,0,8,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending // Waiting for sending to finish
	if(i>=0XFFF)return 1;	//Send failure //Send failure
	return 0;
}

u8 CAN1_Send_EXTid_Num(u32 id,u8* msg)
{
	u8 mbox;
	u16 i=0;	  	 						       
	mbox=CAN1_Tx_Msg(id,1,0,8,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending // Waiting for sending to finish
	if(i>=0XFFF)return 1;	//Send failure //Send failure
	return 0;
}


