#include "can.h"
#include "system.h"

//Initialization of pins corresponding to the CAN interface
static void GPIO_CANPort_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	ENABLE_CAN1_TX_PIN_CLOCK;
	ENABLE_CAN1_RX_PIN_CLOCK;
	
	//Initialize GPIO
	//CAN1
	GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //Pull up
	GPIO_Init(CAN1_TX_PORT, &GPIO_InitStructure);           
	
	GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //Pull up
	GPIO_Init(CAN1_RX_PORT, &GPIO_InitStructure);    
	
	//Pin multiplexing mapping configuration
	GPIO_PinAFConfig(CAN1_TX_PORT,CAN1_TX_Soure,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(CAN1_RX_PORT,CAN1_RX_Soure,GPIO_AF_CAN1);
}

//Initialization of pins corresponding to the CAN interface
static void V1_0_GPIO_CANPort_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	V1_0_ENABLE_CAN1_TX_PIN_CLOCK;
	V1_0_ENABLE_CAN1_RX_PIN_CLOCK;
	
	//Initialize GPIO
	//CAN1
	GPIO_InitStructure.GPIO_Pin = V1_0_CAN1_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //Pull up
	GPIO_Init(V1_0_CAN1_TX_PORT, &GPIO_InitStructure);           
	
	GPIO_InitStructure.GPIO_Pin = V1_0_CAN1_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //Pull up
	GPIO_Init(V1_0_CAN1_RX_PORT, &GPIO_InitStructure);    
	
	//Pin multiplexing mapping configuration
	GPIO_PinAFConfig(V1_0_CAN1_TX_PORT,V1_0_CAN1_TX_Soure,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(V1_0_CAN1_RX_PORT,V1_0_CAN1_TX_Soure,GPIO_AF_CAN1);
}

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
Baud rate = Fpclk1 / ((tbs1 + tbs2 + 1) * brp), Fpclk1 is 36M
                =42M/((3+2+1)*6)
						    =1M
**************************************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	//-1 is because the actual values of these configurations start from 0
	//brp does not require -1 because its configuration values start from 1
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0) return 1;
	tsjw-=1; //Subtract 1 before setting // Subtract 1 before setting
	tbs2-=1;
	tbs1-=1;
	
	//Initialize the GPIO corresponding to the CAN port
	GPIO_CANPort_Init();
	
	//Enable the related clock
	ENABLE_CAN1_CLOCK;
	
	//CAN unit settings
	CAN_InitStructure.CAN_TTCM=DISABLE;	//Non-time-triggered communication mode
	CAN_InitStructure.CAN_ABOM=DISABLE;	//Automatic software offline management
	CAN_InitStructure.CAN_AWUM=DISABLE; //Sleep mode is activated via software (clearing the SLEEP bit of the CAN->MCR)
	CAN_InitStructure.CAN_NART=ENABLE;	//Disable automatic transmission of messages
	CAN_InitStructure.CAN_RFLM=DISABLE;	//Messages are not locked; new ones overwrite old ones
	CAN_InitStructure.CAN_TXFP=DISABLE;	//Priority is determined by the message identifier
	CAN_InitStructure.CAN_Mode= mode;	//Mode settings
	
	//Baud rate-related configurations
	CAN_InitStructure.CAN_SJW=tsjw;	    //Resynchronization jump width (Tsjw) is tsjw+1 time quanta; CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1;     //Tbs1 range CAN_BS1_1tq ~ CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;     //Tbs2 range CAN_BS2_1tq ~ CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;//Frequency division factor (Fdiv) is brp+1
	
	CAN_Init(CAN1, &CAN_InitStructure);   // Initialize CAN1
	
	//Configure filter
	//CAN1
	CAN_FilterInitStructure.CAN_FilterNumber=0;	                   //Filter 0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //Mask mode
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32 bits
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;               //32-bit ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; 	

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32-bit MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//Filter 0 assigned to FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //Activate Filter 0
	CAN_FilterInit(&CAN_FilterInitStructure);            //Filter initialization
	
	//CAN1 FIFO0 interrupt enabled
	#if CAN1_RX0_INT_ENABLE
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0 message register interrupt enabled.
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;// Main priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // secondary priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	//CAN2 FIFO1 interrupt enabled
	#if CAN2_RX1_INT_ENABLE
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO1 message register interrupt enabled.
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // Main priority is 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // Secondary priority is 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	return 0;
}   

//CAN port initialization function in version V1.00
u8 V1_0_CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	//-1 is because the actual values of these configurations start from 0
	//brp does not require -1 because its configuration values start from 1
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0) return 1;
	tsjw-=1; //Subtract 1 before setting // Subtract 1 before setting
	tbs2-=1;
	tbs1-=1;
	
	//Initialize the GPIO corresponding to the CAN port
	V1_0_GPIO_CANPort_Init();
	
	//Enable the related clock
	ENABLE_CAN1_CLOCK;       
	
	//Pin multiplexing mapping configuration
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	
	//CAN unit settings
	CAN_InitStructure.CAN_TTCM=DISABLE;	//Non-time-triggered communication mode
	CAN_InitStructure.CAN_ABOM=DISABLE;	//Automatic software offline management
	CAN_InitStructure.CAN_AWUM=DISABLE; //Sleep mode is activated via software (clearing the SLEEP bit of the CAN->MCR)
	CAN_InitStructure.CAN_NART=ENABLE;	//Disable automatic transmission of messages
	CAN_InitStructure.CAN_RFLM=DISABLE;	//Messages are not locked; new ones overwrite old ones
	CAN_InitStructure.CAN_TXFP=DISABLE;	//Priority is determined by the message identifier
	CAN_InitStructure.CAN_Mode= mode;	//Mode settings
	
	//Baud rate-related configurations
	CAN_InitStructure.CAN_SJW=tsjw;	    //Resynchronization jump width (Tsjw) is tsjw+1 time quanta; CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1;     //Tbs1 range CAN_BS1_1tq ~ CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;     //Tbs2 range CAN_BS2_1tq ~ CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;//Frequency division factor (Fdiv) is brp+1
	
	CAN_Init(CAN1, &CAN_InitStructure);   // Initialize CAN1
	
	//Configure filter
	//CAN1
	CAN_FilterInitStructure.CAN_FilterNumber=0;	                   //Filter 0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //Mask mode
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32 bits
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;               //32-bit ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; 	

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32-bit MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//Filter 0 assigned to FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //Activate Filter 0
	CAN_FilterInit(&CAN_FilterInitStructure);            //Filter initialization
	
	//CAN1 FIFO0 interrupt enabled
	#if CAN1_RX0_INT_ENABLE
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0 message register interrupt enabled.
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;// Main priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // secondary priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	//CAN2 FIFO1 interrupt enabled
	#if CAN2_RX1_INT_ENABLE
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO1 message register interrupt enabled.
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // Main priority is 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // Secondary priority is 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	return 0;
}   

// CAN1 sends functions
u8 CAN1_Send_Num(u32 id,u8 *data)
{
	//CAN sends data structures
	CanTxMsg msg;
	
	u16 i;   //timeout
	u8 mbox; //
	
	#if 1 //Configure standard frame and ID
	msg.StdId = id;
	msg.IDE = CAN_Id_Standard;
	#else //Configure extended frame and ID
	msg.ExtId = id;
	msg.IDE = CAN_Id_Extended;
	#endif
	
	//The sent message belongs to the data frame
	//Data Frame CAN_RTR_DATA
	//Remote frame: CAN_RTR_REMOTE
	msg.RTR = CAN_RTR_DATA;
	
	//Length of data sent
	// 1~8
	msg.DLC = 8;
	
	//Copy the data to be sent into msg Data
	memcpy(msg.Data,data,8);
	
	//CAN1 sends
	CAN_Transmit(CAN1,&msg);
	
	//Wait for CAN1 to complete sending
	while( CAN_TransmitStatus(CAN1,mbox)== CAN_TxStatus_Failed && i<0xffff ) i++;
	
	//Send timeout
	if(i>=0xffff) return 1;
	
	return 0;
}

u8 CAN1_Send_EXTid_Num(u32 id,u8 *data)
{
	//CAN sends data structures
	CanTxMsg msg;
	
	u16 i;   //timeout
	u8 mbox; //
	
	#if 0 //Configure standard frame and ID
	msg.StdId = id;
	msg.IDE = CAN_Id_Standard;
	#else //Configure extended frame and ID
	msg.ExtId = id;
	msg.IDE = CAN_Id_Extended;
	#endif
	
	//The sent message belongs to the data frame
	//Data Frame CAN_RTR_DATA
	//Remote frame: CAN_RTR_REMOTE
	msg.RTR = CAN_RTR_DATA;
	
	//Length of data sent
	// 1~8
	msg.DLC = 8;
	
	//Copy the data to be sent into msg Data
	memcpy(msg.Data,data,8);
	
	//CAN1 sends
	CAN_Transmit(CAN1,&msg);
	
	//Wait for CAN1 to complete sending
	while( CAN_TransmitStatus(CAN1,mbox)== CAN_TxStatus_Failed && i<0xffff ) i++;
	
	//Send timeout
	if(i>=0xffff) return 1;
	
	return 0;
}
