#include "bsp_can.h"
#include "can.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define CAN1_ID_H      0x0000 //32-bit basic ID setting (16-bit high)
#define CAN1_ID_L      0x0000 //32-bit basic ID settings (16 bits lower)
#define CAN1_MASK_H    0x0000 //32-bit mask setting (16-bit high)
#define CAN1_MASK_L    0x0000 //32-bit mask setting (16-bit lower)

static CAN_HandleTypeDef* UserCan1 = &hcan1; //The name of the CAN handle used by the user
static CAN_HandleTypeDef* UserCan2 = &hcan2; //The name of the CAN handle used by the user

static SemaphoreHandle_t can1_mutex = NULL;
static SemaphoreHandle_t can2_mutex = NULL;

//Initialize the CAN bus filter
static void CAN1_Filter_Init(void)
{
	static uint8_t init = 0;
    CAN_FilterTypeDef  sFilterConfig;
    HAL_StatusTypeDef  HAL_Status;
	
	//Create CAN1 mutex
	can1_mutex = xSemaphoreCreateMutex();
	
	if( 1==init ) return;
	
    sFilterConfig.FilterBank = 0;						//Use filter 0
    sFilterConfig.FilterMode =   CAN_FILTERMODE_IDMASK; //Set to IDLIST list mode / IDMASK masking mode
    sFilterConfig.FilterScale =  CAN_FILTERSCALE_32BIT; //Filter bit width
    sFilterConfig.FilterIdHigh = CAN1_ID_H;				//32-bit basic ID setting (16-bit high)
    sFilterConfig.FilterIdLow  = CAN1_ID_L;				//32-bit basic ID settings (16 bits lower)
    sFilterConfig.FilterMaskIdHigh =  CAN1_MASK_H;		//32-bit mask setting (16-bit high)
    sFilterConfig.FilterMaskIdLow  =  CAN1_MASK_L;		//32-bit mask setting (16-bit lower)
    sFilterConfig.FilterFIFOAssignment =  CAN_RX_FIFO0; //Received packets are placed into FIFO0
    sFilterConfig.FilterActivation =  ENABLE;			//ENABLE to activate the filter, DISABLE to forbid the filter
    sFilterConfig.SlaveStartFilterBank  = 14;			//Filter bank configuration; with dual CAN communication, CAN1 must start at 14
	
    HAL_Status=HAL_CAN_ConfigFilter(UserCan1,&sFilterConfig);//Set the above structure parameters into the CAN register
	
	//Prompts required for CAN filter initialization failure
    if(HAL_Status!=HAL_OK) {}
	
	//Enable the CAN bus function
    HAL_Status=HAL_CAN_Start(UserCan1);
	
	//prompt required for CAN bus boot failure
    if(HAL_Status!=HAL_OK){}

	//Enable CAN bus interrupt
    HAL_Status=HAL_CAN_ActivateNotification(UserCan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    
	//prompt required for CAN bus boot failure
	if(HAL_Status!=HAL_OK){}
	
	init = 1;
}

//Initialize the CAN bus filter
static void CAN2_Filter_Init(void)
{
	static uint8_t init = 0;
    CAN_FilterTypeDef  sFilterConfig;
    HAL_StatusTypeDef  HAL_Status;
	
	can2_mutex = xSemaphoreCreateMutex();
	
	if( 1==init ) return;
	
    sFilterConfig.FilterBank = 14;						//Use filter 0
    sFilterConfig.FilterMode =   CAN_FILTERMODE_IDMASK; //Set to IDLIST list mode / IDMASK masking mode
    sFilterConfig.FilterScale =  CAN_FILTERSCALE_32BIT; //Filter bit width
    sFilterConfig.FilterIdHigh = CAN1_ID_H;				//32-bit basic ID setting (16-bit high)
    sFilterConfig.FilterIdLow  = CAN1_ID_L;				//32-bit basic ID settings (16 bits lower)
    sFilterConfig.FilterMaskIdHigh =  CAN1_MASK_H;		//32-bit mask setting (16-bit high)
    sFilterConfig.FilterMaskIdLow  =  CAN1_MASK_L;		//32-bit mask setting (16-bit lower)
    sFilterConfig.FilterFIFOAssignment =  CAN_RX_FIFO1; //Insert the received packet into FIFO1
    sFilterConfig.FilterActivation =  ENABLE;			//ENABLE to activate the filter, DISABLE to forbid the filter
	
    HAL_Status=HAL_CAN_ConfigFilter(UserCan2,&sFilterConfig);//Set the above structure parameters into the CAN register
	
	//Prompts required for CAN filter initialization failure
    if(HAL_Status!=HAL_OK) {}
	
	//Enable the CAN bus function
    HAL_Status=HAL_CAN_Start(UserCan2);
	
	//prompt required for CAN bus boot failure
    if(HAL_Status!=HAL_OK){}

	//Enable CAN bus interrupt
    HAL_Status=HAL_CAN_ActivateNotification(UserCan2,CAN_IT_RX_FIFO1_MSG_PENDING);
    
	//prompt required for CAN bus boot failure
	if(HAL_Status!=HAL_OK){}
	
	init = 1;
}



////Compatible with older C50C
//static void CAN1_Filter_Init_V1_00(void)
//{
//	static uint8_t init = 0;
//    CAN_FilterTypeDef  sFilterConfig;
//    HAL_StatusTypeDef  HAL_Status;
//	
//	if( 1==init ) return;
//	
//	//Older C50C version without USB port, requiring reverse initialization of the USB port
//    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);
//    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
//	
//	//Deinitialize the CAN of the cubemx configuration version
//    __HAL_RCC_CAN1_CLK_DISABLE();
//    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);
//    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
//	
//	//Initialize CAN IO
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	__HAL_RCC_CAN1_CLK_ENABLE();
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//	HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
//	HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
//	
//	//Filter initialization
//    sFilterConfig.FilterBank = 0; Use filter 0
//    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; Set to IDLIST list mode / IDMASK masking mode
//    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; Filter bit width
//    sFilterConfig.FilterIdHigh = CAN1_ID_H; 32-bit basic ID setting (16 bits high)
//    sFilterConfig.FilterIdLow = CAN1_ID_L; 32-bit base ID setting (16 bits lower)
//    sFilterConfig.FilterMaskIdHigh = CAN1_MASK_H; 32-bit mask setting (16-bit high)
//    sFilterConfig.FilterMaskIdLow = CAN1_MASK_L; 32-bit mask setting (16 bits lower)
//    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; The received packet is placed into FIFO1
//    sFilterConfig.FilterActivation = ENABLE; ENABLE activates the filter, DISABLE disables the filter
//    sFilterConfig.SlaveStartFilterBank = 0; Filter group settings require CAN1 to start from 14 for dual CAN communication
//	
//    HAL_Status=HAL_CAN_ConfigFilter(UserCan,&sFilterConfig); // Set the above struct parameters to the CAN register
//	
//	//Prompts required for CAN filter initialization failure
//    if(HAL_Status!=HAL_OK) {}
//	
//	//Enable the CAN bus function
//    HAL_Status=HAL_CAN_Start(UserCan);
//	
//	//prompt required for CAN bus boot failure
//    if(HAL_Status!=HAL_OK){}

//	//Enable CAN bus interrupt
//    HAL_Status=HAL_CAN_ActivateNotification(UserCan,CAN_IT_RX_FIFO1_MSG_PENDING);
//    
//	//prompt required for CAN bus boot failure
//	if(HAL_Status!=HAL_OK){}
//	
//	init = 1;
//}

static uint8_t can1_send_StdNum(uint32_t id,uint8_t *pData,uint8_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR; //Enumeration of CAN transport status
	uint8_t  FreeTxNum=0;         //Number of free mailboxes
	uint32_t TxMailBox = 0;       //Receive and send data via mailbox, hardware determination, and assignment
	CAN_TxHeaderTypeDef TxMsg;    //Struct for sending data
	
	//Validity assessment of data
	if( Len<1 || Len>8 || pData==NULL ) return HAL_ERROR;
	
	if (xSemaphoreTake(can1_mutex, portMAX_DELAY) == pdTRUE)
	{
		//Wait for an available mailbox
		while(0 == FreeTxNum)
		{
			FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(UserCan1);
		}
		
		TxMsg.StdId = id;      //Frame ID
		TxMsg.IDE = CAN_ID_STD;//Standard frame
		
		//CAN_RTR_DATA Data frame
		//CAN_RTR_REMOTE Remote frame
		TxMsg.RTR = CAN_RTR_DATA;
		
		//Data length, values 1~8
		TxMsg.DLC = Len;
		
		HAL_RetVal = HAL_CAN_AddTxMessage(UserCan1,&TxMsg,pData,&TxMailBox);
		
		xSemaphoreGive(can1_mutex);
	}
		
	if( 0!=HAL_RetVal ) return 1;
	
	return 0;
}

static uint8_t can1_send_ExtNum(uint32_t id,uint8_t *pData,uint8_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR; //Enumeration of CAN transport status
	uint8_t  FreeTxNum=0;         //Number of free mailboxes
	uint32_t TxMailBox = 0;       //Receive and send data via mailbox, hardware determination, and assignment
	CAN_TxHeaderTypeDef TxMsg;    //Struct for sending data
	
	//Validity assessment of data
	if( Len<1 || Len>8 || pData==NULL ) return HAL_ERROR;
	
	if (xSemaphoreTake(can1_mutex, portMAX_DELAY) == pdTRUE)
	{
		//Wait for an available mailbox
		while(0 == FreeTxNum)
		{
			FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(UserCan1);
		}
		
		TxMsg.ExtId = id;      //Frame ID
		TxMsg.IDE = CAN_ID_EXT;//Extended frame
		
		//CAN_RTR_DATA Data frame
		//CAN_RTR_REMOTE Remote frame
		TxMsg.RTR = CAN_RTR_DATA;
		
		//Data length, values 1~8
		TxMsg.DLC = Len;
		
		HAL_RetVal = HAL_CAN_AddTxMessage(UserCan1,&TxMsg,pData,&TxMailBox);
		
		xSemaphoreGive(can1_mutex);
	}

	if( 0!=HAL_RetVal ) return 1;
	
	return 0;
}

static uint8_t can2_send_StdNum(uint32_t id,uint8_t *pData,uint8_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR; //Enumeration of CAN transport status
	uint8_t  FreeTxNum=0;         //Number of free mailboxes
	uint32_t TxMailBox = 0;       //Receive and send data via mailbox, hardware determination, and assignment
	CAN_TxHeaderTypeDef TxMsg;    //Struct for sending data
	
	//Validity assessment of data
	if( Len<1 || Len>8 || pData==NULL ) return HAL_ERROR;
	
	if (xSemaphoreTake(can2_mutex, portMAX_DELAY) == pdTRUE)
	{
		//Wait for an available mailbox
		while(0 == FreeTxNum)
		{
			FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(UserCan2);
		}
		
		TxMsg.StdId = id;      //Frame ID
		TxMsg.IDE = CAN_ID_STD;//Standard frame
		
		//CAN_RTR_DATA Data frame
		//CAN_RTR_REMOTE Remote frame
		TxMsg.RTR = CAN_RTR_DATA;
		
		//Data length, values 1~8
		TxMsg.DLC = Len;
		
		HAL_RetVal = HAL_CAN_AddTxMessage(UserCan2,&TxMsg,pData,&TxMailBox);
		
		xSemaphoreGive(can2_mutex);
	}
		
	if( 0!=HAL_RetVal ) return 1;
	
	return 0;
}

static uint8_t can2_send_ExtNum(uint32_t id,uint8_t *pData,uint8_t Len)
{
	HAL_StatusTypeDef HAL_RetVal = HAL_ERROR; //Enumeration of CAN transport status
	uint8_t  FreeTxNum=0;         //Number of free mailboxes
	uint32_t TxMailBox = 0;       //Receive and send data via mailbox, hardware determination, and assignment
	CAN_TxHeaderTypeDef TxMsg;    //Struct for sending data
	
	//Validity assessment of data
	if( Len<1 || Len>8 || pData==NULL ) return HAL_ERROR;
	
	if (xSemaphoreTake(can2_mutex, portMAX_DELAY) == pdTRUE)
	{
		//Wait for an available mailbox
		while(0 == FreeTxNum)
		{
			FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(UserCan2);
		}
		
		TxMsg.ExtId = id;      //Frame ID
		TxMsg.IDE = CAN_ID_EXT;//Extended frame
		
		//CAN_RTR_DATA Data frame
		//CAN_RTR_REMOTE Remote frame
		TxMsg.RTR = CAN_RTR_DATA;
		
		//Data length, values 1~8
		TxMsg.DLC = Len;
		
		HAL_RetVal = HAL_CAN_AddTxMessage(UserCan2,&TxMsg,pData,&TxMailBox);
		
		xSemaphoreGive(can2_mutex);
	}

	if( 0!=HAL_RetVal ) return 1;
	
	return 0;
}

CANInterface_t UserCAN1Dev = {
	.init = CAN1_Filter_Init,
	.sendStd = can1_send_StdNum,
	.sendExt = can1_send_ExtNum,
};

CANInterface_t UserCAN2Dev = {
	.init = CAN2_Filter_Init,
	.sendStd = can2_send_StdNum,
	.sendExt = can2_send_ExtNum,
};
