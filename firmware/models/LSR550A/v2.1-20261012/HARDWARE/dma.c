#include "dma.h"
#include "sys.h"

DMA_InitTypeDef DMA_InitStructure;
u16 DMA1_MEM_LEN2;//Saves the length of each DMA data transfer

/**************************************************************************
Function: Serial Port 2 DMA initialization
Entry parameters: DMA channel, target address of data, starting address of data, amount of data sent
Return value: none
**************************************************************************/
void MYDMA_Init2(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//Enable DMA transfer
	
  DMA_DeInit(DMA_CHx);	//Reset the DMA's channel 1 register to its default value
	DMA1_MEM_LEN2=cndtr;		//Data transfer volume
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA peripheral ADC base address
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA memory base address
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //Data transfer direction: read from memory to send to peripherals
	DMA_InitStructure.DMA_BufferSize = cndtr;  //The size of the DMA cache for the DMA channel
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //The peripheral address register remains unchanged
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //Memory address register increment
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //Data width is 8 bits
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //Data width is 8 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //Operates in normal caching mode
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA channel x has high priority
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA channel x is not set for memory-to-memory transfer
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //Initialize the DMA channel USART1_Tx_DMA_Channel identified registers according to the parameters specified in the DMA_InitStruct
} 
/**************************************************************************
Function: Enable a single DMA transfer
Entry parameters: DMA channel, target address of data, starting address of data, amount of data sent
Return value: none
**************************************************************************/
void MYDMA_Enable2(DMA_Channel_TypeDef*DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //Close the channel indicated by USART1 TX DMA1
 	DMA_SetCurrDataCounter(DMA1_Channel7,DMA1_MEM_LEN2);//The size of the DMA cache for the DMA channel
 	DMA_Cmd(DMA_CHx, ENABLE);  //Enable the channel indicated by USART1 TX DMA1
}	  

///**************************************************************************
//Function: printf in DMA mode
//Entry parameters: What to print
//Return value: none
//**************************************************************************/
void DMA_printf(const char *format,...)
{
	u32 length;
	va_list args;
	
	va_start(args, format);
	length = vsnprintf((char*)t2xbuf, sizeof(t2xbuf), (char*)format, args);
    va_end(args);
	USART_SendBuffer((const char*)t2xbuf,length);
}

u32 USART_SendBuffer(const char* buffer, u32 length)
{
	if( (buffer==NULL) || (length==0) )
	{
		return 0;
	}
 
	DMA_Cmd(DMA1_Channel7, DISABLE); //Disable DMA channel
	DMA_SetCurrDataCounter(DMA1_Channel7, length);
	DMA_Cmd(DMA1_Channel7, ENABLE); //Enable DMA channels
	while(1)
	{
		if(DMA_GetITStatus(DMA1_IT_TC7)!=RESET)	//Confirm that channel 7 (uart2_tx) transmission is complete
		{
			DMA_ClearFlag(DMA1_IT_TC7);//Clear the channel 7 transfer completion flag
			break;
		}
	}
	return length;
}
