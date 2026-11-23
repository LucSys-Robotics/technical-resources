#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//If using ucos, just include the following header files.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS use
#endif  
 

//////////////////////////////////////////////////////////////////
//Add the following code to support the printf function without needing to choose use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//Support functions required by the standard library
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//Define _sys_exit() to avoid using semi-host mode
void _sys_exit(int x) 
{ 
	x = x; 
} 
//Redefine fputc
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//Send in a loop until finished
	USART2->DR = (u8) ch;      
	return ch;
	
//	while((USART1->SR&0X40)==0); // Loop sending until finished
//	USART1->DR = (u8) ch;      
//	return ch;

//	while((USART3->SR&0X40)==0); // Loop sending until finished
//	USART3->DR = (u8) ch;      
//	return ch;
	
}

//Any serial port printf
#include "stdarg.h"
uint8_t Ux_TxBuff[256];     //Serial port x send buffer
void any_printf(USART_TypeDef* USARTx,char *format,...)
{	
	uint8_t i;                                           //Used for for-loops
	
	va_list listdata;                                     //Create a va_list variable listdata
	va_start(listdata,format);                            //Load listdata... Represents an irregular parameter
	vsprintf((char *)Ux_TxBuff,format,listdata);          //Format output to buffer U0_TxBuff
	va_end(listdata);                                     //Free listdata

	for(i=0;i<strlen((const char*)Ux_TxBuff);i++){        //Send byte by byte in loops based on the U0_TxBuff buffer data volume
		while((USARTx->SR&0x40)==0);
		USARTx->DR = Ux_TxBuff[i];
			
	}
	while((USARTx->SR&0x40)==0);	     //Wait until the last byte of data is sent before exiting the function
}

#endif
 
#if EN_USART1_RX   //If receiving is enabled
//Serial port 1 interrupt service program
//Note: Reading USARTx->SR can avoid unexplained errors
u8 USART_RX_BUF[USART_REC_LEN];     //Receive buffer, up to USART_REC_LEN bytes.
//Reception status
//bit15, receive completion flag
//bit14, received 0x0d
//bit13~0, the number of valid bytes received
u16 USART_RX_STA=0;       //Receive status tags

//Initialize IO UART1
//bound: baud rate
void uart_init(u32 bound){
	//GPIO port settings
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //Enable GPIOA clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//Enable the USART1 clock
 
	//Serial port 1 corresponds to pin multiplexing mapping
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9 is multiplexed to USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10 reused as USART1
	
	//USART1 port configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9 and GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//Reuse function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//Speed: 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Push-pull reuse output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //Pull up
	GPIO_Init(GPIOA,&GPIO_InitStructure); //Initialize PA9, PA10

   //USART1 initialization settings
	USART_InitStructure.USART_BaudRate = bound;//Baud rate settings
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//Word length is 8-bit data format
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//A stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;//No parity bits
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//No hardware data flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving modes
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1
	
	USART_Cmd(USART1, ENABLE);  //Enable serial port 1
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//Enable related interrupts

	//Usart1 NVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//Serial port 1 interrupts the channel
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//Preemption priority 3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//Sub-priority 3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ channel enabled
	NVIC_Init(&NVIC_InitStructure);	//Initialize VIC registers according to specified parameters,


	
}
#endif





 



