#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define USART_REC_LEN  			200  	//Define the maximum number of bytes to be received: 200
#define EN_USART1_RX 			1		//Enable (1)/Disable (0) serial port 1 reception
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //Receive buffer, up to USART_REC_LEN bytes. The last byte is a line break
extern u16 USART_RX_STA;         		//Receive status tags
//If you want UART interrupt reception, do not comment out the following macro definition
void uart_init(u32 bound);
void any_printf(USART_TypeDef* USARTx,char *format,...);

#endif


