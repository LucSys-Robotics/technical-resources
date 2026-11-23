#include "uartx.h"

//App key definition
APP_CONTROL_t appkey;

void APPKey_Param_Init(APP_CONTROL_t *p)
{
	p->DirectionFlag = 0;
	p->ParamSaveFlag = 0;
	p->ParamSendflag = 0;
	p->TurnFlag = 0;
	p->TurnPage = 0;
}

/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
Function: Initialize Serial Port 1
Entry parameters: None
Return value: none
**************************************************************************/
void UART1_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //Corresponding pin and serial port clock on
    ENABLE_UART1_TX_PIN_CLOCK;
    ENABLE_UART1_RX_PIN_CLOCK;
    ENABLE_UART1_CLOCK;

    //AF initialization
    GPIO_PinAFConfig(UART1_TX_PORT,UART1_TX_Soure,GPIO_AF_USART1);
    GPIO_PinAFConfig(UART1_RX_PORT,UART1_RX_Soure,GPIO_AF_USART1);

    //Initialize the corresponding pin port
    GPIO_InitStructure.GPIO_Pin = UART1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(UART1_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = UART1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(UART1_RX_PORT, &GPIO_InitStructure);

    //UsartNVIC configuration // UsartNVIC configuration
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    //Preempt priority //Preempt priority
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 4;
    //Subpriority // Subpriority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //Initialize the VIC register according to the specified parameters
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings initialization settings
    USART_InitStructure.USART_BaudRate = bound; //Port rate // Serial port baud rate
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Word length is 8-bit data format // Word length is 8-bit data format
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit // A stop bit
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //No parity bits
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control // No hardware data flow control
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //Sending and receiving mode
    USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts
    USART_Cmd(USART1, ENABLE);                     //Enable serial port 1
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
Function: Send data via Serial Port 1
Entry parameters: Data to send
Return value: none
**************************************************************************/
void uart1_send(u8 data)
{
    USART1->DR = data;
    while((USART1->SR&0x40)==0);
}


/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
Function: Initialize Serial Port 3
Entry parameters: None
Return value: none
**************************************************************************/
void UART3_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //Corresponding pin and serial port clock on
    ENABLE_UART3_TX_PIN_CLOCK;
    ENABLE_UART3_RX_PIN_CLOCK;
    ENABLE_UART3_CLOCK;

    //AF initialization
    GPIO_PinAFConfig(UART3_TX_PORT,UART3_TX_Soure,GPIO_AF_USART3);
    GPIO_PinAFConfig(UART3_RX_PORT,UART3_RX_Soure,GPIO_AF_USART3);

    //Initialize the corresponding pin port
    GPIO_InitStructure.GPIO_Pin = UART3_TX_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(UART3_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = UART3_RX_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(UART3_RX_PORT, &GPIO_InitStructure);

    //UsartNVIC configuration // UsartNVIC configuration
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    //Preempt priority //Preempt priority
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;
    //Preempt priority //Preempt priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //Initialize the VIC register according to the specified parameters
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings initialization settings
    USART_InitStructure.USART_BaudRate = bound; //Port rate // Serial port baud rate
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Word length is 8-bit data format // Word length is 8-bit data format
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit // A stop
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //No parity bits
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control // No hardware data flow control
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //Sending and receiving mode
    USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts
    USART_Cmd(USART3, ENABLE);                     //Enable serial port 3
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
Function: Send data via Serial Port 3
Entry parameters: Data to send
Return value: none
**************************************************************************/
void uart3_send(u8 data)
{
    USART3->DR = data;
    while((USART3->SR&0x40)==0);
}

/**************************************************************************
Function: Serial port 4 initialization
Input   : none
Output  : none
Function: Initialize Serial Port 4
Entry parameters: None
Return value: none
**************************************************************************/
void UART4_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //Corresponding pin and serial port clock on
    ENABLE_UART4_TX_PIN_CLOCK;
    ENABLE_UART4_RX_PIN_CLOCK;
    ENABLE_UART4_CLOCK;

    //AF initialization
    GPIO_PinAFConfig(UART4_TX_PORT,UART4_TX_Soure,GPIO_AF_UART4);
    GPIO_PinAFConfig(UART4_RX_PORT,UART4_RX_Soure,GPIO_AF_UART4);

    //Initialize the corresponding pin port
    GPIO_InitStructure.GPIO_Pin = UART4_TX_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(UART4_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = UART4_RX_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(UART4_RX_PORT, &GPIO_InitStructure);

    //UsartNVIC configuration // UsartNVIC configuration
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    //Preempt priority //Preempt priority
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;
    //Subpriority // Subpriority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //Initialize the VIC register according to the specified parameters
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings initialization settings
    USART_InitStructure.USART_BaudRate = bound; //Port rate // Serial port baud rate
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Word length is 8-bit data format // Word length is 8-bit data format
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit // A stop
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //No parity bits
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control // No hardware data flow control
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //Sending and receiving mode
    USART_Init(UART4, &USART_InitStructure);      //Initialize serial port 4
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts
	
    USART_Cmd(UART4, ENABLE);                     //Enable serial port 4
}




/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
Function: Send data via Serial Port 4
Entry parameters: Data to send
Return value: none
**************************************************************************/
void uart4_send(u8 data)
{
    UART4->DR = data;
    while((UART4->SR&0x40)==0);
}


void UART2_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //Corresponding pin and serial port clock on
    ENABLE_UART2_TX_PIN_CLOCK;
    ENABLE_UART2_RX_PIN_CLOCK;
    ENABLE_UART2_CLOCK;

    //AF initialization
    GPIO_PinAFConfig(UART2_TX_PORT,UART2_TX_Soure,GPIO_AF_USART2);
    GPIO_PinAFConfig(UART2_RX_PORT,UART2_RX_Soure,GPIO_AF_USART2);

    //Initialize the corresponding pin port
    GPIO_InitStructure.GPIO_Pin = UART2_TX_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(UART2_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = UART2_RX_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(UART2_RX_PORT, &GPIO_InitStructure);

    //UsartNVIC configuration // UsartNVIC configuration
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    //Preempt priority //Preempt priority
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 5;
    //Preempt priority //Preempt priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //Initialize the VIC register according to the specified parameters
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings initialization settings
    USART_InitStructure.USART_BaudRate = bound; //Port rate // Serial port baud rate
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Word length is 8-bit data format // Word length is 8-bit data format
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit // A stop
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //No parity bits
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control // No hardware data flow control
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //Sending and receiving mode
    USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts
    USART_Cmd(USART2, ENABLE);                     //Enable serial port 2
}

void uart2_send(u8 data)
{
    USART2->DR = data;
    while((USART2->SR&0x40)==0);
}
