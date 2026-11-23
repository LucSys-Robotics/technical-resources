#ifndef __UARTX_CALLBACK_H
#define __UARTX_CALLBACK_H 

#include "system.h"

//The length of data the robot receives control commands
#define RECEIVE_DATA_SIZE 11

//The structure where the robot receives control commands
typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1 byte
		float X_speed;	            //4 bytes // 4 bytes
		float Y_speed;              //4 bytes // 4 bytes
		float Z_speed;              //4 bytes // 4 bytes
		unsigned char Frame_Tail;   //1 bytes //1 byte
	}Control_Str;
}RECEIVE_DATA;
// IP frame configuration parameters
#define IP_REQUEST_HEADER_1  0xAA   // The first byte of the IP request frame header
#define IP_REQUEST_HEADER_2  0xBB   // Second byte of the IP request frame header
#define IP_CONFIRM_COUNT     50     // Number of confirmations: Stop the request after receiving 50 valid IPs (about 2.5 seconds @20Hz)
#define IP_REQUEST_TIMEOUT   6000    // Request timeout: 300 seconds @ 20Hz


//Internal function
static float XYZ_Target_Speed_transition(u8 High,u8 Low);
static u8 AT_Command_Capture(u8 uart_recv);
static void _System_Reset_(u8 uart_recv);
void RobotControl_SetDebugLevel(char uart_recv);

extern u8 Debug_Flag;
extern u8 IP_Frame_Valid;           
extern u8 IP_Request_Enable ;                  // IP request enable flag (1=send request, 0=stop request)


extern u8 Received_IP[4];  // Stores 4 bytes of the IP address
#endif

