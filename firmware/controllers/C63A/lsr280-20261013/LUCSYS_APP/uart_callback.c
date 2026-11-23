#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//Serial port controls the mutex lock
static uint8_t serial_control_lock = 0;
static uint32_t serial_control_tick = 0;

//Serial port receives completion callback functions
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	extern uint8_t BlueToothBuffer;
	extern uint8_t rosbuffer;
	extern uint8_t usart1_buffer;
	extern uint8_t rs485_buffer;
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//Serial port used by Bluetooth
	if( huart == &huart2 )
	{
		/* APP writes instructions */
		extern QueueHandle_t g_xQueueBlueTooth;
		
		//The interrupt write queue must perform a non-null check to prevent exceptions caused by writes at startup
		if( g_xQueueBlueTooth!=NULL )
			xQueueSendFromISR(g_xQueueBlueTooth,&BlueToothBuffer,&xHigherPriorityTaskWoken);
		
		HAL_UART_Receive_IT(&huart2,&BlueToothBuffer,1);
	}
	
	//Serial port used by ROS
	else if( huart == &huart4 )
	{
		extern QueueHandle_t g_xQueueROSserial;
		
		//The value 1 is the highest priority mutex lock
		serial_control_lock = 1;
		
		if( g_xQueueROSserial!=NULL )
		{
			xQueueSendFromISR(g_xQueueROSserial,&rosbuffer,&xHigherPriorityTaskWoken);
			serial_control_tick = xTaskGetTickCountFromISR();
		}
			
		HAL_UART_Receive_IT(&huart4,&rosbuffer,1);
	}
	
	//Serial port 1 receives buffer
	else if( huart == &huart1 )
	{
		extern QueueHandle_t g_xQueueROSserial;
		
		//Priority mutex lock detection prevents multiple serial ports from writing to a single queue simultaneously
		// 2 is an artificially defined priority; the lower the value, the higher the priority
		if( 2 <= serial_control_lock )
		{
			serial_control_lock=2;
			if( g_xQueueROSserial!=NULL )
			{
				xQueueSendFromISR(g_xQueueROSserial,&usart1_buffer,&xHigherPriorityTaskWoken);
				serial_control_tick = xTaskGetTickCountFromISR();
			}
		}
		else
		{
			if((xTaskGetTickCountFromISR() - serial_control_tick) > 2000)
				serial_control_lock = 2;
		}

		HAL_UART_Receive_IT(&huart1,&usart1_buffer,1);
	}
	
	//485 receive buffer
	else if( huart == &huart3 )
	{
		//HAL_UART_Transmit(&huart3,&rs485_buffer,1,200);
		HAL_UART_Receive_IT(&huart3,&rs485_buffer,1);
	}
	
	//Initiate scheduling based on specific circumstances
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
