#ifndef __CHECK_H
#define __CHECK_H

#include "sys.h"
#include "system.h"

typedef struct
{
	unsigned char Process;				//Flag the progress of the self-test mode
	int Motor_forward;						//Motor forward count value
	int Motor_retreat;						//Motor reverse count value
	unsigned char Servo_Dir[6];		//Six servo directions
	int Servo_count[6];						//PWM values for six servos
	unsigned char Usart2_check_dir;			//Bluetooth receives remote control direction markers
	unsigned char Usart3_check_flag;		//Serial port 3 self-test marker position
	unsigned char Usart3_rec_buffer[50];//Serial port 3 receives the cache
	unsigned char Usart3_rec_count;
}Check_parameter;

#define CHECK_TASK_PRIO		4     //Task priority //Task Priority
#define CHECK_STK_SIZE 		512   //Task stack size //Task Stack Size

extern Check_parameter check_parameter;

void Check_task(void *pvParameters);
void Check_Key(void);

#endif
