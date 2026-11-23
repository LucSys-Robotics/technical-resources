#ifndef __PS2_TASK_H
#define __PS2_TASK_H

#include "system.h"

#define PS2_TASK_PRIO		4     //Task priority //Task Priority
#define PS2_STK_SIZE 		256   //Task stack size //Task Stack Size
#define PS2_TASK_RATE       RATE_20_HZ 

void pstwo_task(void *pvParameters);

#endif
