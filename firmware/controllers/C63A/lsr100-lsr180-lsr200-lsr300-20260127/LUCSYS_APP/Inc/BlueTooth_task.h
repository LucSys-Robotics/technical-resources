#ifndef __BLUETOOTH_TASK_H
#define __BLUETOOTH_TASK_H

#include <stdint.h>

typedef struct{
	uint8_t dirkey; //Joystick direction button
	uint8_t page;   //APP page records
	uint8_t saveflash;//Save flash commands
	uint8_t reportparam;//Submit data request instructions
}LUCSYS_APPKey_t;


extern LUCSYS_APPKey_t lucsysApp;

#endif
