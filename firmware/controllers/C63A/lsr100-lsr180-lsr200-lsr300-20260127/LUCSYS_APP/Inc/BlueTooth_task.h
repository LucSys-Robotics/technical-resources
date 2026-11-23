#ifndef __BLUETOOTH_TASK_H
#define __BLUETOOTH_TASK_H

#include <stdint.h>

typedef struct{
	uint8_t dirkey; //摇杆方向按键
	uint8_t page;   //APP页面记录
	uint8_t saveflash;//保存flash指令
	uint8_t reportparam;//上报数据请求指令
}LUCSYS_APPKey_t;


extern LUCSYS_APPKey_t lucsysApp;

#endif
