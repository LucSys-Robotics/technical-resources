#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include <stdint.h>

typedef struct{
	void (*init)(void);                                        //CAN initialization
	uint8_t (*sendStd)(uint32_t id,uint8_t *pData,uint8_t Len); //Send standard frame data
	uint8_t (*sendExt)(uint32_t id,uint8_t *pData,uint8_t Len); //Send extended frame data
}CANInterface_t,*pCANInterface_t; 

typedef struct{
	uint32_t id;
	uint8_t buffer[8];
}CANmsgType_t;

extern CANInterface_t UserCAN1Dev , UserCAN2Dev;

#endif /* __BSP_CAN_H */
