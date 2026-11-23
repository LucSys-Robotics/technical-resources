#ifndef __AUTORECHARGE_TASK_H
#define __AUTORECHARGE_TASK_H

#include <stdint.h>

typedef struct{
	uint8_t online;
	uint8_t R_A;
	uint8_t R_B;
	uint8_t L_A;
	uint8_t L_B;
	uint8_t RedNum;
	uint8_t ChargingFlag;//Whether it is already at the charging marker
	uint8_t devState;    //Internal state machine for charging equipment
	float  ChargingCur;  //Charging current
	float  ChargingVol;  //Charging voltage
	
	float dockerX;
	float dockerY;
	float dockerZ;
}ChargeDevType_t;

extern ChargeDevType_t ChargeDev;

#endif

