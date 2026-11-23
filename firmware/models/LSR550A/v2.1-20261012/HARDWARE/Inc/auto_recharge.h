#ifndef __AUTORECHARGE_H
#define __AUTORECHARGE_H
#include "sys.h"

typedef struct{
	u8 L_A;          //Four-channel infrared signal status. 1 Signal received, 0 No signal
	u8 L_B;
	u8 R_A;
	u8 R_B;
	u8 RED_STATE;    //Summary of four infrared signals: values 0~4
	u8 NavWalk;      //Navigation takes over the recharge control flag 1 taken over 0 Status
	u8 AllowRecharge;//Automatic recharge mode flag 1 Recharge mode 0 Normal mode
	u8 Charging;     //Charging marker position: 1 Charged 0 Not charging
	
	u8 RedRefalsh_Time; //Refresh time when infrared signal recognition, unit: 10ms.
	
	float ChargingCurrent;//Charging current magnitude
	
	float Dock_MoveX; //Set the speed during docking, which is directly assigned to the Red_Move
	float Dock_MoveY;
	float Dock_MoveZ;
	
	float Red_MoveX; //Recognize the three-axis speed of the cart as provided by the infrared signal
	float Red_MoveY;
	float Red_MoveZ;
	
	float Up_MoveX;//When navigation takes over, the car's speed of 3 axles is provided
	float Up_MoveY;
	float Up_MoveZ;
	
	u8 OutLine_Check;//Offline inspection of charging equipment
	
}CHARGER_t;


//External interface
extern CHARGER_t charger;
void auto_recharge_reset(void);
void CAN_Send_AutoRecharge(void);
void Find_Charging_HardWare(void);

//Logic functions for handling automatic recharge
void Handle_Normal_AutoRecharge(uint8_t* temp_rxbuf);
void Handle_AKM_AutoRecharge(uint8_t* temp_rxbuf);

#if defined AKM_CAR
//The following is dedicated to automatic recharging for Ackermann cars. When using automatic recharge in Ackermann, speed execution comes from the vehicle itself, not the speed logic of charging equipment. \
(原因是阿克曼对接逻辑不同,充电装备上的逻辑需要满足其他小车,不能只满足阿克曼.故阿克曼独立在控制板端)
#define cur_front_left do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ =  0.1f ;\
}while(0)

#define cur_front_right do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ = -0.1f ;\
}while(0)


#define front_left do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ =  charger.Dock_MoveZ ;\
}while(0)

#define front_right do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ = -charger.Dock_MoveZ ;\
}while(0)

#define back_left do{                          \
			charger.Red_MoveX = charger.Dock_MoveX ,\
			charger.Red_MoveZ = -charger.Dock_MoveZ ;\
}while(0)

#define back_right do{                          \
			charger.Red_MoveX = charger.Dock_MoveX ,\
			charger.Red_MoveZ = charger.Dock_MoveZ ;\
}while(0)

#define back do{                          \
			charger.Red_MoveX = charger.Dock_MoveX ,\
			charger.Red_MoveZ = 0 ;\
}while(0)

#define front do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ = 0 ;\
}while(0)

#define stop do{                          \
			charger.Red_MoveX = 0 ,\
			charger.Red_MoveZ = 0 ;\
}while(0)
#endif //#if defined AKM_CAR


#endif
