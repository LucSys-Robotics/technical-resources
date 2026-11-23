#ifndef __SYS_H
#define __SYS_H	 
#include "stm32f4xx.h" 

//TODO: Used for debugging variables
typedef struct{
	float   f_val1;
	float   f_val2;
	int     int_val;
	short   s_val;
	uint8_t  u8_val;
	uint16_t u16_val;
	
	//TODO: Added to Ackermann debugging
//	float akm_left_angle;
//	float akm_right_angle;
//	float akm_turnR;
//	
//	float odom_x;
//	float odom_y;
//	float odom_z;
//	
//	uint16_t set_turnangle;
	//1. Be sure to pay attention to division-related statements; denominator variables must be initialized and cannot be zero
}DEBUG_t;


//Hardware version, starting with V1.0
typedef enum{
	HW_NONE = 0,
	V1_0 = 0x0001,  //Ver1.0: C50C, gyroscope MPU6050, PS2 controller with a standard socket
	V1_1 = 0x0002,  //Ver1.1: C50C, gyroscope ICM20948, PS2 controller upgraded to USB controller
}HARDWARE_VERSION;

//Software version, starting at V1.00.
typedef enum{
	SW_NONE = 0,
	V1_00 = 0x0001, //Ver1.00: For software version updates, please refer to the Updatalog/Update Log .txt
}SOFTWARE_VERSION;

//System-level variables related to the robot
typedef struct{
	HARDWARE_VERSION HardWare_Ver;  //Hardware version
	SOFTWARE_VERSION SoftWare_Ver;  //Software version
	int Time_count;      // System power-on timer
	u8 HardWare_charger; // Whether the system has charging equipment hardware 1: Present 0: Not present
	u8 SecurityLevel;    // System safety level Default is 1 0: Highest level, continuously monitors robot control commands; automatically stops if lost. 1: Does not monitor control commands; if command loss, the last speed command is retained 2~255£º... TBA
	u16 LED_delay;       // System status indicator flashing time in milliseconds
}SYS_VAL_t;

extern SYS_VAL_t SysVal;
void SYS_VAL_t_Init(SYS_VAL_t* p);
const uint8_t* getSW_Ver(SOFTWARE_VERSION ver);
const uint8_t* getHW_Ver(HARDWARE_VERSION ver);

//extern DEBUG_t debug;

//Math ABS interface
//abs(): Used for int type
//fabs(): used for float type
//labs(): used for long int type data.
//llabs(): for data of type long long int.
//fabsf(): used for float type data
//fabsl(): used for long double type data

//0, OS not supported
//1, supports OS
#define SYSTEM_SUPPORT_OS		1		//Defines whether system folders support the OS
																	    
	 
//bit band operation enables GPIO control functions similar to 51
//For the specific implementation idea, refer to <<CM3 Authority Guide>> Chapter 5 (pages 87~92). M4 is similar to M3, only the register addresses have changed.
//IO port macro definition
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO port address mapping
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO port operation, only for a single IO port!
//Ensure the value of n is less than 16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //Output
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //Input

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //Output
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //Input

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //Output
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //Input

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //Output
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //Input

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //Output
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //Input

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //Output
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //Input

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //Output
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //Input

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //Output
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //Input

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //Output
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //Input

//The following are assembly functions
void WFI_SET(void);		//Execute WFI instructions
void INTX_DISABLE(void);//Close all interrupts
void INTX_ENABLE(void);	//Enable all interrupts
void MSR_MSP(u32 addr);	//Set the stack address
#endif











