#include "stmflash.h"
#include "delay.h"
#include "stmflash.h"  
#define FLASH_SAVE_ADDR  0X0800E000 	//Set the FLASH save address (must be an even number, and its value must be greater than the FLASH size occupied by this code + 0X08000000)
//Unlock FLASH for STM32
void STMFLASH_Unlock(void)
{
  FLASH->KEYR=FLASH_KEY1;//Write unlock sequence.
  FLASH->KEYR=FLASH_KEY2;
}
//flash locked
void STMFLASH_Lock(void)
{
  FLASH->CR|=1<<7;//Lock
}
//Get FLASH status
u8 STMFLASH_GetStatus(void)
{	
	u32 res;		
	res=FLASH->SR; 
	if(res&(1<<0))return 1;		    //Busy
	else if(res&(1<<2))return 2;	//Programming error
	else if(res&(1<<4))return 3;	//Write protection error
	return 0;						//Operation complete
}
//Wait for the operation to complete
//time: length of delay
//Return value: status.
u8 STMFLASH_WaitDone(u16 time)
{
	u8 res;
	do
	{
		res=STMFLASH_GetStatus();
		if(res!=1)break;//Not busy; no need to wait, return immediately.
		delay_us(1);
		time--;
	 }while(time);
	 if(time==0)res=0xff;//TIMEOUT
	 return res;
}
//Erase page
//paddr: page address
//Return value: execution result
u8 STMFLASH_ErasePage(u32 paddr)
{
	u8 res=0;
	res=STMFLASH_WaitDone(0X5FFF);//Wait for the last operation to finish, > 20ms
	if(res==0)
	{ 
		FLASH->CR|=1<<1;//Page erase
		FLASH->AR=paddr;//Set the page address
		FLASH->CR|=1<<6;//Start erasing
		res=STMFLASH_WaitDone(0X5FFF);//Wait for the operation to finish, > 20ms
		if(res!=1)//Not busy
		{
			FLASH->CR&=~(1<<1);//Clear page erase flag.
		}
	}
	return res;
}
//Write half a character at the specified address in FLASH
//faddr: specified address (this address must be a multiple of 2!!)
//dat: Data to be written
//Return value: write result
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat)
{
	u8 res;	   	    
	res=STMFLASH_WaitDone(0XFF);	 
	if(res==0)//OK
	{
		FLASH->CR|=1<<0;//Programming
		*(vu16*)faddr=dat;//Write data
		res=STMFLASH_WaitDone(0XFF);//Wait for the operation to complete
		if(res!=1)//Operation successful
		{
			FLASH->CR&=~(1<<0);//Clear PG bits.
		}
	} 
	return res;
} 
//Read half-word (16-bit data) of a specified address
//faddr: read address
//Return value: corresponding data.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}
#if STM32_FLASH_WREN	//If writing is enabled
//Unchecked writes
//WriteAddr: Starting address
//pBuffer: Data pointer
//NumToWrite: Half-word (16-bit) count
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		STMFLASH_WriteHalfWord(WriteAddr,pBuffer[i]);
	    WriteAddr+=2;//Address increase 2.
	}  
} 
//Write data of the specified length starting from the specified address
//WriteAddr: Starting address (this address must be a multiple of 2!!)
//pBuffer: Data pointer
//NumToWrite: Number of half-words (16 bits) (the number of 16-bit data to be written).
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //bytes
#else 
#define STM_SECTOR_SIZE	2048
#endif		 
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//Up to 2K bytes
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
	u32 secpos;	   //sector address
	u16 secoff;	   //Offset address within a sector (calculated by 16-bit words)
	u16 secremain; //Remaining addresses within the sector (calculated with 16-bit words)
 	u16 i;    
	u32 offaddr;   //Remove the address after 0X08000000
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//Invalid address
	STMFLASH_Unlock();						//Unlock
	offaddr=WriteAddr-STM32_FLASH_BASE;		//Actual offset address.
	secpos=offaddr/STM_SECTOR_SIZE;			//Sector address 0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//Offset within a sector (2 bytes as the base unit).
	secremain=STM_SECTOR_SIZE/2-secoff;		//Size of the remaining sector space
	if(NumToWrite<=secremain)secremain=NumToWrite;//Not larger than the sector range
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//Read the entire sector's content
		for(i=0;i<secremain;i++)//Validate data
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//Erase required
		}
		if(i<secremain)//Erase required
		{
			STMFLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//Erase this sector
			for(i=0;i<secremain;i++)//Replication
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//Write to the entire sector
		}else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//Write deleted sections directly to the remaining sector intervals.
		if(NumToWrite==secremain)break;//Write has finished
		else//Write not completed
		{
			secpos++;				//Sector address increments by 1
			secoff=0;				//Offset position is 0
		   	pBuffer+=secremain;  	//Pointer offset
			WriteAddr+=secremain*2;	//Write address offset (16-bit data address, requires *2)
		   	NumToWrite-=secremain;	//Decreasing bytes (16 bits).
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//The next sector still can't be finished
			else secremain=NumToWrite;//The next sector can be written
		}	 
	};	
	STMFLASH_Lock();//Lock
}
#endif
//Read data of the specified length starting from the specified address
//ReadAddr: starting address
//pBuffer: Data pointer
//NumToWrite: Half-word (16-bit) count
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//Read 2 bytes.
		ReadAddr+=2;//Offset by 2 bytes.
	}
}

//WriteAddr: Starting address
//WriteData: Data to be written
void Test_Write(u32 WriteAddr,u16 WriteData)   	
{
	STMFLASH_Write(WriteAddr,&WriteData,1);//Write one word
}
 
/**************************************************************************
Function: Read specified data from Flash
Entry parameters: None
Return value: none
**************************************************************************/
void Flash_Read(void)
{
	STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)PID_Parameter,10);
	if(PID_Parameter[0]==65535&&PID_Parameter[1]==65535&&PID_Parameter[2]==65535&&PID_Parameter[3]==65535&&PID_Parameter[4]==65535&&PID_Parameter[5]==65535&&PID_Parameter[6]==65535)
	{
		RC_Velocity=30;
		Velocity_KP=12;
		Velocity_KI=12;
	}
  else
	{		
		RC_Velocity=PID_Parameter[0];//Read data from Flash
		Velocity_KP=PID_Parameter[1];
		Velocity_KI=PID_Parameter[2];
	}
}	
/**************************************************************************
Function: Write specified data to Flash
Entry parameters: None
Return value: none
**************************************************************************/
void Flash_Write(void)
{
Flash_Parameter[0]=RC_Velocity;	//Write data
Flash_Parameter[1]=Velocity_KP;	//
Flash_Parameter[2]=Velocity_KI;	//
STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)Flash_Parameter,10);	
}	









