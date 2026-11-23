#include "I2C.h"
/**************************************************************************
Function: IIC pin initialization
Input   : none
Output  : none
Function: IIC pin initialization
Entry parameters: None
Return value: none
**************************************************************************/
void I2C_GPIOInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//Enable the GPIOB clock

  //GPIOB8, B9 initialization settings
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Normal output mode
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//Push-pull output
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Pull up
  GPIO_Init(GPIOB, &GPIO_InitStructure);//Initialize
	IIC_SCL=1;
	IIC_SDA=1;
	
}

/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : none
Function: simulates IIC start signal
Entry parameters: None
Return value: none
**************************************************************************/
void I2C_Start(void)
{
	SDA_OUT();     //SDA line output
	IIC_SDA=1;
	if(!READ_SDA)return ;	
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return ;
	delay_us(1);
	IIC_SCL=0;//Hold the I2C bus to prepare to send or receive data
	return ;
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
Function: simulates IIC end signals
Entry parameters: None
Return value: none
**************************************************************************/
void I2C_Stop(void)
{
	SDA_OUT();//SDA line output
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1; 
	IIC_SDA=1;//Send the I2C bus end signal
	delay_us(1);	
}



bool I2C_WaiteForAck(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA is set as input
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			I2C_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//Clock output 0
	return 1;
}

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
Function: IIC response
Entry parameters: None
Return value: none
**************************************************************************/
void I2C_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}

/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
Function: IIC does not respond
Entry parameters: None
Return value: none
**************************************************************************/ 
void I2C_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}



bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;
    
    Dat =I2C_ReadOneByte(DevAddr, RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}




bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{

    uint8_t Dat, Mask;
    
	Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}

/**************************************************************************
Function: IIC sends a bit
Input   : none
Output  : none
Function: IIC sends one bit
Entry parameters: None
Return value: none
**************************************************************************/
void I2C_WriteByte(uint8_t Data)
{
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//Pull down the clock to start data transfer
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(Data&0x80)>>7;
        Data<<=1; 	  
		delay_us(1);   
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
}


u8 I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_WriteByte(Data);
	I2C_WaiteForAck();
	I2C_Stop();
	return 1;
}


bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	
	for(i = 0; i < Num; i ++)
	{
		I2C_WriteByte(*(pBuff + i));
		I2C_WaiteForAck();
	}
	I2C_Stop();

	return true;
}

/**************************************************************************
Function: IIC reads a bit
Input   : none
Output  : none
Function: IIC reads one bit
Entry parameters: None
Return value: none
**************************************************************************/
uint8_t I2C_ReadByte(uint8_t Ack)
{
	uint8_t i, RecDat = 0;

	SDA_IN();
	for(i = 0; i < 8; i ++)
	{
	//	I2C_SCL_Clr();
		IIC_SCL=0;
		delay_us(1);
//		I2C_SCL_Set();
				IIC_SCL=1;
		RecDat <<= 1;
		if(READ_SDA)
			RecDat |= 0x01;
		else
			RecDat &= ~0x01;
		delay_us(1);
	}
	if(I2C_ACK == Ack)
		I2C_Ack();
	else
		I2C_NAck();

	return RecDat;
}




uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t TempVal = 0;
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();
	TempVal = I2C_ReadByte(I2C_NACK);
	I2C_Stop();
	
	return TempVal;
}

bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();

	for(i = 0; i < Num; i ++)
	{
		if((Num - 1) == i)
		{
			*(pBuff + i) = I2C_ReadByte(I2C_NACK);
		}
		else
		{
			*(pBuff + i) = I2C_ReadByte(I2C_ACK);
		}
	}

	I2C_Stop();
	
	return true;
}


///**************************************************************************
//Function: IIC continuous reading data
//Input   : dev£ºTarget device IIC address£»reg:Register address£»
//					length£ºNumber of bytes£»*data:The pointer where the read data will be stored
//Output  : count£ºNumber of bytes read out-1
//Function: IIC continuous data reading
//Entry parameters: dev: target device IIC address; reg: register address; length: number of bytes;
//					*data: The pointer to which the read data will be stored
//Return value: count: number of bytes read minus 1
//**************************************************************************/ 
//u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
//    u8 count = 0;
//	
//	IIC_Start();
//	IIC_Send_Byte(dev); // Send write command
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg); // Send address
//  IIC_Wait_Ack();	  
//	IIC_Start();
//	IIC_Send_Byte(dev+1); // Enter receive mode
//	IIC_Wait_Ack();
//	
//    for(count=0;count<length;count++){
//		 
//		 if(count != length - 1) data[count] = IIC_Read_Byte(1); // Read data with ACK
//		 else data[count] = IIC_Read_Byte(0); // last byte NACK
//	}
//    IIC_Stop(); // Generate a stop condition
//    return count;
//}



