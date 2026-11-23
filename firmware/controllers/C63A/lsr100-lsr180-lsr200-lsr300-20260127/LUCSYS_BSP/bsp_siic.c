#include "bsp_siic.h"
#include "bsp_dwt.h"

#define userconfig_delayus 1

//Configure gyroscope software IIC interface
static GPIOConfigType_t imu_scl = {
	.pin = IIC_SCL_Pin,
	.port = IIC_SCL_GPIO_Port
};

static GPIOConfigType_t imu_sda = {
	.pin = IIC_SDA_Pin,
	.port = IIC_SDA_GPIO_Port
};


//Confirm that the device's IIC interface configuration is deprecated
//static GPIOConfigType_t ver_scl = {
//	.pin = VerIIC_SCL_Pin,
//	.port = VerIIC_SCL_GPIO_Port
//};

//static GPIOConfigType_t ver_sda = {
//	.pin = VerIIC_SDA_Pin,
//	.port = VerIIC_SDA_GPIO_Port
//};

//Implementation of basic io operations
static void sethigh(GPIOConfigType_t* io)
{
	HAL_GPIO_WritePin(io->port,io->pin,GPIO_PIN_SET);
}

static void setlow(GPIOConfigType_t* io)
{
	HAL_GPIO_WritePin(io->port,io->pin,GPIO_PIN_RESET);
}

static void setInputMode(GPIOConfigType_t* io)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = io->pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(io->port, &GPIO_InitStruct);
}

static void setOutputMode(GPIOConfigType_t* io)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = io->pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(io->port, &GPIO_InitStruct);
}

static uint8_t readPins(GPIOConfigType_t* io)
{
	return HAL_GPIO_ReadPin(io->port,io->pin);
}


//IIC Sub-protocol implementation
static void sIIC_Start(sIIC_IOType_t* io)
{
	setOutputMode(io->SDA);
	setlow(io->SCL);
	sethigh(io->SDA);
	sethigh(io->SCL);
	delay_us(userconfig_delayus);
	setlow(io->SDA);
	delay_us(userconfig_delayus);
	setlow(io->SCL);
//	delay_us(userconfig_delayus);
}

static void sIIC_Stop(sIIC_IOType_t* io)
{
    setOutputMode(io->SDA);   // Set SDA as output
    setlow(io->SCL);  // Pull down the SCL
    setlow(io->SDA);  // Pull down the SDA
	
    sethigh(io->SCL);  // Set the SCL to high level
	
	delay_us(userconfig_delayus);
	
    sethigh(io->SDA);  // When SCL is high, raise the SDA to generate a stop signal
    delay_us(userconfig_delayus);
}

static uint8_t sIIC_WaitAck(sIIC_IOType_t* io,uint32_t timeout)
{
    uint32_t time = 0;

    setInputMode(io->SDA);    // Set SDA as input
    sethigh(io->SDA);  // Raise the SDA
	
	delay_us(userconfig_delayus);
	
    sethigh(io->SCL);  // Raise the SCL and wait for a response signal
	
	delay_us(userconfig_delayus);
	
    while (readPins(io->SDA)) {  // Detects whether the slave pulls down the SDA to generate an ACK signal
        time++;
        delay_us(userconfig_delayus);
        if (time > timeout) {
            sIIC_Stop(io);  // If time expires, a stop signal is sent
            return 0;     // No response
        }
    }

    setlow(io->SCL);  // Pull down the SCL to continue transferring
    setOutputMode(io->SDA);
    return 1;    // Received a response
}

static void sIIC_Ack(sIIC_IOType_t* io)
{
    setOutputMode(io->SDA);   // Set SDA as output
    setlow(io->SCL);  // Pull down the SCL
    setlow(io->SDA);  // Pull down the SDA and send the ACK
	
	delay_us(userconfig_delayus);

    setlow(io->SDA);
    sethigh(io->SCL);  // Pull up the SCL to confirm the transfer
	
	delay_us(userconfig_delayus);

    setlow(io->SCL);  // Pull down the SCL to end the ACK signal
    sethigh(io->SDA);
}

static void sIIC_NAck(sIIC_IOType_t* io)
{
    setOutputMode(io->SDA);   // Set SDA as output
    setlow(io->SCL);  // Pull down the SCL
    setlow(io->SDA);  // Pull down the SDA and send the ACK
	
	delay_us(userconfig_delayus);

    sethigh(io->SDA);
    sethigh(io->SCL);  // Pull up the SCL to confirm the transfer
	
	delay_us(userconfig_delayus);

    setlow(io->SCL);  // Pull down the SCL to end the ACK signal
    sethigh(io->SDA);
}

static void sIIC_SendByte(sIIC_IOType_t* io,uint8_t byte)
{
    uint8_t i;

    setOutputMode(io->SDA);  // Set SDA as output
    setlow(io->SCL); // Pull down the SCL

    for (i = 0; i < 8; i++) {
        if (byte & 0x80) {
            sethigh(io->SDA);  // Send high order
        } else {
            setlow(io->SDA);  // Send low position
        }
        delay_us(1);
        byte <<= 1;       // Move to the next position
        sethigh(io->SCL);       // Raise the SCL to transfer the current position
		delay_us(1);
        setlow(io->SCL);       // Pull down the SCL and prepare for the next position
  
    }
}

static uint8_t sIIC_ReadByte(sIIC_IOType_t* io,uint8_t ack)
{
    uint8_t i, byte = 0;

    setInputMode(io->SDA);  // Set SDA as input

    for (i = 0; i < 8; i++) {
        setlow(io->SCL);  // Pull down the SCL
        delay_us(1);	
        sethigh(io->SCL);  // Pull up the SCL to read the current bit
		delay_us(1);
        byte <<= 1;
        if (readPins(io->SDA)) {
            byte |= 0x01;  // Read at a high level
        }
//        delay_us(1);
    }

    if (!ack) {
        sIIC_NAck(io);  // If no response is needed, send NACK
    } else {
        sIIC_Ack(io);   // Send ACK if acknowledgment is required
    }

    return byte;
}

//IIC Encapsulated protocol implementation
static IIC_Status_t IIC_Master_Transmit(sIIC_IOType_t* io,uint16_t dev_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start(io);    // Send the start signal

    sIIC_SendByte(io,dev_addr);        // Send slave device address (write mode)
    if (!sIIC_WaitAck(io,timeout)) {          // Wait for a response
        sIIC_Stop(io);                // If there is no response, stop communication
        return IIC_TIMEOUT;
    }

    for (uint16_t i = 0; i < size; i++) {
        sIIC_SendByte(io,data[i]);     // Send data byte by byte
        if (!sIIC_WaitAck(io,timeout)) {      // Wait for a response after each byte sent
            sIIC_Stop(io);            // If there is no response, stop communication
            return IIC_TIMEOUT;
        }
    }

    sIIC_Stop(io);                    // Send a stop signal
    return IIC_OK;                       // Success
}

static IIC_Status_t IIC_Master_Receive(sIIC_IOType_t* io,uint16_t dev_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start(io);    // Send the start signal

    sIIC_SendByte(io,dev_addr | 0x01); // Send slave device address (read mode)
    if (!sIIC_WaitAck(io,timeout)) {          // Wait for a response
        sIIC_Stop(io);                // If there is no response, stop communication
        return IIC_TIMEOUT;
    }

    for (uint16_t i = 0; i < size; i++) {
        data[i] = sIIC_ReadByte(io,i == (size - 1) ? 0 : 1); // Read data byte by byte; send NACK for the final byte
    }

    sIIC_Stop(io);                    // Send a stop signal
    return IIC_OK;                       // Success
}


static IIC_Status_t IIC_Mem_Write(sIIC_IOType_t* io,uint16_t dev_addr, uint16_t mem_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start(io);                  // Send the start signal
    
    sIIC_SendByte(io,dev_addr);       // Send slave device address (write mode)
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT; // Wait for a response

    sIIC_SendByte(io,mem_addr);       // Send memory/register addresses
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT; // Wait for a response

    for (uint16_t i = 0; i < size; i++) {
        sIIC_SendByte(io,data[i]);    // Send data byte by byte
        if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT; // Wait for a response
    }
    
    sIIC_Stop(io);                   // Send a stop signal
    return IIC_OK;                      // Write successful
}

static IIC_Status_t IIC_Mem_Read(sIIC_IOType_t* io,uint16_t dev_addr, uint16_t mem_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start(io);                   // Send the start signal
    
    sIIC_SendByte(io,dev_addr);        // Send slave device address (write mode)
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT;  // Wait for a response

    sIIC_SendByte(io,mem_addr);        // Send memory/register addresses
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT;  // Wait for a response
    
    sIIC_Start(io);                    // Repeated START condition; prepare to read data
    sIIC_SendByte(io,dev_addr | 0x01); // Send slave device address (read mode)
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT;  // Wait for a response

    for (uint16_t i = 0; i < size; i++) {
        data[i] = sIIC_ReadByte(io,i == (size - 1) ? 0 : 1); // Read data byte by byte; send NACK for the final byte
    }

    sIIC_Stop(io);                    // Send a stop signal
    return IIC_OK;                       // Read successful
}

static void iic_delayms(uint16_t ms)
{
	delay_ms(ms);
}


//Mount the driver
IICInterface_t UserIMU_sIICDev = {
	.iic_io = {
		.SCL = &imu_scl,
		.SDA = &imu_sda
	},
	.write = IIC_Master_Transmit , 
	.read = IIC_Master_Receive ,
	.write_reg = IIC_Mem_Write ,
	.read_reg = IIC_Mem_Read ,
	.delay_ms = iic_delayms
};

//Mount the driver
//IICInterface_t UserVer_sIICDev = {
//	.iic_io = {
//		.SCL = &ver_scl,
//		.SDA = &ver_sda
//	},
//	.write = IIC_Master_Transmit , 
//	.read = IIC_Master_Receive ,
//	.write_reg = IIC_Mem_Write ,
//	.read_reg = IIC_Mem_Read ,
//	.delay_ms = iic_delayms
//};


#if 0 //High-speed version
#include "bsp_siic.h"
#include "gpio.h"

#define userconfig_USE_DELAY  1 //Whether to use delay functionality: Some devices do not support high speed and require delay to communicate normally
#define userconfig_DELAY_TIME 25 //Delay time, while( userconfig_DELAY_TIME-- ) \
                                      主频越高的单片机,数值需要越大.可以根据实际通信效果来调整.注：数值调整的过低,可能会导致时序不对而无法正常通信

#define sIIC_SDA_H HAL_GPIO_WritePin(IIC_SDA_GPIO_Port,IIC_SDA_Pin,GPIO_PIN_SET)
#define sIIC_SDA_L HAL_GPIO_WritePin(IIC_SDA_GPIO_Port,IIC_SDA_Pin,GPIO_PIN_RESET)
#define sIIC_SCL_H HAL_GPIO_WritePin(IIC_SCL_GPIO_Port,IIC_SCL_Pin,GPIO_PIN_SET)
#define sIIC_SCL_L HAL_GPIO_WritePin(IIC_SCL_GPIO_Port,IIC_SCL_Pin,GPIO_PIN_RESET)

static void SDA_IN(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = IIC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IIC_SDA_GPIO_Port, &GPIO_InitStruct);
}

static void SDA_OUT(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = IIC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IIC_SDA_GPIO_Port, &GPIO_InitStruct);
}

static uint8_t READ_SDA(void)
{
	return HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port,IIC_SDA_Pin);
}

static void sIIC_Start(void)
{
	SDA_OUT();
	sIIC_SDA_H;
	sIIC_SCL_H;
	
	#if userconfig_USE_DELAY == 1
		uint8_t delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
	sIIC_SDA_L;
	
	#if userconfig_USE_DELAY == 1
		delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
	sIIC_SCL_L;
}

static void sIIC_Stop(void)
{
    SDA_OUT();   // Set SDA as output
    sIIC_SCL_L;  // Pull down the SCL
    sIIC_SDA_L;  // Pull down the SDA
	
	#if userconfig_USE_DELAY == 1
		uint8_t delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
    sIIC_SCL_H;  // Set the SCL to high level
	
	#if userconfig_USE_DELAY == 1
		delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
    sIIC_SDA_H;  // When SCL is high, raise the SDA to generate a stop signal
}

static uint8_t sIIC_WaitAck(uint32_t timeout)
{
    uint32_t time = 0;

    SDA_IN();    // Set SDA as input
    sIIC_SDA_H;  // Raise the SDA
	
	#if userconfig_USE_DELAY == 1
		uint8_t delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
    sIIC_SCL_H;  // Raise the SCL and wait for a response signal
	
	#if userconfig_USE_DELAY == 1
		delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
    while (READ_SDA()) {  // Detects whether the slave pulls down the SDA to generate an ACK signal
        time++;
        if (time > timeout) {
            sIIC_Stop();  // If time expires, a stop signal is sent
            return 0;     // No response
        }
    }

    sIIC_SCL_L;  // Pull down the SCL to continue transferring
    return 1;    // Received a response
}

static void sIIC_Ack(void)
{
    sIIC_SCL_L;  // Pull down the SCL
    SDA_OUT();   // Set SDA as output
    sIIC_SDA_L;  // Pull down the SDA and send the ACK
	
	#if userconfig_USE_DELAY == 1
		uint8_t delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
    sIIC_SCL_H;  // Pull up the SCL to confirm the transfer
	
	#if userconfig_USE_DELAY == 1
		delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
    sIIC_SCL_L;  // Pull down the SCL to end the ACK signal
}

static void sIIC_NAck(void)
{
    sIIC_SCL_L;  // Pull down the SCL
    SDA_OUT();   // Set SDA as output
    sIIC_SDA_H;  // Raise SDA and send NACK
	
	#if userconfig_USE_DELAY == 1
		uint8_t delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
    sIIC_SCL_H;  // Pull up the SCL to confirm the transfer
	
	#if userconfig_USE_DELAY == 1
		delay = userconfig_DELAY_TIME;
		while(delay--);
	#endif
	
    sIIC_SCL_L;  // Pull down the SCL to end the NACK signal
}

static void sIIC_SendByte(uint8_t byte)
{
    uint8_t i;

    SDA_OUT();  // Set SDA as output
    sIIC_SCL_L; // Pull down the SCL

    for (i = 0; i < 8; i++) {
        if (byte & 0x80) {
            sIIC_SDA_H;  // Send high order
        } else {
            sIIC_SDA_L;  // Send low position
        }
        byte <<= 1;       // Move to the next position
		
		#if userconfig_USE_DELAY == 1
			uint8_t delay = userconfig_DELAY_TIME;
			while(delay--);
		#endif
		
        sIIC_SCL_H;       // Raise the SCL to transfer the current position
		
		#if userconfig_USE_DELAY == 1
			delay = userconfig_DELAY_TIME;
			while(delay--);
		#endif
		
        sIIC_SCL_L;       // Pull down the SCL and prepare for the next position

    }
}

static uint8_t sIIC_ReadByte(uint8_t ack)
{
    uint8_t i, byte = 0;

    SDA_IN();  // Set SDA as input

    for (i = 0; i < 8; i++) {
        sIIC_SCL_L;  // Pull down the SCL

		#if userconfig_USE_DELAY == 1
			uint8_t delay = userconfig_DELAY_TIME;
			while(delay--);
		#endif
		
        sIIC_SCL_H;  // Pull up the SCL to read the current bit
		
		#if userconfig_USE_DELAY == 1
			delay = userconfig_DELAY_TIME;
			while(delay--);
		#endif
		
        byte <<= 1;
        if (READ_SDA()) {
            byte |= 0x01;  // Read at a high level
        }
    }

    if (!ack) {
        sIIC_NAck();  // If no response is needed, send NACK
    } else {
        sIIC_Ack();   // Send ACK if acknowledgment is required
    }

    return byte;
}

static IIC_Status_t IIC_Master_Transmit(sIIC_IOType_t* io,uint16_t dev_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start();    // Send the start signal

    sIIC_SendByte(dev_addr);        // Send slave device address (write mode)
    if (!sIIC_WaitAck(timeout)) {          // Wait for a response
        sIIC_Stop();                // If there is no response, stop communication
        return IIC_TIMEOUT;
    }

    for (uint16_t i = 0; i < size; i++) {
        sIIC_SendByte(data[i]);     // Send data byte by byte
        if (!sIIC_WaitAck(timeout)) {      // Wait for a response after each byte sent
            sIIC_Stop();            // If there is no response, stop communication
            return IIC_TIMEOUT;
        }
    }

    sIIC_Stop();                    // Send a stop signal
    return IIC_OK;                       // Success
}

static IIC_Status_t IIC_Master_Receive(sIIC_IOType_t* io,uint16_t dev_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start();    // Send the start signal

    sIIC_SendByte(dev_addr | 0x01); // Send slave device address (read mode)
    if (!sIIC_WaitAck(timeout)) {          // Wait for a response
        sIIC_Stop();                // If there is no response, stop communication
        return IIC_TIMEOUT;
    }

    for (uint16_t i = 0; i < size; i++) {
        data[i] = sIIC_ReadByte(i == (size - 1) ? 0 : 1); // Read data byte by byte; send NACK for the final byte
    }

    sIIC_Stop();                    // Send a stop signal
    return IIC_OK;                       // Success
}


static IIC_Status_t IIC_Mem_Write(sIIC_IOType_t* io,uint16_t dev_addr, uint16_t mem_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start();                  // Send the start signal
    
    sIIC_SendByte(dev_addr);       // Send slave device address (write mode)
    if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT; // Wait for a response

    sIIC_SendByte(mem_addr);       // Send memory/register addresses
    if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT; // Wait for a response

    for (uint16_t i = 0; i < size; i++) {
        sIIC_SendByte(data[i]);    // Send data byte by byte
        if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT; // Wait for a response
    }
    
    sIIC_Stop();                   // Send a stop signal
    return IIC_OK;                      // Write successful
}

static IIC_Status_t IIC_Mem_Read(sIIC_IOType_t* io,uint16_t dev_addr, uint16_t mem_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start();                   // Send the start signal
    
    sIIC_SendByte(dev_addr);        // Send slave device address (write mode)
    if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT;  // Wait for a response

    sIIC_SendByte(mem_addr);        // Send memory/register addresses
    if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT;  // Wait for a response
    
    sIIC_Start();                    // Repeated START condition; prepare to read data
    sIIC_SendByte(dev_addr | 0x01); // Send slave device address (read mode)
    if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT;  // Wait for a response

    for (uint16_t i = 0; i < size; i++) {
        data[i] = sIIC_ReadByte(i == (size - 1) ? 0 : 1); // Read data byte by byte; send NACK for the final byte
    }

    sIIC_Stop();                    // Send a stop signal
    return IIC_OK;                       // Read successful
}

static void iic_delayms(uint16_t ms)
{
	HAL_Delay(ms);
}

//Mount the driver
IICInterface_t UserIMU_sIICDev = {
	.write = IIC_Master_Transmit , 
	.read = IIC_Master_Receive ,
	.write_reg = IIC_Mem_Write ,
	.read_reg = IIC_Mem_Read ,
	.delay_ms = iic_delayms
};

//use demo
//void mpu6050_init(void)
//{
//	#define MPU6050_DEV 0x68
//	pIICInterface_t iic = &UserII2Dev;
//	uint8_t writebuf = 0;
//	
//	writebuf=0;
//	iic->write_reg(MPU6050_DEV<<1,0x6B,&writebuf,1,200);
//	
//	writebuf=0x07;
//	iic->write_reg(MPU6050_DEV<<1,0x19,&writebuf,1,200);
//	
//	writebuf=0x06;
//	iic->write_reg(MPU6050_DEV<<1,0x1A,&writebuf,1,200);
//	
//	writebuf=0x01;
//	iic->write_reg(MPU6050_DEV<<1,0x1C,&writebuf,1,200);//±2g
//	
//	writebuf=0x18;
//	iic->write_reg(MPU6050_DEV<<1,0x1B,&writebuf,1,200);//±2000
//	
//	printf("read mpu:%d\r\n",iic->read_reg(0x68<<1,0x3B,accel,6,200));
//	printf("X:%.2f\r\n",(float)(short)(accel[0]<<8|accel[1])*0.00059814453125f);
//	printf("Y:%.2f\r\n",(float)(short)(accel[2]<<8|accel[3])*0.00059814453125f);
//	printf("Z:%.2f\r\n\r\n",(float)(short)(accel[4]<<8|accel[5])*0.00059814453125f);
//}

//void IO_TOP_demo(void* param)
//{
//	pIICInterface_t iic = &UserII2Dev;

//	#define IO_TOP_Dev 0x20 //IO TOP device address
//	
//	uint8_t buf[2] = { 0,0 };
//	uint8_t readbuf[2] = { 0xAA,0xBB };
//	uint8_t flag=0;
//	iic->write(IO_TOP_Dev << 1, buf, 2, 200); // Turn off all levels
//	
//	for(;;)
//	{
//		flag=!flag;
//		if( flag ) buf[0]=0,buf[1]=0;
//		else buf[0] = 0xff,buf[1]=0xff;
//		
//		printf("wrtie:%d\r\n",iic->write(IO_TOP_Dev<<1,buf,2,200));
//		
//		printf("read:%d\r\n",iic->read(IO_TOP_Dev<<1,readbuf,2,200));
//		printf("0x%2X    0x%2X\r\n",readbuf[0],readbuf[1]);
//	}
//}
#endif


