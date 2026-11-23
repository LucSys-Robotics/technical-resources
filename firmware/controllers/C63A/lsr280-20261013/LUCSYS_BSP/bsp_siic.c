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




