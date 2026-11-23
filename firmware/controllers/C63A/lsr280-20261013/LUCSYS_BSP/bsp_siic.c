#include "bsp_siic.h"
#include "bsp_dwt.h"

#define userconfig_delayus 1

//陀螺仪软件iic接口配置
static GPIOConfigType_t imu_scl = {
	.pin = IIC_SCL_Pin,
	.port = IIC_SCL_GPIO_Port
};

static GPIOConfigType_t imu_sda = {
	.pin = IIC_SDA_Pin,
	.port = IIC_SDA_GPIO_Port
};


//io口基础操作实现
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


//IIC子协议实现
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
    setOutputMode(io->SDA);   // 设置 SDA 为输出
    setlow(io->SCL);  // 拉低 SCL
    setlow(io->SDA);  // 拉低 SDA
	
    sethigh(io->SCL);  // 将 SCL 置为高电平
	
	delay_us(userconfig_delayus);
	
    sethigh(io->SDA);  // 在 SCL 为高时，将 SDA 拉高生成停止信号
    delay_us(userconfig_delayus);
}

static uint8_t sIIC_WaitAck(sIIC_IOType_t* io,uint32_t timeout)
{
    uint32_t time = 0;

    setInputMode(io->SDA);    // 设置 SDA 为输入
    sethigh(io->SDA);  // 拉高 SDA
	
	delay_us(userconfig_delayus);
	
    sethigh(io->SCL);  // 拉高 SCL，等待应答信号
	
	delay_us(userconfig_delayus);
	
    while (readPins(io->SDA)) {  // 检测从设备是否拉低 SDA，产生ACK信号
        time++;
        delay_us(userconfig_delayus);
        if (time > timeout) {
            sIIC_Stop(io);  // 超时则发送停止信号
            return 0;     // 无应答
        }
    }

    setlow(io->SCL);  // 拉低 SCL，继续传输
    setOutputMode(io->SDA);
    return 1;    // 收到应答
}

static void sIIC_Ack(sIIC_IOType_t* io)
{
    setOutputMode(io->SDA);   // 设置 SDA 为输出
    setlow(io->SCL);  // 拉低 SCL
    setlow(io->SDA);  // 拉低 SDA，发送 ACK
	
	delay_us(userconfig_delayus);

    setlow(io->SDA);
    sethigh(io->SCL);  // 拉高 SCL，确认传输
	
	delay_us(userconfig_delayus);

    setlow(io->SCL);  // 拉低 SCL，结束 ACK 信号
    sethigh(io->SDA);
}

static void sIIC_NAck(sIIC_IOType_t* io)
{
    setOutputMode(io->SDA);   // 设置 SDA 为输出
    setlow(io->SCL);  // 拉低 SCL
    setlow(io->SDA);  // 拉低 SDA，发送 ACK
	
	delay_us(userconfig_delayus);

    sethigh(io->SDA);
    sethigh(io->SCL);  // 拉高 SCL，确认传输
	
	delay_us(userconfig_delayus);

    setlow(io->SCL);  // 拉低 SCL，结束 ACK 信号
    sethigh(io->SDA);
}

static void sIIC_SendByte(sIIC_IOType_t* io,uint8_t byte)
{
    uint8_t i;

    setOutputMode(io->SDA);  // 设置 SDA 为输出
    setlow(io->SCL); // 拉低 SCL

    for (i = 0; i < 8; i++) {
        if (byte & 0x80) {
            sethigh(io->SDA);  // 发送高位
        } else {
            setlow(io->SDA);  // 发送低位
        }
        delay_us(1);
        byte <<= 1;       // 移动到下一位
        sethigh(io->SCL);       // 拉高 SCL，传输当前位
		delay_us(1);
        setlow(io->SCL);       // 拉低 SCL，准备下一位
  
    }
}

static uint8_t sIIC_ReadByte(sIIC_IOType_t* io,uint8_t ack)
{
    uint8_t i, byte = 0;

    setInputMode(io->SDA);  // 设置 SDA 为输入

    for (i = 0; i < 8; i++) {
        setlow(io->SCL);  // 拉低 SCL
        delay_us(1);	
        sethigh(io->SCL);  // 拉高 SCL，读取当前位
		delay_us(1);
        byte <<= 1;
        if (readPins(io->SDA)) {
            byte |= 0x01;  // 读取到高电平
        }
//        delay_us(1);
    }

    if (!ack) {
        sIIC_NAck(io);  // 如果不需要应答，发送 NACK
    } else {
        sIIC_Ack(io);   // 需要应答则发送 ACK
    }

    return byte;
}

//IIC封装协议实现
static IIC_Status_t IIC_Master_Transmit(sIIC_IOType_t* io,uint16_t dev_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start(io);    // 发送起始信号

    sIIC_SendByte(io,dev_addr);        // 发送从设备地址（写模式）
    if (!sIIC_WaitAck(io,timeout)) {          // 等待应答
        sIIC_Stop(io);                // 如果没有应答，则停止通信
        return IIC_TIMEOUT;
    }

    for (uint16_t i = 0; i < size; i++) {
        sIIC_SendByte(io,data[i]);     // 逐字节发送数据
        if (!sIIC_WaitAck(io,timeout)) {      // 每发送一个字节后等待应答
            sIIC_Stop(io);            // 如果没有应答，则停止通信
            return IIC_TIMEOUT;
        }
    }

    sIIC_Stop(io);                    // 发送停止信号
    return IIC_OK;                       // 成功
}

static IIC_Status_t IIC_Master_Receive(sIIC_IOType_t* io,uint16_t dev_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start(io);    // 发送起始信号

    sIIC_SendByte(io,dev_addr | 0x01); // 发送从设备地址（读模式）
    if (!sIIC_WaitAck(io,timeout)) {          // 等待应答
        sIIC_Stop(io);                // 如果没有应答，则停止通信
        return IIC_TIMEOUT;
    }

    for (uint16_t i = 0; i < size; i++) {
        data[i] = sIIC_ReadByte(io,i == (size - 1) ? 0 : 1); // 逐字节读取数据，最后一个字节发送NACK
    }

    sIIC_Stop(io);                    // 发送停止信号
    return IIC_OK;                       // 成功
}


static IIC_Status_t IIC_Mem_Write(sIIC_IOType_t* io,uint16_t dev_addr, uint16_t mem_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start(io);                  // 发送起始信号
    
    sIIC_SendByte(io,dev_addr);       // 发送从设备地址（写模式）
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT; // 等待应答

    sIIC_SendByte(io,mem_addr);       // 发送内存/寄存器地址
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT; // 等待应答

    for (uint16_t i = 0; i < size; i++) {
        sIIC_SendByte(io,data[i]);    // 逐字节发送数据
        if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT; // 等待应答
    }
    
    sIIC_Stop(io);                   // 发送停止信号
    return IIC_OK;                      // 写入成功
}

static IIC_Status_t IIC_Mem_Read(sIIC_IOType_t* io,uint16_t dev_addr, uint16_t mem_addr, uint8_t *data, uint16_t size,uint32_t timeout)
{
    sIIC_Start(io);                   // 发送起始信号
    
    sIIC_SendByte(io,dev_addr);        // 发送从设备地址（写模式）
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT;  // 等待应答

    sIIC_SendByte(io,mem_addr);        // 发送内存/寄存器地址
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT;  // 等待应答
    
    sIIC_Start(io);                    // 重新启动信号，准备读数据
    sIIC_SendByte(io,dev_addr | 0x01); // 发送从设备地址（读模式）
    if (!sIIC_WaitAck(io,timeout)) return IIC_TIMEOUT;  // 等待应答

    for (uint16_t i = 0; i < size; i++) {
        data[i] = sIIC_ReadByte(io,i == (size - 1) ? 0 : 1); // 逐字节读取数据，最后一个字节发送NACK
    }

    sIIC_Stop(io);                    // 发送停止信号
    return IIC_OK;                       // 读取成功
}

static void iic_delayms(uint16_t ms)
{
	delay_ms(ms);
}


//挂载驱动
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




