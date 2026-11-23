#include "MPU6050.h"
#include "imu_task.h"
#include "I2C.h"

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Ax, Ay, Az: The original reading of the accelerometer on the x,y, and z axes (plus or minus)
Output  : 0: success, others: error code
Function: obtain acceleration value (raw value)
**************************************************************************/
void MPU6050_Get_Accelscope(void)
{
    imu.accel.x =(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //Read the X-axis accelerometer
    imu.accel.y =(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //Read the X-axis accelerometer
    imu.accel.z =(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //Read the Z-axis accelerometer

    if(SysVal.Time_count<CONTROL_DELAY) // 10 seconds before starting // 10 seconds before startup
    {
		
    }
    else //10 seconds after starting // 10 seconds after booting
    {
        //Save the raw data to update zero by clicking the user button
        //Save raw data for updating zero points with a single user key
		imu.Original_accel.x = imu.accel.x;
		imu.Original_accel.y = imu.accel.y;
		imu.Original_accel.z = imu.accel.z;

        //Removes zero drift data
        //Remove data from zero drift
        imu.accel.x = imu.Original_accel.x - imu.Deviation_accel.x ;
        imu.accel.y = imu.Original_accel.y - imu.Deviation_accel.y ;
        imu.accel.z = imu.Original_accel.z - imu.Deviation_accel.z + 16384;
    }

}

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Gx, Gy, Gz: raw readings (plus or minus) of the x,y, and z axes of the gyroscope
Output  : 0: success, others: error code
Function: Obtain gyroscope values (raw values)
**************************************************************************/
void MPU6050_Get_Gyroscope(void)
{
    imu.gyro.x = (I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //Read the X-axis gyroscope
    imu.gyro.y = (I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //Read the Y-axis gyroscope
    imu.gyro.z = (I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //Read the Z-axis gyroscope

    if(SysVal.Time_count<CONTROL_DELAY) // 10 seconds before starting // 10 seconds before startup
    {

    }
    else //10 seconds after starting // 10 seconds after booting
    {

        //Save the raw data to update zero by clicking the user button
        //Save raw data for updating zero points with a single user key
		imu.Original_gyro.x = imu.gyro.x;
		imu.Original_gyro.y = imu.gyro.y;
		imu.Original_gyro.z = imu.gyro.z;

        //Removes zero drift data
        //Remove data from zero drift
		imu.gyro.x = imu.Original_gyro.x - imu.Deviation_gyro.x;
		imu.gyro.y = imu.Original_gyro.y - imu.Deviation_gyro.y;
		imu.gyro.z = imu.Original_gyro.z - imu.Deviation_gyro.z;
    }
}

/**************************************************************************
Function: The new ADC data is updated to FIFO array for filtering
Input   : ax£¬ay£¬az£ºx£¬y, z-axis acceleration data£»gx£¬gy£¬gz£ºx. Y, z-axis angular acceleration data
Output  : none
Function: Updates new ADC data to the FIFO array for filtering
Entry parameters: ax, ay, az: acceleration data along the x, y, z axes; gx, gy, gz: angular acceleration data along the x, y, z axes
Return value: none
**************************************************************************/
int16_t  MPU6050_FIFO[6][11];
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
    unsigned char i ;
    int32_t sum=0;
    for(i=1; i<10; i++) {	//FIFO operation
        MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
        MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
        MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
        MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
        MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
        MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
    }
    MPU6050_FIFO[0][9]=ax;//Place new data at the end of the data
    MPU6050_FIFO[1][9]=ay;
    MPU6050_FIFO[2][9]=az;
    MPU6050_FIFO[3][9]=gx;
    MPU6050_FIFO[4][9]=gy;
    MPU6050_FIFO[5][9]=gz;

    sum=0;
    for(i=0; i<10; i++) {	//Find the sum of the current array, then take the average
        sum+=MPU6050_FIFO[0][i];
    }
    MPU6050_FIFO[0][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[1][i];
    }
    MPU6050_FIFO[1][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[2][i];
    }
    MPU6050_FIFO[2][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[3][i];
    }
    MPU6050_FIFO[3][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[4][i];
    }
    MPU6050_FIFO[4][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[5][i];
    }
    MPU6050_FIFO[5][10]=sum/10;
}

/**************************************************************************
Function: Setting the clock source of mpu6050
Input   : source£ºClock source number
Output  : none
Function: Set the clock source for MPU6050
Entry parameter: source: clock source number
Return value: none
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
**************************************************************************/
void MPU6050_setClockSource(uint8_t source) {
    I2C_WriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    I2C_WriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Setting the maximum range of mpu6050 accelerometer
Input   : range£ºAcceleration maximum range number
Output  : none
Function: Set the maximum range of the MPU6050 accelerometer
Entry parameters: range: Maximum acceleration range number
Return value: none
**************************************************************************/
//#define MPU6050_ACCEL_FS_2 0x00 //Max range ¡À2G
//#define MPU6050_ACCEL_FS_4 0x01 //Max range ¡À4G
//#define MPU6050_ACCEL_FS_8 0x02 //Max range ¡À8G
//#define MPU6050_ACCEL_FS_16 0x03 //Max range ¡À16G
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    I2C_WriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Set mpu6050 to sleep mode or not
Input   : enable£º1£¬sleep£»0£¬work£»
Output  : none
Function: Set whether MPU6050 enters sleep mode
Entry parameters: enable: 1, sleep; 0, work;
Return value: none
**************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************************************************************
Function: Read identity
Input   : none
Output  : 0x68
Function: Read MPU6050 WHO_AM_I identifier
Entry parameters: None
Return value: 0x68
**************************************************************************/
uint8_t MPU6050_getDeviceID(void) {
    return I2C_ReadOneByte(devAddr,MPU6050_RA_WHO_AM_I);
}

/**************************************************************************
Function: Check whether mpu6050 is connected
Input   : none
Output  : 1£ºConnected£»0£ºNot connected
Function: Detect whether the MPU6050 is connected
Entry parameters: None
Return value: 1-connected; 0-disconnected
**************************************************************************/
uint8_t MPU6050_testConnection(void) {
    if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
        return 1;
    else return 0;
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable£º1£¬yes£»0;not
Output  : none
Function: Set whether MPU6050 is the host for the AUX I2C line
Entry parameters: enable: 1, yes; 0: no
Return value: none
**************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable£º1£¬yes£»0;not
Output  : none
Function: Set whether MPU6050 is the host for the AUX I2C line
Entry parameters: enable: 1, yes; 0: no
Return value: none
**************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************************************************************
Function: Read mpu6050 built-in temperature sensor data
Input   : none
Output  : Centigrade temperature
Function: Read data from MPU6050 built-in temperature sensor
Entry parameters: None
Return value: temperature in Celsius
**************************************************************************/
int Read_Temperature(void)
{
    float Temp;
    Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
    if(Temp>32768) Temp-=65536;	//Data type conversion
    Temp=(36.53f+Temp/340)*10;	  //Store at ten times the temperature
    return (int)Temp;
}

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : LPF: Digital low-pass filtering frequency (Hz)
Output  : 0: Settings successful, others: Settings failed
Function: Set the digital low-pass filter for MPUrobot_select_init.h
Input parameters: lpf: Digital low-pass filter frequency (Hz)
Return value: 0-configuration succeeded; otherwise, configuration failed
**************************************************************************/
unsigned char MPU6050_Set_LPF(u16 lpf)
{
    u8 data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return I2C_WriteOneByte(devAddr,MPU6050_RA_CONFIG,data); //Set the digital lowpass filter
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : rate:4~1000(Hz)
Output  : 0: Settings successful, others: Settings failed
Function: Set the sampling rate for MPUrobot_select_init.h (assuming Fs = 1KHz)
Input parameters: rate: 4~1000 (Hz)
Return value: 0-configuration succeeded; otherwise, configuration failed
**************************************************************************/
unsigned char MPU6050_Set_Rate(u16 rate)
{
    u8 data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=I2C_WriteOneByte(devAddr,MPU6050_RA_SMPLRT_DIV,data);	//Set the digital lowpass filter
    return MPU6050_Set_LPF(rate/2);	//Automatically sets LPF to half of the sampling rate // Automatically sets LPF to half of the sampling rate
}

/**************************************************************************
Function: initialization Mpu6050 to enter the available state
Input   : none
Output  : none
Function: Initialize MPU6050 to enter usable state
Entry parameters: None
Return value: none
**************************************************************************/
u8 MPU6050_initialize(void) {
//    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //Set clock
//    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000); //Gyro range setting
//    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2); //Max acceleration range ¡À2G
//    MPU6050_setSleepEnabled(0); //Enter working state
//	  MPU6050_setI2CMasterModeEnabled(0); //Prevent MPU6050 from controlling AUXI2C
//	  MPU6050_setI2CBypassEnabled(0); //Disable bypass of MCU I2C to MPU6050 AUXI2C

    u8 res;
    //IIC_Init(); // Initialize the IIC bus
    I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X80);	//Reset MPUrobot_select_init.h // Reset MPUrobot_select_init.h
    delay_ms(200); //Delay 200 ms // Delay 200 ms
    I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X00);	//Wake mpurobot_select_init.h // Wake up MPUrobot_select_init.h

    //MPU6050_Set_Gyro_Fsr(1); //Gyroscope sensor, ¡À500dps = ¡À500¡ã/s ¡À32768 (gyro/32768*500)*PI/180(rad/s) = gyro/3754.9(rad/s)
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    //MPU6050_Set_Accel_Fsr(0); //Acceleration sensor, ¡À2g = ¡À2*9.8m/s^2 ¡À32768 accel/32768*19.6 = accel/1671.84
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    MPU6050_Set_Rate(50);			//Set the sampling rate to 50Hz // Set sampling rate to 50Hz

    I2C_WriteOneByte(devAddr,MPU6050_RA_INT_ENABLE,0X00);	  //Turn off all interrupts // Turn off all interrupts
    I2C_WriteOneByte(devAddr,MPU6050_RA_USER_CTRL,0X00);	//The I2C main mode is off //I2C main mode off
    I2C_WriteOneByte(devAddr,MPU6050_RA_FIFO_EN,0X00);	  //Close the FIFO // Close the FIFO
    //The INT pin is low, enabling bypass mode to read the magnetometer directly
    //INT pin is active at low level; when bypass mode is enabled, the magnetometer can be read directly
    I2C_WriteOneByte(devAddr,MPU6050_RA_INT_PIN_CFG,0X80);
    //Read the ID of MPU6050
    //Read the ID of the MPU6050
    res=I2C_ReadOneByte(devAddr,MPU6050_RA_WHO_AM_I);
    if(res==MPU6050_DEFAULT_ADDRESS) //The device ID is correct, the correct device ID depends on the AD pin // Device ID is correct; correct ID depends on AD pin
    {
        I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X01);	//Set CLKSEL, PLL X axis as reference // Set CLKSEL, PLL X axis as reference
        I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_2,0X00);	//Both Acceleration and gyroscope work
        MPU6050_Set_Rate(50);	                      //Set the sampling rate to 50Hz // Set sampling rate to 50Hz
    } else return 1;
    return 0;
}


//------------------End of File----------------------------
