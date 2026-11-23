#include "bsp_icm20948.h"

//ICM20948 Registers
#include "icm20948_reg.h"

//imu requires iic support package
#include "bsp_siic.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

//When publishing data, critical section is needed, see ImuUpdate
#include "FreeRTOS.h"
#include "task.h"

//Used to store IMU data
IMU_DATA_t axis_9Val = { 0 };      
IMU_DATA_t axis_9ValOri = { 0 };  
ATTITUDE_DATA_t AttitudeVal = { 0 };

static uint8_t ICM20948_Init(void)
{
	pIICInterface_t iicdev = &UserIMU_sIICDev;//Initialize the IIC device
	
	IIC_Status_t check_state = IIC_OK;//Check if the initialization step is error
	
	uint8_t writebuf = 0;
	
	//Select bank0
	writebuf = REG_VAL_SELECT_BANK_0;
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,REG_BANK_SEL,&writebuf,1,500);
	
	//Access the icm20948 WHO AM I register and check if the device is correct
	check_state += iicdev -> read_reg(&iicdev->iic_io,ICM20948_DEV<<1,WHO_AM_I,&writebuf,1,500);
	if( 0xEA != writebuf ) return 1;//Data error
	
	//Operate PWR1 to reset ICM20948
	writebuf = (1<<7);
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,PWR_MGMT_1,&writebuf,1,500);
	iicdev -> delay_ms(100); //Delay waiting for the reset to complete
	
	//Configure user_ctrl
	// DMP not enabled: 0<<7
	// FIFO not enabled: 0<<6
	// Do not enable the internal IIC reading of ICM20948 0<<5
	// Non-reset slave IIC: 0 << 4
	// No reset DMP: 0<<3
	// Do not reset SRAM modules: 0 << 2
	// No reset of the IIC host: 0 << 1
	writebuf = 0x00;
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,USER_CTRL,&writebuf,1,500);
	
	//Turn off low power mode, enable temperature sensor, and set automatic selection of the optimal clock source
	writebuf = 0x01;
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,PWR_MGMT_1,&writebuf,1,100);
	
	//Select bank2
	writebuf = REG_VAL_SELECT_BANK_2;
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,REG_BANK_SEL,&writebuf,1,100);
	
	//Set frequency The frequency of GYRO data output (ODR: output data rate = 1.1KHz / 1 + configuration value)
	writebuf = 0x07; //Set GYRO's sampling frequency to 1.1KHz (requires FCHOICE=1 to enable filtering, DLPF_CFG=1~7 to configure filtering information)
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,GYRO_SMPLRT_DIV,&writebuf,1,100);
	
	//Configure GYRO_CONFIG_1, DLPCFG, range and DLFP
	//FS_SEL[2:1] Gyroscope range setting: (3<<1: ¡À2000dps 2<<1: ¡À1000dps 1<<1: ¡À500dps 0<<1: ¡À250dps)
	//FCHOICE[0] filter enable: 1 --> enable filtering
	//DLPFCFG [5:3] Filtering configuration: 6 (3DB BW: 11.6Hz NBW: 17.8Hz) 3DB BW: higher frequency, faster gyroscope response, but noise may increase. NBW: lower frequency, more stable data
	writebuf = (1<<1) | (1<<0) | (6<<3);
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,GYRO_CONFIG_1,&writebuf,1,100);
	
	//Set frequency ACCEL_SMPLRT_DIV_2 (output frequency configuration: 1.125kHz / (1 + configuration value))
	writebuf = 0x07;
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,ACCEL_SMPLRT_DIV_2,&writebuf,1,100);
	
	//ACCEL_DLPFCFG [5:3] Filter Configuration:
	//FS_SEL[2:1] Accelerometer range: (3<<1: ¡À16g 2<<1: ¡À8g 1<<1: ¡À4g 0<<1: ¡À2g)
	//FCHOICE[0] filter enable bit:
	writebuf = (0<<1) | (1<<0) | (6<<3);
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,ACCEL_CONFIG,&writebuf,1,100);
	
	//Switch back to bank0
	writebuf = REG_VAL_SELECT_BANK_0;
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,REG_BANK_SEL,&writebuf,1,100);
	
	//Configure magnetometer
	
	//Initiates the ICM20948's bypass mode, allowing direct access to the magnetometer via the IIC bus without passing through the ICM20948
	writebuf = (1<<1);
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,INT_PIN_CFG,&writebuf,1,100);
	
	//Access the magnetometer to confirm the device is correct
	check_state += iicdev -> read_reg(&iicdev->iic_io,AK09916_DEV<<1,WIA,&writebuf,1,100);
	if( 0x09 != writebuf ) return 1;//Data error returned
	
	/* Configure magnetometer output frequency
	doc:
	0: Turn off output
	1<<0: Single measurement mode, automatically returns to closed output mode after measurement
	1<<1: Continuous measurement mode 1, frequency 10Hz
	1<<2: Continuous measurement mode 2, frequency 20Hz
	(1<<2)| (1<<1): Continuous measurement mode 3, frequency 50Hz
	1<<3: Continuous measurement mode 4, frequency 100Hz
	*/
	writebuf = (1<<3);//Configure magnetometer for 100Hz output
	check_state += iicdev -> write_reg(&iicdev->iic_io,AK09916_DEV<<1,CNTL2,&writebuf,1,100);
	//Magnetometer Range Description: ¡À4900 uT, 16-bit resolution
	
	//Error during initialization
	if( 0 != check_state ) return 1;
	
	return 0;
}

static uint8_t ICM20948_DeInit(void)
{
	pIICInterface_t iicdev = &UserIMU_sIICDev;//Initialize the IIC device
	
	IIC_Status_t check_state = IIC_OK;//Check if the initialization step is error
	
	uint8_t writebuf = 0;
	
	//Select bank0
	writebuf = REG_VAL_SELECT_BANK_0;
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,REG_BANK_SEL,&writebuf,1,500);
	
	//Access the icm20948 WHO AM I register and check if the device is correct
	check_state += iicdev -> read_reg(&iicdev->iic_io,ICM20948_DEV<<1,WHO_AM_I,&writebuf,1,500);
	if( 0xEA != writebuf ) return 1;//Data error
	
	//Set the icm20948 to sleep mode
	writebuf = (1<<6);
	check_state += iicdev -> write_reg(&iicdev->iic_io,ICM20948_DEV<<1,PWR_MGMT_1,&writebuf,1,500);
	
	//An error occurred during the process
	if( 0 != check_state ) return 1;
	
	return 0;
}


static IMU_DATA_t ZeroPoint = { 0 };//Zero point
static ATTITUDE_DATA_t ZeroAttitude = { 0 };

//Internal data for auxiliary zero calibration
static IMU_DATA_t priv_imu = { 0 };
static ATTITUDE_DATA_t priv_attit = { 0 };

/**
 * Read 9-axis data and convert it into physical units
 *
 * Calculate on local variables throughout the process, then publish it all at once to *data / *ori.
 * Cannot be rewritten on the spot ¡ª the conversion must be done in three steps (original count - > dps - > rad/s), with one insert in between
 * Blocking magnetometer IIC read; writing directly to globals would let other tasks read partially updated data:
 * It could be an ADC count of ¡À32768, or a dps value of ¡À500, yet it is used as rad/s.
 * In actual tests, I encountered this: Vz's PD output shot up to 583 (should be in the 0.04 order), and was cut off by the limiter to full rudder.
 * Straight Path Assist (Linear_IMUHelp, D gain 0.15) and automatic heading maintenance both use this amount.
 */
static void ImuUpdate(IMU_DATA_t *data,IMU_DATA_t *ori)
{
	pIICInterface_t iicdev = &UserIMU_sIICDev;//Specify the IIC device

	//Start with the current value: If the magnetometer reading is unreliable, use the previous data
	IMU_DATA_t d = *data;

	//Obtain the raw values of the 6-axis data
	uint8_t tmpbuf[12];
	iicdev -> read_reg(&iicdev->iic_io,ICM20948_DEV<<1,ACCEL_XOUT_H,tmpbuf,12,100);

	//Obtain the original value of acceleration along the 3 axes
	d.accel.x = (short)(tmpbuf[0]<<8 | tmpbuf[1]);
	d.accel.y = (short)(tmpbuf[2]<<8 | tmpbuf[3]);
	d.accel.z = (short)(tmpbuf[4]<<8 | tmpbuf[5]);

	//Obtain the original angular velocity value on 3 axes
	d.gyro.x = (short)(tmpbuf[6]<<8 | tmpbuf[7]);
	d.gyro.y = (short)(tmpbuf[8]<<8 | tmpbuf[9]);
	d.gyro.z = (short)(tmpbuf[10]<<8 | tmpbuf[11]);

	//Obtain 3-axis magnetometer data
	uint8_t magnbuf[8]; //byte[6] is invalid, address 0x17 no data. byte[7] is register ST2, address 0x18. To improve efficiency, continuous reading is used
	iicdev->read_reg(&iicdev->iic_io,AK09916_DEV<<1,HXL,magnbuf,8,100);//Read magnetometer data

	uint8_t mangbit = (magnbuf[7]>>3)&0x01;
	if( 0 == mangbit )
	{
		//Obtain the original values of the magnetometer on the 3-axis axis
		d.magn.x = (short)(magnbuf[1]<<8 | magnbuf[0]);
		d.magn.y = (short)(magnbuf[3]<<8 | magnbuf[2]);
		d.magn.z = (short)(magnbuf[5]<<8 | magnbuf[4]);
	}
	else
	{
		//Data is unreliable, not processed, and the previous data is retained
	}

	//Zero calibration
	d.gyro.x -= ZeroPoint.gyro.x;
	d.gyro.y -= ZeroPoint.gyro.y;
	d.gyro.z -= ZeroPoint.gyro.z;

	d.accel.x -= ZeroPoint.accel.x;
	d.accel.y -= ZeroPoint.accel.y;
	d.accel.z -= ZeroPoint.accel.z + 0;

//	d.magn.x -= ZeroPoint.magn.x;
//	d.magn.y -= ZeroPoint.magn.y;
//	d.magn.z -= ZeroPoint.magn.z;

	//At this moment, d is the original value after zero correction, i.e., the content of ori
	IMU_DATA_t raw = d;

	//Convert units to m/s^2. Range: plus or minus 2g, resolution 2^16
	d.accel.x *= 0.00059814453125f;
	d.accel.y *= 0.00059814453125f;
	d.accel.z *= 0.00059814453125f;

	//Convert units to rad/s. Range: ¡À500 dps, resolution 2^16
	d.gyro.x *= 0.0152587890625f;//Unit dps
	d.gyro.y *= 0.0152587890625f;
	d.gyro.z *= 0.0152587890625f;

	d.gyro.x *= 0.01745329252f;//unit: rad/s
	d.gyro.y *= 0.01745329252f;
	d.gyro.z *= 0.01745329252f;

	if( mangbit==0 )
	{
		//Convert units to uT. Range: ¡À4900 uT, resolution 2^16
		d.magn.x *= 0.1495361328125f;
		d.magn.y *= 0.1495361328125f;
		d.magn.z *= 0.1495361328125f;
	}

	//Private data copy for zero marking
	priv_imu = raw;

	//One-time release: Readers either get the full set of old values or the full set of new values, without reading the intermediate state
	taskENTER_CRITICAL();
	*ori  = raw;
	*data = d;
	taskEXIT_CRITICAL();
}

//Methods for calibrating zeros
static void setZeroPoint_axis(void) 
{
	//Clear the current zero data first
	priv_imu.gyro.x += ZeroPoint.gyro.x;
	priv_imu.gyro.y += ZeroPoint.gyro.y;
	priv_imu.gyro.z += ZeroPoint.gyro.z;
	
	priv_imu.accel.x += ZeroPoint.accel.x;
	priv_imu.accel.y += ZeroPoint.accel.y;
	priv_imu.accel.z += ZeroPoint.accel.z;
	priv_imu.accel.z -= 16384; //16384 corresponds to 9.8 m/s^2 at ¡À2g scale
	
	memcpy(&ZeroPoint,&priv_imu,sizeof(IMU_DATA_t));
}

static void setZeroPoint_attitude(void)
{
	priv_attit.pitch += ZeroAttitude.pitch;
	priv_attit.roll += ZeroAttitude.roll;
	priv_attit.yaw += ZeroAttitude.yaw;
	memcpy(&ZeroAttitude,&priv_attit,sizeof(ATTITUDE_DATA_t));
}

static const float Kp = 0.8;   // Proportionality coefficient
static const float Ki = 0.0001; // Integral coefficients

//Quick calculation 1/sqrt(x)
static float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
	return y;
} 

static void attitudeUpdate(IMU_DATA_t imudata, ATTITUDE_DATA_t *attitude) // Display complementary filtering (gyroscope + accelerometer)
{
    static float eInt[3] = {0}; // Initialize cumulative error
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Initialize quaternions
    float norm;
    float vx, vy, vz;
    float ex, ey, ez; // Error variable
	float q0_t,q1_t,q2_t,q3_t;
	
	float halftime = 0.01f * 0.5f; // 10ms sampling cycle
	
	
	// Helper variables (shorter variable names)
	float gx = imudata.gyro.x;  // Angular velocity (rad/s) rotating around the X-axis
	float gy = imudata.gyro.y;  // Angular velocity (rad/s) of rotation around the Y-axis
	float gz = imudata.gyro.z;  // Angular velocity around the Z-axis (rad/s)
	float ax = imudata.accel.x; // Acceleration along the X-axis (m¡¤s^-2)
	float ay = imudata.accel.y; // Acceleration along the Y-axis (m¡¤s^-2)
	float az = imudata.accel.z; // Acceleration along the Z-axis (m¡¤s^-2)

    // Helper variables (avoid long calculation expressions)
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
	
    // Extract the gravity component from the equivalent rotation matrix
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
	vz = 1-2*(q1q1+q2q2);
	
	//Accelerometer normalization
	norm = Q_rsqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Calculation error
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // Cumulative error
    eInt[0] += ex;
    eInt[1] += ey;
    eInt[2] += ez;

    // PI control
    gx += Kp * ex + Ki * eInt[0];
    gy += Kp * ey + Ki * eInt[1];
    gz += Kp * ez + Ki * eInt[2];

    // Update quaternions
    q0_t =  (-q1 * gx - q2 * gy - q3 * gz) * halftime;
    q1_t =  ( q0 * gx + q2 * gz - q3 * gy) * halftime;
    q2_t =  ( q0 * gy - q1 * gz + q3 * gx) * halftime;
    q3_t =  ( q0 * gz + q1 * gy - q2 * gx) * halftime;
	
	q0+=q0_t;
	q1+=q1_t;
	q2+=q2_t;
	q3+=q3_t;
	
    // Normalization (quaternions)
    norm = Q_rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

	// Convert quaternions to Euler angles
	attitude -> roll  = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.296f;
	attitude -> pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.296f;
	attitude -> yaw   = atan2(2 *(q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.296f;

	// Zero-drift correction
	attitude -> roll  -= ZeroAttitude.roll ;
	attitude -> pitch -= ZeroAttitude.pitch ;
	attitude -> yaw   -= ZeroAttitude.yaw;
	
	//Normalize angles
	if( attitude->yaw > 180.0f )   attitude -> yaw -= 360.0f;
	if( attitude->yaw < -180.0f )  attitude -> yaw += 360.0f;
	
	memcpy(&priv_attit,attitude,sizeof(ATTITUDE_DATA_t));

}


//Attach driver
IMUInterface_t UserICM20948 = {
	.Init = ICM20948_Init,
	.DeInit = ICM20948_DeInit,
	.UpdateZeroPoint_axis = setZeroPoint_axis,
	.UpdateZeroPoint_attitude = setZeroPoint_attitude,
	.Update_9axisVal = ImuUpdate,
	.UpdateAttitude = attitudeUpdate
};


