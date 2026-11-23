#include "imu_task.h"

//imu data structure
IMU_DATA_t imu;

void MPU6050_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        //This task runs at 100Hz
        //This task runs at a frequency of 100Hz
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_TASK_RATE));

        //Read the gyroscope zero before starting
        //Before powering on, read the gyroscope zero
        if(SysVal.Time_count<CONTROL_DELAY)
        {	
            ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
            ImuData_copy(&imu.Deviation_accel,&imu.accel);
        }
        //Get acceleration sensor data
        MPU6050_Get_Accelscope();

        //Get gyroscope data
        MPU6050_Get_Gyroscope(); //Get gyroscope data

    }
}

void ICM20948_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        //This task runs at 100Hz
        //This task runs at a frequency of 100Hz
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_TASK_RATE));

        //Read the gyroscope zero before starting
        //Before powering on, read the gyroscope zero
        if(SysVal.Time_count<CONTROL_DELAY)
        {
            ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
            ImuData_copy(&imu.Deviation_accel,&imu.accel);
        }

        //Get acceleration sensor data
        ICM20948_Get_Accel(); //Get accelerometer data

        //Get gyroscope data
        ICM20948_Get_Gyroscope(); //Get gyroscope data

#if 0 // No magnetometer data is used, so do not enable collection
        static u8 mag_count=0;
        //Get magnetometer data
        mag_count++;
        if(mag_count>=13)	 //The maximum sampling rate of the magnetometer is 8Hz, so the magnetometer information is sampled only once every 13 times this function is entered
        {
            invMSMagRead(&magnet[0], &magnet[1], &magnet[2]); //Get magnetometer data
            mag_count=0;
        }
#endif
    }
}


/**************************************************************************
Function: Copy the value of the IMU
Entry parameters: the structure variable to be assigned and the structure variable to be copied
Return value: none
Author: LUCSYS
**************************************************************************/
void ImuData_copy(IMU_BASE_t* req_val,const IMU_BASE_t* copied_val)
{
	req_val->x = copied_val -> x;
	req_val->y = copied_val -> y;
	req_val->z = copied_val -> z;
}


