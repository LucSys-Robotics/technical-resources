#ifndef __BSP_ICM20948_H
#define __BSP_ICM20948_H

#include <stdint.h>

typedef struct{ //Intermediate private variables
	float x;
	float y;
	float z;
}PrivateBuf_t;
	
typedef struct{
	PrivateBuf_t gyro; //3-axis angular velocity
	PrivateBuf_t accel;//3-axis acceleration
	PrivateBuf_t magn;//3-axis magnetometer
}IMU_DATA_t;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}ATTITUDE_DATA_t;


//IMU operation interface
typedef struct {
    uint8_t (*Init)(void);   //Return value: 1-error; 0-no error
    uint8_t (*DeInit)(void); //Return value: 1-error; 0-no error
	
	void (*UpdateZeroPoint_axis)(void);               //Update the zeros of the 9-axis data
	void (*UpdateZeroPoint_attitude)(void);          //Updated posture zero point
    void (*Update_9axisVal)(IMU_DATA_t* imudata,IMU_DATA_t* ori);                         //Update 9-axis data
	void (*UpdateAttitude)(IMU_DATA_t imudata,ATTITUDE_DATA_t *attitude); //Updated attitude angles
	
}IMUInterface_t,*pIMUInterface_t;

extern IMUInterface_t UserICM20948;

//Used to store IMU data
extern IMU_DATA_t axis_9Val;      
extern IMU_DATA_t axis_9ValOri;  
extern ATTITUDE_DATA_t AttitudeVal;

#endif /* __BSP_IMU_H */
