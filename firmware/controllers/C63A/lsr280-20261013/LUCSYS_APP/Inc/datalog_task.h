#ifndef __DATALOG_TASK_H
#define __DATALOG_TASK_H

#include <stdint.h>

/**
 * @file    datalog_task.h
 * @brief Temporary data observation task: Reporting complete vehicle operation data via serial port 4 (huart4) via CSV text
 *
 * @note
 * - This feature is only for long-term data collection during the research and development phase; for shipped firmware, set DATALOG_ENABLE to 0.
 * - When this feature is enabled, data_task's UART4 transmission is commented out; both cannot use huart4 simultaneously.
 * - All values are reported as fixed-point integers (to avoid floating-point printf); the scaling factors are given in the macro definitions below and the explanation in row H.
 */

//Debug the main switch
#define DATALOG_ENABLE          0

//Reporting cycle: data lines 10Hz, event detection 50Hz, statistics rows every 10 seconds
#define DATALOG_TICK_MS         20      //Task cycle (ms), event detection frequency
#define DATALOG_DATA_DIV        5       //Data line division: 20ms x 5 = 100ms (10Hz)
#define DATALOG_STAT_DIV        500     //Count row division: 20ms x 500 = 10s

//Fixed-point scaling factor (restore on host computer)
#define DATALOG_SCALE_VEL       1000    //Linear velocity m/s - > mm/s
#define DATALOG_SCALE_OMG       1000    //Angular velocity rad/s - > mrad/s
#define DATALOG_SCALE_RPM       10      //Rotational speed: rpm -> 0.1rpm
#define DATALOG_SCALE_CUR       10      //Current A - > 0.1A
#define DATALOG_SCALE_TMP       10      //Temperature degC -> 0.1degC
#define DATALOG_SCALE_VOL       100     //Voltage V -> 0.01V
#define DATALOG_SCALE_ANG       100     //Angle deg -> 0.01deg
#define DATALOG_SCALE_SCL       1000    //Speed-limit ratio -> per mille

/**
 * @brief Control task snapshots
 *
 * RobotControl_task internal local/static variables are not visible to the log task, managed by the control task each time
 * At the end of the cycle, a one-time copy is made to this struct to avoid the log task reading half-new, half-old torn data.
 * The rest of the data (actual steering position/wheel speed/current/temperature/IMU/driver status) is already in the global structure.
 * Read directly from log tasks.
 */
typedef struct{
	uint32_t tick;          //Timestamp (ms) of this control cycle
	uint8_t  reorientHold;  //Redirection deceleration hold gate state
	float    steerScale;    //Actual steering speed limit ratio applied in this cycle (0 = Downward stop)
	int32_t  maxProgress;   //Maximum Steering Counts among the Four Wheels
	float    cmdVx,cmdVy,cmdVz;  //Raw command received from the queue
	float    smVx,smVy,smVz;     //Actual executed value after smoothing the speed limit
	float    stVx,stVy,stVz;     //Limited and smoothed command actually used for steering calculation (SteerCmd)
	uint32_t steerSendCnt;       //set_MotorPos the cumulative number of calls, the difference between adjacent samples is the CAN dispatch rate
	float    steerTgt[4];   //Target steering positions (counts), order A/B/C/D
	float    rpmTgt[4];     //Target hub rotational speeds (rpm), order A/B/C/D
}RobotLogSnap_t;

extern volatile RobotLogSnap_t g_RobotLogSnap;

void DataLog_task(void* param);

#endif
