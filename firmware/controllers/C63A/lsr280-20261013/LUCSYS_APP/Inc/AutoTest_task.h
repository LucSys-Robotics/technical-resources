#ifndef __AUTOTEST_TASK_H
#define __AUTOTEST_TASK_H

#include <stdint.h>

//On-site automated testing is a R&D facility and shares the same main switch as the data observation system.
//DATALOG_ENABLE=0, this file does not export any symbols, and tasks will not be created —
//In the shipping firmware, customers long-pressing the handle button will not accidentally trigger the test script.
#include "datalog_task.h"

#if DATALOG_ENABLE

/**
 * Heading control internal quantity, used to diagnose oscillations in the translation segment Vz
 *
 * The log task samples at 10Hz while this task runs at 50Hz. Reading instantaneous values directly causes aliasing — this was the case before
 * only saw Vz jumping between +0.30/-0.03, unable to figure out what happened in between.
 * Change to this task: cumulative envelope (vzMin/vzMax) and symbol flip count per beat, called after reading the log
 * AutoTest_DbgWindowReset clears the window so that every tick within 100ms is counted, regardless of the sampling timing.
 */
typedef struct{
	float   yaw0;     //Test coordinate system reference heading (degrees). After starting, it should remain constant; any changes indicate it has been retaken
	float   yawErr;   //Heading error (degrees) for this shot, input for PD
	float   vzRaw;    //The PD output (rad/s) before the current limit to see if it really reaches the limit
	float   vzLast;   //The actual Vz issued in this auction
	float   vzMin;    //Lower envelope of Vz within the window
	float   vzMax;    //Send down the upper envelope of Vz within the window
	uint8_t flipCnt;  //The number of times the Vz symbol flips in the window, a direct indicator of oscillation
	uint8_t seg;      //Current segment index
	uint8_t state;    //State machine state 0 = standby 1 = execute 2 = pause between segments 3 = finish
}AutoTestDbg_t;

extern volatile AutoTestDbg_t g_AutoTestDbg;

//After reading the number of windows from the log task, call to reopen the next statistics window
void AutoTest_DbgWindowReset(void);

//Automated site testing task: Perform cyclical vertical movement/lateral movement/backward/tilt/rotation within a 2.4m x 2.4m site
void AutoTest_task(void* param);

//Request to start a test (triggered by long-pressing the controller upper, can be called in any task/callback)
void AutoTest_Start(void);

//Request to stop testing (triggered by a controller click/double-click)
void AutoTest_Stop(void);

//Query whether the test is running
uint8_t AutoTest_IsRunning(void);

#endif //DATALOG_ENABLE

#endif
