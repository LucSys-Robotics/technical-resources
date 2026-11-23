#ifndef __DRIVER_D50A_H
#define __DRIVER_D50A_H

#include "system.h"

/*
 * D50A drive management module
 *
 * This module centrally manages D50A drive communication on USART2/RS485.
 * Other modules should not send D50A protocol frames directly to serial port 2 to avoid multiple tasks competing for the 485 bus at the same time.
 * If drive parameters need to be modified, the D50A_PostXXX() series interface should be called to deliver the request to the D50A_Task.
 * If you need to check drive status, read the D50A_State_t cache instead of actively sending read commands.
 *
 * Precautions for multi-driver series:
 * 1. Each driver on the bus must have a unique address.
 * 2. Broadcast address 0xFF is only suitable when there is only one drive on the bus.
 * 3. When multiple drives are connected simultaneously to the 485 bus, do not broadcast the address lookup, otherwise multiple devices will reply simultaneously, causing bus conflicts.
 */

#define D50A_TASK_PRIO       2
#define D50A_STK_SIZE        256
#define D50A_TASK_RATE       10

#define D50A_DEFAULT_ADDR    0x01
#define D50A_BROADCAST_ADDR  0xFF
#define D50A_MAX_DEVICES     4

/* Driver communication result codes: submission interfaces return these values, and the state cache records the latest error. */
typedef enum
{
	D50A_OK = 0,
	D50A_ERR_TIMEOUT,
	D50A_ERR_FRAME,
	D50A_ERR_FUNC,
	D50A_ERR_PARAM,
	D50A_ERR_EXCEPTION,
	D50A_ERR_QUEUE_FULL,
	D50A_ERR_NOT_INIT,
}D50A_Result_t;

/* D50A_Task Types of requests handled internally. */
typedef enum
{
	D50A_REQ_WRITE_OVERCURRENT = 0,
	D50A_REQ_WRITE_OVERTIME,
	D50A_REQ_RESET_PARAM,
	D50A_REQ_WRITE_ADDR,
	D50A_REQ_SAVE,
	D50A_REQ_READ_ADDR,
}D50A_RequestType_t;

/*
 * Submit request objects to the D50A_Task queue.
 *
 * m1_value/m2_value Use practical units that are easier to understand from the upper levels:
 * - When writing overcurrent threshold: units are mA, for example, 8000 means 8000mA
 * - When writing over flow time: units are ms, for example, 5000 means 5000ms
 *
 * save_after_write Only valid for writing to parameters B0/B1:
 * - 0: Write RAM only, do not save immediately
 * - 1: Automatically append B4 save command after successful write
 */
typedef struct
{
	D50A_RequestType_t type;
	uint8_t addr;
	uint8_t new_addr;
	uint16_t m1_value;
	uint16_t m2_value;
	uint8_t save_after_write;
}D50A_Request_t;

/*
 * Alarm position analysis results.
 *
 * The meaning of alarm bits in the D50A protocol is easier to write backwards:
 * - A bit value of 1 indicates normal
 * - A bit value of 0 indicates a corresponding protection trigger
 */
typedef struct
{
	uint8_t undervoltage_ok;
	uint8_t overvoltage_ok;
	uint8_t overtemp_ok;
	uint8_t overcurrent_ok;
}D50A_AlarmBits_t;

/*
 * Status cache corresponding to a single D50A address.
 *
 * D50A_Task periodically reads drive data and updates the structure.
 * When Bluetooth apps, OLED menus, or other modules need to display status, simply read the cache directly.
 * It is not recommended to bypass D50A_Task to directly access USART2 through these modules.
 */
typedef struct
{
	uint8_t enable;              // Whether to enable the device slot: 0 = do not poll, 1 = participate in polling
	uint8_t addr;                // Driver address
	uint8_t online;              // Whether the most recent communication was successful: 1 = online, 0 = offline or timeout
	uint8_t last_error;          // Error code read in the most recent cycle
	uint8_t last_request;        // The type of last write request processed
	uint8_t last_request_error;  // The execution result of the most recent write request

	uint8_t alarm;               // Original alarm code
	D50A_AlarmBits_t alarm_bits; // Result of bitwise parsing of the alarm code
	float m1_current_a;          // M1 real-time current, unit A
	float m2_current_a;          // M2 Real-time current, unit A
	int16_t m1_current_raw;      // M1 real-time current raw value, protocol value = actual current A * 100
	int16_t m2_current_raw;      // M2 real-time current raw value, protocol value = actual current A * 100

	uint8_t m1_temp;             // M1 temperature, unit Celsius
	uint8_t m2_temp;             // M2 Temperature, unit Celsius

	uint16_t m1_overcurrent_ma;  // M1 overcurrent threshold set value, unit mA
	uint16_t m2_overcurrent_ma;  // M2 overcurrent threshold setting, unit mA
	uint16_t m1_overtime_ms;     // M1 overcurrent time set value, unit ms
	uint16_t m2_overtime_ms;     // M2 Overcurrent time setting, unit ms

	uint32_t last_update_tick;   // FreeRTOS tick for the most recent successful communication

	uint32_t comm_success_count;    // Cumulative communication successes: Increases after receiving valid reply frames and completing protocol verification
	uint32_t comm_timeout_count;    // Cumulative communication timeout: increases if no complete valid reply is received within the timeout after sending the command
	uint32_t comm_frame_err_count;  // Cumulative number of data errors returned: increments when frame format errors, abnormal frames, address/function code/data mismatches
}D50A_State_t;


/*
================================================================================
D50A API call examples and considerations
================================================================================

1. General principles
--------------------------------------------------------------------------------
1. Serial 2/485 bus is managed centrally by this module, other modules should not directly send D50A protocol frames.
2. If you want to write drive parameters before FreeRTOS starts, use the D50A_SyncXXX() synchronization interface.
3. After FreeRTOS is running normally, it is recommended to use the D50A_PostXXX() asynchronous interface to hand over requests to D50A_Task.
4. When reading current, temperature, alarm, and parameter setting values, there is no need to actively send read commands; the D50A_State_t cache is read directly.
5. When multiple drives are connected in series, each drive must have a unique address, such as 0x01, 0x02, 0x03.
6. Broadcast address 0xFF can only be used to query addresses when there is only one drive on the bus; do not broadcast queries when there are multiple drives.

Second, hardware initialization stage: synchronously write initial parameters
--------------------------------------------------------------------------------
Applicable scenarios:
- FreeRTOS scheduler has not yet started.
- Initial values such as overcurrent threshold and overcurrent time need to be written to the drive during the systemInit() phase.
- Do not use D50A_PostXXX() at this time, as D50A_Task is not yet running.

Prerequisites:
- UART2_Init (115200) has been completed.
- D50A_Init() completed.
- USART2 receives interrupts enabled, and the interrupt function calls D50A_RxByte().

Example: Set default parameters for the drive at address 0x01 and save immediately.

    UART2_Init(115200);
    D50A_Init();

    // Set the M1/M2 overcurrent threshold to 8000mA and save automatically.
    // The fourth parameter save=1 means that after a successful write, the B4 save command is automatically sent.
    D50A_SyncWriteOverCurrent(0x01, 8000, 8000, 1);

    // Set M1/M2 overcurrent time to 5000ms and save automatically.
    D50A_SyncWriteOverTime(0x01, 5000, 5000, 1);

Example: Write multiple parameters consecutively and save them all together.

    D50A_SyncWriteOverCurrent(0x01, 8000, 8000, 0);
    D50A_SyncWriteOverTime(0x01, 5000, 5000, 0);
    D50A_SyncSave(0x01);

Notes:
- The synchronization interface blocks waiting for drive return; single commands default to a maximum wait of D50A_TIMEOUT_MS.
- Synchronous interfaces are suitable for initialization phases; It is not recommended to call them again after the system is running to avoid accessing the 485 bus simultaneously with D50A_Task.
- Before modifying parameters, it is recommended to ensure the motor is stopped.

3. FreeRTOS runtime: Asynchronous write requests
--------------------------------------------------------------------------------
Applicable scenarios:
- Runtime modification parameters for Bluetooth apps, OLED menus, button menus, etc.
- Do not want the caller to block waiting for a serial reply.
- It is expected that all write operations are queued by D50A_Task to avoid conflicts on the 485 bus.

Example: The Bluetooth app sets the overcurrent threshold at address 0x02 to 8000mA and saves it.

    uint8_t ret;
    ret = D50A_PostWriteOverCurrent(0x02, 8000, 8000, 1);
    if(ret != D50A_OK)
    {
        // This only means the request could not be queued, for example because the queue is full.
        // does not indicate a drive execution failure.
    }

Example: runtime writes over flow time.

    D50A_PostWriteOverTime(0x02, 5000, 5000, 1);

Example: Write multiple parameters consecutively and save them all together.

    D50A_PostWriteOverCurrent(0x02, 8000, 8000, 0);
    D50A_PostWriteOverTime(0x02, 5000, 5000, 0);
    D50A_PostSave(0x02);

Check if the asynchronous write request is actually executed successfully:

    const D50A_State_t *drv = D50A_GetStateByAddr(0x02);
    if(drv != NULL)
    {
        if(drv->last_request_error == D50A_OK)
        {
            // The most recent write request executed successfully.
        }
        else
        {
            // The most recent write request failed to execute; error codes are shown in D50A_Result_t.
        }
    }

Notes:
- The return value of D50A_PostXXX() only indicates whether the delivery queue was successful.
- The actual execution results of the drive depend on the last_request_error in the corresponding device status.
- D50A_Task will prioritize write requests, and the write request processing cycle will pause automatic polling once.

4. Read the drive state cache
--------------------------------------------------------------------------------
D50A_Task Periodic reading:
- Alarm code and real-time current A0
- Temperature A3
- overcurrent threshold setting value A1
- Overcurrent time setting value A2

The caller does not need to actively send read commands; they can directly read the cache.

Example: Read real-time current, temperature, and alarm status at address 0x01.

    const D50A_State_t *drv = D50A_GetStateByAddr(0x01);
    if(drv != NULL && drv->online)
    {
        float m1_current = drv->m1_current_a; // M1 real-time current, unit A
        float m2_current = drv->m2_current_a; M2 real-time current, unit: A
        uint8_t m1_temp = drv->m1_temp; // M1 temperature, unit Celsius
        uint8_t m2_temp = drv->m2_temp; // M2 temperature, unit Celsius

        if(drv->alarm_bits.overcurrent_ok == 0)
        {
            // Overcurrent protection triggered.
        }
        if(drv->alarm_bits.overtemp_ok == 0)
        {
            // Overtemperature protection triggered.
        }
    }

Alarm Position Precautions:
- In the D50A protocol, alarm bit 1 indicates normal.
- In the D50A protocol, an alarm bit of 0 indicates a corresponding protection trigger.
- For example, overcurrent_ok == 0 indicates overcurrent protection triggered, not normal.

Five, multiple drivers used in series
--------------------------------------------------------------------------------
Applicable scenarios:
- Multiple D50A drives mounted on one 485 bus.
- Each drive uses a different address.

Example: Register 3 drive addresses.

    D50A_AddDevice(0x01);
    D50A_AddDevice(0x02);
    D50A_AddDevice(0x03);

Example: Read the status of a drive by address.

    const D50A_State_t *drv2 = D50A_GetStateByAddr(0x02);

Example: Traverse all registered devices, suitable for batch online display.

    uint8_t i;
    for(i = 0; i < D50A_GetDeviceCount(); i++)
    {
        const D50A_State_t *drv = D50A_GetStateByIndex(i);
        if(drv != NULL)
        {
            // drv->addr Driver address
            // drv->online Online status
            // drv->m1_temp M1 temperature
            // drv->m1_current_a M1 current
        }
    }

Differences between ByAddr and ByIndex:
- D50A_GetStateByAddr (addr): Check status by drive address; business code is more recommended for this one.
- D50A_GetStateByIndex (index): Status is checked by internal device table sublabels, suitable for traversing all devices.

Multi-driver precautions:
- Do not allow multiple drives to share the same address.
- Do not use D50A_PostReadAddress() broadcast lookup addresses on multi-driver buses.
- If you want to change the address, it is recommended to keep only the target driver on the bus to avoid misoperation or address conflicts.

6. Modify the drive address
--------------------------------------------------------------------------------
Example: Change address 0x01 to 0x02.

    D50A_PostWriteAddress(0x01, 0x02);

or synchronize modifications during initialization:

    D50A_SyncWriteAddress(0x01, 0x02);

Notes:
- B3 The address modification command will be saved automatically, so you don't need to call Save again.
- Before modifying the address, it is recommended to connect only one target drive on the bus.
- The new address cannot be 0x00 or the broadcast address 0xFF.
- After modifying the address, the address in the device status table will also be updated.

7. Restore default overcurrent parameters
--------------------------------------------------------------------------------
Asynchronous runtime recovery:

    D50A_PostResetParam(0x01);

Initialization phase synchronous recovery:

    D50A_SyncResetParam(0x01);

Notes:
- B2 will restore the default overcurrent threshold and overcurrent time and save automatically.
- After recovery, D50A_Task will read back the set values during subsequent polling.
================================================================================
*/extern TaskHandle_t d50a_TaskHandle;

/* D50A initialization: On hardware versions supporting the 485 interface, it should be called after UART2_Init(). */
void D50A_Init(void);

/*
 * D50A Management Tasks:
 * 1. Handle all write requests uniformly to avoid multitasking conflicts on the 485 bus.
 * 2. Polling drive status at a fixed frequency during idle times.
 */
void D50A_Task(void *pvParameters);

/* USART2 receive interrupt call: feeds a received byte to the D50A frame capture state machine. */
void D50A_RxByte(uint8_t data);

/*
 * Asynchronous write request interface.
 *
 * The return value only indicates whether the request was queued successfully, not whether the driver executed it successfully.
 * Check last_request_error in the corresponding device state for actual driver execution results.
 */
uint8_t D50A_PostWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save);
uint8_t D50A_PostWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save);
uint8_t D50A_PostResetParam(uint8_t addr);
uint8_t D50A_PostWriteAddress(uint8_t old_addr,uint8_t new_addr);
uint8_t D50A_PostSave(uint8_t addr);
uint8_t D50A_PostReadAddress(void);

/*
 * Synchronous write interface used during hardware initialization.
 *
 * These interfaces do not depend on D50A_Task or use the FreeRTOS queue; suitable for setting driver parameters before the scheduler starts.
 * Before use, UART2_Init() and D50A_Init() must be completed, and it must be ensured that USART2 can call D50A_RxByte() during reception interrupts.
 * Prefer D50A_PostXXX() at runtime to avoid accessing the 485 bus concurrently with D50A_Task.
 */
uint8_t D50A_SyncWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save);
uint8_t D50A_SyncWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save);
uint8_t D50A_SyncResetParam(uint8_t addr);
uint8_t D50A_SyncWriteAddress(uint8_t old_addr,uint8_t new_addr);
uint8_t D50A_SyncSave(uint8_t addr);

/* Multi-drive device table management interface. */
uint8_t D50A_AddDevice(uint8_t addr);
uint8_t D50A_RemoveDevice(uint8_t addr);
uint8_t D50A_GetDeviceCount(void);
const D50A_State_t* D50A_GetStateByAddr(uint8_t addr);    // Query status by driver address, suitable for business code use
const D50A_State_t* D50A_GetStateByIndex(uint8_t index);  // Query status by device table index, suitable for iterating through all registered devices


#endif
