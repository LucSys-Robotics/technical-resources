#include "driver_d50a.h"

/*
 * D50A serial port protocol fixed at 13 bytes:
 * AA AA AA ADDR FUNC D0 D1 D2 D3 BB BB BB BB
 *
 * The protocol itself has no checksum, so here we can only rely on:
 * 1. Fixed frame header
 * 2. Fix frame tail
 * 3. Fixed length
 * 4. Address/function code match
 */
#define D50A_FRAME_LEN       13
#define D50A_HEAD            0xAA
#define D50A_TAIL            0xBB

/* The maximum time a single command waits for a response, in ms. */
#define D50A_TIMEOUT_MS      80

/* The maximum number of write requests other tasks can queue. */
#define D50A_REQ_QUEUE_LEN   8

/* D50A protocol function code definition. */
#define D50A_FUNC_READ_STATUS       0xA0
#define D50A_FUNC_READ_OVERCURRENT  0xA1
#define D50A_FUNC_READ_OVERTIME     0xA2
#define D50A_FUNC_READ_TEMP         0xA3
#define D50A_FUNC_READ_ADDR         0xA4
#define D50A_FUNC_WRITE_OVERCURRENT 0xB0
#define D50A_FUNC_WRITE_OVERTIME    0xB1
#define D50A_FUNC_RESET_PARAM       0xB2
#define D50A_FUNC_WRITE_ADDR        0xB3
#define D50A_FUNC_SAVE              0xB4

TaskHandle_t d50a_TaskHandle = NULL;

/* Write request queue: Bluetooth/OLED/other tasks only deliver requests and do not directly operate serial port 2. */
static QueueHandle_t d50a_req_queue = NULL;

/* Multi-drive status table. Add 0x01 by default, which can be added later via D50A_AddDevice(). */
static D50A_State_t d50a_devices[D50A_MAX_DEVICES];
static uint8_t d50a_device_count = 0;

/* Device index used during automatic polling, polling one device at a time to avoid prolonged usage in a single cycle. */
static uint8_t d50a_poll_device_index = 0;

/*
 * Current pointer to the status of the device in communication.
 * D50A_RequestAndWait() updates online, last_error, last_update_tick accordingly.
 */
static D50A_State_t *d50a_active_state = NULL;

/*
 * USART2 interrupts receive buffer.
 * rx_buf For the frame currently being received.
 * rx_frame Used to save a frame that has been fully collected and passed the frame tail check.
 */
static volatile uint8_t rx_buf[D50A_FRAME_LEN];
static volatile uint8_t rx_frame[D50A_FRAME_LEN];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_frame_ready = 0;

static void D50A_ClearRxFrame(void);
static void D50A_BuildFrame(uint8_t *frame,uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3);
static uint8_t D50A_CheckFrame(const uint8_t *frame);
static uint8_t D50A_IsExceptionFrame(const uint8_t *frame);
static uint8_t D50A_WaitFrame(uint8_t *frame,uint16_t timeout_ms);
static uint8_t D50A_WaitFrameNoRTOS(uint8_t *frame,uint16_t timeout_ms);
static void D50A_SendFrame(const uint8_t *frame);
static int16_t D50A_ToInt16(uint8_t high,uint8_t low);
static uint16_t D50A_ToUInt16(uint8_t high,uint8_t low);
static void D50A_UpdateAlarmBits(D50A_State_t *state,uint8_t alarm);
static void D50A_UpdateCommStats(D50A_State_t *state,uint8_t result);
static uint8_t D50A_ReturnFrameError(D50A_State_t *state);
static uint8_t D50A_ReturnSuccess(D50A_State_t *state);
static uint8_t D50A_PostRequest(const D50A_Request_t *request);
static uint8_t D50A_ProcessRequest(const D50A_Request_t *request);
static void D50A_PollStep(uint8_t step);
static D50A_State_t* D50A_FindState(uint8_t addr);
static D50A_State_t* D50A_FindAnyState(uint8_t addr);
static D50A_State_t* D50A_GetOrAddState(uint8_t addr);

static uint8_t D50A_ReadStatus(D50A_State_t *state);
static uint8_t D50A_ReadOverCurrent(D50A_State_t *state);
static uint8_t D50A_ReadOverTime(D50A_State_t *state);
static uint8_t D50A_ReadTemperature(D50A_State_t *state);
static uint8_t D50A_ReadAddress(void);
static uint8_t D50A_WriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma);
static uint8_t D50A_WriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms);
static uint8_t D50A_ResetParam(uint8_t addr);
static uint8_t D50A_WriteAddress(uint8_t old_addr,uint8_t new_addr);
static uint8_t D50A_Save(uint8_t addr);
static uint8_t D50A_RequestAndWaitNoRTOS(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp);

void D50A_Init(void)
{
	/* Clear the device table and enable address 0x01 by default. */
	memset(d50a_devices,0,sizeof(d50a_devices));
	d50a_device_count = 0;
	d50a_poll_device_index = 0;
	d50a_active_state = NULL;
	D50A_AddDevice(D50A_DEFAULT_ADDR);
	D50A_ClearRxFrame();

}

void D50A_Task(void *pvParameters)
{
	D50A_Request_t request;
	uint8_t poll_step = 0;
	u32 lastWakeTime = getSysTickCnt();

	/* Create the request queue after runtime tasks start; D50A_Init() itself does not depend on the queue. */
	if(d50a_req_queue == NULL)
	{
		d50a_req_queue = xQueueCreate(D50A_REQ_QUEUE_LEN,sizeof(D50A_Request_t));
	}

	while(1)
	{
		/* Task runs at 10Hz, write requests first, and periodic polling status when idle. */
		vTaskDelayUntil(&lastWakeTime,F2T(D50A_TASK_RATE));

		if(d50a_req_queue == NULL)
		{
			continue;
		}

		/*
		 * Prioritize write requests for external deliveries.
		 * Only one request per task cycle is processed, ensuring a controllable communication pace.
		 */
		if(xQueueReceive(d50a_req_queue,&request,0) == pdTRUE)
		{
			D50A_State_t *state = D50A_GetOrAddState(request.addr == D50A_BROADCAST_ADDR ? D50A_DEFAULT_ADDR : request.addr);
			if(state != NULL)
			{
				state->last_request = request.type;
				state->last_request_error = D50A_ProcessRequest(&request);
			}
		}
		else
		{
			/*
			 * Perform automatic polling when no request is written.
			 * poll_step Decide which type of data to read this time, D50A_PollStep() decide which device to access.
			 */
			D50A_PollStep(poll_step);
			poll_step++;
			if(poll_step >= 10) poll_step = 0;
		}
	}
}

void D50A_RxByte(uint8_t data)
{
	/*
	 * Frame Capture State Machine:
	 * 1. The first 3 bytes must consecutively receive 0xAA.
	 * 2. After collecting 13 bytes, check if the 9th~12th bytes are 0xBB.
	 * 3. After verification, copy to the rx_frame and place the rx_frame_ready.
	 *
	 * This runs in the USART2 interrupt; do minimal work and no application-level parsing.
	 */
	if(rx_index < 3)
	{
		if(data == D50A_HEAD)
		{
			rx_buf[rx_index++] = data;
		}
		else
		{
			rx_index = 0;
		}
		return;
	}

	rx_buf[rx_index++] = data;

	if(rx_index >= D50A_FRAME_LEN)
	{
		uint8_t i;
		if(rx_buf[9] == D50A_TAIL && rx_buf[10] == D50A_TAIL &&
		   rx_buf[11] == D50A_TAIL && rx_buf[12] == D50A_TAIL)
		{
			for(i=0;i<D50A_FRAME_LEN;i++)
			{
				rx_frame[i] = rx_buf[i];
			}
			rx_frame_ready = 1;
		}
		rx_index = 0;
	}
}

uint8_t D50A_PostWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save)
{
	D50A_Request_t request;

	/* The upper-layer incoming unit is mA, and internally it is converted to protocol units of 10mA before transmission. */
	request.type = D50A_REQ_WRITE_OVERCURRENT;
	request.addr = addr;
	request.new_addr = 0;
	request.m1_value = m1_ma;
	request.m2_value = m2_ma;
	request.save_after_write = save;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save)
{
	D50A_Request_t request;

	/* The upper-layer incoming unit is ms, and the protocol also uses ms. */
	request.type = D50A_REQ_WRITE_OVERTIME;
	request.addr = addr;
	request.new_addr = 0;
	request.m1_value = m1_ms;
	request.m2_value = m2_ms;
	request.save_after_write = save;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostResetParam(uint8_t addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_RESET_PARAM;
	request.addr = addr;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostWriteAddress(uint8_t old_addr,uint8_t new_addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_WRITE_ADDR;
	request.addr = old_addr;
	request.new_addr = new_addr;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostSave(uint8_t addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_SAVE;
	request.addr = addr;
	return D50A_PostRequest(&request);
}

uint8_t D50A_PostReadAddress(void)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));

	/*
	 * Broadcast lookup address is only suitable for a single drive on the bus.
	 * Do not call this interface when multiple drives are connected in series, otherwise multiple devices may respond simultaneously.
	 */
	request.type = D50A_REQ_READ_ADDR;
	request.addr = D50A_BROADCAST_ADDR;
	return D50A_PostRequest(&request);
}

uint8_t D50A_SyncWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint16_t m1_raw,m2_raw;
	uint8_t ret;

	if(m1_ma < 1000 || m1_ma > 16000 || m2_ma < 1000 || m2_ma > 16000)
		return D50A_ERR_PARAM;

	d50a_active_state = D50A_GetOrAddState(addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	m1_raw = m1_ma/10;
	m2_raw = m2_ma/10;
	ret = D50A_RequestAndWaitNoRTOS(addr,D50A_FUNC_WRITE_OVERCURRENT,m1_raw>>8,m1_raw,m2_raw>>8,m2_raw,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_WRITE_OVERCURRENT ||
	   D50A_ToUInt16(resp[5],resp[6]) != m1_raw || D50A_ToUInt16(resp[7],resp[8]) != m2_raw)
		return D50A_ReturnFrameError(d50a_active_state);

	d50a_active_state->m1_overcurrent_ma = m1_raw*10;
	d50a_active_state->m2_overcurrent_ma = m2_raw*10;
	D50A_UpdateCommStats(d50a_active_state,D50A_OK);

	if(save)
		return D50A_SyncSave(addr);

	return D50A_OK;
}

uint8_t D50A_SyncWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;

	if(m1_ms < 10 || m1_ms > 10000 || m2_ms < 10 || m2_ms > 10000)
		return D50A_ERR_PARAM;

	d50a_active_state = D50A_GetOrAddState(addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	ret = D50A_RequestAndWaitNoRTOS(addr,D50A_FUNC_WRITE_OVERTIME,m1_ms>>8,m1_ms,m2_ms>>8,m2_ms,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_WRITE_OVERTIME ||
	   D50A_ToUInt16(resp[5],resp[6]) != m1_ms || D50A_ToUInt16(resp[7],resp[8]) != m2_ms)
		return D50A_ReturnFrameError(d50a_active_state);

	d50a_active_state->m1_overtime_ms = m1_ms;
	d50a_active_state->m2_overtime_ms = m2_ms;
	D50A_UpdateCommStats(d50a_active_state,D50A_OK);

	if(save)
		return D50A_SyncSave(addr);

	return D50A_OK;
}

uint8_t D50A_SyncResetParam(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;

	d50a_active_state = D50A_GetOrAddState(addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	ret = D50A_RequestAndWaitNoRTOS(addr,D50A_FUNC_RESET_PARAM,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_RESET_PARAM ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_ReturnFrameError(d50a_active_state);

	return D50A_ReturnSuccess(d50a_active_state);
}

uint8_t D50A_SyncWriteAddress(uint8_t old_addr,uint8_t new_addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;
	D50A_State_t *new_state;

	if(new_addr == D50A_BROADCAST_ADDR || new_addr == 0)
		return D50A_ERR_PARAM;

	d50a_active_state = D50A_GetOrAddState(old_addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	new_state = D50A_FindState(new_addr);
	if(new_state != NULL && new_state != d50a_active_state)
		return D50A_ERR_PARAM;

	ret = D50A_RequestAndWaitNoRTOS(old_addr,D50A_FUNC_WRITE_ADDR,new_addr,0xFF,0xFF,0xFF,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != old_addr || resp[4] != D50A_FUNC_WRITE_ADDR || resp[5] != new_addr)
		return D50A_ReturnFrameError(d50a_active_state);

	d50a_active_state->addr = new_addr;
	return D50A_ReturnSuccess(d50a_active_state);
}

uint8_t D50A_SyncSave(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;

	d50a_active_state = D50A_GetOrAddState(addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	ret = D50A_RequestAndWaitNoRTOS(addr,D50A_FUNC_SAVE,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_SAVE ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_ReturnFrameError(d50a_active_state);

	return D50A_ReturnSuccess(d50a_active_state);
}

uint8_t D50A_AddDevice(uint8_t addr)
{
	D50A_State_t *state;

	/* 0 and broadcast addresses cannot be added to the poll table as generic device addresses. */
	if(addr == 0 || addr == D50A_BROADCAST_ADDR)
		return D50A_ERR_PARAM;

	/* If the device slot once existed but is disabled, re-enable it directly. */
	state = D50A_FindAnyState(addr);
	if(state != NULL)
	{
		state->enable = 1;
		return D50A_OK;
	}

	if(d50a_device_count >= D50A_MAX_DEVICES)
		return D50A_ERR_QUEUE_FULL;

	/* Create a new device slot and wait for subsequent polling to update the status. */
	state = &d50a_devices[d50a_device_count++];
	memset(state,0,sizeof(D50A_State_t));
	state->enable = 1;
	state->addr = addr;
	state->last_error = D50A_ERR_NOT_INIT;
	return D50A_OK;
}

uint8_t D50A_RemoveDevice(uint8_t addr)
{
	D50A_State_t *state = D50A_FindState(addr);
	if(state == NULL)
		return D50A_ERR_PARAM;

	/* Do not compress arrays to avoid sudden state changes when reading external indexes. */
	state->enable = 0;
	state->online = 0;
	return D50A_OK;
}

uint8_t D50A_GetDeviceCount(void)
{
	return d50a_device_count;
}

const D50A_State_t* D50A_GetStateByAddr(uint8_t addr)
{
	return D50A_FindState(addr);
}

const D50A_State_t* D50A_GetStateByIndex(uint8_t index)
{
	if(index >= d50a_device_count)
		return NULL;
	if(d50a_devices[index].enable == 0)
		return NULL;

	/* Return a read-only pointer; callers must not cast it to modify internal state. */
	return &d50a_devices[index];
}


static uint8_t D50A_PostRequest(const D50A_Request_t *request)
{
	if(d50a_req_queue == NULL)
	{
		return D50A_ERR_NOT_INIT;
	}
	if(xQueueSend(d50a_req_queue,request,0) != pdTRUE)
	{
		return D50A_ERR_QUEUE_FULL;
	}

	/*
	 * After successfully joining the team, add this address to the device table in advance.
	 * This lets external code find the status slot through D50A_GetStateByAddr() even before execution finishes.
	 */
	if(request->addr != D50A_BROADCAST_ADDR)
	{
		D50A_AddDevice(request->addr);
	}
	return D50A_OK;
}

static void D50A_ClearRxFrame(void)
{
	/* Receive status is accessed jointly by interrupts and tasks, with interrupts briefly closed during cleanup. */
	__disable_irq();
	rx_index = 0;
	rx_frame_ready = 0;
	memset((void*)rx_buf,0,sizeof(rx_buf));
	memset((void*)rx_frame,0,sizeof(rx_frame));
	__enable_irq();
}

static void D50A_BuildFrame(uint8_t *frame,uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3)
{
	frame[0] = D50A_HEAD;
	frame[1] = D50A_HEAD;
	frame[2] = D50A_HEAD;
	frame[3] = addr;
	frame[4] = func;
	frame[5] = d0;
	frame[6] = d1;
	frame[7] = d2;
	frame[8] = d3;
	frame[9] = D50A_TAIL;
	frame[10] = D50A_TAIL;
	frame[11] = D50A_TAIL;
	frame[12] = D50A_TAIL;
}

static uint8_t D50A_CheckFrame(const uint8_t *frame)
{
	/* D50A has no checksum, only checking fixed frame heads and fixed frame tails. */
	if(frame[0] != D50A_HEAD || frame[1] != D50A_HEAD || frame[2] != D50A_HEAD)
		return 0;
	if(frame[9] != D50A_TAIL || frame[10] != D50A_TAIL || frame[11] != D50A_TAIL || frame[12] != D50A_TAIL)
		return 0;
	return 1;
}

static uint8_t D50A_IsExceptionFrame(const uint8_t *frame)
{
	/* When no function code is defined, the drive consistently returns AA AA AA AA FF FF FF FF FF BB. */
	return frame[3] == 0xAA && frame[4] == 0xFF && frame[5] == 0xFF &&
	       frame[6] == 0xFF && frame[7] == 0xFF && frame[8] == 0xFF;
}

static uint8_t D50A_WaitFrame(uint8_t *frame,uint16_t timeout_ms)
{
	TickType_t start = xTaskGetTickCount();
	TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
	uint8_t i;

	while((xTaskGetTickCount() - start) <= timeout)
	{
		if(rx_frame_ready)
		{
			/* rx_frame Written by interrupt, entering critical zone during copying, and clearing the ready flag after copying. */
			__disable_irq();
			for(i=0;i<D50A_FRAME_LEN;i++)
			{
				frame[i] = rx_frame[i];
			}
			rx_frame_ready = 0;
			__enable_irq();

			if(D50A_CheckFrame(frame) == 0)
				return D50A_ERR_FRAME;
			if(D50A_IsExceptionFrame(frame))
				return D50A_ERR_EXCEPTION;
			return D50A_OK;
		}
		vTaskDelay(1);
	}
	return D50A_ERR_TIMEOUT;
}

static uint8_t D50A_WaitFrameNoRTOS(uint8_t *frame,uint16_t timeout_ms)
{
	uint16_t wait_ms = 0;
	uint8_t i;

	/*
	 * Wait function used before the scheduler starts.
	 * Do not call vTaskDelay()/xTaskGetTickCount() here; use only delay_ms(1) for busy-wait timeout handling.
	 * USART2 reception still depends on interrupts; after receiving a complete frame, D50A_RxByte() sets the rx_frame_ready.
	 */
	while(wait_ms <= timeout_ms)
	{
		if(rx_frame_ready)
		{
			__disable_irq();
			for(i=0;i<D50A_FRAME_LEN;i++)
			{
				frame[i] = rx_frame[i];
			}
			rx_frame_ready = 0;
			__enable_irq();

			if(D50A_CheckFrame(frame) == 0)
				return D50A_ERR_FRAME;
			if(D50A_IsExceptionFrame(frame))
				return D50A_ERR_EXCEPTION;
			return D50A_OK;
		}

		delay_ms(1);
		wait_ms++;
	}

	return D50A_ERR_TIMEOUT;
}

static void D50A_SendFrame(const uint8_t *frame)
{
	uint8_t i;

	/* The current serial port transmission still follows the original blocking single-byte transmission style of the project. */
	for(i=0;i<D50A_FRAME_LEN;i++)
	{
		uart2_send(frame[i]);
	}
}

static int16_t D50A_ToInt16(uint8_t high,uint8_t low)
{
	return (int16_t)(((uint16_t)high<<8)|low);
}

static uint16_t D50A_ToUInt16(uint8_t high,uint8_t low)
{
	return (uint16_t)(((uint16_t)high<<8)|low);
}

static void D50A_UpdateAlarmBits(D50A_State_t *state,uint8_t alarm)
{
	/* The protocol stipulates: alarm level 1 indicates normal, 0 indicates protection triggered. */
	state->alarm_bits.undervoltage_ok = (alarm & 0x08) ? 1 : 0;
	state->alarm_bits.overvoltage_ok = (alarm & 0x04) ? 1 : 0;
	state->alarm_bits.overtemp_ok = (alarm & 0x02) ? 1 : 0;
	state->alarm_bits.overcurrent_ok = (alarm & 0x01) ? 1 : 0;
}

static void D50A_UpdateCommStats(D50A_State_t *state,uint8_t result)
{
	if(state == NULL)
		return;

	/*
	 * These three counters only count communication results actually transmitted on the 485 bus.
	 * Local errors such as parameter errors, queue not initialized, or queue full are not included in communication statistics.
	 */
	if(result == D50A_OK)
	{
		state->comm_success_count++;
	}
	else if(result == D50A_ERR_TIMEOUT)
	{
		state->comm_timeout_count++;
	}
	else if(result == D50A_ERR_FRAME || result == D50A_ERR_EXCEPTION)
	{
		state->comm_frame_err_count++;
	}
}

static uint8_t D50A_ReturnFrameError(D50A_State_t *state)
{
	if(state != NULL)
	{
		state->online = 0;
		state->last_error = D50A_ERR_FRAME;
	}
	D50A_UpdateCommStats(state,D50A_ERR_FRAME);
	return D50A_ERR_FRAME;
}

static uint8_t D50A_ReturnSuccess(D50A_State_t *state)
{
	if(state != NULL)
	{
		state->online = 1;
		state->last_error = D50A_OK;
	}
	D50A_UpdateCommStats(state,D50A_OK);
	return D50A_OK;
}

static D50A_State_t* D50A_FindState(uint8_t addr)
{
	uint8_t i;

	/* Only searches for enabled devices. */
	for(i=0;i<d50a_device_count;i++)
	{
		if(d50a_devices[i].enable && d50a_devices[i].addr == addr)
			return &d50a_devices[i];
	}
	return NULL;
}

static D50A_State_t* D50A_GetOrAddState(uint8_t addr)
{
	D50A_State_t *state = D50A_FindState(addr);
	if(state != NULL)
		return state;

	/* When writing requests to access new addresses, the device table is automatically supplemented. */
	if(D50A_AddDevice(addr) != D50A_OK)
		return NULL;

	return D50A_FindState(addr);
}

static D50A_State_t* D50A_FindAnyState(uint8_t addr)
{
	uint8_t i;

	/* Find all slots, including those that have been removed/disabled. */
	for(i=0;i<d50a_device_count;i++)
	{
		if(d50a_devices[i].addr == addr)
			return &d50a_devices[i];
	}
	return NULL;
}

static uint8_t D50A_RequestAndWaitNoRTOS(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp)
{
	uint8_t tx[D50A_FRAME_LEN];
	uint8_t ret;

	/*
	 * Synchronous send/receive functions used before the scheduler starts.
	 * This function does not go through the request queue nor calls vTaskDelay(), suitable for writing drive parameters during the systemInit() phase.
	 * Note: During runtime, if D50A_Task has already started, it is not recommended to call this function again to avoid accessing the 485 bus simultaneously with the task.
	 */
	D50A_ClearRxFrame();
	D50A_BuildFrame(tx,addr,func,d0,d1,d2,d3);
	D50A_SendFrame(tx);
	ret = D50A_WaitFrameNoRTOS(resp,D50A_TIMEOUT_MS);

	if(d50a_active_state != NULL)
	{
		if(ret == D50A_OK)
		{
			d50a_active_state->online = 1;
			d50a_active_state->last_error = D50A_OK;
			d50a_active_state->last_update_tick = 0;
		}
		else
		{
			d50a_active_state->online = 0;
			d50a_active_state->last_error = ret;
			D50A_UpdateCommStats(d50a_active_state,ret);
		}
	}

	return ret;
}

static uint8_t D50A_RequestAndWait(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp)
{
	uint8_t tx[D50A_FRAME_LEN];
	uint8_t ret;

	/*
	 * Clear old received frames before sending commands to prevent previous frame residues from being misused by this command.
	 * This function should only be called by D50A_Task, ensuring there is only one command on the bus at any given time.
	 */
	D50A_ClearRxFrame();
	D50A_BuildFrame(tx,addr,func,d0,d1,d2,d3);
	D50A_SendFrame(tx);
	ret = D50A_WaitFrame(resp,D50A_TIMEOUT_MS);
	if(ret != D50A_OK)
	{
		if(d50a_active_state != NULL)
		{
			d50a_active_state->online = 0;
			d50a_active_state->last_error = ret;
			D50A_UpdateCommStats(d50a_active_state,ret);
		}
		return ret;
	}

	if(d50a_active_state != NULL)
	{
		d50a_active_state->online = 1;
		d50a_active_state->last_error = D50A_OK;
		d50a_active_state->last_update_tick = xTaskGetTickCount();
	}
	return D50A_OK;
}

static uint8_t D50A_ReadStatus(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_STATUS,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr) return D50A_ReturnFrameError(state);

	/*
	 * A0 return frame special: the 4th byte is not function code A0, but an alarm code.
	 * The next 4 bytes are M1/M2 real-time current, int16, actual current A = raw / 100.
	 */
	state->alarm = resp[4];
	D50A_UpdateAlarmBits(state,state->alarm);
	state->m1_current_raw = D50A_ToInt16(resp[5],resp[6]);
	state->m2_current_raw = D50A_ToInt16(resp[7],resp[8]);
	state->m1_current_a = state->m1_current_raw/100.0f;
	state->m2_current_a = state->m2_current_raw/100.0f;
	return D50A_ReturnSuccess(state);
}

static uint8_t D50A_ReadOverCurrent(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_OVERCURRENT,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_OVERCURRENT) return D50A_ReturnFrameError(state);

	/* A1 returns an overcurrent threshold unit of 10mA, which is converted to mA stored in the cache. */
	state->m1_overcurrent_ma = D50A_ToUInt16(resp[5],resp[6])*10;
	state->m2_overcurrent_ma = D50A_ToUInt16(resp[7],resp[8])*10;
	return D50A_ReturnSuccess(state);
}

static uint8_t D50A_ReadOverTime(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_OVERTIME,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_OVERTIME) return D50A_ReturnFrameError(state);

	/* A2 returns the overcurrent time unit in ms. */
	state->m1_overtime_ms = D50A_ToUInt16(resp[5],resp[6]);
	state->m2_overtime_ms = D50A_ToUInt16(resp[7],resp[8]);
	return D50A_ReturnSuccess(state);
}

static uint8_t D50A_ReadTemperature(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_TEMP,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_TEMP) return D50A_ReturnFrameError(state);

	/* A3 temperature format: 00 M1_TEMP 00 M2_TEMP. */
	state->m1_temp = resp[6];
	state->m2_temp = resp[8];
	return D50A_ReturnSuccess(state);
}

static uint8_t D50A_ReadAddress(void)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret = D50A_RequestAndWait(D50A_BROADCAST_ADDR,D50A_FUNC_READ_ADDR,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[4] != D50A_FUNC_READ_ADDR) return D50A_ReturnFrameError(d50a_active_state);

	/* Automatically adds the address to the device table after querying it. It is not recommended to use broadcast address lookup in multi-device buses. */
	D50A_AddDevice(resp[3]);
	d50a_active_state = D50A_FindState(resp[3]);
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_WriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint16_t m1_raw,m2_raw;
	uint8_t ret;

	if(m1_ma < 1000 || m1_ma > 16000 || m2_ma < 1000 || m2_ma > 16000)
		return D50A_ERR_PARAM;

	/* B0 protocol unit is 10mA, while upper-layer APIs use mA. */
	m1_raw = m1_ma/10;
	m2_raw = m2_ma/10;
	ret = D50A_RequestAndWait(addr,D50A_FUNC_WRITE_OVERCURRENT,m1_raw>>8,m1_raw,m2_raw>>8,m2_raw,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_WRITE_OVERCURRENT ||
	   D50A_ToUInt16(resp[5],resp[6]) != m1_raw || D50A_ToUInt16(resp[7],resp[8]) != m2_raw)
		return D50A_ReturnFrameError(d50a_active_state);

	if(d50a_active_state != NULL)
	{
		d50a_active_state->m1_overcurrent_ma = m1_raw*10;
		d50a_active_state->m2_overcurrent_ma = m2_raw*10;
	}
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_WriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;

	if(m1_ms < 10 || m1_ms > 10000 || m2_ms < 10 || m2_ms > 10000)
		return D50A_ERR_PARAM;

	/* B1 protocol unit is ms. */
	ret = D50A_RequestAndWait(addr,D50A_FUNC_WRITE_OVERTIME,m1_ms>>8,m1_ms,m2_ms>>8,m2_ms,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_WRITE_OVERTIME ||
	   D50A_ToUInt16(resp[5],resp[6]) != m1_ms || D50A_ToUInt16(resp[7],resp[8]) != m2_ms)
		return D50A_ReturnFrameError(d50a_active_state);

	if(d50a_active_state != NULL)
	{
		d50a_active_state->m1_overtime_ms = m1_ms;
		d50a_active_state->m2_overtime_ms = m2_ms;
	}
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_ResetParam(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_RESET_PARAM,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_RESET_PARAM ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_ReturnFrameError(d50a_active_state);

	/* B2 restores default overflow parameters and saves them automatically, and subsequent polling reads the latest values. */
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_WriteAddress(uint8_t old_addr,uint8_t new_addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;
	D50A_State_t *new_state;

	if(new_addr == D50A_BROADCAST_ADDR || new_addr == 0)
		return D50A_ERR_PARAM;

	/* B3 After changing the address, the drive will automatically save and no need to send B4 again. */
	ret = D50A_RequestAndWait(old_addr,D50A_FUNC_WRITE_ADDR,new_addr,0xFF,0xFF,0xFF,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != old_addr || resp[4] != D50A_FUNC_WRITE_ADDR || resp[5] != new_addr)
		return D50A_ReturnFrameError(d50a_active_state);

	new_state = D50A_FindState(new_addr);
	if(new_state != NULL && new_state != d50a_active_state)
		return D50A_ERR_PARAM;

	if(d50a_active_state != NULL)
	{
		d50a_active_state->addr = new_addr;
	}
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_Save(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_SAVE,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_SAVE ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_ReturnFrameError(d50a_active_state);

	/* B4 is only used to store parameters written to B0/B1. */
	return D50A_ReturnSuccess(d50a_active_state);
}

static uint8_t D50A_ProcessRequest(const D50A_Request_t *request)
{
	uint8_t ret;

	/* Before processing write requests, first determine which device's state cache to update for this communication. */
	d50a_active_state = D50A_GetOrAddState(request->addr == D50A_BROADCAST_ADDR ? D50A_DEFAULT_ADDR : request->addr);
	if(d50a_active_state == NULL)
		return D50A_ERR_PARAM;

	switch(request->type)
	{
		case D50A_REQ_WRITE_OVERCURRENT:
			ret = D50A_WriteOverCurrent(request->addr,request->m1_value,request->m2_value);
			/* save_after_write is 1, parameters are saved automatically after successful write. */
			if(ret == D50A_OK && request->save_after_write) ret = D50A_Save(request->addr);
			return ret;

		case D50A_REQ_WRITE_OVERTIME:
			ret = D50A_WriteOverTime(request->addr,request->m1_value,request->m2_value);
			/* save_after_write is 1, parameters are saved automatically after successful write. */
			if(ret == D50A_OK && request->save_after_write) ret = D50A_Save(request->addr);
			return ret;

		case D50A_REQ_RESET_PARAM:
			return D50A_ResetParam(request->addr);

		case D50A_REQ_WRITE_ADDR:
			return D50A_WriteAddress(request->addr,request->new_addr);

		case D50A_REQ_SAVE:
			return D50A_Save(request->addr);

		case D50A_REQ_READ_ADDR:
			return D50A_ReadAddress();

		default:
			return D50A_ERR_PARAM;
	}
}

static void D50A_PollStep(uint8_t step)
{
	uint8_t ret;
	D50A_State_t *state;

	if(d50a_device_count == 0)
		return;

	if(d50a_poll_device_index >= d50a_device_count)
		d50a_poll_device_index = 0;

	state = &d50a_devices[d50a_poll_device_index];
	d50a_poll_device_index++;

	if(state->enable == 0)
		return;

	d50a_active_state = state;

	/*
	 * Automated polling strategy:
	 * D50A_Task runs at 10Hz, poll_step cycles every 10 cycles.
	 *
	 * For single devices:
	 * - A0 alarm/real-time current: 5Hz
	 * - A3 temperature: 2Hz
	 * - A1 overcurrent threshold setting: 1Hz
	 * - A2 overcurrent time setting: 1Hz
	 *
	 * When there are multiple devices, only one device is accessed at a time, so the frequency of a single device is approximately equal to the frequency per device count mentioned above.
	 */
	switch(step)
	{
		case 0:
		case 2:
		case 4:
		case 6:
		case 8:
			ret = D50A_ReadStatus(state);
			break;
		case 1:
		case 5:
			ret = D50A_ReadTemperature(state);
			break;
		case 3:
			ret = D50A_ReadOverCurrent(state);
			break;
		case 7:
			ret = D50A_ReadOverTime(state);
			break;
		default:
			ret = D50A_OK;
			break;
	}

	if(ret != D50A_OK)
	{
		state->last_error = ret;
	}
}
