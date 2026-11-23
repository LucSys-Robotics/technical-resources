#include "driver_d50a.h"
#include "buzzer.h"

/*
 * D50A serial port protocol fixed at 13 bytes:
 * AA AA AA ADDR FUNC D0 D1 D2 D3 BB BB BB BB
 *
 * The protocol itself has no checksum, so here we can only rely on:
 * 1. Fixed frame header
 * 2. Fix frame tail
 * 3. Fixed length
 * 4. Address/function code match; A0 resp[4] is an alarm code and needs to be parsed separately.
 * All array indices in the text start from 0; resp[3] is the address, and resp[5]~resp[8] are data.
 * D50A_Task exclusively owns runtime communication; interrupts only receive frames, and display/control modules only read cached data.
 */
#define D50A_FRAME_LEN       13
#define D50A_HEAD            0xAA
#define D50A_TAIL            0xBB

/* Reply wait threshold, unit: ms; Sync paths are counted by 1ms, task paths are determined by RTOS tick. */
#define D50A_TIMEOUT_MS      80
#define D50A_POLL_FAIL_LIMIT 3       // After a single slot has failed consecutively until this number of failures is reached, the polling frequency is reduced
#define D50A_POLL_RETRY_MS   1000    // After reducing polling frequency, allow at least 1000ms between attempts

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

#define D50A_INIT_RETRY_DELAY_MS   100    // After configuration fails, the additional waiting time for this round, in milliseconds
#define D50A_INIT_SAVE_DELAY_MS    100    // B4 response, wait for EEPROM to save, then read back the parameters
/* Starting current and overcurrent time are set Robot_Select each model, and the same configuration is used for sending and recall. */
#define D50A_ADDRESS_TARGET_ADDR   D50A_SECOND_ADDR

TaskHandle_t d50a_TaskHandle = NULL;

/* Write request queue: Bluetooth/OLED/other tasks only deliver requests and do not directly operate serial port 2. */
static QueueHandle_t d50a_req_queue = NULL;

/* Multi-drive status table. Register addresses 1 or 1 and 2 by model type, which can be added later via D50A_AddDevice(). */
static D50A_State_t d50a_devices[D50A_MAX_DEVICES];
static uint8_t d50a_device_count = 0; // Number of registered slots, including disabled/offline slots, not the number of online drives

/* Device index used during automatic polling, polling one device at a time to avoid prolonged usage in a single cycle. */
static uint8_t d50a_poll_device_index = 0;
static uint8_t d50a_poll_failures[D50A_MAX_DEVICES];
static TickType_t d50a_poll_retry_tick[D50A_MAX_DEVICES];

/*
 * Current pointer to the status of the device in communication.
 * D50A_RequestAndWait() updates online, last_error, last_update_tick accordingly.
 */
static D50A_State_t *d50a_active_state = NULL;

/* All registration slots are configured or disabled postset 1; window timeout exit does not mean configuration was successful. */
static volatile uint8_t d50a_startup_configured = 0;

volatile uint8_t D50A_AddressSetRequest = 0;
volatile uint8_t D50A_AddressSetConfirmed = 0;

/*
 * USART2 interrupts receive buffer.
 * rx_buf For the frame currently being received.
 * rx_frame Used to save a frame that has been fully collected and passed the frame tail check.
 */
static volatile uint8_t rx_buf[D50A_FRAME_LEN];
static volatile uint8_t rx_frame[D50A_FRAME_LEN];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_frame_ready = 0;
static volatile uint8_t rx_uart_error = 0; // After a serial port error occurs in this transaction, it remains at 1 and is cleared by the next transaction

/*
 * Discard old frames and serial port error locks before starting a new transaction to avoid treating the last reply as the current data.
 * Retains the interrupt switch state before the call; can also be used before the scheduler starts.
 */
static void D50A_ClearRxFrame(void)
{
	uint32_t irq_state = __get_PRIMASK();
	volatile uint32_t status;

	/* Clear hardware errors in SR¡úDR order, then clear old software frames; Restore the original PRIMASK at the end. */
	__disable_irq();
	status = USART2->SR;
	if(status & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE |
	             USART_FLAG_FE | USART_FLAG_PE))
		status = USART2->DR;
	rx_index = 0;
	rx_frame_ready = 0;
	rx_uart_error = 0;
	memset((void*)rx_buf,0,sizeof(rx_buf));
	memset((void*)rx_frame,0,sizeof(rx_frame));
	__set_PRIMASK(irq_state);
}

/* Frames are grouped according to a fixed 13-byte protocol frame; d0~d3 have been converted by the caller into protocol data bytes. */
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

/*
 * Send a full frame; Return D50A_OK successfully; if any byte fails, the stop immediately stops and returns TX_TIMEOUT.
 * On failure, only the local USART2 is rebuilt; drive is not reset, and retransmission is not done here.
 */
static uint8_t D50A_SendFrame(const uint8_t *frame)
{
	uint8_t i;

	/* If any byte fails to be sent, the entire frame is aborted and the error is returned to task processing. */
	for(i=0;i<D50A_FRAME_LEN;i++)
	{
		if(uart2_send(frame[i]) != 0)
		{
			/* Only restore the serial port of the machine, do not reset the driver, and do not change the motor control status. */
			USART_Cmd(USART2,DISABLE);
			UART2_Init(115200);
			return D50A_ERR_TX_TIMEOUT;
		}
	}
	return D50A_OK;
}

/*
 * Record the result of a single communication and return the result as is, allowing upstream to return directly or continue processing.
 * Called only after actual send/receive failure or command response validation has completed, not for local parameter/queue failures.
 */
static uint8_t D50A_SetResult(D50A_State_t *state,uint8_t result)
{
	if(state == NULL) return result;

	//Success count increases after command response validation passes; Timeouts and frame errors are accumulated separately.
	//This function does not clear status_valid, change timestamps or alter confirmed protection alarms.
	state->online = (result == D50A_OK) ? 1 : 0;
	state->last_error = result;
	if(result == D50A_OK)
		state->comm_success_count++;
	else if(result == D50A_ERR_TIMEOUT || result == D50A_ERR_TX_TIMEOUT)
		state->comm_timeout_count++;
	else if(result == D50A_ERR_FRAME || result == D50A_ERR_EXCEPTION || result == D50A_ERR_UART)
		state->comm_frame_err_count++;

	return result;
}

/* Search all registration slots by address, including disabled slots; Protect queries by retaining alarms before offline/deactivation. */
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

/* Enable slots by address; Online status is not a filtering criterion; offline device caches can still be read. */
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

/* Called when a write request requires a state slot; If an existing slot is reused, it will be reused; otherwise, registering the address will result in a NULL return. */
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

/*
 * is only responsible for joining the team and pre-registering addresses; Return D50A_OK indicates successful delivery but does not indicate the drive has been executed.
 * Return NOT_INIT when the task has not yet created a queue; return QUEUE_FULL when the queue is full; Serial port data is not sent here.
 */
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

/*
 * Synchronous transactions used before scheduler start: clear old frames¡ú send¡ú wait in 1ms stepping for response.
 * Do not call RTOS delay or tick interfaces; Return format/serial port results, address, function, and data for continued validation by the caller.
 * When sending or receiving successfully, set online and set last_update_tick to 0; success statistics should be recorded only after command validation.
 */
static uint8_t D50A_RequestAndWaitNoRTOS(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp)
{
	uint8_t tx[D50A_FRAME_LEN];
	uint8_t ret,i;
	uint32_t irq_state;
	uint16_t wait_ms = 0;

	//Clean up old frames, group frames, and send them; if any byte fails to be sent, no reply will be waited for.
	D50A_ClearRxFrame();
	D50A_BuildFrame(tx,addr,func,d0,d1,d2,d3);
	ret = D50A_SendFrame(tx);
	if(ret == D50A_OK)
	{

		ret = D50A_ERR_TIMEOUT;
		while(wait_ms <= D50A_TIMEOUT_MS)
		{
			if(rx_uart_error)
			{
				ret = D50A_ERR_UART;
				break;
			}
			if(rx_frame_ready)
			{
				//Interrupt writes to receive frames, retains and restores the original interrupt state during copying.
				irq_state = __get_PRIMASK();
				__disable_irq();
				if(rx_uart_error)
				{
					__set_PRIMASK(irq_state);
					ret = D50A_ERR_UART;
					break;
				}
				for(i=0;i<D50A_FRAME_LEN;i++) resp[i] = rx_frame[i];
				rx_frame_ready = 0;
				__set_PRIMASK(irq_state);

				//First handle serial port errors, then check fixed frame headers, frames, and protocol exception responses.
				if(rx_uart_error)
					ret = D50A_ERR_UART;
				else if(resp[0] != D50A_HEAD || resp[1] != D50A_HEAD || resp[2] != D50A_HEAD ||
				        resp[9] != D50A_TAIL || resp[10] != D50A_TAIL || resp[11] != D50A_TAIL || resp[12] != D50A_TAIL)
					ret = D50A_ERR_FRAME;
				else if(resp[3] == 0xAA && resp[4] == 0xFF && resp[5] == 0xFF &&
				        resp[6] == 0xFF && resp[7] == 0xFF && resp[8] == 0xFF)
					ret = D50A_ERR_EXCEPTION;
				else
					ret = D50A_OK;
				break;
			}
			delay_ms(1);
			wait_ms++;
		}
	}

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
			d50a_active_state->status_valid = 0;
			D50A_SetResult(d50a_active_state,ret);
		}
	}
	return ret;
}

/*
 * Transactions called only by D50A_Task serial number; Give up the CPU with vTaskDelay(1) while waiting for a response.
 * First check serial port errors, then check frame head/end and error responses; After success, the caller continues to verify business fields.
 * Send and receive failure clears online/status_valid and logs errors, but does not clear protection_fault and historical parameter caches.
 */
static uint8_t D50A_RequestAndWait(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp)
{
	uint8_t tx[D50A_FRAME_LEN];
	uint8_t ret,i;
	uint32_t irq_state;
	TickType_t start,timeout;

	//Clean up old frames, group frames, and send them; if any byte fails to be sent, no reply will be waited for.
	D50A_ClearRxFrame();
	D50A_BuildFrame(tx,addr,func,d0,d1,d2,d3);
	ret = D50A_SendFrame(tx);
	if(ret == D50A_OK)
	{
		start = xTaskGetTickCount();
		timeout = pdMS_TO_TICKS(D50A_TIMEOUT_MS);
		ret = D50A_ERR_TIMEOUT;
		while((xTaskGetTickCount() - start) <= timeout)
		{
			if(rx_uart_error)
			{
				ret = D50A_ERR_UART;
				break;
			}
			if(rx_frame_ready)
			{
				//Interrupt writes to receive frames, retains and restores the original interrupt state during copying.
				irq_state = __get_PRIMASK();
				__disable_irq();
				if(rx_uart_error)
				{
					__set_PRIMASK(irq_state);
					ret = D50A_ERR_UART;
					break;
				}
				for(i=0;i<D50A_FRAME_LEN;i++) resp[i] = rx_frame[i];
				rx_frame_ready = 0;
				__set_PRIMASK(irq_state);

				//First handle serial port errors, then check fixed frame headers, frames, and protocol exception responses.
				if(rx_uart_error)
					ret = D50A_ERR_UART;
				else if(resp[0] != D50A_HEAD || resp[1] != D50A_HEAD || resp[2] != D50A_HEAD ||
				        resp[9] != D50A_TAIL || resp[10] != D50A_TAIL || resp[11] != D50A_TAIL || resp[12] != D50A_TAIL)
					ret = D50A_ERR_FRAME;
				else if(resp[3] == 0xAA && resp[4] == 0xFF && resp[5] == 0xFF &&
				        resp[6] == 0xFF && resp[7] == 0xFF && resp[8] == 0xFF)
					ret = D50A_ERR_EXCEPTION;
				else
					ret = D50A_OK;
				break;
			}
			vTaskDelay(1);
		}
	}

	if(d50a_active_state != NULL)
	{
		if(ret == D50A_OK)
		{
			d50a_active_state->online = 1;
			d50a_active_state->last_error = D50A_OK;
			d50a_active_state->last_update_tick = xTaskGetTickCount();
		}
		else
		{
			d50a_active_state->online = 0;
			d50a_active_state->status_valid = 0;
			D50A_SetResult(d50a_active_state,ret);
		}
	}
	return ret;
}

/*
 * Read A0 and update alarms, M1/M2 currents and A0 timestamp; parse currents as signed 16-bit big-endian values.
 * Only valid replies with matching addresses can update protection_fault; Failure only invalidates this A0 cache.
 */
static uint8_t D50A_ReadStatus(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_STATUS,0,0,0,0,resp);
	state->status_valid = 0;
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr) return D50A_SetResult(state,D50A_ERR_FRAME);

	/*
	 * A0 return frame special: resp[4] is an alarm code, not function code A0.
	 * The next 4 bytes are M1/M2 real-time current, int16, actual current A = raw / 100.
	 */
	state->alarm = resp[4];
	//The lower four bits from bit3 to bit0 are, in order: undervoltage, overvoltage, overtemperature, overcurrent; 1 = normal, 0 = protection trigger.
	state->alarm_bits.undervoltage_ok = (state->alarm & 0x08) ? 1 : 0;
	state->alarm_bits.overvoltage_ok = (state->alarm & 0x04) ? 1 : 0;
	state->alarm_bits.overtemp_ok = (state->alarm & 0x02) ? 1 : 0;
	state->alarm_bits.overcurrent_ok = (state->alarm & 0x01) ? 1 : 0;
	state->m1_current_raw = (int16_t)(((uint16_t)resp[5]<<8)|resp[6]);
	state->m2_current_raw = (int16_t)(((uint16_t)resp[7]<<8)|resp[8]);
	state->m1_current_a = state->m1_current_raw/100.0f;
	state->m2_current_a = state->m2_current_raw/100.0f;
	state->last_status_tick = xTaskGetTickCount();
	/* Only a valid A0 reply from this driver can change its protection flag; protection is cleared only when all four low bits are 1. */
	state->protection_fault = ((state->alarm & 0x0F) != 0x0F) ? 1 : 0;
	state->status_valid = 1;
	return D50A_SetResult(state,D50A_OK);
}

/* Read A1: protocol unit is 10mA, converted to mA, then update the two-channel threshold cache; If failure, retain the old value. */
static uint8_t D50A_ReadOverCurrent(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_OVERCURRENT,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_OVERCURRENT) return D50A_SetResult(state,D50A_ERR_FRAME);

	/* A1 returns an overcurrent threshold unit of 10mA, which is converted to mA stored in the cache. */
	state->m1_overcurrent_ma = (uint16_t)(((uint16_t)resp[5]<<8)|resp[6])*10;
	state->m2_overcurrent_ma = (uint16_t)(((uint16_t)resp[7]<<8)|resp[8])*10;
	return D50A_SetResult(state,D50A_OK);
}

/* Read A2: Both channels of overflow time are in ms; Failure to retain old cache. */
static uint8_t D50A_ReadOverTime(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_OVERTIME,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_OVERTIME) return D50A_SetResult(state,D50A_ERR_FRAME);

	/* A2 returns the overcurrent time unit in ms. */
	state->m1_overtime_ms = (uint16_t)(((uint16_t)resp[5]<<8)|resp[6]);
	state->m2_overtime_ms = (uint16_t)(((uint16_t)resp[7]<<8)|resp[8]);
	return D50A_SetResult(state,D50A_OK);
}

/*
 * Send B0, allowed range 1000¨C16000mA; divide by 10 and round down, then send and update cache based on quantized value
 * This function only writes RAM; Whether to append B4 is determined by the startup process or requesting branch.
 */
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
	   (uint16_t)(((uint16_t)resp[5]<<8)|resp[6]) != m1_raw || (uint16_t)(((uint16_t)resp[7]<<8)|resp[8]) != m2_raw)
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	if(d50a_active_state != NULL)
	{
		d50a_active_state->m1_overcurrent_ma = m1_raw*10;
		d50a_active_state->m2_overcurrent_ma = m2_raw*10;
	}
	return D50A_SetResult(d50a_active_state,D50A_OK);
}

/* Send B1, allowed range 10¨C10000ms; after verifying echo on both channels, update cache, this function does not save proactively */
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
	   (uint16_t)(((uint16_t)resp[5]<<8)|resp[6]) != m1_ms || (uint16_t)(((uint16_t)resp[7]<<8)|resp[8]) != m2_ms)
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	if(d50a_active_state != NULL)
	{
		d50a_active_state->m1_overtime_ms = m1_ms;
		d50a_active_state->m2_overtime_ms = m2_ms;
	}
	return D50A_SetResult(d50a_active_state,D50A_OK);
}

/*
 * Send B3 internally to the task; the driver will automatically save the address; After success, modify the original slot address without moving the slot.
 * Follow the existing process to check target slot conflicts after B3 response; the Bluetooth process will disable offline target occupancy in advance.
 */
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
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	new_state = D50A_FindState(new_addr);
	if(new_state != NULL && new_state != d50a_active_state)
		return D50A_ERR_PARAM;

	if(d50a_active_state != NULL)
	{
		d50a_active_state->addr = new_addr;
	}
	return D50A_SetResult(d50a_active_state,D50A_OK);
}

/* Send B4 and verify address, function code, and four 0xFF; here just confirms save acknowledgment, no parameter readback */
static uint8_t D50A_Save(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_SAVE,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_SAVE ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	/* B4 is only used to store parameters written to B0/B1. */
	return D50A_SetResult(d50a_active_state,D50A_OK);
}

/*
 * Perform one round of boot configuration on the specified address: B0¡úB1¡ú B4¡ú etc. 100ms¡úA1¡úA2, and verify the two parameters.
 * This function does not retry repeatedly; failure is passed to the task to retry within a 10-second window; Parameters already written are not rolled back.
 * D50A_OK means all configuration and readback checks passed in this cycle; a readback mismatch returns ERR_FRAME.
 */
static uint8_t D50A_RunStandbyInit(uint8_t addr)
{
	D50A_State_t *state = D50A_FindState(addr);
	uint8_t ret;

	if(state == NULL)
		return D50A_ERR_NOT_INIT;

	d50a_active_state = state;
	state->online = 0;

	/* B0¡úB1¡úB4: Write two RAM parameters first, then save again; If any step fails, skip the next command of this round. */
	ret = D50A_WriteOverCurrent(addr,
		robot.D50A_OverCurrent_mA,robot.D50A_OverCurrent_mA);
	if(ret == D50A_OK)
	{
		ret = D50A_WriteOverTime(addr,
			robot.D50A_OverTime_ms,robot.D50A_OverTime_ms);
	}
	if(ret == D50A_OK)
		ret = D50A_Save(addr);

	/* Allow the drive to save the EEPROM and then read and confirm the actual parameters. */
	if(ret == D50A_OK)
	{
		delay_ms(D50A_INIT_SAVE_DELAY_MS);
		ret = D50A_ReadOverCurrent(state);
	}
	if(ret == D50A_OK)
		ret = D50A_ReadOverTime(state);

	if(ret == D50A_OK &&
	   (state->m1_overcurrent_ma != robot.D50A_OverCurrent_mA ||
	    state->m2_overcurrent_ma != robot.D50A_OverCurrent_mA ||
	    state->m1_overtime_ms != robot.D50A_OverTime_ms ||
	    state->m2_overtime_ms != robot.D50A_OverTime_ms))
		ret = D50A_ERR_FRAME;

	if(ret == D50A_OK)
		state->online = 1;
	else
		state->online = 0;
	state->last_error = ret;
	return ret;
}

/*
 * Dedicated readdress process for Bluetooth O/P: handles address conflicts and offline slots, then reads A0 confirmation from the new address after B3.
 * If the target is online or the old address no longer exists, only perform a lookback on target A0; Refuse operation if both addresses are online.
 * B3 fails to recover target placeholder; B3 succeeds but A0 fails without rolling back address. Acknowledgment flags and beeps are handled by the task after success. After
 */
static uint8_t D50A_ProcessAddressSet(uint8_t old_addr,uint8_t new_addr)
{
	D50A_State_t *source = D50A_FindState(old_addr);
	D50A_State_t *target = D50A_FindState(new_addr);
	uint8_t target_disabled = 0;
	uint8_t ret;

	/* If the target address is already online, only do one check to avoid conflicts between two online devices. */
	if(target != NULL && target != source && target->online != 0)
	{
		if(source != NULL && source->online != 0)
		{
			source->last_error = D50A_ERR_PARAM;
			return D50A_ERR_PARAM;
		}

		d50a_active_state = target;
		ret = D50A_ReadStatus(target);
		return ret;
	}

	/* After the last address change, the status table may no longer have the old address; just check the new address. */
	if(source == NULL)
	{
		if(target == NULL)
			return D50A_ERR_NOT_INIT;

		d50a_active_state = target;
		ret = D50A_ReadStatus(target);
		return ret;
	}

	/* Temporarily disable the new address when it is registered by default but not online, to avoid repeated polling after the address change. */
	if(target != NULL && target != source)
	{
		target->enable = 0;
		target_disabled = 1;
	}

	d50a_active_state = source;
	ret = D50A_WriteAddress(old_addr,new_addr);
	if(ret != D50A_OK)
	{
		if(target_disabled)
			target->enable = 1;
		return ret;
	}

	/* B3 succeeds, read the A0 status using the new address to confirm that the module has indeed responded according to the new address. */
	ret = D50A_ReadStatus(source);
	if(ret != D50A_OK)
		return ret;
	if(source->addr != new_addr)
	{
		source->last_error = D50A_ERR_FRAME;
		return D50A_ERR_FRAME;
	}

	return D50A_OK;
}

/*
 * After powering on UART2_Init() and Robot_Select(), call: clear the software status and register the address according to the configuration, without sending parameter commands.
 * Fixed parameters are configured by the task in the original IMU waiting window; During runtime, serial port recovery does not call this function to avoid clearing historical alarms.
 */
void D50A_Init(void)
{
	/* Initialize only the device table and receive state here; configure fixed parameters in the standby window after startup. */
	memset(d50a_devices,0,sizeof(d50a_devices));
	d50a_device_count = 0;
	d50a_poll_device_index = 0;
	memset(d50a_poll_failures,0,sizeof(d50a_poll_failures));
	memset(d50a_poll_retry_tick,0,sizeof(d50a_poll_retry_tick));
	d50a_active_state = NULL;
	d50a_startup_configured = 0;
	D50A_AddressSetRequest = 0;
	D50A_AddressSetConfirmed = 0;
	if(robot.D50A_Enable!=0)
	{
		D50A_AddDevice(D50A_DEFAULT_ADDR);
		if(robot.D50A_Number==2) D50A_AddDevice(D50A_SECOND_ADDR);
	}
	D50A_ClearRxFrame();
}

/*
 * The only runtime D50A communication tasks: startup window configuration¡ú Bluetooth readdress request¡ú queue write request¡ú idle polling.
 * Only normal requests/polls are scheduled at 10Hz; Start configuration and readdress follow their own processes, and do not use parameter success as a condition for motor release.
 */
void D50A_Task(void *pvParameters)
{
	D50A_Request_t request;
	D50A_State_t *state;
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;
	uint8_t device_index;
	uint8_t addr;
	uint8_t poll_step = 0;
	uint8_t startup_window_finished = 0; // The window has closed, which does not mean all driver configurations were successful
	uint8_t startup_device_index = 0;
	uint8_t startup_device_ok_mask = 0; // bit i = slot i has been successfully configured or disabled; this window will not be overridden
	u32 lastWakeTime = getSysTickCnt();

	/* Create the request queue after runtime tasks start; D50A_Init() itself does not depend on the queue. */
	if(d50a_req_queue == NULL)
	{
		d50a_req_queue = xQueueCreate(D50A_REQ_QUEUE_LEN,sizeof(D50A_Request_t));
	}

	while(1)
	{
		/*
		 * Only configure within the existing IMU waiting window; successful slots are not overwritten, failed slots retry in rotation.
		 * Exit when all  complete or when the Time_count reaches CONTROL_DELAY, without blocking the original motor's self-test.
		 */
		if(startup_window_finished == 0)
		{
			if(d50a_startup_configured != 0 || SysVal.Time_count >= CONTROL_DELAY)
			{
				startup_window_finished = 1;
				lastWakeTime = getSysTickCnt();
			}
			else
			{
				if(d50a_device_count == 0)
				{
					d50a_startup_configured = 1;
				}
				else
				{
					uint8_t device_bit;

					if(startup_device_index >= d50a_device_count)
						startup_device_index = 0;
					device_bit = (uint8_t)(1U << startup_device_index);

					if((startup_device_ok_mask & device_bit) == 0)
					{
						if(d50a_devices[startup_device_index].enable == 0 ||
						   D50A_RunStandbyInit(d50a_devices[startup_device_index].addr) == D50A_OK)
						{
							startup_device_ok_mask |= device_bit;
						}
						else
						{
							delay_ms(D50A_INIT_RETRY_DELAY_MS);
						}
					}

					startup_device_index++;
					if(startup_device_index >= d50a_device_count)
						startup_device_index = 0;

					if(startup_device_ok_mask == (uint8_t)((1U << d50a_device_count) - 1U))
						d50a_startup_configured = 1;
				}
				continue;
			}
		}

		//UART4 interrupt requests only; Here, readdress and A0 callback are executed, and only after success is the acknowledgment flag set and a beep sounds.
		if(D50A_AddressSetRequest != D50A_ADDRESS_SET_NONE)
		{
			uint8_t address_set_request = D50A_AddressSetRequest;
			uint8_t old_addr;
			uint8_t new_addr;

			D50A_AddressSetRequest = D50A_ADDRESS_SET_NONE;
			D50A_AddressSetConfirmed = 0;

			if(address_set_request == D50A_ADDRESS_SET_1_TO_2)
			{
				old_addr = D50A_DEFAULT_ADDR;
				new_addr = D50A_ADDRESS_TARGET_ADDR;
			}
			else if(address_set_request == D50A_ADDRESS_SET_2_TO_1)
			{
				old_addr = D50A_ADDRESS_TARGET_ADDR;
				new_addr = D50A_DEFAULT_ADDR;
			}
			else
			{
				continue;
			}

			if(D50A_ProcessAddressSet(old_addr,new_addr) == D50A_OK)
			{
				D50A_AddressSetConfirmed = 1;
				Buzzer_AddTask(1,20);
			}
			continue;
		}

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
			state = D50A_GetOrAddState(request.addr == D50A_BROADCAST_ADDR ? D50A_DEFAULT_ADDR : request.addr);
			if(state != NULL)
			{
				state->last_request = request.type;
				d50a_active_state = state;
				switch(request.type)
				{
					case D50A_REQ_WRITE_OVERCURRENT:
						ret = D50A_WriteOverCurrent(request.addr,request.m1_value,request.m2_value);
						if(ret == D50A_OK && request.save_after_write) ret = D50A_Save(request.addr);
						break;
					case D50A_REQ_WRITE_OVERTIME:
						ret = D50A_WriteOverTime(request.addr,request.m1_value,request.m2_value);
						if(ret == D50A_OK && request.save_after_write) ret = D50A_Save(request.addr);
						break;
					case D50A_REQ_RESET_PARAM:
						ret = D50A_RequestAndWait(request.addr,D50A_FUNC_RESET_PARAM,0,0,0,0,resp);
						if(ret == D50A_OK)
						{
							if(resp[3] != request.addr || resp[4] != D50A_FUNC_RESET_PARAM ||
							   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
								ret = D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);
							else
								ret = D50A_SetResult(d50a_active_state,D50A_OK);
						}
						break;
					case D50A_REQ_WRITE_ADDR:
						ret = D50A_WriteAddress(request.addr,request.new_addr);
						break;
					case D50A_REQ_SAVE:
						ret = D50A_Save(request.addr);
						break;
					case D50A_REQ_READ_ADDR:
						ret = D50A_RequestAndWait(D50A_BROADCAST_ADDR,D50A_FUNC_READ_ADDR,0,0,0,0,resp);
						if(ret == D50A_OK)
						{
							if(resp[4] != D50A_FUNC_READ_ADDR)
								ret = D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);
							else
							{
								//Broadcast address lookup is only applicable to a single driver; register the reply address after success.
								D50A_AddDevice(resp[3]);
								d50a_active_state = D50A_FindState(resp[3]);
								ret = D50A_SetResult(d50a_active_state,D50A_OK);
							}
						}
						break;
					default:
						ret = D50A_ERR_PARAM;
						break;
				}
				//Record to the original request slot; A4 address lookup may have switched active_state to the reply address.
				state->last_request_error = ret;
			}
		}
		else
		{
			/*
			 * Access up to one slot per cycle; 0/2/4/6/8 read A0, 1/5 read A3, 3 read A1, 7 read A2, 9 idle.
			 * Same step: first traverse all registered slots, then switch types; Offline/deactivated slots also consume one scheduling cycle.
			 * 10Hz is the task frequency, not the update frequency driving each data class per unit; Write requests occupy polling in this cycle.
			 */
			if(d50a_device_count != 0)
			{
				if(d50a_poll_device_index >= d50a_device_count) d50a_poll_device_index = 0;
				device_index = d50a_poll_device_index++;
				state = &d50a_devices[device_index];

				//After 3 consecutive failures in a single slot, retry at least 1 second after the last failure; Clear the number of failed attempts after success.
				//This backoff only restricts automatic polling and does not restrict startup configuration, Bluetooth readdressing, or queue write requests.
				if(state->enable != 0 &&
				   (d50a_poll_failures[device_index] < D50A_POLL_FAIL_LIMIT ||
				    (TickType_t)(xTaskGetTickCount() - d50a_poll_retry_tick[device_index]) >= pdMS_TO_TICKS(D50A_POLL_RETRY_MS)))
				{
					d50a_active_state = state;
					switch(poll_step)
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
							addr = state->addr;
							ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_TEMP,0,0,0,0,resp);
							if(ret == D50A_OK)
							{
									if(resp[3] != addr || resp[4] != D50A_FUNC_READ_TEMP)
									ret = D50A_SetResult(state,D50A_ERR_FRAME);
								else
								{
									state->m1_temp = resp[6];
									state->m2_temp = resp[8];
									ret = D50A_SetResult(state,D50A_OK);
								}
							}
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

					//Step 9 is a free step and does not clear the consecutive failure count.
					if(poll_step < 9)
					{
						if(ret != D50A_OK)
						{
							state->last_error = ret;
							if(d50a_poll_failures[device_index] < D50A_POLL_FAIL_LIMIT) d50a_poll_failures[device_index]++;
							if(d50a_poll_failures[device_index] >= D50A_POLL_FAIL_LIMIT) d50a_poll_retry_tick[device_index] = xTaskGetTickCount();
						}
						else
							d50a_poll_failures[device_index] = 0;
					}
				}
			}
			/* Polling all devices in the same query step, then switching query types. */
			if(d50a_poll_device_index >= d50a_device_count)
			{
				poll_step++;
				if(poll_step >= 10) poll_step = 0;
			}
		}
	}
}

/* USART2 normally receives byte inputs, only group frames are juxed ready; Command parsing and retry are completed by the task. */
void D50A_RxByte(uint8_t data)
{
	/*
	 * Frame Capture State Machine:
	 * 1. The first 3 bytes must consecutively receive 0xAA.
	 * 2. After collecting 13 bytes, check whether subscripts 9~12 are all 0xBB.
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

/* USART2 Error Reception Entry: Discard the current frame and lock the transaction error, waiting for the function to return ERR_UART. */
void D50A_RxError(void)
{
	/* Only bad frames are dropped during interrupts, with no retries, no delays, and no commands sent. */
	rx_index = 0;
	rx_frame_ready = 0;
	rx_uart_error = 1;
}

/* Submit B0 request, m1_ma/m2_ma in mA; Save is non-zero, and after the task writes successfully, append B4. */
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

/* Submit B1 request, m1_ms/m2_ms units ms; Save non-zero, append B4 after the task writes successfully. */
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

/* Submit B2 recovery parameter requests; Drive auto-save, cache refreshed by subsequent A1/A2 polls. */
uint8_t D50A_PostResetParam(uint8_t addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_RESET_PARAM;
	request.addr = addr;
	return D50A_PostRequest(&request);
}

/* Only B3 readdress requests are delivered; A0 callbacks, acknowledgments, or beeps do not include Bluetooth O/P processes. */
uint8_t D50A_PostWriteAddress(uint8_t old_addr,uint8_t new_addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_WRITE_ADDR;
	request.addr = old_addr;
	request.new_addr = new_addr;
	return D50A_PostRequest(&request);
}

/* Only B4 saves the request; the actual result is written by the task to the last_request_error of the corresponding slot. */
uint8_t D50A_PostSave(uint8_t addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_SAVE;
	request.addr = addr;
	return D50A_PostRequest(&request);
}

/* Deliver broadcast A4 address lookup requests; the reply address will be added to the device table; Only used for scenarios with a single driver on the bus. */
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

/*
 * Power on synchronizes write B0; Parameter ranges and quantization rules match task writes; save continues synchronization B4 when non-zero.
 * Return actual response verification result; if saving fails, retain the RAM parameter cache already written successfully.
 */
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
	   (uint16_t)(((uint16_t)resp[5]<<8)|resp[6]) != m1_raw || (uint16_t)(((uint16_t)resp[7]<<8)|resp[8]) != m2_raw)
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	d50a_active_state->m1_overcurrent_ma = m1_raw*10;
	d50a_active_state->m2_overcurrent_ma = m2_raw*10;
	D50A_SetResult(d50a_active_state,D50A_OK);

	if(save)
		return D50A_SyncSave(addr);

	return D50A_OK;
}

/* Power on synchronously write B1, unit ms; Save continues synchronization with B4 when non-zero, returning the actual execution result. */
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
	   (uint16_t)(((uint16_t)resp[5]<<8)|resp[6]) != m1_ms || (uint16_t)(((uint16_t)resp[7]<<8)|resp[8]) != m2_ms)
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	d50a_active_state->m1_overtime_ms = m1_ms;
	d50a_active_state->m2_overtime_ms = m2_ms;
	D50A_SetResult(d50a_active_state,D50A_OK);

	if(save)
		return D50A_SyncSave(addr);

	return D50A_OK;
}

/* Power-on synchronously sends B2 and verifies the reply; Do not immediately reread parameters or append B4. */
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
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	return D50A_SetResult(d50a_active_state,D50A_OK);
}

/*
 * Power-on synchronized address readdress: Before sending, check whether the new address is valid and whether other active slots are occupied.
 * Only verifies B3 responses and modifies the original slot address, without performing Bluetooth process A0 callbacks or buzzes.
 */
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
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	d50a_active_state->addr = new_addr;
	return D50A_SetResult(d50a_active_state,D50A_OK);
}

/* Power-on synchronously sends B4 and verifies response; Additional save delays and parameter readback are arranged by the caller. */
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
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	return D50A_SetResult(d50a_active_state,D50A_OK);
}

/*
 * Register the address or re-enable the existing slot; new slots default to offline, and absence of communication does not imply a driver protection alarm.
 * Retains up to D50A_MAX_DEVICES slots; re-enabling does not clear old caches or acknowledged alarms.
 */
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

/* Disable the slot and mark it offline; do not delete/compact the array, reduce the registered count, or clear parameter/protection caches. */
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

/* Return number of registered slots, including disabled and offline slots; do not use as actual connected device count or protection-check scope. */
uint8_t D50A_GetDeviceCount(void)
{
	return d50a_device_count;
}

/*
 * Query the protection cache of the drive required by the model without sending commands or directly changing motor activation.
 * Check address 1 or addresses 1 and 2 by Robot_Select configuration; Returns 1 when automatic management is off or protection is not acknowledged; returns 0 when protection is confirmed.
 * It has been confirmed that alarms do not disappear offline and can only be cleared by a valid normal A0 reply for that slot.
 */
uint8_t D50A_IsProtectionNormal(void)
{
	D50A_State_t *state;

	//For uncompatible models, disable automatic management; lack of drive data does not affect original control.
	if(robot.D50A_Enable==0) return 1;

	//After readdressing, there may be disabled slots at the same address; prioritize checking slots currently in use.
	state = D50A_FindState(D50A_DEFAULT_ADDR);
	if(state == NULL) state = D50A_FindAnyState(D50A_DEFAULT_ADDR);
	if(state != NULL && state->protection_fault != 0) return 0;

	if(robot.D50A_Number==2)
	{
		state = D50A_FindState(D50A_SECOND_ADDR);
		if(state == NULL) state = D50A_FindAnyState(D50A_SECOND_ADDR);
		if(state != NULL && state->protection_fault != 0) return 0;
	}
	return 1;
}

/* Returns read-only cache of enabled slots by address; Returns NULL if unregistered or disabled; offline can still return historical cache. */
const D50A_State_t* D50A_GetStateByAddr(uint8_t addr)
{
	return D50A_FindState(addr);
}

/* Returns read-only cache based on fixed slot subscripts; Returns NULL when out of bounds or disabled; readdressing does not move slot subscripts. */
const D50A_State_t* D50A_GetStateByIndex(uint8_t index)
{
	if(index >= d50a_device_count)
		return NULL;
	if(d50a_devices[index].enable == 0)
		return NULL;

	/* Return a read-only pointer; callers must not cast it to modify internal state. */
	return &d50a_devices[index];
}
