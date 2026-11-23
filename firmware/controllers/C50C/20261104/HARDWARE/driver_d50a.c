#include "driver_d50a.h"
#include "buzzer.h"

/*
 * D50A 串口协议固定为 13 字节：
 * AA AA AA ADDR FUNC D0 D1 D2 D3 BB BB BB BB
 *
 * 协议本身没有校验和，所以这里只能依赖：
 * 1. 固定帧头
 * 2. 固定帧尾
 * 3. 固定长度
 * 4. 地址/功能码匹配；A0 回复的 resp[4] 为报警码，需要单独解析。
 * 文中数组下标均从 0 开始；resp[3] 是地址，resp[5]～resp[8] 是数据。
 * 运行期由 D50A_Task 独占通信；中断只收帧，显示和控制模块只读缓存。
 */
#define D50A_FRAME_LEN       13
#define D50A_HEAD            0xAA
#define D50A_TAIL            0xBB

/* 回复等待门限，单位 ms；同步路径按 1ms 累计，任务路径按 RTOS tick 判断。 */
#define D50A_TIMEOUT_MS      80
#define D50A_POLL_FAIL_LIMIT 3       // 单个槽位连续失败达到此次数后降低轮询频率
#define D50A_POLL_RETRY_MS   1000    // 降频后相邻轮询尝试至少间隔 1000ms

/* 其他任务最多可以排队的写请求数量。 */
#define D50A_REQ_QUEUE_LEN   8

/* D50A 协议功能码定义。 */
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

#define D50A_INIT_RETRY_DELAY_MS   100    // 启动配置失败后，本轮额外等待的时间，单位 ms
#define D50A_INIT_SAVE_DELAY_MS    100    // B4 应答后等待 EEPROM 保存，再回读参数
/* 启动电流和过流时间由各车型Robot_Select设置，发送与回读使用同一份配置。 */
#define D50A_ADDRESS_TARGET_ADDR   D50A_SECOND_ADDR

TaskHandle_t d50a_TaskHandle = NULL;

/* 写请求队列：蓝牙/OLED/其他任务只投递请求，不直接操作串口2。 */
static QueueHandle_t d50a_req_queue = NULL;

/* 多驱动器状态表。按车型配置登记地址1或1和2，后续可通过D50A_AddDevice()增加。 */
static D50A_State_t d50a_devices[D50A_MAX_DEVICES];
static uint8_t d50a_device_count = 0; // 已登记槽位数，包含停用/离线槽位，不是在线驱动数量

/* 自动轮询时使用的设备索引，每次轮询一个设备，避免单周期占用太久。 */
static uint8_t d50a_poll_device_index = 0;
static uint8_t d50a_poll_failures[D50A_MAX_DEVICES];
static TickType_t d50a_poll_retry_tick[D50A_MAX_DEVICES];

/*
 * 当前正在通信的设备状态指针。
 * D50A_RequestAndWait() 根据它更新 online、last_error、last_update_tick。
 */
static D50A_State_t *d50a_active_state = NULL;

/* 所有登记槽位配置完成或被停用后置 1；窗口超时退出不等于配置成功。 */
static volatile uint8_t d50a_startup_configured = 0;

volatile uint8_t D50A_AddressSetRequest = 0;
volatile uint8_t D50A_AddressSetConfirmed = 0;

/*
 * USART2 中断接收缓冲。
 * rx_buf 用于正在接收的一帧。
 * rx_frame 用于保存已经收完整并通过帧尾检查的一帧。
 */
static volatile uint8_t rx_buf[D50A_FRAME_LEN];
static volatile uint8_t rx_frame[D50A_FRAME_LEN];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_frame_ready = 0;
static volatile uint8_t rx_uart_error = 0; // 本次事务出现串口错误后保持为 1，由下次事务清除

/*
 * 新事务开始前丢弃旧帧和串口错误锁存，避免把上次回复当作本次数据。
 * 保留调用前的中断开关状态，调度器启动前也可使用。
 */
static void D50A_ClearRxFrame(void)
{
	uint32_t irq_state = __get_PRIMASK();
	volatile uint32_t status;

	/* 先按 SR→DR 顺序清硬件错误，再清软件旧帧；结束时恢复原 PRIMASK。 */
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

/* 按固定 13 字节协议组帧；d0～d3 已由调用者换算为协议数据字节。 */
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
 * 发送一整帧；成功返回 D50A_OK，任一字节失败立即停止并返回 TX_TIMEOUT。
 * 失败时仅重建本机 USART2，不复位驱动器、不在这里重发。
 */
static uint8_t D50A_SendFrame(const uint8_t *frame)
{
	uint8_t i;

	/* 任一字节发送失败便中止整帧，将错误交回任务处理。 */
	for(i=0;i<D50A_FRAME_LEN;i++)
	{
		if(uart2_send(frame[i]) != 0)
		{
			/* 只恢复本机串口，不复位驱动器，不改变电机控制状态。 */
			USART_Cmd(USART2,DISABLE);
			UART2_Init(115200);
			return D50A_ERR_TX_TIMEOUT;
		}
	}
	return D50A_OK;
}

/*
 * 记录一次通信结果并原样返回 result，供上层直接返回或继续处理。
 * 只在实际收发失败或命令应答校验完成后调用，不用于本地参数/入队失败。
 */
static uint8_t D50A_SetResult(D50A_State_t *state,uint8_t result)
{
	if(state == NULL) return result;

	//成功计数在命令应答校验通过后增加；超时和帧错误分别累计.
	//本函数不清 status_valid、不改时间戳，也不改变已确认的保护报警.
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

/* 按地址查所有登记槽位，包括停用槽位；保护查询用它保留离线/停用前的报警。 */
static D50A_State_t* D50A_FindAnyState(uint8_t addr)
{
	uint8_t i;

	/* 查找所有槽位，包括已经 Remove/disable 的槽位。 */
	for(i=0;i<d50a_device_count;i++)
	{
		if(d50a_devices[i].addr == addr)
			return &d50a_devices[i];
	}
	return NULL;
}

/* 按地址查启用槽位；在线状态不作为筛选条件，离线设备的缓存仍可读取。 */
static D50A_State_t* D50A_FindState(uint8_t addr)
{
	uint8_t i;

	/* 只查找启用中的设备。 */
	for(i=0;i<d50a_device_count;i++)
	{
		if(d50a_devices[i].enable && d50a_devices[i].addr == addr)
			return &d50a_devices[i];
	}
	return NULL;
}

/* 写请求需要状态槽位时调用；已有槽位直接复用，否则登记该地址，失败返回 NULL。 */
static D50A_State_t* D50A_GetOrAddState(uint8_t addr)
{
	D50A_State_t *state = D50A_FindState(addr);
	if(state != NULL)
		return state;

	/* 写请求访问新地址时，自动补进设备表。 */
	if(D50A_AddDevice(addr) != D50A_OK)
		return NULL;

	return D50A_FindState(addr);
}

/*
 * 只负责入队和提前登记地址；返回 D50A_OK 表示投递成功，不表示驱动已执行。
 * 任务尚未创建队列时返回 NOT_INIT，队列满时返回 QUEUE_FULL；这里不发送串口数据。
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
	 * 请求入队成功后，提前把该地址加入设备表。
	 * 这样即使还没执行完成，外部也可以通过 D50A_GetStateByAddr() 找到状态槽位。
	 */
	if(request->addr != D50A_BROADCAST_ADDR)
	{
		D50A_AddDevice(request->addr);
	}
	return D50A_OK;
}

/*
 * 调度器启动前使用的同步事务：清旧帧→发送→按 1ms 步进等待回复。
 * 不调用 RTOS 延时或 tick 接口；返回格式/串口结果，地址、功能及数据由调用者继续校验。
 * 收发成功时先置 online 并将 last_update_tick 置 0；成功统计要等命令校验通过后再记。
 */
static uint8_t D50A_RequestAndWaitNoRTOS(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp)
{
	uint8_t tx[D50A_FRAME_LEN];
	uint8_t ret,i;
	uint32_t irq_state;
	uint16_t wait_ms = 0;

	//清理旧帧、组帧并发送，任一字节发送失败后不再等待回复.
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
				//中断写入接收帧，复制时保留并恢复原来的中断状态.
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

				//先处理串口错误，再检查固定帧头、帧尾及协议异常回复.
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
 * 仅由 D50A_Task 串行调用的事务；等待回复期间用 vTaskDelay(1) 让出 CPU。
 * 先检查串口错误，再检查帧头/帧尾及异常回复；成功后由调用者继续校验业务字段。
 * 收发失败清 online/status_valid 并记录错误，但不清 protection_fault 和历史参数缓存。
 */
static uint8_t D50A_RequestAndWait(uint8_t addr,uint8_t func,uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t *resp)
{
	uint8_t tx[D50A_FRAME_LEN];
	uint8_t ret,i;
	uint32_t irq_state;
	TickType_t start,timeout;

	//清理旧帧、组帧并发送，任一字节发送失败后不再等待回复.
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
				//中断写入接收帧，复制时保留并恢复原来的中断状态.
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

				//先处理串口错误，再检查固定帧头、帧尾及协议异常回复.
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
 * 读取 A0 并更新报警、M1/M2 电流及 A0 时间戳；电流按有符号 16 位大端值解析。
 * 只有有效且地址匹配的回复才能更新 protection_fault；失败只使本次 A0 缓存无效。
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
	 * A0 返回帧特殊：resp[4] 是报警码，不是功能码 A0。
	 * 后续4字节为 M1/M2 实时电流，int16，实际电流A = raw / 100。
	 */
	state->alarm = resp[4];
	//低四位从 bit3 到 bit0 依次为欠压、过压、过温、过流；1=正常，0=保护触发.
	state->alarm_bits.undervoltage_ok = (state->alarm & 0x08) ? 1 : 0;
	state->alarm_bits.overvoltage_ok = (state->alarm & 0x04) ? 1 : 0;
	state->alarm_bits.overtemp_ok = (state->alarm & 0x02) ? 1 : 0;
	state->alarm_bits.overcurrent_ok = (state->alarm & 0x01) ? 1 : 0;
	state->m1_current_raw = (int16_t)(((uint16_t)resp[5]<<8)|resp[6]);
	state->m2_current_raw = (int16_t)(((uint16_t)resp[7]<<8)|resp[8]);
	state->m1_current_a = state->m1_current_raw/100.0f;
	state->m2_current_a = state->m2_current_raw/100.0f;
	state->last_status_tick = xTaskGetTickCount();
	/* 只有本驱动有效 A0 回复能改变保护标志；低四位全为 1 才表示保护已清除。 */
	state->protection_fault = ((state->alarm & 0x0F) != 0x0F) ? 1 : 0;
	state->status_valid = 1;
	return D50A_SetResult(state,D50A_OK);
}

/* 读取 A1：协议单位为 10mA，换算成 mA 后更新两路阈值缓存；失败保留旧值。 */
static uint8_t D50A_ReadOverCurrent(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_OVERCURRENT,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_OVERCURRENT) return D50A_SetResult(state,D50A_ERR_FRAME);

	/* A1 返回的过流电流单位为 10mA，这里转换成 mA 存入缓存。 */
	state->m1_overcurrent_ma = (uint16_t)(((uint16_t)resp[5]<<8)|resp[6])*10;
	state->m2_overcurrent_ma = (uint16_t)(((uint16_t)resp[7]<<8)|resp[8])*10;
	return D50A_SetResult(state,D50A_OK);
}

/* 读取 A2：两路过流时间单位均为 ms；失败保留旧缓存。 */
static uint8_t D50A_ReadOverTime(D50A_State_t *state)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t addr = state->addr;
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_READ_OVERTIME,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_READ_OVERTIME) return D50A_SetResult(state,D50A_ERR_FRAME);

	/* A2 返回的过流时间单位为 ms。 */
	state->m1_overtime_ms = (uint16_t)(((uint16_t)resp[5]<<8)|resp[6]);
	state->m2_overtime_ms = (uint16_t)(((uint16_t)resp[7]<<8)|resp[8]);
	return D50A_SetResult(state,D50A_OK);
}

/*
 * 发送 B0，允许范围 1000～16000mA；除以 10 取整后发送并按量化值更新缓存。
 * 本函数只写 RAM；是否追加 B4 由启动流程或请求分支决定。
 */
static uint8_t D50A_WriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint16_t m1_raw,m2_raw;
	uint8_t ret;

	if(m1_ma < 1000 || m1_ma > 16000 || m2_ma < 1000 || m2_ma > 16000)
		return D50A_ERR_PARAM;

	/* B0 协议单位为 10mA，上层 API 使用 mA。 */
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

/* 发送 B1，允许范围 10～10000ms；校验两路回显后更新缓存，本函数不主动保存。 */
static uint8_t D50A_WriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;

	if(m1_ms < 10 || m1_ms > 10000 || m2_ms < 10 || m2_ms > 10000)
		return D50A_ERR_PARAM;

	/* B1 协议单位为 ms。 */
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
 * 任务内部发送 B3，驱动会自动保存地址；成功后修改原槽位地址，不移动槽位。
 * 按现有流程在 B3 应答后检查目标槽位冲突；蓝牙流程会事先停用离线目标占位。
 */
static uint8_t D50A_WriteAddress(uint8_t old_addr,uint8_t new_addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret;
	D50A_State_t *new_state;

	if(new_addr == D50A_BROADCAST_ADDR || new_addr == 0)
		return D50A_ERR_PARAM;

	/* B3 修改地址后驱动器会自动保存，不需要再发送 B4。 */
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

/* 发送 B4 并校验地址、功能码和四个 0xFF；这里只确认保存应答，不回读参数。 */
static uint8_t D50A_Save(uint8_t addr)
{
	uint8_t resp[D50A_FRAME_LEN];
	uint8_t ret = D50A_RequestAndWait(addr,D50A_FUNC_SAVE,0,0,0,0,resp);
	if(ret != D50A_OK) return ret;
	if(resp[3] != addr || resp[4] != D50A_FUNC_SAVE ||
	   resp[5] != 0xFF || resp[6] != 0xFF || resp[7] != 0xFF || resp[8] != 0xFF)
		return D50A_SetResult(d50a_active_state,D50A_ERR_FRAME);

	/* B4 只用于保存 B0/B1 写入的参数。 */
	return D50A_SetResult(d50a_active_state,D50A_OK);
}

/*
 * 对指定地址执行一轮启动配置：B0→B1→B4→等 100ms→A1→A2，并核对两路参数。
 * 本函数不循环重试，失败交给任务在 10 秒窗口内重试；已写入的参数不回滚。
 * 返回 D50A_OK 才表示本轮配置及回读全部通过；回读值不符返回 ERR_FRAME。
 */
static uint8_t D50A_RunStandbyInit(uint8_t addr)
{
	D50A_State_t *state = D50A_FindState(addr);
	uint8_t ret;

	if(state == NULL)
		return D50A_ERR_NOT_INIT;

	d50a_active_state = state;
	state->online = 0;

	/* B0→B1→B4：先写两个 RAM 参数，再保存一次；任一步失败就跳过本轮后续命令。 */
	ret = D50A_WriteOverCurrent(addr,
		robot.D50A_OverCurrent_mA,robot.D50A_OverCurrent_mA);
	if(ret == D50A_OK)
	{
		ret = D50A_WriteOverTime(addr,
			robot.D50A_OverTime_ms,robot.D50A_OverTime_ms);
	}
	if(ret == D50A_OK)
		ret = D50A_Save(addr);

	/* 给驱动器留出 EEPROM 保存时间，再读取确认实际参数。 */
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
 * 蓝牙 o/p 的专用改址流程：处理地址冲突和离线占位，B3 后以新地址读取 A0 确认。
 * 若目标已在线或旧地址已不存在，只做目标 A0 回查；两个地址均在线时拒绝操作。
 * B3 失败恢复目标占位；B3 成功但 A0 失败不回滚地址。确认标志和蜂鸣由任务在成功后处理。
 */
static uint8_t D50A_ProcessAddressSet(uint8_t old_addr,uint8_t new_addr)
{
	D50A_State_t *source = D50A_FindState(old_addr);
	D50A_State_t *target = D50A_FindState(new_addr);
	uint8_t target_disabled = 0;
	uint8_t ret;

	/* 如果目标地址已经在线，只做一次回查，避免两个在线设备地址冲突。 */
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

	/* 上一次改址后状态表中可能已经没有旧地址，只需回查新地址。 */
	if(source == NULL)
	{
		if(target == NULL)
			return D50A_ERR_NOT_INIT;

		d50a_active_state = target;
		ret = D50A_ReadStatus(target);
		return ret;
	}

	/* 新地址只是默认登记但未在线时，暂时禁用它，避免改址后重复轮询。 */
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

	/* B3 成功后使用新地址读取 A0 状态，确认模块确实已经按新地址响应。 */
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
 * 上电在 UART2_Init() 和 Robot_Select() 后调用：清软件状态并按配置登记地址，不发送参数命令。
 * 固定参数由任务在原有 IMU 等待窗口配置；运行期串口恢复不调用本函数，以免清掉历史报警。
 */
void D50A_Init(void)
{
	/* 这里只初始化设备表和接收状态，固定参数放到启动后的待机窗口中配置。 */
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
 * 唯一的运行期 D50A 通信任务：启动窗口配置→蓝牙改址请求→队列写请求→空闲轮询。
 * 只有普通请求/轮询按 10Hz 调度；启动配置及改址走各自流程，不以参数成功作为电机放行条件。
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
	uint8_t startup_window_finished = 0; // 窗口已结束，不代表所有驱动配置成功
	uint8_t startup_device_index = 0;
	uint8_t startup_device_ok_mask = 0; // bit i=槽位 i 已配置成功或被停用，本窗口不再重写
	u32 lastWakeTime = getSysTickCnt();

	/* 运行期任务启动后再创建请求队列，D50A_Init() 本身不依赖队列。 */
	if(d50a_req_queue == NULL)
	{
		d50a_req_queue = xQueueCreate(D50A_REQ_QUEUE_LEN,sizeof(D50A_Request_t));
	}

	while(1)
	{
		/*
		 * 仅在原有 IMU 等待窗口内配置；成功槽位不再重写，失败槽位轮转重试。
		 * 全部完成或 Time_count 达到 CONTROL_DELAY 都会退出，不阻挡原有电机自检。
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

		//UART4 中断只置请求；这里执行改址和 A0 回查，成功后才置确认标志并蜂鸣.
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

		/* 任务 10Hz 运行，写请求优先，空闲时周期轮询状态。 */
		vTaskDelayUntil(&lastWakeTime,F2T(D50A_TASK_RATE));

		if(d50a_req_queue == NULL)
		{
			continue;
		}

		/*
		 * 优先处理外部投递的写请求。
		 * 一次任务周期只处理一条请求，保证通信节奏可控。
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
								//广播查址只适用于单个驱动，成功后登记回复地址.
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
				//记录到原请求槽位；A4 查址可能已经将 active_state 切换到回复地址.
				state->last_request_error = ret;
			}
		}
		else
		{
			/*
			 * 每周期最多访问一个槽位；0/2/4/6/8 读 A0，1/5 读 A3，3 读 A1，7 读 A2，9 空闲。
			 * 同一步骤先遍历全部登记槽位，再切换类型；离线/停用槽位也会消耗一个调度周期。
			 * 10Hz 是任务频率，不是每台驱动每类数据的更新频率；写请求会占用本周期轮询。
			 */
			if(d50a_device_count != 0)
			{
				if(d50a_poll_device_index >= d50a_device_count) d50a_poll_device_index = 0;
				device_index = d50a_poll_device_index++;
				state = &d50a_devices[device_index];

				//单槽连续失败 3 次后，按上次失败时间至少间隔 1 秒重试；成功后清失败次数.
				//此退避只限制自动轮询，不限制启动配置、蓝牙改址或队列写请求.
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

					//步骤9为空闲步骤，不清除连续失败次数.
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
			/* 同一查询步骤轮询完全部设备，再切换查询类型。 */
			if(d50a_poll_device_index >= d50a_device_count)
			{
				poll_step++;
				if(poll_step >= 10) poll_step = 0;
			}
		}
	}
}

/* USART2 正常接收字节入口，仅组帧并置 ready；命令解析和重试均由任务完成。 */
void D50A_RxByte(uint8_t data)
{
	/*
	 * 收帧状态机：
	 * 1. 前3个字节必须连续收到 0xAA。
	 * 2. 收满13字节后检查下标9～12是否均为 0xBB。
	 * 3. 通过检查后复制到 rx_frame，并置位 rx_frame_ready。
	 *
	 * 这里运行在 USART2 中断中，只做极少量工作，不做业务解析。
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

/* USART2 接收错误入口：丢弃当前帧并锁存本事务错误，等待函数返回 ERR_UART。 */
void D50A_RxError(void)
{
	/* 中断内只丢弃坏帧，不重试、不延时、不发送命令。 */
	rx_index = 0;
	rx_frame_ready = 0;
	rx_uart_error = 1;
}

/* 投递 B0 请求，m1_ma/m2_ma 单位 mA；save 非 0 时，任务在写成功后追加 B4。 */
uint8_t D50A_PostWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save)
{
	D50A_Request_t request;

	/* 上层传入单位为 mA，内部发送前再换算为协议单位 10mA。 */
	request.type = D50A_REQ_WRITE_OVERCURRENT;
	request.addr = addr;
	request.new_addr = 0;
	request.m1_value = m1_ma;
	request.m2_value = m2_ma;
	request.save_after_write = save;
	return D50A_PostRequest(&request);
}

/* 投递 B1 请求，m1_ms/m2_ms 单位 ms；save 非 0 时，任务在写成功后追加 B4。 */
uint8_t D50A_PostWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save)
{
	D50A_Request_t request;

	/* 上层传入单位为 ms，协议中也是 ms。 */
	request.type = D50A_REQ_WRITE_OVERTIME;
	request.addr = addr;
	request.new_addr = 0;
	request.m1_value = m1_ms;
	request.m2_value = m2_ms;
	request.save_after_write = save;
	return D50A_PostRequest(&request);
}

/* 投递 B2 恢复参数请求；驱动自动保存，缓存由后续 A1/A2 轮询刷新。 */
uint8_t D50A_PostResetParam(uint8_t addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_RESET_PARAM;
	request.addr = addr;
	return D50A_PostRequest(&request);
}

/* 只投递 B3 改址请求；不包含蓝牙 o/p 流程的 A0 回查、确认标志或蜂鸣。 */
uint8_t D50A_PostWriteAddress(uint8_t old_addr,uint8_t new_addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_WRITE_ADDR;
	request.addr = old_addr;
	request.new_addr = new_addr;
	return D50A_PostRequest(&request);
}

/* 只投递 B4 保存请求，实际结果由任务写入对应槽位的 last_request_error。 */
uint8_t D50A_PostSave(uint8_t addr)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));
	request.type = D50A_REQ_SAVE;
	request.addr = addr;
	return D50A_PostRequest(&request);
}

/* 投递广播 A4 查址请求，回复地址会加入设备表；只用于总线上单个驱动的场景。 */
uint8_t D50A_PostReadAddress(void)
{
	D50A_Request_t request;
	memset(&request,0,sizeof(request));

	/*
	 * 广播查询地址只适合总线上单个驱动器。
	 * 多驱动器串联时不要调用该接口，否则多个设备可能同时回复。
	 */
	request.type = D50A_REQ_READ_ADDR;
	request.addr = D50A_BROADCAST_ADDR;
	return D50A_PostRequest(&request);
}

/*
 * 上电同步写 B0；参数范围和量化规则与任务写入一致，save 非 0 时继续同步 B4。
 * 返回实际应答校验结果；保存失败仍保留已经写成功的 RAM 参数缓存。
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

/* 上电同步写 B1，单位 ms；save 非 0 时继续同步 B4，返回实际执行结果。 */
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

/* 上电同步发送 B2 并校验回复；不立即重读参数，也不追加 B4。 */
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
 * 上电同步改址，发送前先检查新地址是否合法、是否已有其他启用槽位占用。
 * 只校验 B3 应答并修改原槽地址，不执行蓝牙流程的 A0 回查或蜂鸣。
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

/* 上电同步发送 B4 并校验应答；额外的保存延时及参数回读由调用者安排。 */
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
 * 登记地址或重新启用原槽位；新槽默认离线，未通信不代表驱动有保护。
 * 最多保留 D50A_MAX_DEVICES 个槽位，重新启用不会清掉旧缓存或已确认报警。
 */
uint8_t D50A_AddDevice(uint8_t addr)
{
	D50A_State_t *state;

	/* 0 和广播地址不能作为普通设备地址加入轮询表。 */
	if(addr == 0 || addr == D50A_BROADCAST_ADDR)
		return D50A_ERR_PARAM;

	/* 如果设备槽位曾经存在但被禁用，则直接重新启用。 */
	state = D50A_FindAnyState(addr);
	if(state != NULL)
	{
		state->enable = 1;
		return D50A_OK;
	}

	if(d50a_device_count >= D50A_MAX_DEVICES)
		return D50A_ERR_QUEUE_FULL;

	/* 新建一个设备槽位，等待后续轮询更新状态。 */
	state = &d50a_devices[d50a_device_count++];
	memset(state,0,sizeof(D50A_State_t));
	state->enable = 1;
	state->addr = addr;
	state->last_error = D50A_ERR_NOT_INIT;
	return D50A_OK;
}

/* 停用槽位并置离线，不删除/压缩数组，不减少登记数，也不清参数和保护缓存。 */
uint8_t D50A_RemoveDevice(uint8_t addr)
{
	D50A_State_t *state = D50A_FindState(addr);
	if(state == NULL)
		return D50A_ERR_PARAM;

	/* 不压缩数组，避免外部按 index 读取时状态突然移动。 */
	state->enable = 0;
	state->online = 0;
	return D50A_OK;
}

/* 返回登记槽位数，包含停用和离线槽位；不能作为实际接入数量或保护检查范围。 */
uint8_t D50A_GetDeviceCount(void)
{
	return d50a_device_count;
}

/*
 * 查询车型所需驱动的保护缓存，不发送命令，也不直接改变电机使能。
 * 由Robot_Select配置检查地址1或地址1和2；关闭自动管理或未确认保护时返回1，已确认保护返回0。
 * 已确认报警不会因离线而消失，只有该槽位有效的正常 A0 回复才能清除。
 */
uint8_t D50A_IsProtectionNormal(void)
{
	D50A_State_t *state;

	//未适配车型关闭自动管理，不因缺少驱动数据影响原有控制.
	if(robot.D50A_Enable==0) return 1;

	//改址后可能存在同地址的停用占位，优先检查正在使用的槽位.
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

/* 按地址返回启用槽位的只读缓存；未登记或已停用返回 NULL，离线仍可返回历史缓存。 */
const D50A_State_t* D50A_GetStateByAddr(uint8_t addr)
{
	return D50A_FindState(addr);
}

/* 按固定槽位下标返回只读缓存；越界或停用返回 NULL，改址不会移动槽位下标。 */
const D50A_State_t* D50A_GetStateByIndex(uint8_t index)
{
	if(index >= d50a_device_count)
		return NULL;
	if(d50a_devices[index].enable == 0)
		return NULL;

	/* 返回只读指针，调用者不要强制转换后修改内部状态。 */
	return &d50a_devices[index];
}
