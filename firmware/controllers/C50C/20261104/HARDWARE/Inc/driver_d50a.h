#ifndef __DRIVER_D50A_H
#define __DRIVER_D50A_H

#include "system.h"

/*
 * D50A 驱动器管理模块
 *
 * 本模块统一管理 USART2/RS485 上的 D50A 驱动器通信。
 * 其他模块不要直接往串口2发送 D50A 协议帧，避免多个任务同时抢占485总线。
 * 如果需要修改驱动器参数，应调用 D50A_PostXXX() 系列接口把请求投递给 D50A_Task。
 * 如果需要查看驱动器状态，应读取 D50A_State_t 缓存，而不是主动发读命令。
 *
 * 多驱动串联注意事项：
 * 1. 总线上每个驱动器都必须有唯一地址。
 * 2. 广播地址 0xFF 只适合总线上只有一个驱动器时使用。
 * 3. 多个驱动器同时接在485总线上时，不要广播查询地址，否则多个设备会同时回复，造成总线冲突。
 */

#define D50A_TASK_PRIO       2
#define D50A_STK_SIZE        256
#define D50A_TASK_RATE       10

#define D50A_DEFAULT_ADDR    0x01
#define D50A_SECOND_ADDR     0x02
#define D50A_BROADCAST_ADDR  0xFF
#define D50A_MAX_DEVICES     4

/* 蓝牙地址设置请求值。 */
#define D50A_ADDRESS_SET_NONE       0
#define D50A_ADDRESS_SET_1_TO_2     1
#define D50A_ADDRESS_SET_2_TO_1     2

/* 驱动器通信结果码：投递接口返回这些值，状态缓存里也会记录最近一次错误。 */
typedef enum
{
	D50A_OK = 0,        // 通信/回显校验成功，不等于驱动无保护报警
	D50A_ERR_TIMEOUT,   // 回复等待超时
	D50A_ERR_FRAME,     // 帧格式或地址/功能码/数据校验不符
	D50A_ERR_FUNC,      // 功能码错误预留结果码
	D50A_ERR_PARAM,     // 本地参数非法或设备槽位冲突
	D50A_ERR_EXCEPTION, // 驱动返回协议规定的异常帧
	D50A_ERR_QUEUE_FULL,// 写队列或设备槽位已满
	D50A_ERR_NOT_INIT,  // 请求队列尚未创建或指定设备不存在
	D50A_ERR_TX_TIMEOUT, // 串口发送未能完成
	D50A_ERR_UART,       // 接收溢出、噪声、帧或校验错误
}D50A_Result_t;

/* D50A_Task 内部处理的请求类型。 */
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
 * 投递到 D50A_Task 队列中的请求对象。
 *
 * m1_value/m2_value 使用上层更容易理解的实际单位：
 * - 写过流电流时：单位为 mA，例如 8000 表示 8000mA
 * - 写过流时间时：单位为 ms，例如 5000 表示 5000ms
 *
 * save_after_write 仅对 B0/B1 参数写入有效：
 * - 0：只写入 RAM，不立刻保存
 * - 1：写入成功后自动追加 B4 保存命令
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
 * 报警位解析结果。
 *
 * D50A 协议中报警位的含义比较容易写反：
 * - 位值为 1 表示正常
 * - 位值为 0 表示对应保护触发
 */
typedef struct
{
	uint8_t undervoltage_ok; // bit3：1=正常，0=欠压保护
	uint8_t overvoltage_ok;  // bit2：1=正常，0=过压保护
	uint8_t overtemp_ok;     // bit1：1=正常，0=过温保护
	uint8_t overcurrent_ok;  // bit0：1=正常，0=过流保护
}D50A_AlarmBits_t;

/*
 * 单个 D50A 地址对应的状态缓存。
 *
 * D50A_Task 会周期读取驱动器数据并更新该结构体。
 * 蓝牙 APP、OLED 菜单或其他模块需要显示状态时，直接读取该缓存即可。
 * 不建议这些模块绕过 D50A_Task 直接访问 USART2。
 */
typedef struct
{
	uint8_t enable;              // 是否启用该设备槽位，0=不轮询，1=参与轮询
	uint8_t addr;                // 驱动器地址
	uint8_t online;              // 最近一次通信是否成功，1=在线，0=离线或超时
	uint8_t last_error;          // 最近一次通信/校验结果，见 D50A_Result_t，不是驱动报警码
	uint8_t last_request;        // 最近一次处理的写请求类型
	uint8_t last_request_error;  // 最近一次写请求的执行结果

	uint8_t alarm;               // 有效 A0 回复的原始报警码；低四位全 1 为正常，默认 0 不能直接当报警
	D50A_AlarmBits_t alarm_bits; // 报警码按位解析后的结果
	uint8_t status_valid;        // 有效 A0 缓存标志；收发失败或本次 A0 读取失败时清 0
	volatile uint8_t protection_fault; // 0=未确认保护，1=已确认保护；仅有效 A0 更新，通信失败不清除
	uint32_t last_status_tick;   // 最近一次有效 A0 状态回复的 tick
	float m1_current_a;          // M1 实时电流，单位 A
	float m2_current_a;          // M2 实时电流，单位 A
	int16_t m1_current_raw;      // M1 实时电流原始值，协议值=实际电流A*100
	int16_t m2_current_raw;      // M2 实时电流原始值，协议值=实际电流A*100

	uint8_t m1_temp;             // M1 温度，单位 摄氏度
	uint8_t m2_temp;             // M2 温度，单位 摄氏度

	uint16_t m1_overcurrent_ma;  // M1 过流电流设置值，单位 mA
	uint16_t m2_overcurrent_ma;  // M2 过流电流设置值，单位 mA
	uint16_t m1_overtime_ms;     // M1 过流时间设置值，单位 ms
	uint16_t m2_overtime_ms;     // M2 过流时间设置值，单位 ms

	uint32_t last_update_tick;   // 最近一次格式完整的成功收发 tick；同步接口记 0，不等于 A0 更新时间

	uint32_t comm_success_count;    // 累计通信成功次数：收到合法回复帧并完成协议校验后递增
	uint32_t comm_timeout_count;    // 累计回复超时或字节发送超时次数，不含本地参数/入队失败
	uint32_t comm_frame_err_count;  // 累计已记录的 ERR_FRAME/ERR_EXCEPTION/ERR_UART 次数
}D50A_State_t;


/*
================================================================================
D50A API 调用示例和注意事项
================================================================================

一、总原则
--------------------------------------------------------------------------------
1. 串口2/485 总线由本模块统一管理，其他模块不要直接发送 D50A 协议帧。
2. FreeRTOS 启动前如果要写驱动器参数，使用 D50A_SyncXXX() 同步接口。
3. FreeRTOS 正常运行后，推荐使用 D50A_PostXXX() 异步接口，把请求交给 D50A_Task 处理。
4. 读取电流、温度、报警、参数设置值时，不需要主动发读命令，直接读取 D50A_State_t 缓存。
5. 多驱动器串联时，每个驱动器必须有唯一地址，例如 0x01、0x02、0x03。
6. 广播地址 0xFF 只能在总线上只有一个驱动器时用于查询地址，多驱动器时不要广播查询。

二、硬件初始化阶段：同步写初始参数
--------------------------------------------------------------------------------
适用场景：
- FreeRTOS 调度器还没有启动。
- 需要在 systemInit() 阶段给驱动器写入过流电流、过流时间等初始值。
- 此时不要使用 D50A_PostXXX()，因为 D50A_Task 还没有运行。

前置条件：
- UART2_Init(115200) 已经完成。
- D50A_Init() 已经完成。
- USART2 接收中断已经打开，并且中断函数中会调用 D50A_RxByte()。

示例：给地址 0x01 的驱动器设置默认参数，并立即保存。

    UART2_Init(115200);
    Robot_Select(); // 先确定D50A启用标志、数量和启动参数。
    D50A_Init();

    // 设置 M1/M2 过流电流为 8000mA，并自动保存。
    // 第4个参数 save=1 表示写成功后自动发送 B4 保存命令。
    D50A_SyncWriteOverCurrent(0x01, 8000, 8000, 1);

    // 设置 M1/M2 过流时间为 5000ms，并自动保存。
    D50A_SyncWriteOverTime(0x01, 5000, 5000, 1);

示例：连续写多个参数，最后统一保存。

    D50A_SyncWriteOverCurrent(0x01, 8000, 8000, 0);
    D50A_SyncWriteOverTime(0x01, 5000, 5000, 0);
    D50A_SyncSave(0x01);

注意事项：
- 同步接口会阻塞等待驱动器返回，单条命令默认最长等待 D50A_TIMEOUT_MS。
- 同步接口适合初始化阶段使用；系统运行后不建议再调用，避免和 D50A_Task 同时访问485总线。
- 修改参数前，建议确保电机停转。

三、FreeRTOS 运行期：异步写请求
--------------------------------------------------------------------------------
适用场景：
- 蓝牙 APP、OLED 菜单、按键菜单等运行期修改参数。
- 不希望调用者阻塞等待串口回复。
- 希望所有写操作都由 D50A_Task 排队处理，避免485总线冲突。

示例：蓝牙 APP 设置地址 0x02 的过流电流为 8000mA，并保存。

    uint8_t ret;
    ret = D50A_PostWriteOverCurrent(0x02, 8000, 8000, 1);
    if(ret != D50A_OK)
    {
        // 这里只表示请求没有成功放入队列，例如队列满。
        // 不代表驱动器执行失败。
    }

示例：运行期写过流时间。

    D50A_PostWriteOverTime(0x02, 5000, 5000, 1);

示例：连续写多个参数，最后统一保存。

    D50A_PostWriteOverCurrent(0x02, 8000, 8000, 0);
    D50A_PostWriteOverTime(0x02, 5000, 5000, 0);
    D50A_PostSave(0x02);

查看异步写请求是否真正执行成功：

    const D50A_State_t *drv = D50A_GetStateByAddr(0x02);
    if(drv != NULL)
    {
        if(drv->last_request_error == D50A_OK)
        {
            // 最近一次写请求执行成功。
        }
        else
        {
            // 最近一次写请求执行失败，错误码见 D50A_Result_t。
        }
    }

注意事项：
- D50A_PostXXX() 的返回值只表示“投递队列是否成功”。
- 驱动器实际执行结果要看对应设备状态中的 last_request_error。
- 投递不会将 last_request_error 改成等待状态，可能暂时读到上一条结果；蓝牙改址另有 Confirmed 标志。
- D50A_Task 会优先处理写请求，处理写请求的周期会暂停一次自动轮询。

四、读取驱动器状态缓存
--------------------------------------------------------------------------------
D50A_Task 会周期读取：
- 报警码和实时电流 A0
- 温度 A3
- 过流电流设置值 A1
- 过流时间设置值 A2

调用者不需要主动发读命令，直接读取缓存即可。

示例：读取地址 0x01 的实时电流、温度和报警状态。

    const D50A_State_t *drv = D50A_GetStateByAddr(0x01);
    if(drv != NULL && drv->online && drv->status_valid)
    {
        float m1_current = drv->m1_current_a;   // M1 实时电流，单位 A
        float m2_current = drv->m2_current_a;   // M2 实时电流，单位 A
        uint8_t m1_temp = drv->m1_temp;         // M1 温度，单位 摄氏度
        uint8_t m2_temp = drv->m2_temp;         // M2 温度，单位 摄氏度

        if(drv->alarm_bits.overcurrent_ok == 0)
        {
            // 过流保护触发。
        }
        if(drv->alarm_bits.overtemp_ok == 0)
        {
            // 过温保护触发。
        }
    }

报警位注意事项：
- D50A 协议中，报警位为 1 表示正常。
- D50A 协议中，报警位为 0 表示对应保护触发。
- 例如 overcurrent_ok == 0 表示过流保护触发，不是正常。
- 未收到有效 A0 时，默认 alarm=0 不能解释成四项保护全部触发。
- online 可被参数读写更新；实时报警应同时检查 status_valid 和 last_status_tick，OLED 当前按 2 秒判过期。
- last_error/D50A_OK 是通信结果；驱动保护看 alarm/alarm_bits，控制查询看 protection_fault。

五、多驱动器串联使用
--------------------------------------------------------------------------------
适用场景：
- 一条485总线上挂多个 D50A 驱动器。
- 每个驱动器使用不同地址。

示例：注册 3 个驱动器地址。

    D50A_AddDevice(0x01);
    D50A_AddDevice(0x02);
    D50A_AddDevice(0x03);

示例：按地址读取某个驱动器状态。

    const D50A_State_t *drv2 = D50A_GetStateByAddr(0x02);

示例：遍历所有已登记设备，适合批量显示在线状态。

    uint8_t i;
    for(i = 0; i < D50A_GetDeviceCount(); i++)
    {
        const D50A_State_t *drv = D50A_GetStateByIndex(i);
        if(drv != NULL)
        {
            // drv->addr          驱动器地址
            // drv->online        在线状态
            // drv->m1_temp       M1 温度
            // drv->m1_current_a  M1 电流
        }
    }

ByAddr 和 ByIndex 的区别：
- D50A_GetStateByAddr(addr)：按驱动器地址查状态，业务代码更推荐用这个。
- D50A_GetStateByIndex(index)：按内部设备表下标查状态，适合遍历所有设备。

多驱动注意事项：
- 不要让多个驱动器使用同一个地址。
- 不要在多驱动总线上使用 D50A_PostReadAddress() 广播查询地址。
- 如果要改地址，建议总线上只保留目标驱动器，避免误操作或地址冲突。

六、修改驱动器地址
--------------------------------------------------------------------------------
示例：把地址 0x01 修改为 0x02。

    D50A_PostWriteAddress(0x01, 0x02);

或在初始化阶段同步修改：

    D50A_SyncWriteAddress(0x01, 0x02);

注意事项：
- B3 修改地址命令会自动保存，不需要再调用 Save。
- 修改地址前，建议总线上只连接一个目标驱动器。
- 新地址不能为 0x00，也不能为广播地址 0xFF。
- 改址成功后，原设备槽位的地址会更新，槽位下标不移动。
- Post/Sync 改址接口只校验 B3；蓝牙 o/p 专用流程另外执行 A0 回查，成功才置确认标志并蜂鸣。
- 默认预登记了地址 1、2：同步改址会在发送前检查槽位冲突；蓝牙流程会处理离线目标占位。

七、恢复默认过流参数
--------------------------------------------------------------------------------
运行期异步恢复：

    D50A_PostResetParam(0x01);

初始化阶段同步恢复：

    D50A_SyncResetParam(0x01);

注意事项：
- B2 会恢复默认过流电流和过流时间，并自动保存。
- 恢复后，D50A_Task 后续轮询会重新读回设置值。
================================================================================
*/
extern TaskHandle_t d50a_TaskHandle;

/*
 * 上电在 UART2_Init() 和 Robot_Select() 之后调用，按车型配置登记地址，不发送配置命令。
 * 固定阈值/时间由任务在 IMU 等待窗口写入；不要把它当作运行期串口恢复接口。
 */
void D50A_Init(void);

/*
 * D50A 管理任务：
 * 1. 原有 IMU 等待窗口内配置固定参数，成功槽位跳过，失败重试，超时退出不阻挡电机自检。
 * 2. 蓝牙改址优先；普通周期按 10Hz 调度，每次处理一条队列请求或轮询一个登记槽位。
 * 3. 10Hz 不是每台驱动每种数据的刷新频率，槽位轮转和写请求都会分摊普通轮询。
 */
void D50A_Task(void *pvParameters);

/* USART2 接收中断中调用：把收到的一个字节喂给 D50A 收帧状态机。 */
void D50A_RxByte(uint8_t data);
/* USART2 接收错误时丢弃当前帧，并通知通信任务返回错误。 */
void D50A_RxError(void);

/*
 * UART4 收到 o/p 只置请求：o 表示 1→2，p 表示 2→1，每次新请求先清 Confirmed。
 * D50A_Task 执行 B3 或目标回查；有效 A0 确认成功后置 Confirmed=1，并调用蜂鸣接口。
 * 此确认标志只表示蓝牙专用改址流程结果，不表示驱动一直在线或没有保护报警。
 */
extern volatile uint8_t D50A_AddressSetRequest;
extern volatile uint8_t D50A_AddressSetConfirmed;

/*
 * 异步写请求接口。
 *
 * 返回值只代表“请求是否成功放入队列”，不代表驱动器已经执行成功。
 * 驱动器实际执行结果请查看对应设备状态中的 last_request_error。
 * B0 输入 1000～16000mA，按 10mA 向下取整；B1 输入 10～10000ms，由任务执行时校验。
 * save 非 0 表示 B0/B1 成功后追加 B4；B2/B3 自带保存，不需要额外 B4。
 */
uint8_t D50A_PostWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save);
uint8_t D50A_PostWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save);
uint8_t D50A_PostResetParam(uint8_t addr);
uint8_t D50A_PostWriteAddress(uint8_t old_addr,uint8_t new_addr);
uint8_t D50A_PostSave(uint8_t addr);
uint8_t D50A_PostReadAddress(void);

/*
 * 硬件初始化阶段使用的同步写接口。
 *
 * 这些接口不依赖 D50A_Task，也不走 FreeRTOS 队列，适合在调度器启动前设置驱动器参数。
 * 使用前必须先完成 UART2_Init() 和 D50A_Init()，并保证 USART2 接收中断能调用 D50A_RxByte()。
 * 运行期仍建议优先使用 D50A_PostXXX()，避免和 D50A_Task 同时访问485总线。
 */
uint8_t D50A_SyncWriteOverCurrent(uint8_t addr,uint16_t m1_ma,uint16_t m2_ma,uint8_t save);
uint8_t D50A_SyncWriteOverTime(uint8_t addr,uint16_t m1_ms,uint16_t m2_ms,uint8_t save);
uint8_t D50A_SyncResetParam(uint8_t addr);
uint8_t D50A_SyncWriteAddress(uint8_t old_addr,uint8_t new_addr);
uint8_t D50A_SyncSave(uint8_t addr);

/*
 * 多驱动器设备表管理：Count 是登记槽位数，包含停用/离线槽位，不是在线数量。
 * Add 同地址只重新启用，Remove 只停用、不压缩或清报警；ByAddr/ByIndex 对停用槽返回 NULL。
 */
uint8_t D50A_AddDevice(uint8_t addr);
uint8_t D50A_RemoveDevice(uint8_t addr);
uint8_t D50A_GetDeviceCount(void);
/*
 * 保护汇总查询，不发送串口命令，可在电机失能判断中直接调用。
 * 按Robot_Select配置检查地址1或地址1和2；未启用自动管理的车型直接返回1。
 * 返回 1=所检查驱动没有已确认的保护报警，0=所检查驱动存在保护报警。
 * 过流、过压、欠压、过温均参与判断，仅有效 A0 正常回复能清除对应驱动报警。
 * 未收到过 A0 的设备不阻止运行，返回 1 不保证驱动在线；离线/停用不清除已确认的报警。
 * 本接口只查询，电机停机由控制任务结合返回值处理；启用和数量不依赖登记/在线槽位数。
 */
uint8_t D50A_IsProtectionNormal(void);
const D50A_State_t* D50A_GetStateByAddr(uint8_t addr);    // 按驱动器地址查询状态，适合业务代码使用
const D50A_State_t* D50A_GetStateByIndex(uint8_t index);  // 按设备表下标查询状态，适合遍历所有已登记设备


#endif
