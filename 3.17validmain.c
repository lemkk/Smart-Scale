
/**
 * @file main.c
 * @brief Smart Scale BLE应用主程序
 *
 * 功能概述：
 *  - 硬件上电后进入待机状态，不做任何工作
 *  - 长按按键1秒后系统开机：开始BLE广播、采集HX711重量数据
 *  - BLE广播名称由8位拨码开关决定，格式为 "Smart Scale-XXX"（000~255）
 *  - 手机连接后实时接收重量数据（格式：x.xxx kg）
 *  - LED状态：广播中闪烁，连接后常亮
 *  - 长按按键3秒后系统关机：断开BLE，进入低功耗模式
 *
 * 硬件平台：nRF52840 自定义PCB
 * 基于 Nordic Semiconductor nRF5 SDK BLE NUS示例
 *
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 */
// ============================================================
// 头文件
// ============================================================
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_saadc.h"
#include "fds.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


// ============================================================
// BLE 参数配置
// ============================================================
#define APP_BLE_CONN_CFG_TAG            1                                /**< SoftDevice BLE配置标签 */
#define DEVICE_NAME                     "Smart Scale"                    /**< 设备名称基础部分，完整名称由拨码开关动态生成 */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN       /**< Nordic UART Service UUID类型 */
#define APP_BLE_OBSERVER_PRIO           3                                /**< BLE观察者优先级 */

#define APP_ADV_INTERVAL                64                               /**< 广播间隔（单位0.625ms，此处为40ms）*/
#define APP_ADV_DURATION                18000                            /**< 广播持续时间（单位10ms，此处为180秒）*/

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< 最小连接间隔 200ms */   //测试
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS) /**< 最大连接间隔 1000ms */  //测试
//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< 最小连接间隔 20ms */
//#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS) /**< 最大连接间隔 75ms */

#define SLAVE_LATENCY                   0                                /**< 从机延迟 */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS) /**< 连接超时 4秒 */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)           /**< 首次连接参数更新延迟 5秒 */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)          /**< 后续连接参数更新间隔 30秒 */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                /**< 连接参数协商最大尝试次数 */

#define DEAD_BEEF                       0xDEADBEEF                      /**< 错误码，用于栈溢出定位 */
#define UART_TX_BUF_SIZE                256                              /**< UART TX缓冲区大小（未使用，保留兼容性）*/
#define UART_RX_BUF_SIZE                256                              /**< UART RX缓冲区大小（未使用，保留兼容性）*/


// ============================================================
// GPIO 引脚定义
// ============================================================

// HX711 称重传感器
#define HX711_DT_PIN    NRF_GPIO_PIN_MAP(0, 4)  /**< HX711 数据引脚 DOUT */
#define HX711_SCK_PIN   NRF_GPIO_PIN_MAP(0, 6)  /**< HX711 时钟引脚 SCK  */

// 电源控制
#define PWR_EN_PIN      NRF_GPIO_PIN_MAP(0, 15) /**< 电源自锁引脚，上电后立即拉高保持供电 */
#define BUTTON_PIN      NRF_GPIO_PIN_MAP(0, 17) /**< 开关机按键引脚，按下时为低电平 */

// LED 指示灯（低电平亮）
#define LED_R           NRF_GPIO_PIN_MAP(0, 12) /**< 红色LED，由BSP模块控制广播/连接状态 */
#define LED_G           NRF_GPIO_PIN_MAP(1, 9)  /**< 绿色LED（预留）*/
#define LED_B           NRF_GPIO_PIN_MAP(0, 8)  /**< 蓝色LED（预留）*/

// 电池电压检测
#define BAT_CTRL_PIN    NRF_GPIO_PIN_MAP(0, 29) /**< 控制分压电路 N-ch MOS Q3（高电平导通）*/
#define BAT_ADC_CHANNEL 0                       /**< SAADC通道0，对应AIN7（P0.31）*/

// 拨码开关（8位，ON=低电平，一端接GND，另一端经5.1k电阻接GPIO）
// 物理左数第1位对应bit0（值=1），左数第8位对应bit7（值=128）
// DIP_ADDR0 使用 p0.09/NFC1 引脚，需在main()中将其切换为GPIO模式
#define DIP_ADDR0       NRF_GPIO_PIN_MAP(0, 9)  /**< 拨码位7（物理左数第8位，值=128）*/
#define DIP_ADDR1       NRF_GPIO_PIN_MAP(1, 6)  /**< 拨码位6（物理左数第7位，值=64） */
#define DIP_ADDR2       NRF_GPIO_PIN_MAP(1, 4)  /**< 拨码位5（物理左数第6位，值=32） */
#define DIP_ADDR3       NRF_GPIO_PIN_MAP(1, 2)  /**< 拨码位4（物理左数第5位，值=16） */
#define DIP_ADDR4       NRF_GPIO_PIN_MAP(1, 0)  /**< 拨码位3（物理左数第4位，值=8）  */
#define DIP_ADDR5       NRF_GPIO_PIN_MAP(0, 24) /**< 拨码位2（物理左数第3位，值=4）  */
#define DIP_ADDR6       NRF_GPIO_PIN_MAP(0, 22) /**< 拨码位1（物理左数第2位，值=2）  */
#define DIP_ADDR7       NRF_GPIO_PIN_MAP(0, 20) /**< 拨码位0（物理左数第1位，值=1）  */


//FDS参数配置
#define TARE_FILE_ID     0x0001  // FDS文件ID
#define TARE_RECORD_KEY  0x0001  // FDS记录key


// ============================================================
// 应用参数配置
// ============================================================
#define GapValue                437              /**< HX711 重量换算系数（每克对应的原始值差）*///419
#define WEIGHT_MEAS_INTERVAL    APP_TIMER_TICKS(1000) /**< 重量采样间隔：1秒 */
#define HOLD_ON_MS              1000             /**< 长按开机所需时间：1秒 */
#define HOLD_OFF_MS             3000             /**< 长按关机所需时间：3秒 */
#define BAT_VOLTAGE_DIV  4.33f   /**< 分压系数：(R10+R13)/R13 = (33.2+10)/10 */


// ============================================================
// 全局变量
// ============================================================
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< 当前BLE连接句柄，无连接时为INVALID */
static uint32_t m_button_hold_count = 0;                 /**< 按键持续按下的100ms计数 */
static bool     m_system_on = false;                     /**< 系统工作状态标志 */
static bool     g_led_state = false;                     /**< LED工作状态标志 */
static long     Weight_Maopi = 0;                        /**< HX711零点基准值（毛皮值）*/
static volatile     bool g_tare_pending = false;                      /**<去皮残差补偿*/
static uint16_t g_bat_mv_history[4] = {0};  // 电池电压滑动平均历史缓冲
static uint8_t  g_bat_mv_index = 0;          // 当前写入索引
static bool     g_bat_history_full = false;  // 历史缓冲是否已填满

static bool g_bat_low = false;  //低电量标志

static uint32_t g_bat_log_counter = 0;  //日志中电量更新计数器

static volatile bool g_tare_requested = false;           /**< 去皮请求标志 */


static volatile bool g_fds_initialized = false;  // FDS初始化完成标志
static volatile bool g_fds_write_done  = false;  // FDS写入完成标志

// ---- 采样状态机 ----
// 每次1秒主定时器触发后，通过单次定时器串联以下阶段（每个回调 < 1ms，零阻塞）：
//   PHASE 0: 拉低BAT_CTRL_PIN → 等50ms
//   PHASE 1: ADC采样         → 等100ms（给HX711第1次就绪）
//   PHASE 2: HX711读数#1     → 等100ms
//   PHASE 3: HX711读数#2     → 等100ms
//   PHASE 4: HX711读数#3，计算，发送，复位
typedef enum { MEAS_PHASE_BAT_ENABLE = 0,
               MEAS_PHASE_ADC_SAMPLE,
               MEAS_PHASE_HX_1,
               MEAS_PHASE_HX_2,
               MEAS_PHASE_HX_3 } meas_phase_t;

static meas_phase_t g_meas_phase  = MEAS_PHASE_BAT_ENABLE;
static float        g_hx_sum      = 0.0f;  /**< HX711三次读数累加 */
static uint8_t      g_bat_percent = 0;     /**< 本轮计算出的电量百分比，供PHASE4发送 */

APP_TIMER_DEF(m_weight_timer_id);  /**< 1秒主采样定时器（重复） */
APP_TIMER_DEF(m_meas_step_timer);  /**< 采样状态机单步定时器（单次） */
APP_TIMER_DEF(m_button_timer_id);  /**< 按键检测定时器 */
APP_TIMER_DEF(m_led_timer_id);     /**< LED闪烁定时器 */
APP_TIMER_DEF(m_bat_low_blink_timer);   /**< 低电量LED闪烁定时器 */

// 调试用计数器
static volatile uint32_t g_interrupt_count = 0;   /**< 按键中断触发次数 */
static volatile uint32_t g_timer_start_count = 0; /**< 按键定时器启动次数 */

// 前向声明
static void advertising_start(void);



// ============================================================
// 电池电压检测（SAADC）
// ============================================================

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    (void)p_event; // 仅用阻塞式单次采样，回调不需要处理
}

static void bat_adc_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t ch_cfg =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);

    err_code = nrf_drv_saadc_channel_init(0, &ch_cfg);
    APP_ERROR_CHECK(err_code);
}

/**@brief 导通电池分压电路（无延迟，立刻返回）*/
static void bat_adc_enable(void)
{
    nrf_gpio_cfg_output(BAT_CTRL_PIN);
    nrf_gpio_pin_clear(BAT_CTRL_PIN);  // 低电平导通 P-ch Q5
}

/**@brief 采样ADC并关断分压电路（无延迟，16次连续采样）
 * @return 电池电压 mV
 */
static uint16_t bat_adc_sample_and_disable(void)
{
    nrf_saadc_value_t adc_val = 0;
    int32_t sum = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
        nrf_drv_saadc_sample_convert(0, &adc_val);
        sum += adc_val;
    }
    adc_val = (nrf_saadc_value_t)(sum / 16);
    nrf_gpio_pin_set(BAT_CTRL_PIN);  // 关断分压电路

    //NRF_LOG_INFO("ADC raw: %d", adc_val);
    if (adc_val < 0) return 0;

    float v_ain_mv = (float)adc_val * 3600.0f / 16384.0f;
    float v_bat_mv = v_ain_mv * BAT_VOLTAGE_DIV;
    return (uint16_t)v_bat_mv;
}

//电量计算
static uint8_t bat_percent_get(uint16_t mv)
{
    if (mv >= 4200) return 100;
    if (mv <= 3500) return 0;
    return (uint8_t)((mv - 3500) * 100 / (4200 - 3500));
}

// ============================================================
// HX711 称重传感器驱动
// ============================================================

/**@brief 从HX711读取一次原始ADC值（24位）
 *
 * @details 通过手动模拟SPI时序读取HX711输出。
 *          读取完成后发送第25个时钟脉冲，设置增益为128（通道A）。
 *          加入超时保护防止HX711未就绪时卡死。
 *
 * @return 24位原始ADC值，超时返回0
 */
static uint32_t HX711_Read(void)
{
    uint32_t count = 0;
    uint8_t  i;

    nrf_gpio_cfg_input(HX711_DT_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_delay_us(1);
    nrf_gpio_pin_clear(HX711_SCK_PIN);
    nrf_delay_us(1);

    // 等待DOUT拉低（表示数据就绪），加入超时保护
    uint32_t timeout = 100000;
    while (nrf_gpio_pin_read(HX711_DT_PIN) && timeout > 0) timeout--;
    if (timeout == 0) return 0;

    // 读取24位数据
    for (i = 0; i < 24; i++)
    {
        nrf_gpio_pin_set(HX711_SCK_PIN);
        nrf_delay_us(1);
        count = count << 1;
        nrf_gpio_pin_clear(HX711_SCK_PIN);
        nrf_delay_us(1);
        if (nrf_gpio_pin_read(HX711_DT_PIN))
            count++;
    }

    // 第25个脉冲：完成本次读取，同时设置下次增益为128（通道A）
    nrf_gpio_pin_set(HX711_SCK_PIN);
    count ^= 0x800000; // 补码转换
    nrf_delay_us(1);
    nrf_gpio_pin_clear(HX711_SCK_PIN);
    nrf_delay_us(1);

    return count;
}

/**@brief 初始化HX711引脚 */
static void Init_Hx711(void)
{
    nrf_gpio_cfg_output(HX711_SCK_PIN);
    nrf_gpio_pin_clear(HX711_SCK_PIN);
    nrf_gpio_cfg_input(HX711_DT_PIN, NRF_GPIO_PIN_NOPULL);
}

/**@brief 采集零点基准值（毛皮值）
 *
 * @details 在系统开机时调用，将当前无负载的ADC值作为零点基准。
 */
static void Get_Maopi(void)
{
    long sum = 0;
    for (uint8_t i = 0; i < 5; i++)
    {
        sum += (long)HX711_Read();
        nrf_delay_ms(100);
    }
    Weight_Maopi = sum / 5;
}

/**@brief 获取当前重量（克）
 *
 * @details 读取HX711原始值，减去零点基准后除以换算系数。
 *
 * @return 重量值，单位：克
 */
static long Get_Weight(void)
{
    float sum = 0;
    for (uint8_t i = 0; i < 3; i++)
    {   
        nrf_delay_ms(100);
        sum += (float)(long)HX711_Read() - (float)Weight_Maopi;
    }
    return (long)(sum / 3.0f / (float)GapValue);
}


// ============================================================
// 拨码开关驱动
// ============================================================

/**@brief 读取8位拨码开关的当前值
 *
 * @details 所有引脚配置为内部上拉输入。拨码开关ON时接GND（低电平），
 *          OFF时由内部上拉保持高电平。读取后取反，使ON=1、OFF=0。
 *          物理左数第1位对应bit0（最低位，值=1），
 *          物理左数第8位对应bit7（最高位，值=128）。
 *          注意：DIP_ADDR0使用p0.09/NFC引脚，需在main()中预先切换为GPIO模式。
 *
 * @return 0~255的拨码开关值
 */
static uint8_t read_dip_switch(void)
{
    // 配置所有拨码引脚为内部上拉输入
    nrf_gpio_cfg_input(DIP_ADDR0, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(DIP_ADDR1, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(DIP_ADDR2, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(DIP_ADDR3, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(DIP_ADDR4, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(DIP_ADDR5, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(DIP_ADDR6, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(DIP_ADDR7, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(1); // 等待电平稳定

    // 读取各位原始电平（物理左数第1位→bit0，第8位→bit7）
    uint32_t p0 = nrf_gpio_pin_read(DIP_ADDR7); // 物理左数第1位 → bit0
    uint32_t p1 = nrf_gpio_pin_read(DIP_ADDR6); // 物理左数第2位 → bit1
    uint32_t p2 = nrf_gpio_pin_read(DIP_ADDR5); // 物理左数第3位 → bit2
    uint32_t p3 = nrf_gpio_pin_read(DIP_ADDR4); // 物理左数第4位 → bit3
    uint32_t p4 = nrf_gpio_pin_read(DIP_ADDR3); // 物理左数第5位 → bit4
    uint32_t p5 = nrf_gpio_pin_read(DIP_ADDR2); // 物理左数第6位 → bit5
    uint32_t p6 = nrf_gpio_pin_read(DIP_ADDR1); // 物理左数第7位 → bit6
    uint32_t p7 = nrf_gpio_pin_read(DIP_ADDR0); // 物理左数第8位 → bit7

    NRF_LOG_INFO("DIP raw 0-3: %d %d %d %d", p0, p1, p2, p3);
    NRF_LOG_INFO("DIP raw 4-7: %d %d %d %d", p4, p5, p6, p7);

    // 组合各位
    uint8_t val = 0;
    val |= (p0 << 0);
    val |= (p1 << 1);
    val |= (p2 << 2);
    val |= (p3 << 3);
    val |= (p4 << 4);
    val |= (p5 << 5);
    val |= (p6 << 6);
    val |= (p7 << 7);

    // 取反：ON（低电平）→ 1，OFF（高电平）→ 0
    val = (~val) & 0xFF;

    NRF_LOG_INFO("DIP val: %d", val);

    return val;
}


// ============================================================
// 电源管理 / 开关机
// ============================================================

/**@brief 系统关机
 *
 * @details 停止重量采样，断开BLE连接，配置按键为低功耗唤醒源，
 *          然后进入system-off深度休眠。释放PWR_EN_PIN断开自锁供电。
 */
static void system_off(void)
{
    NRF_LOG_INFO("System OFF");
    app_timer_stop(m_led_timer_id);        // 停止闪烁
    nrf_gpio_pin_set(LED_B);               // 熄灭
    nrf_delay_ms(500);

    // 停止重量采样定时器
    app_timer_stop(m_weight_timer_id);
    app_timer_stop(m_meas_step_timer);     // 停止状态机步进定时器
    nrf_gpio_pin_set(BAT_CTRL_PIN);        // 确保分压电路关断
    g_meas_phase = MEAS_PHASE_BAT_ENABLE; // 复位状态机

    // 如果当前有BLE连接，先断开
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        nrf_delay_ms(200);
    }

    // 配置按键为低电平唤醒源（下次长按可重新开机）
    nrf_gpio_cfg_sense_input(BUTTON_PIN,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_SENSE_LOW);

    m_system_on = false;

    // 进入深度休眠（此函数不返回）
    sd_power_system_off();

    // 释放自锁，断开硬件供电（理论上不会执行到这里）
    nrf_gpio_pin_clear(PWR_EN_PIN);
}

/**@brief 系统开机
 *
 * @details 采集HX711零点基准，启动重量采样定时器，开始BLE广播。
 */
static void system_on(void)
{
    m_system_on = true;
    NRF_LOG_INFO("System ON");

    nrf_gpio_cfg_output(LED_B);  // 初始化蓝灯引脚为输出
    nrf_gpio_pin_set(LED_B);  // 先熄灭
    

    //nrf_delay_ms(2000);  // 等待HX711稳定
    // 如果Flash里有有效baseline就直接用，否则自动归零
    if (Weight_Maopi == 0)
    {
        g_tare_pending = true;
    }
    NRF_LOG_INFO("Maopi baseline: %ld", Weight_Maopi);

    // 启动1秒周期的重量采样定时器
    app_timer_start(m_weight_timer_id, WEIGHT_MEAS_INTERVAL, NULL);
    nrf_gpio_pin_set(LED_B);  // 确保timer启动时LED是熄灭状态
    app_timer_start(m_led_timer_id, APP_TIMER_TICKS(500), NULL);  // 开始闪烁

    // 开始BLE广播，手机端可搜索到设备
    advertising_start();
}

// ============================================================
// FDS事件回调和读写函数
// ============================================================
// FDS事件回调
static void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                g_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        case FDS_EVT_UPDATE:
            if (p_evt->result == NRF_SUCCESS)
            {
                g_fds_write_done = true;
            }
            break;

        default:
            break;
    }
}

// 保存baseline到Flash
static void fds_baseline_save(long baseline)
{
    static uint32_t data;  // static，生命周期贯穿整个程序
    data = (uint32_t)baseline;

    fds_record_t        record;
    fds_record_desc_t   desc = {0};
    fds_find_token_t    tok  = {0};

    record.file_id           = TARE_FILE_ID;
    record.key               = TARE_RECORD_KEY;
    record.data.p_data       = &data;
    record.data.length_words = 1;

    g_fds_write_done = false;

    if (fds_record_find(TARE_FILE_ID, TARE_RECORD_KEY, &desc, &tok) == NRF_SUCCESS)
    {
        fds_record_update(&desc, &record);
    }
    else
    {
        fds_record_write(&desc, &record);
    }

    NRF_LOG_INFO("Baseline saved to flash: %ld", baseline);
}

// 从Flash读取baseline，返回true表示读到有效值
static bool fds_baseline_load(long * p_baseline)
{
    fds_record_desc_t   desc = {0};
    fds_find_token_t    tok  = {0};
    fds_flash_record_t  flash_record;

    if (fds_record_find(TARE_FILE_ID, TARE_RECORD_KEY, &desc, &tok) == NRF_SUCCESS)
    {
        if (fds_record_open(&desc, &flash_record) == NRF_SUCCESS)
        {
            uint32_t * p_data = (uint32_t *)flash_record.p_data;
            *p_baseline = (long)(*p_data);
            NRF_LOG_INFO("Raw data from flash: %u", *p_data);  // 先打印原始值确认
            fds_record_close(&desc);
            NRF_LOG_INFO("Baseline loaded from flash: %ld", *p_baseline);
            return true;
        }
    }
    return false;
}

// ============================================================
// 按键检测
// ============================================================

/**@brief 按键检测定时器回调（每100ms触发一次）
 *
 * @details 检测按键是否持续按下：
 *          - 系统关闭时：按满1秒触发开机
 *          - 系统开启时：按满3秒触发关机
 *          - 中途松开：重置计数
 */
static void button_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (nrf_gpio_pin_read(BUTTON_PIN) == 0) // 按键仍在按下（低电平）
    {
        m_button_hold_count++;
        if (m_system_on && m_button_hold_count >= (HOLD_OFF_MS / 100))
        {
            // 开机状态下按满3秒：关机
            m_button_hold_count = 0;
            app_timer_stop(m_button_timer_id);
            system_off();
            return;
        }
    }
    else // 按键松开，重置计数
    {
        if (!m_system_on && m_button_hold_count > 0)    //点按开机
        {
        m_button_hold_count = 0;
        app_timer_stop(m_button_timer_id);
        system_on();
        return;
        }
        if (m_system_on && m_button_hold_count > 0 
            && m_button_hold_count < (HOLD_OFF_MS / 100))   //去皮
        {
            g_tare_requested = true;  // 只设标志，立刻返回
        }
        m_button_hold_count = 0;
        app_timer_stop(m_button_timer_id);
    }
}

/**@brief 按键GPIO中断回调
 *
 * @details 检测到按键下降沿（按下瞬间）时启动100ms周期的检测定时器。
 */
static void button_interrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    g_interrupt_count++;
    if (pin == BUTTON_PIN)
    {
        m_button_hold_count = 0;
        ret_code_t err = app_timer_start(m_button_timer_id, APP_TIMER_TICKS(100), NULL);
        if (err == NRF_SUCCESS) g_timer_start_count++;
    }
}

/**@brief 初始化按键GPIO中断
 *
 * @details 配置BUTTON_PIN为下降沿中断（内部上拉），按下时触发检测定时器。
 */
static void button_init(void)
{
    ret_code_t err_code;

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    config.pull = NRF_GPIO_PIN_PULLUP;

    nrf_drv_gpiote_in_init(BUTTON_PIN, &config, button_interrupt_handler);
    nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
}


// ============================================================
// BLE 模块实例
// ============================================================
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);  /**< BLE NUS服务实例 */
NRF_BLE_GATT_DEF(m_gatt);                          /**< GATT模块实例 */
NRF_BLE_QWR_DEF(m_qwr);                            /**< Queued Write模块实例 */
BLE_ADVERTISING_DEF(m_advertising);                /**< 广播模块实例 */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< BLE单包最大数据长度 */
static ble_uuid_t m_adv_uuids[] =                                       /**< 广播UUID列表 */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


// ============================================================
// 定时器
// ============================================================

/**@brief 初始化app_timer模块 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief 重量采样定时器回调（每1秒触发）
 *
 * @details 读取HX711重量数据，通过RTT日志输出，
 *          如果当前有BLE连接则同时通过NUS发送到手机。
 *          数据格式："%d.%03d kg\r\n"
 */
 
/**@brief 采样状态机单步回调（单次定时器，每步 < 1ms，零阻塞）
 *
 * 状态转移：
 *   BAT_ENABLE  →(50ms)→ ADC_SAMPLE →(100ms)→ HX_1 →(100ms)→ HX_2 →(100ms)→ HX_3
 *   HX_3完成后计算均值、发送数据，复位状态机等待下一个1秒主定时器。
 */
static void meas_step_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    switch (g_meas_phase)
    {
        // ----------------------------------------------------------------
        // PHASE 1: 分压电路已稳定50ms，采样ADC，然后等100ms给HX711第1次就绪
        // ----------------------------------------------------------------
        case MEAS_PHASE_ADC_SAMPLE:
        {
            uint16_t bat_mv_now = bat_adc_sample_and_disable();

            // 滑动平均
            g_bat_mv_history[g_bat_mv_index] = bat_mv_now;
            g_bat_mv_index = (g_bat_mv_index + 1) % 4;
            if (g_bat_mv_index == 0) g_bat_history_full = true;

            uint8_t  valid_count = g_bat_history_full ? 4 : g_bat_mv_index;
            uint32_t sum = 0;
            for (uint8_t i = 0; i < valid_count; i++) sum += g_bat_mv_history[i];
            uint16_t bat_mv  = (uint16_t)(sum / valid_count);
            g_bat_percent    = bat_percent_get(bat_mv);
            g_bat_log_counter++;
            if (g_bat_log_counter >= 30)
            {
                g_bat_log_counter = 0;
                NRF_LOG_INFO("BAT: %d mV, %d%%", bat_mv, g_bat_percent);
            }

            if (g_bat_percent == 0)
            {
                NRF_LOG_INFO("Battery low, shutting down...");
                system_off();
                return;
            }

            // 低电量红灯控制
            if (g_bat_percent <= 20 && !g_bat_low)
            {
                g_bat_low = true;
                nrf_gpio_pin_set(LED_R);  // 确保从熄灭状态开始
                app_timer_start(m_bat_low_blink_timer, APP_TIMER_TICKS(500), NULL);
            }
            else if (g_bat_percent > 20 && g_bat_low)
            {
                g_bat_low = false;
                app_timer_stop(m_bat_low_blink_timer);
                nrf_gpio_pin_set(LED_R);  // 熄灭
            }

            g_hx_sum   = 0.0f;
            g_meas_phase = MEAS_PHASE_HX_1;
            app_timer_start(m_meas_step_timer, APP_TIMER_TICKS(100), NULL);
            break;
        }

        // ----------------------------------------------------------------
        // PHASE 2: HX711 第1次读数
        // ----------------------------------------------------------------
        case MEAS_PHASE_HX_1:
            g_hx_sum += (float)(long)HX711_Read() - (float)Weight_Maopi;
            g_meas_phase = MEAS_PHASE_HX_2;
            app_timer_start(m_meas_step_timer, APP_TIMER_TICKS(100), NULL);
            break;

        // ----------------------------------------------------------------
        // PHASE 3: HX711 第2次读数
        // ----------------------------------------------------------------
        case MEAS_PHASE_HX_2:
            g_hx_sum += (float)(long)HX711_Read() - (float)Weight_Maopi;
            g_meas_phase = MEAS_PHASE_HX_3;
            app_timer_start(m_meas_step_timer, APP_TIMER_TICKS(100), NULL);
            break;

        // ----------------------------------------------------------------
        // PHASE 4: HX711 第3次读数，计算，发送，复位
        // ----------------------------------------------------------------
        case MEAS_PHASE_HX_3:
        {
            g_hx_sum += (float)(long)HX711_Read() - (float)Weight_Maopi;

            if (g_tare_pending)
            {
                g_tare_pending = false;
                Weight_Maopi += (long)(g_hx_sum / 3.0f);
                NRF_LOG_INFO("Tare done, new baseline=%ld", Weight_Maopi);
                fds_baseline_save(Weight_Maopi);    //向flash中存入basline值
                g_meas_phase = MEAS_PHASE_BAT_ENABLE;
                break;
            }
            
            NRF_LOG_INFO("Raw diff: %ld", (long)(g_hx_sum / 3.0f)); //标定时用
            long weight_g = (long)(g_hx_sum / 3.0f / (float)GapValue);
            long kg   = weight_g / 1000;
            long gram = weight_g % 1000;
            if (gram < 0) gram = -gram;

            char buf[48];
            int len;
            if (weight_g < 0)
            {
                len = snprintf(buf, sizeof(buf), "-%ld.%03ld kg BAT:%d%%\r\n",
                               -kg, gram, g_bat_percent);
                NRF_LOG_INFO("Weight: -%ld.%03ld kg", -kg, gram);
            }
            else
            {
                len = snprintf(buf, sizeof(buf), "%ld.%03ld kg BAT:%d%%\r\n",
                               kg, gram, g_bat_percent);
                NRF_LOG_INFO("Weight: %ld.%03ld kg", kg, gram);
            }

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                uint16_t length = (uint16_t)len;
                uint32_t err_code = ble_nus_data_send(&m_nus, (uint8_t *)buf,
                                                      &length, m_conn_handle);
            if ((err_code != NRF_SUCCESS)             &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES)     &&
                (err_code != NRF_ERROR_NOT_FOUND)     &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) &&
                (err_code != NRF_ERROR_NO_MEM))
            {
                APP_ERROR_CHECK(err_code);
            }
            }

            // 复位，等待下一次1秒主定时器
            g_meas_phase = MEAS_PHASE_BAT_ENABLE;
            break;
        }

        default:
            g_meas_phase = MEAS_PHASE_BAT_ENABLE;
            break;
    }
}

/**@brief 1秒主定时器回调：处理去皮请求，或启动采样状态机第一步（零阻塞）*/
static void weight_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    // 去皮
    if (g_tare_requested)
    {
        g_tare_requested = false;
        g_tare_pending = true;
        // 不return，直接fall through让状态机正常启动这一轮采样
    }

    // PHASE 0: 导通分压电路，50ms后进入ADC_SAMPLE
    bat_adc_enable();
    g_meas_phase = MEAS_PHASE_ADC_SAMPLE;
    app_timer_start(m_meas_step_timer, APP_TIMER_TICKS(50), NULL);
}


/**@brief LED闪烁定时器回调（每500ms触发一次）
 *
 * @details 每次触发时切换蓝色LED状态，实现闪烁效果。
 *          仅在系统开机且蓝牙未连接时运行，连接后由ble_evt_handler停止。
 */
 /*
static void led_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_gpio_pin_toggle(LED_B);
}*/
static void led_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    g_led_state = !g_led_state;
    if (g_led_state)
        nrf_gpio_pin_clear(LED_B);  // 点亮
    else
        nrf_gpio_pin_set(LED_B);    // 熄灭
}

//低电量红灯闪烁回调函数
static void bat_low_blink_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_gpio_pin_toggle(LED_R);
}

/**@brief 创建应用定时器
 *
 * @details 创建三个定时器，均为重复模式，仅创建不启动：
 *          - 重量采样定时器：在system_on()中启动，每1秒采集一次HX711数据
 *          - 按键检测定时器：在button_interrupt_handler()中启动，每100ms检测一次按键状态
 *          - LED闪烁定时器：在system_on()中启动，每500ms切换一次绿灯状态
 *                          连接时停止并常亮，断开时重新启动闪烁，关机时停止并熄灭
 */
static void timers_create(void)
{
    ret_code_t err_code;
 
    err_code = app_timer_create(&m_weight_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                weight_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // 单次定时器：驱动采样状态机各步骤之间的等待
    err_code = app_timer_create(&m_meas_step_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                meas_step_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_button_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                button_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_led_timer_id,  
                                APP_TIMER_MODE_REPEATED,
                                led_timer_handler);
    APP_ERROR_CHECK(err_code);

    //低电量闪烁定时器
    err_code = app_timer_create(&m_bat_low_blink_timer,
                            APP_TIMER_MODE_REPEATED,
                            bat_low_blink_handler);
    APP_ERROR_CHECK(err_code);
}


// ============================================================
// BLE 初始化与事件处理
// ============================================================

/**@brief SoftDevice断言回调 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief GAP参数初始化
 *
 * @details 读取拨码开关值生成动态设备名称，格式为 "Smart Scale-XXX"（XXX为000~255）。
 *          设置连接参数（连接间隔、超时等）。
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    // 读取拨码开关值，生成动态设备名称
    uint8_t dip_val = read_dip_switch();
    char device_name[20];
    snprintf(device_name, sizeof(device_name), "Smart Scale-%03d", dip_val);
    NRF_LOG_INFO("Device name: %s", NRF_LOG_PUSH(device_name));

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)device_name,
                                          strlen(device_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Queued Write模块错误处理 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief NUS数据接收回调
 *
 * @details 手机端通过BLE发送数据时触发。当前仅记录日志，可根据需要扩展命令解析。
 */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_DEBUG("Received data from BLE NUS.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        NRF_LOG_INFO("Notification enabled");
    }
}

/**@brief 初始化BLE服务（NUS + Queued Write） */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief 连接参数事件处理 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief 连接参数模块错误处理 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief 初始化连接参数模块 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief 广播超时处理：回到待机状态，等待下次按键 */
static void sleep_mode_enter(void)
{
    NRF_LOG_INFO("Advertising timeout, entering standby.");
    app_timer_stop(m_led_timer_id);        // 停止闪烁
    nrf_gpio_pin_set(LED_B);               // 熄灭
    app_timer_stop(m_weight_timer_id);
    app_timer_stop(m_meas_step_timer);     // 停止状态机步进定时器
    nrf_gpio_pin_set(BAT_CTRL_PIN);        // 确保分压电路关断
    g_meas_phase = MEAS_PHASE_BAT_ENABLE; // 复位状态机
    m_system_on = false;
    // 注意：此处不进入system-off，保持芯片运行等待下次按键
}

/**@brief 广播事件处理
 *
 * @details 广播开始时LED闪烁；广播超时（180秒无连接）时回到待机状态。
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            // 广播中：LED闪烁
            break;

        case BLE_ADV_EVT_IDLE:
            // 广播超时：回到待机状态
            sleep_mode_enter();
            break;

        default:
            break;
    }
}

/**@brief BLE事件处理
 *
 * @details 处理连接、断开、PHY更新、安全参数、超时等BLE事件。
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("BLE Connected step 1");
            app_timer_stop(m_led_timer_id);   // 停止闪烁
            NRF_LOG_INFO("BLE Connected step 2");
            nrf_gpio_pin_clear(LED_B);        // 常亮
            NRF_LOG_INFO("BLE Connected step 3");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            NRF_LOG_INFO("BLE Connected step 4");
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            NRF_LOG_INFO("BLE Connected step 5");
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("BLE Connected step 6");
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("BLE Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            nrf_gpio_pin_set(LED_B);          // 先熄灭
            app_timer_start(m_led_timer_id, APP_TIMER_TICKS(500), NULL);  // 重新闪烁
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // 不支持配对
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

/**@brief 初始化BLE协议栈（SoftDevice） */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief GATT事件处理（MTU更新） */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) &&
        (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("MTU updated: %d bytes", m_ble_nus_max_data_len);
    }
}

/**@brief 初始化GATT模块 */
void gatt_init(void)
{
    ret_code_t err_code;
    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief BSP事件处理（断开、白名单等） */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF: 
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}



/**@brief 初始化广播模块 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.config.ble_adv_fast_enabled    = true;
    init.config.ble_adv_fast_interval   = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout    = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief 初始化LED（不使用BSP按键，仅初始化LED） */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief 初始化日志模块 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief 初始化电源管理模块 */
static void power_management_init(void)
{ 
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief 主循环空闲处理：处理日志，然后进入低功耗等待事件 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief 启动BLE广播 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


// ============================================================
// 主函数
// ============================================================

/**@brief 应用程序入口
 *
 * 启动流程：
 *  1. 将NFC引脚切换为GPIO（用于拨码开关，仅首次上电时执行并复位）
 *  2. 立即拉高PWR_EN_PIN，保持硬件自锁供电
 *  3. 初始化各模块
 *  4. 进入主循环，等待按键中断触发system_on()
 */
int main(void)
{
    // --------------------------------------------------------
    // 步骤1：将 p0.09/NFC1 引脚切换为普通GPIO（仅首次执行）
    // 写入UICR寄存器后需复位生效，下次启动时跳过此步骤
    // --------------------------------------------------------



    if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) ==
        (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
        NRF_UICR->NFCPINS = 0;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        NVIC_SystemReset(); // 复位以使UICR配置生效
    }

    // --------------------------------------------------------
    // 步骤2：立即自锁供电
    // 按键触发硬件上电后，程序必须尽快拉高PWR_EN_PIN，
    // 否则用户松开按键后硬件断电，程序无法继续运行
    // --------------------------------------------------------
    nrf_gpio_cfg_output(PWR_EN_PIN);
    nrf_gpio_pin_set(PWR_EN_PIN);

    // --------------------------------------------------------
    // 步骤3：初始化各模块
    // --------------------------------------------------------
    bool erase_bonds;

    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();      // 读取拨码开关，设置BLE设备名称
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    Init_Hx711();           // 初始化HX711引脚
    bat_adc_init();          // 初始化电池ADC
    // FDS初始化
    fds_register(fds_evt_handler);
    fds_init();
    // 等待FDS初始化完成
    while (!g_fds_initialized) { idle_state_handle(); }

    // 从Flash读取上次的baseline
    long saved_baseline = 0;
    if (fds_baseline_load(&saved_baseline))
    {
        Weight_Maopi = saved_baseline;
        NRF_LOG_INFO("Restored baseline from flash: %ld", Weight_Maopi);
    }
    timers_create();        // 创建重量采样和按键检测定时器
    button_init();          // 注册按键下降沿中断

    NRF_LOG_INFO("System ready. Press and hold button for 1s to power on.");

    // --------------------------------------------------------
    // 步骤4：主循环
    // 系统在此低功耗等待，由按键中断驱动状态切换：
    //   按键按下 → button_interrupt_handler → button_timer_handler
    //   → system_on() / system_off()
    // --------------------------------------------------------
    for (;;)
    {

        idle_state_handle();    
    }
}
