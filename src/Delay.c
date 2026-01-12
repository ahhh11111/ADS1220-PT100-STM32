/**
 * @file    Delay.c
 * @brief   延时函数库 - 实现文件
 * @details 实现两种延时方式，通过宏定义选择:
 *          - ADS1220_DELAY_SYSTICK: SysTick定时器精确延时
 *          - ADS1220_DELAY_SIMPLE: 简单循环延时
 * @version 1.0
 * @date    2024-01-11
 */

#include "Delay.h"

/* ====================================================================
 * 方法1: SysTick定时器精确延时 (推荐)
 * 特点: 精度高(±1μs)，不受编译器优化影响
 * ==================================================================== */
#if defined(ADS1220_DELAY_SYSTICK)

volatile uint32_t g_systick_ms = 0;

/**
 * @brief  SysTick定时器初始化
 * @note   配置SysTick为1ms中断
 */
void SysTick_Init(void)
{
    // 配置SysTick 1ms中断
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1)
            ;
    }

    // 注意: 在STM32中，数值越小优先级越高
    NVIC_SetPriority(SysTick_IRQn, 0x0F);
}

/**
 * @brief  SysTick中断处理函数
 * @note   增加 __weak 属性，防止用户工程中已有 SysTick_Handler 导致重定义报错
 */
__attribute__((weak)) void SysTick_Handler(void)
{
    g_systick_ms++;
}

/**
 * @brief  获取毫秒时间戳
 * @retval 当前毫秒时间戳
 */
uint32_t GetMillis(void)
{
    return g_systick_ms;
}

/**
 * @brief  获取微秒时间戳
 * @retval 当前微秒时间戳
 * @note   基于SysTick计数器和毫秒计数器计算
 */
uint32_t GetMicros(void)
{
    uint32_t ms, ticks;
    uint32_t load = SysTick->LOAD;

    // 读取当前毫秒数和SysTick计数值
    // 注意: 需要处理SysTick中断可能导致的不一致性
    do
    {
        ms = g_systick_ms;
        ticks = SysTick->VAL;
    } while (ms != g_systick_ms);

    // SysTick是向下计数的，计算已经过的ticks
    uint32_t elapsed_ticks = load - ticks;

    // 将ticks转换为微秒: ticks * 1000000 / SystemCoreClock
    // 为避免溢出，改为: (ticks * 1000) / (SystemCoreClock / 1000)
    uint32_t us_in_tick = (elapsed_ticks * 1000) / (SystemCoreClock / 1000);

    return (ms * 1000) + us_in_tick;
}

/**
 * @brief  毫秒级延时
 * @param  ms: 延时时间(毫秒)
 */
void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Delay_us(1000U);
    }
}

/**
 * @brief  微秒级延时
 * @param  us: 延时时间(微秒)
 * @note   基于SysTick计数器实现，精度较高
 */
void Delay_us(uint32_t us)
{
    if (us == 0U)
        return;
    uint32_t load = SysTick->LOAD;
    uint32_t val = SysTick->VAL;
    uint32_t ticks = (SystemCoreClock / 1000000) * us;
    uint32_t elapsed = 0;

    // 处理简单的us延时，注意SysTick是向下计数的
    while (elapsed < ticks)
    {
        uint32_t current = SysTick->VAL;
        if (current <= val)
            elapsed += (val - current);
        else
            elapsed += (val + load - current); // Wrap around
        val = current;
    }
}

/* ====================================================================
 * 方法2: 简单循环延时
 * 特点: 不占用硬件资源，但精度依赖编译器优化和CPU频率
 * ==================================================================== */
#elif defined(ADS1220_DELAY_SIMPLE)

/**
 * @brief  微秒级延时(简单循环)
 * @param  us: 延时时间(微秒)
 * @note   精度依赖于SystemCoreClock设置是否准确
 *         假设72MHz系统时钟，每个循环约4个时钟周期
 */
void Delay_us(uint32_t us)
{
    uint32_t ticks = (SystemCoreClock / 1000000) * us;
    // 每个循环约4个时钟周期 (编译器优化级别会影响这个值)
    ticks = ticks / 4;

    while (ticks--)
    {
        __NOP(); // 空操作，防止被编译器优化掉
    }
}

/**
 * @brief  毫秒级延时(简单循环)
 * @param  ms: 延时时间(毫秒)
 */
void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Delay_us(1000);
    }
}

#endif