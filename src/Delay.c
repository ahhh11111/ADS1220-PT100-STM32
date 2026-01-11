#include "ADS1220.h"

/* ====================================================================
 * 延时函数实现 - 根据宏定义选择不同实现方式
 * ==================================================================== */

#if defined(ADS1220_DELAY_SYSTICK)
/* ====================================================================
 * SysTick精确延时实现 (推荐)
 * ==================================================================== */

static volatile uint32_t g_systick_ms = 0;  // 毫秒计数器

/**
 * @brief  初始化SysTick定时器
 * @param  无
 * @retval 无
 * @note   配置SysTick为1ms中断一次 (72MHz系统时钟)
 */
void SysTick_Init(void)
{
    // 配置SysTick:  72MHz / 1000 = 72kHz = 1ms
    // SysTick重装载值 = 72000 - 1 = 71999
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        // 配置失败
        while(1);
    }
    
    // 设置SysTick中断优先级 (设置为较低优先级，避免影响其他中断)
    NVIC_SetPriority(SysTick_IRQn, 0x0F);
}

/**
 * @brief  SysTick中断服务函数
 * @param  无
 * @retval 无
 */
void SysTick_Handler(void)
{
    g_systick_ms++;
}

/**
 * @brief  获取系统运行微秒数
 * @param  无
 * @retval 微秒数
 * @note   基于毫秒计数器估算，精度为1ms
 */
uint32_t GetMicros(void)
{
    return g_systick_ms * 1000;
}

/**
 * @brief  获取系统运行毫秒数
 * @param  无
 * @retval 毫秒数
 */
uint32_t GetMillis(void)
{
    return g_systick_ms;
}

/**
 * @brief  微秒级精确延时
 * @param  us:  延时微秒数
 * @retval 无
 * @note   由于SysTick为1ms中断，微秒延时使用忙等待循环实现
 *         对于1ms以上的延时，建议使用Delay_ms()
 */
void Delay_us(uint32_t us)
{
    if (us == 0U) {
        return;
    }

    uint64_t ticks = (uint64_t)us * (uint64_t)(SystemCoreClock / 1000000);
    uint32_t load = SysTick->LOAD + 1U;
    uint32_t start = SysTick->VAL;
    uint64_t elapsed = 0;

    while (elapsed < ticks) {
        uint32_t current = SysTick->VAL;

        if (current <= start) {
            elapsed += (start - current);
        } else {
            elapsed += (start + load - current);
        }

        start = current;
    }
}

/**
 * @brief  毫秒级延时
 * @param  ms: 延时毫秒数
 * @retval 无
 */
void Delay_ms(uint32_t ms)
{
    uint32_t start = g_systick_ms;
    
    while((g_systick_ms - start) < ms)
    {
        // 等待
    }
}

#elif defined(ADS1220_DELAY_SIMPLE)
/* ====================================================================
 * 简单循环延时实现 (默认，精度较低)
 * ==================================================================== */

/**
 * @brief  简单微秒延时
 * @param  us:   延时微秒数
 * @retval 无
 * @note   精度依赖于系统时钟和编译器优化等级
 */
void Delay_us(uint32_t us)
{
    uint32_t ticks;
    
    // 72MHz系统时钟，每次循环约9个时钟周期 (根据编译器优化可能不同)
    ticks = us * (SystemCoreClock / 1000000) / 9;
    
    while(ticks--)
    {
        __NOP();
    }
}

/**
 * @brief  毫秒延时
 * @param  ms:  延时毫秒数
 * @retval 无
 */
void Delay_ms(uint32_t ms)
{
    while(ms--)
    {
        Delay_us(1000);
    }
}

#elif defined(ADS1220_DELAY_EXTERNAL)
/* ====================================================================
 * 使用外部延时函数 (用户需在其他地方实现)
 * ==================================================================== */
// 外部延时函数声明在头文件中

#endif
