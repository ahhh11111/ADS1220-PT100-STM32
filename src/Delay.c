#include "ADS1220.h"

#if defined(ADS1220_DELAY_SYSTICK)

static volatile uint32_t g_systick_ms = 0;

void SysTick_Init(void)
{
    // 配置SysTick 1ms中断
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1)
            ;
    }

    // 修复: 将SysTick优先级设为最高 (0x00)，防止被其他中断打断导致计时不准
    // 注意: 在STM32中，数值越小优先级越高
    NVIC_SetPriority(SysTick_IRQn, 0x0F);
}

// 增加 __weak 属性，防止用户工程中已有 SysTick_Handler 导致重定义报错
__attribute__((weak)) void SysTick_Handler(void)
{
    g_systick_ms++;
}

uint32_t GetMillis(void) { return g_systick_ms; }

void Delay_ms(uint32_t ms)
{
    uint32_t start = g_systick_ms;
    while ((g_systick_ms - start) < ms)
        ;
}

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

#endif