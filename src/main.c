#include "stm32f10x.h"
#include "ADS1220.h"
#include "PT100.h"
#include <stdio.h>

/* USART配置 */
void USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    // TX - PA9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/* printf重定向 */
int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        ;
    return ch;
}

/**
 * @brief  延时功能测试
 */
void DelayTest(void)
{
    printf("\r\n========================================\r\n");
    printf("  延时功能测试\r\n");
    printf("========================================\r\n");

#if defined(ADS1220_DELAY_SYSTICK)
    printf("使用:  SysTick精确延时\r\n");

    printf("\r\n测试1ms延时 x 5次:\r\n");
    for (int i = 0; i < 5; i++)
    {
        uint32_t start = GetMillis();
        Delay_ms(1);
        uint32_t end = GetMillis();
        printf("  实际延时: %lu ms\r\n", end - start);
    }

    printf("\r\n测试100us延时 x 5次:\r\n");
    for (int i = 0; i < 5; i++)
    {
        uint32_t start = GetMicros();
        Delay_us(100);
        uint32_t end = GetMicros();
        printf("  实际延时: %lu us\r\n", end - start);
    }

    printf("\r\n系统运行时间:\r\n");
    printf("  %lu ms = %lu. %03lu 秒\r\n",
           GetMillis(), GetMillis() / 1000, GetMillis() % 1000);

#elif defined(ADS1220_DELAY_SIMPLE)
    printf("使用:  简单循环延时\r\n");
    printf("  (精度依赖编译器优化)\r\n");

#elif defined(ADS1220_DELAY_EXTERNAL)
    printf("使用:  外部延时函数\r\n");
#endif

    printf("========================================\r\n\r\n");
}

/**
 * @brief  PT100测量示例 (2线制)
 * @note   电阻单位mΩ，温度单位0.01°C
 */
void PT100_MeasureExample_2Wire(void)
{
    PT100_Config_t pt100_config;
    int32_t temperature, resistance;
    uint32_t count = 0;

    printf("\r\n========================================\r\n");
    printf("  PT100温度测量 (2线制)\r\n");
    printf("  增益8倍，250μA激励电流\r\n");
    printf("  纯整数运算，无浮点\r\n");
    printf("========================================\r\n");

    // 配置PT100参数 (2线制)
    pt100_config.type = PT100_TYPE;
    pt100_config.idac = PT100_IDAC_250UA;
    pt100_config.gain = 8;
    pt100_config.vref_mv = 2048;  // 2048mV = 2.048V
    pt100_config.input_p = ADS1220_MUX_AIN0_AIN1;
    pt100_config.wire_mode = PT100_2WIRE; // 2线制

    PT100_Init(&pt100_config);

    printf("配置完成，开始测量...\r\n\r\n");

    for (int i = 0; i < 5; i++) // 测量5次后退出
    {
        resistance = PT100_ReadResistance_Int(&pt100_config);

        if (resistance > 0)
        {
            temperature = PT100_ResistanceToTemperature_Int(resistance, PT100_TYPE);

            /* 输出: 电阻mΩ转Ω，温度0.01°C转°C */
            printf("[%lu] [运行:%lus] 电阻: %ld.%03ldΩ, 温度: %ld.%02ld°C\r\n",
                   ++count, GetMillis() / 1000,
                   (long)(resistance / 1000), (long)(resistance % 1000),
                   (long)(temperature / 100), (long)(temperature >= 0 ? temperature % 100 : (-temperature) % 100));
        }
        else
        {
            printf("[%lu] 读取错误! (错误码: %d)\r\n", ++count, ADS1220_GetLastError());
        }

        Delay_ms(1000);
    }
}

/**
 * @brief  PT100测量示例 (3线制)
 * @note   电阻单位mΩ，温度单位0.01°C
 */
void PT100_MeasureExample_3Wire(void)
{
    PT100_Config_t pt100_config;
    int32_t temperature, resistance;
    uint32_t count = 0;

    printf("\r\n========================================\r\n");
    printf("  PT100温度测量 (3线制)\r\n");
    printf("  IDAC1->AIN0, IDAC2->AIN2 (导线补偿)\r\n");
    printf("  纯整数运算，无浮点\r\n");
    printf("========================================\r\n");

    // 配置PT100参数 (3线制)
    pt100_config.type = PT100_TYPE;
    pt100_config.idac = PT100_IDAC_250UA;
    pt100_config.gain = 8;
    pt100_config.vref_mv = 2048;              // 2048mV = 2.048V
    pt100_config.input_p = ADS1220_MUX_AIN0_AIN1; // PT100测量通道
    pt100_config.wire_mode = PT100_3WIRE;         // 3线制
    pt100_config.idac2_pin = ADS1220_I2MUX_AIN2;  // IDAC2输出到AIN2

    PT100_Init(&pt100_config);

    printf("配置完成，开始测量...\r\n");
    printf("提示: 3线制可消除导线电阻影响\r\n\r\n");

    for (int i = 0; i < 5; i++) // 测量5次后退出
    {
        resistance = PT100_ReadResistance_Int(&pt100_config);

        if (resistance > 0)
        {
            temperature = PT100_ResistanceToTemperature_Int(resistance, PT100_TYPE);

            /* 输出: 电阻mΩ转Ω，温度0.01°C转°C */
            printf("[%lu] [运行:%lus] 电阻: %ld.%03ldΩ, 温度: %ld.%02ld°C\r\n",
                   ++count, GetMillis() / 1000,
                   (long)(resistance / 1000), (long)(resistance % 1000),
                   (long)(temperature / 100), (long)(temperature >= 0 ? temperature % 100 : (-temperature) % 100));
        }
        else
        {
            printf("[%lu] 读取错误! (错误码: %d)\r\n", ++count, ADS1220_GetLastError());
        }

        Delay_ms(1000);
    }
}

/**
 * @brief  主函数
 */
int main(void)
{
    // 系统初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // 初始化串口
    USART_Config();

    printf("\r\n\r\n");
    printf("========================================\r\n");
    printf("  ADS1220 + PT100 温度测量系统\r\n");
    printf("  STM32F103C8T6 @ 72MHz\r\n");
    printf("  固件版本: v2.0\r\n");
    printf("  计算模式: 纯整数运算(无FPU)\r\n");
    printf("  温度精度: ±0.1°C\r\n");
    printf("========================================\r\n");

    // 初始化ADS1220 (内部会初始化SysTick)
    ADS1220_Init();
    printf("ADS1220初始化完成\r\n");

    Delay_ms(100);

    // 延时功能测试
    DelayTest();

    Delay_ms(1000);

    // PT100测量示例 - 2线制
    PT100_MeasureExample_2Wire();

    Delay_ms(2000);

    // PT100测量示例 - 3线制
    PT100_MeasureExample_3Wire();

    printf("\r\n所有测试完成!\r\n");

    while (1)
    {
        Delay_ms(1000);
    }
}
