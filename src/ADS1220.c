/**
 * @file    ADS1220.c
 * @brief   ADS1220 24位高精度ADC驱动库 - 实现文件 (已修复SPI时序)
 * @details 实现ADS1220的完整驱动功能，包括:
 * - SPI通信(硬件/软件可选)
 * - 修复软件SPI Mode 1时序问题
 * - 寄存器配置
 * - 数据采集和转换
 * - 错误处理
 * @version 1.2
 * @date    2026-01-12
 */

#include "ADS1220.h"

/* ====================================================================
 * 私有函数声明
 * ==================================================================== */
static void ADS1220_GPIO_Init(void);                   /**< GPIO初始化 */
static void ADS1220_SPI_Init(void);                    /**< SPI初始化 */
static uint8_t ADS1220_SPI_TransferByte(uint8_t data); /**< SPI字节传输 */
static void ADS1220_CS_Low(void);                      /**< 片选拉低 */
static void ADS1220_CS_High(void);                     /**< 片选拉高 */

/* ====================================================================
 * 私有变量
 * ==================================================================== */
static int g_last_error = ADS1220_ERROR_NONE; /**< 全局错误状态 */

/* ====================================================================
 * GPIO控制函数实现
 * ==================================================================== */
/**
 * @brief  片选信号拉低
 */
static void ADS1220_CS_Low(void)
{
    GPIO_ResetBits(ADS1220_CS_PORT, ADS1220_CS_PIN);
}

/**
 * @brief  片选信号拉高
 */
static void ADS1220_CS_High(void)
{
    GPIO_SetBits(ADS1220_CS_PORT, ADS1220_CS_PIN);
}

/* ====================================================================
 * 软件SPI实现
 * ==================================================================== */
#ifdef ADS1220_USE_SOFTWARE_SPI

/**
 * @brief  软件SPI - SCK拉低
 */
static void ADS1220_SCK_Low(void)
{
    GPIO_ResetBits(ADS1220_SCK_PORT, ADS1220_SCK_PIN);
}

/**
 * @brief  软件SPI - SCK拉高
 */
static void ADS1220_SCK_High(void)
{
    GPIO_SetBits(ADS1220_SCK_PORT, ADS1220_SCK_PIN);
}

/**
 * @brief  软件SPI - MOSI写入
 * @param  bit: 要写入的位值(0或1)
 */
static void ADS1220_MOSI_Write(uint8_t bit)
{
    if (bit)
        GPIO_SetBits(ADS1220_MOSI_PORT, ADS1220_MOSI_PIN);
    else
        GPIO_ResetBits(ADS1220_MOSI_PORT, ADS1220_MOSI_PIN);
}

/**
 * @brief  软件SPI - MISO读取
 * @retval MISO引脚电平(0或1)
 */
static uint8_t ADS1220_MISO_Read(void)
{
    return GPIO_ReadInputDataBit(ADS1220_MISO_PORT, ADS1220_MISO_PIN);
}

/**
 * @brief  软件SPI字节传输 (SPI Mode 1 严格时序)
 * @note   SPI Mode 1: CPOL=0 (空闲低电平), CPHA=1 (第二边沿采样)
 * @param  data: 要发送的字节
 * @retval 接收到的字节
 *
 * 时序说明:
 * - 初始状态: SCK Low
 * - 边沿1 (Rising): Host写MOSI (Shift), Slave写MISO (Shift)
 * - 边沿2 (Falling): Host读MISO (Sample), Slave读MOSI (Sample)
 */
static uint8_t ADS1220_SPI_TransferByte(uint8_t data)
{
    uint8_t i;
    uint8_t recv = 0;

    ADS1220_SCK_Low(); // 确保起始状态为低电平

    for (i = 0; i < 8; i++)
    {
        /* 1. 准备MOSI数据 
         * 虽然Mode 1是在Rising Edge Shift，但这里在Rising之前准备好数据
         * 可以保证更充裕的Setup时间，对Slave来说是完全合法的。
         */
        ADS1220_MOSI_Write(data & 0x80);
        data <<= 1;
        Delay_us(1);

        /* 2. SCK上升沿 (Leading Edge)
         * Slave在此边沿输出MISO数据
         */
        ADS1220_SCK_High();
        Delay_us(1);

        /* 3. SCK下降沿 (Trailing Edge)
         * 这是SPI Mode 1的采样时刻
         * Slave在此边沿采样MOSI
         */
        ADS1220_SCK_Low();
        
        /* 4. 主机采样MISO数据
         * 在SCK拉低后立即读取，模拟在下降沿采样
         * 此时数据在下一个上升沿之前都是有效的
         */
        recv <<= 1;
        if (ADS1220_MISO_Read())
            recv |= 0x01;
            
        Delay_us(1);
    }

    return recv;
}

#else /* 使用硬件SPI */

/* ====================================================================
 * 硬件SPI实现
 * ==================================================================== */

/**
 * @brief  硬件SPI字节传输
 * @param  data: 要发送的字节
 * @retval 接收到的字节
 * @note   包含超时处理，防止死锁
 */
static uint8_t ADS1220_SPI_TransferByte(uint8_t data)
{
    uint16_t retry = 0;

    /* 等待发送缓冲区空 */
    while (SPI_I2S_GetFlagStatus(ADS1220_SPI, SPI_I2S_FLAG_TXE) == RESET)
    {
        if (++retry > 5000)
        {
            g_last_error = ADS1220_ERROR_TIMEOUT;
            return 0;
        }
    }

    SPI_I2S_SendData(ADS1220_SPI, data);

    retry = 0;
    /* 等待接收缓冲区非空 */
    while (SPI_I2S_GetFlagStatus(ADS1220_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if (++retry > 5000)
        {
            g_last_error = ADS1220_ERROR_TIMEOUT;
            return 0;
        }
    }

    return SPI_I2S_ReceiveData(ADS1220_SPI);
}

#endif /* ADS1220_USE_SOFTWARE_SPI */

/* ====================================================================
 * 硬件初始化函数
 * ==================================================================== */

/**
 * @brief  GPIO初始化
 * @note   初始化CS、DRDY和SPI相关引脚
 */
static void ADS1220_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能GPIO时钟 */
    RCC_APB2PeriphClockCmd(ADS1220_CS_CLK | ADS1220_DRDY_CLK, ENABLE);

    /* 配置CS引脚(推挽输出) */
    GPIO_InitStructure.GPIO_Pin = ADS1220_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1220_CS_PORT, &GPIO_InitStructure);
    ADS1220_CS_High(); // 默认拉高，禁用芯片

    /* 配置DRDY引脚(上拉输入) */
    GPIO_InitStructure.GPIO_Pin = ADS1220_DRDY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ADS1220_DRDY_PORT, &GPIO_InitStructure);

#ifdef ADS1220_USE_SOFTWARE_SPI
    /* 软件SPI引脚配置 */
    RCC_APB2PeriphClockCmd(ADS1220_SCK_CLK | ADS1220_MISO_CLK | ADS1220_MOSI_CLK, ENABLE);

    /* SCK和MOSI配置为推挽输出 */
    GPIO_InitStructure.GPIO_Pin = ADS1220_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1220_SCK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ADS1220_MOSI_PIN;
    GPIO_Init(ADS1220_MOSI_PORT, &GPIO_InitStructure);

    /* MISO配置为上拉输入 */
    GPIO_InitStructure.GPIO_Pin = ADS1220_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ADS1220_MISO_PORT, &GPIO_InitStructure);

    ADS1220_SCK_Low(); // SCK初始状态为低(SPI Mode 1)
#else
    /* 硬件SPI引脚配置 */
    RCC_APB2PeriphClockCmd(ADS1220_SPI_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

    /* SCK和MOSI配置为复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = ADS1220_PIN_SCK | ADS1220_PIN_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1220_SPI_GPIO_PORT, &GPIO_InitStructure);

    /* MISO配置为上拉输入 */
    GPIO_InitStructure.GPIO_Pin = ADS1220_PIN_MISO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ADS1220_SPI_GPIO_PORT, &GPIO_InitStructure);
#endif
}

/**
 * @brief  SPI初始化
 * @note   仅在使用硬件SPI时有效，配置为SPI Mode 1
 */
static void ADS1220_SPI_Init(void)
{
#ifndef ADS1220_USE_SOFTWARE_SPI
    SPI_InitTypeDef SPI_InitStructure;

    /* 使能SPI时钟 */
    RCC_APB2PeriphClockCmd(ADS1220_SPI_CLK, ENABLE);

    /* 配置SPI参数 - ADS1220要求SPI Mode 1 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  // 全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                       // 主机模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                   // 8位数据
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                          // CPOL=0: 空闲低电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                        // CPHA=1: 第二边沿采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                           // 软件NSS
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // 分频系数32 (~2.25MHz @ 72MHz)
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                  // MSB先行
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(ADS1220_SPI, &SPI_InitStructure);

    /* 使能SPI */
    SPI_Cmd(ADS1220_SPI, ENABLE);
#endif
}

/* ====================================================================
 * 公共API函数实现
 * ==================================================================== */

/**
 * @brief  初始化ADS1220
 */
void ADS1220_Init(void)
{
#if defined(ADS1220_DELAY_SYSTICK)
    SysTick_Init(); // 初始化SysTick定时器
#endif
    ADS1220_GPIO_Init(); // 初始化GPIO
    ADS1220_SPI_Init();  // 初始化SPI
    Delay_ms(10);        // 等待芯片上电稳定
    ADS1220_Reset();     // 复位芯片
}

/**
 * @brief  反初始化ADS1220
 */
void ADS1220_DeInit(void)
{
    ADS1220_PowerDown(); // 进入掉电模式
#ifndef ADS1220_USE_SOFTWARE_SPI
    SPI_Cmd(ADS1220_SPI, DISABLE); // 禁用SPI
#endif
}

/**
 * @brief  发送命令字节
 */
void ADS1220_SendCommand(uint8_t cmd)
{
    ADS1220_CS_Low();
    Delay_us(2);
    ADS1220_SPI_TransferByte(cmd);
    Delay_us(2);
    ADS1220_CS_High();
}

/**
 * @brief  复位ADS1220
 */
void ADS1220_Reset(void)
{
    ADS1220_SendCommand(ADS1220_CMD_RESET);
    Delay_ms(2); // 数据手册建议等待至少50us，这里保守等待2ms
}

/**
 * @brief  启动同步转换
 */
void ADS1220_StartSync(void)
{
    ADS1220_SendCommand(ADS1220_CMD_START);
}

/**
 * @brief  进入掉电模式
 */
void ADS1220_PowerDown(void)
{
    ADS1220_SendCommand(ADS1220_CMD_POWERDOWN);
}

/**
 * @brief  写入单个寄存器
 * @note   寄存器地址必须为0-3
 */
void ADS1220_WriteRegister(uint8_t reg, uint8_t value)
{
    if (reg > 3)
        return;

    uint8_t cmd = ADS1220_CMD_WREG | (reg << 2);

    ADS1220_CS_Low();
    Delay_us(2);
    ADS1220_SPI_TransferByte(cmd);
    ADS1220_SPI_TransferByte(value);
    Delay_us(2);
    ADS1220_CS_High();
}

/**
 * @brief  读取单个寄存器
 * @note   寄存器地址必须为0-3
 */
uint8_t ADS1220_ReadRegister(uint8_t reg)
{
    if (reg > 3)
        return 0;

    uint8_t cmd = ADS1220_CMD_RREG | (reg << 2);
    uint8_t value;

    ADS1220_CS_Low();
    Delay_us(2);
    ADS1220_SPI_TransferByte(cmd);
    value = ADS1220_SPI_TransferByte(0xFF); // 发送dummy byte读取数据
    Delay_us(2);
    ADS1220_CS_High();

    return value;
}

/**
 * @brief  写入完整配置
 */
void ADS1220_WriteConfig(ADS1220_Config_t *config)
{
    ADS1220_WriteRegister(ADS1220_REG0, config->reg0);
    ADS1220_WriteRegister(ADS1220_REG1, config->reg1);
    ADS1220_WriteRegister(ADS1220_REG2, config->reg2);
    ADS1220_WriteRegister(ADS1220_REG3, config->reg3);
}

/**
 * @brief  读取完整配置
 */
void ADS1220_ReadConfig(ADS1220_Config_t *config)
{
    config->reg0 = ADS1220_ReadRegister(ADS1220_REG0);
    config->reg1 = ADS1220_ReadRegister(ADS1220_REG1);
    config->reg2 = ADS1220_ReadRegister(ADS1220_REG2);
    config->reg3 = ADS1220_ReadRegister(ADS1220_REG3);
}

/**
 * @brief  检查数据是否就绪
 * @retval 1=数据就绪(DRDY为低), 0=数据未就绪
 */
uint8_t ADS1220_IsDataReady(void)
{
    return (GPIO_ReadInputDataBit(ADS1220_DRDY_PORT, ADS1220_DRDY_PIN) == Bit_RESET);
}

/**
 * @brief  等待 ADS1220 数据就绪（基于轮询次数）
 * @param  max_try: 最大轮询次数
 * @retval 1=数据就绪, 0=超时
 */
uint8_t ADS1220_WaitForData(uint32_t max_try)
{
    while (max_try--)
    {
        if (ADS1220_IsDataReady())
        {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief  等待 ADS1220 数据就绪（基于超时时间，毫秒）
 * @param  timeout_ms: 超时时间(毫秒)
 * @retval 1=数据就绪, 0=超时
 * @note   使用 GetMillis() 实现精确的毫秒级超时
 */
uint8_t ADS1220_WaitForDataTimeout_ms(uint32_t timeout_ms)
{
#if defined(ADS1220_DELAY_SYSTICK)
    uint32_t start_time = GetMillis();
    
    while ((GetMillis() - start_time) < timeout_ms)
    {
        if (ADS1220_IsDataReady())
        {
            return 1;
        }
    }
    return 0;
#else
    /* 无 SysTick 时，回退到轮询方式 */
    /* 假设每次轮询约 1us，timeout_ms * 1000 次轮询 */
    return ADS1220_WaitForData(timeout_ms * 1000);
#endif
}

/**
 * @brief  启动ADC转换（非阻塞）
 * @note   调用后需要使用 ADS1220_PollConversion 轮询转换状态
 */
void ADS1220_StartConversion(void)
{
    ADS1220_StartSync();
}

/**
 * @brief  轮询ADC转换状态（非阻塞）
 * @param  timeout_ms: 超时时间(毫秒)
 * @param  start_time_ms: 转换开始时的时间戳(毫秒)
 * @retval ADS1220_CONV_WAITING - 仍在等待数据
 * @retval ADS1220_CONV_READY - 数据已就绪
 * @retval ADS1220_CONV_TIMEOUT - 等待超时
 */
ADS1220_ConvState_t ADS1220_PollConversion(uint32_t timeout_ms, uint32_t start_time_ms)
{
#if defined(ADS1220_DELAY_SYSTICK)
    /* 检查是否超时 */
    if ((GetMillis() - start_time_ms) >= timeout_ms)
    {
        g_last_error = ADS1220_ERROR_TIMEOUT;
        return ADS1220_CONV_TIMEOUT;
    }
    
    /* 检查数据是否就绪 */
    if (ADS1220_IsDataReady())
    {
        return ADS1220_CONV_READY;
    }
    
    return ADS1220_CONV_WAITING;
#else
    /* 无 SysTick 时，仅检查数据是否就绪 */
    if (ADS1220_IsDataReady())
    {
        return ADS1220_CONV_READY;
    }
    return ADS1220_CONV_WAITING;
#endif
}

/**
 * @brief  读取ADC数据（带超时的完整流程）
 * @param  timeout_ms: 超时时间(毫秒)
 * @param  data: 输出参数，存储读取的ADC数据
 * @retval ADS1220_CONV_READY - 读取成功
 * @retval ADS1220_CONV_TIMEOUT - 等待超时
 */
ADS1220_ConvState_t ADS1220_ReadDataWithTimeout(uint32_t timeout_ms, int32_t *data)
{
    ADS1220_ClearError();
    ADS1220_StartSync();
    
    if (ADS1220_WaitForDataTimeout_ms(timeout_ms))
    {
        *data = ADS1220_ReadData();
        return ADS1220_CONV_READY;
    }
    
    g_last_error = ADS1220_ERROR_TIMEOUT;
    *data = 0x7FFFFFFF;
    return ADS1220_CONV_TIMEOUT;
}


/**
 * @brief  读取24位ADC原始数据
 * @retval 24位ADC数据(符号扩展到32位)
 * @note   数据格式为补码，自动进行符号扩展
 */
int32_t ADS1220_ReadData(void)
{
    int32_t data = 0;
    uint8_t buf[3];

    ADS1220_CS_Low();
    Delay_us(2);

    /* 发送读数据命令并接收3字节数据 */
    ADS1220_SPI_TransferByte(ADS1220_CMD_RDATA);
    buf[0] = ADS1220_SPI_TransferByte(0xFF); // MSB
    buf[1] = ADS1220_SPI_TransferByte(0xFF); // 中间字节
    buf[2] = ADS1220_SPI_TransferByte(0xFF); // LSB

    Delay_us(2);
    ADS1220_CS_High();

    /* 组合成24位数据 */
    data = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2];

    /* 符号扩展: 如果最高位为1，则为负数 */
    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return data;
}

/**
 * @brief  读取电压值
 * @param  gain: 增益值(1,2,4,8,16,32,64,128)
 * @param  vref: 基准电压(V)
 * @retval 电压值(V)
 * @note   转换公式: V = (ADC_Code / 2^23) * (Vref / Gain)
 */
float ADS1220_ReadVoltage_Float(uint8_t gain, float vref)
{
    int32_t raw_data = ADS1220_ReadData();
    /* 24位ADC满量程为 2^23 = 8388608 */
    return ((float)raw_data / 8388608.0f) * (vref / (float)gain);
}

/**
 * @brief  读取电压值（整数版本）
 * @param  gain: 增益值(1,2,4,8,16,32,64,128)
 * @param  vref_unit: 基准电压(单位由调用者决定)
 * @retval 电压值(与vref_unit相同单位)
 * @note   转换公式: V = (ADC_Code * Vref) / (2^23 * Gain)
 *         使用整数运算，避免浮点计算
 *         
 * 使用示例: 
 * - 微伏:  ADS1220_ReadVoltage_Int(1, 2048000)  // vref=2.048V, 返回µV
 * - 毫伏: ADS1220_ReadVoltage_Int(1, 2048)     // vref=2.048V, 返回mV  
 * - 伏特: ADS1220_ReadVoltage_Int(1, 2)        // vref=2.048V, 返回V(整数部分)
 */
int32_t ADS1220_ReadVoltage_Int(uint8_t gain, int32_t vref_unit)
{
    int32_t raw_data = ADS1220_ReadData();
    /* 24位ADC满量程为 2^23 = 8388608 */
    int64_t voltage = ((int64_t)raw_data * vref_unit) / ((int64_t)8388608 * gain);
    return (int32_t)voltage;
}

/**
 * @brief  读取内部温度传感器值
 * @retval 温度值(°C)
 * @note   需要先配置TS=1使能温度传感器
 * 分辨率: 0.03125°C/LSB
 */
int16_t ADS1220_ReadTemperature(void)
{
    int32_t raw_data = ADS1220_ReadData();
    /* 内部温度传感器输出14位结果
     * 转换系数: 0.03125°C/LSB = 3125/100000
     */
    return (int16_t)((raw_data * 3125) / 100000);
}

/* ====================================================================
 * 快速配置辅助函数
 * ==================================================================== */

/**
 * @brief  设置输入多路复用器
 * @param  mux: 多路复用器配置值(MUX[3:0])
 * @note   只修改REG0的高4位，保留低4位
 */
void ADS1220_SetInputMux(uint8_t mux)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG0);
    ADS1220_WriteRegister(ADS1220_REG0, (reg & 0x0F) | (mux & 0xF0));
}

/**
 * @brief  设置增益
 * @param  gain: 增益配置值(GAIN[2:0])
 * @note   只修改REG0的GAIN位，保留其他位
 */
void ADS1220_SetGain(uint8_t gain)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG0);
    ADS1220_WriteRegister(ADS1220_REG0, (reg & 0xF1) | (gain & 0x0E));
}

/**
 * @brief  设置数据速率
 * @param  rate: 数据速率配置值(DR[2:0])
 * @note   只修改REG1的高3位，保留其他位
 */
void ADS1220_SetDataRate(uint8_t rate)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG1);
    ADS1220_WriteRegister(ADS1220_REG1, (reg & 0x1F) | (rate & 0xE0));
}

/**
 * @brief  设置转换模式
 * @param  mode: 转换模式(CM位)
 * @note   只修改REG1的CM位，保留其他位
 */
void ADS1220_SetConversionMode(uint8_t mode)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG1);
    ADS1220_WriteRegister(ADS1220_REG1, (reg & 0xFB) | (mode & 0x04));
}

/**
 * @brief  设置基准电压源
 * @param  vref: 基准电压源配置值(VREF[1:0])
 * @note   只修改REG2的高2位，保留其他位
 */
void ADS1220_SetVref(uint8_t vref)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG2);
    ADS1220_WriteRegister(ADS1220_REG2, (reg & 0x3F) | (vref & 0xC0));
}

/**
 * @brief  获取默认配置
 * @param  config: 配置结构体指针(输出)
 * @note   默认配置:
 * - 输入: AIN0-AIN1差分
 * - 增益: 1
 * - 采样率: 20 SPS
 * - 模式: 正常模式，单次转换
 * - 基准: 内部2.048V
 * - IDAC: 关闭
 */
void ADS1220_GetDefaultConfig(ADS1220_Config_t *config)
{
    config->reg0 = ADS1220_MUX_AIN0_AIN1 | ADS1220_GAIN_1 | ADS1220_PGA_ENABLED;
    config->reg1 = ADS1220_DR_20SPS | ADS1220_MODE_NORMAL | ADS1220_CM_SINGLE;
    config->reg2 = ADS1220_VREF_INT | ADS1220_FIR_NONE;
    config->reg3 = 0x00;
}

/* ====================================================================
 * 错误处理函数
 * ==================================================================== */

/**
 * @brief  获取最后一次错误码
 */
int ADS1220_GetLastError(void)
{
    return g_last_error;
}

/**
 * @brief  清除错误状态
 */
void ADS1220_ClearError(void)
{
    g_last_error = ADS1220_ERROR_NONE;
}