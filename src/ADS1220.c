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
    uint32_t ticks;
    
    // 72MHz系统时钟，每次循环约9个时钟周期
    // 实际循环次数需要根据编译器优化调整
    ticks = us * (SystemCoreClock / 1000000) / 9;
    
    while(ticks--)
    {
        __NOP();
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

/* ====================================================================
 * 私有函数声明
 * ==================================================================== */
static void ADS1220_GPIO_Init(void);
static void ADS1220_SPI_Init(void);
static uint8_t ADS1220_SPI_TransferByte(uint8_t data);
static void ADS1220_CS_Low(void);
static void ADS1220_CS_High(void);

#ifdef ADS1220_USE_SOFTWARE_SPI
static void ADS1220_SCK_Low(void);
static void ADS1220_SCK_High(void);
static void ADS1220_MOSI_Write(uint8_t bit);
static uint8_t ADS1220_MISO_Read(void);
#endif

/* ====================================================================
 * GPIO 引脚控制函数
 * ==================================================================== */
static void ADS1220_CS_Low(void)
{
    GPIO_ResetBits(ADS1220_CS_PORT, ADS1220_CS_PIN);
}

static void ADS1220_CS_High(void)
{
    GPIO_SetBits(ADS1220_CS_PORT, ADS1220_CS_PIN);
}

#ifdef ADS1220_USE_SOFTWARE_SPI
static void ADS1220_SCK_Low(void)
{
    GPIO_ResetBits(ADS1220_SCK_PORT, ADS1220_SCK_PIN);
}

static void ADS1220_SCK_High(void)
{
    GPIO_SetBits(ADS1220_SCK_PORT, ADS1220_SCK_PIN);
}

static void ADS1220_MOSI_Write(uint8_t bit)
{
    if (bit)
        GPIO_SetBits(ADS1220_MOSI_PORT, ADS1220_MOSI_PIN);
    else
        GPIO_ResetBits(ADS1220_MOSI_PORT, ADS1220_MOSI_PIN);
}

static uint8_t ADS1220_MISO_Read(void)
{
    return GPIO_ReadInputDataBit(ADS1220_MISO_PORT, ADS1220_MISO_PIN);
}
#endif

/* ====================================================================
 * SPI 传输函数
 * ==================================================================== */
#ifdef ADS1220_USE_SOFTWARE_SPI
/**
 * @brief  软件SPI传输一个字节
 * @param  data: 要发送的数据
 * @retval 接收到的数据
 */
static uint8_t ADS1220_SPI_TransferByte(uint8_t data)
{
    uint8_t i;
    uint8_t recv = 0;
    
    for (i = 0; i < 8; i++)
    {
        ADS1220_SCK_Low();
        
        // 发送数据 (MSB先发)
        ADS1220_MOSI_Write(data & 0x80);
        data <<= 1;
        
        Delay_us(1); // 调整延时以匹配时序
        
        ADS1220_SCK_High();
        
        // 读取数据 (MSB先读)
        recv <<= 1;
        if (ADS1220_MISO_Read())
            recv |= 0x01;
        
        Delay_us(1);
    }
    
    ADS1220_SCK_Low();
    
    return recv;
}
#else
/**
 * @brief  硬件SPI传输一个字节
 * @param  data: 要发送的数据
 * @retval 接收到的数据
 */
static uint8_t ADS1220_SPI_TransferByte(uint8_t data)
{
    uint16_t retry = 0;
    
    // 等待发送缓冲区空
    while (SPI_I2S_GetFlagStatus(ADS1220_SPI, SPI_I2S_FLAG_TXE) == RESET)
    {
        if (++retry > 5000)
            return 0;
    }
    
    // 发送数据
    SPI_I2S_SendData(ADS1220_SPI, data);
    
    retry = 0;
    // 等待接收缓冲区非空
    while (SPI_I2S_GetFlagStatus(ADS1220_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if (++retry > 5000)
            return 0;
    }
    
    // 返回接收的数据
    return SPI_I2S_ReceiveData(ADS1220_SPI);
}
#endif

/* ====================================================================
 * GPIO 初始化
 * ==================================================================== */
static void ADS1220_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能时钟
    RCC_APB2PeriphClockCmd(ADS1220_CS_CLK | ADS1220_DRDY_CLK, ENABLE);
    
    // 配置CS引脚为推挽输出
    GPIO_InitStructure.GPIO_Pin = ADS1220_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1220_CS_PORT, &GPIO_InitStructure);
    ADS1220_CS_High(); // 默认不选中
    
    // 配置DRDY引脚为上拉输入
    GPIO_InitStructure.GPIO_Pin = ADS1220_DRDY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ADS1220_DRDY_PORT, &GPIO_InitStructure);
    
#ifdef ADS1220_USE_SOFTWARE_SPI
    // 软件SPI GPIO配置
    RCC_APB2PeriphClockCmd(ADS1220_SCK_CLK | ADS1220_MISO_CLK | ADS1220_MOSI_CLK, ENABLE);
    
    // SCK和MOSI为推挽输出
    GPIO_InitStructure.GPIO_Pin = ADS1220_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1220_SCK_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = ADS1220_MOSI_PIN;
    GPIO_Init(ADS1220_MOSI_PORT, &GPIO_InitStructure);
    
    // MISO为上拉输入
    GPIO_InitStructure.GPIO_Pin = ADS1220_MISO_PIN;
    GPIO_InitStructure. GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ADS1220_MISO_PORT, &GPIO_InitStructure);
    
    // 初始状态
    ADS1220_SCK_Low();
#else
    // 硬件SPI GPIO配置
    RCC_APB2PeriphClockCmd(ADS1220_SPI_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
    
    // SCK和MOSI为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = ADS1220_PIN_SCK | ADS1220_PIN_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure. GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1220_SPI_GPIO_PORT, &GPIO_InitStructure);
    
    // MISO为上拉输入
    GPIO_InitStructure.GPIO_Pin = ADS1220_PIN_MISO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ADS1220_SPI_GPIO_PORT, &GPIO_InitStructure);
#endif
}

/* ====================================================================
 * SPI 初始化
 * ==================================================================== */
static void ADS1220_SPI_Init(void)
{
#ifndef ADS1220_USE_SOFTWARE_SPI
    SPI_InitTypeDef SPI_InitStructure;
    
    // 使能SPI时钟
    RCC_APB2PeriphClockCmd(ADS1220_SPI_CLK, ENABLE);
    
    // SPI配置
    // ADS1220支持SPI Mode 1 (CPOL=0, CPHA=1)
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        // 时钟空闲为低
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;      // 第二个时钟边沿采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;         // 软件NSS管理
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // 72MHz/32=2.25MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // MSB先传输
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(ADS1220_SPI, &SPI_InitStructure);
    
    // 使能SPI
    SPI_Cmd(ADS1220_SPI, ENABLE);
#endif
}

/* ====================================================================
 * 公共API函数
 * ==================================================================== */

/**
 * @brief  初始化ADS1220
 * @param  无
 * @retval 无
 */
void ADS1220_Init(void)
{
    // 如果使用SysTick延时，先初始化SysTick
#if defined(ADS1220_DELAY_SYSTICK)
    SysTick_Init();
#endif
    
    ADS1220_GPIO_Init();
    ADS1220_SPI_Init();
    
    Delay_ms(10); // 等待芯片上电稳定
    
    // 复位芯片
    ADS1220_Reset();
    Delay_ms(10);
}

/**
 * @brief  反初始化ADS1220
 * @param  无
 * @retval 无
 */
void ADS1220_DeInit(void)
{
    ADS1220_PowerDown();
#ifndef ADS1220_USE_SOFTWARE_SPI
    SPI_Cmd(ADS1220_SPI, DISABLE);
#endif
}

/**
 * @brief  发送命令到ADS1220
 * @param  cmd: 命令字节
 * @retval 无
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
 * @param  无
 * @retval 无
 */
void ADS1220_Reset(void)
{
    ADS1220_SendCommand(ADS1220_CMD_RESET);
    Delay_ms(1); // 等待复位完成 (典型值50us)
}

/**
 * @brief  启动/同步转换
 * @param  无
 * @retval 无
 */
void ADS1220_StartSync(void)
{
    ADS1220_SendCommand(ADS1220_CMD_START);
}

/**
 * @brief  进入掉电模式
 * @param  无
 * @retval 无
 */
void ADS1220_PowerDown(void)
{
    ADS1220_SendCommand(ADS1220_CMD_POWERDOWN);
}

/**
 * @brief  写ADS1220寄存器
 * @param  reg: 寄存器地址 (0-3)
 * @param  value: 要写入的值
 * @retval 无
 */
void ADS1220_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t cmd;
    
    if (reg > 3)
        return;
    
    cmd = ADS1220_CMD_WREG | (reg << 2);
    
    ADS1220_CS_Low();
    Delay_us(2);
    ADS1220_SPI_TransferByte(cmd);
    ADS1220_SPI_TransferByte(value);
    Delay_us(2);
    ADS1220_CS_High();
}

/**
 * @brief  读ADS1220寄存器
 * @param  reg:   寄存器地址 (0-3)
 * @retval 寄存器值
 */
uint8_t ADS1220_ReadRegister(uint8_t reg)
{
    uint8_t cmd, value;
    
    if (reg > 3)
        return 0;
    
    cmd = ADS1220_CMD_RREG | (reg << 2);
    
    ADS1220_CS_Low();
    Delay_us(2);
    ADS1220_SPI_TransferByte(cmd);
    value = ADS1220_SPI_TransferByte(0xFF);
    Delay_us(2);
    ADS1220_CS_High();
    
    return value;
}

/**
 * @brief  写配置到ADS1220
 * @param  config: 配置结构体指针
 * @retval 无
 */
void ADS1220_WriteConfig(ADS1220_Config_t *config)
{
    ADS1220_WriteRegister(ADS1220_REG0, config->reg0);
    ADS1220_WriteRegister(ADS1220_REG1, config->reg1);
    ADS1220_WriteRegister(ADS1220_REG2, config->reg2);
    ADS1220_WriteRegister(ADS1220_REG3, config->reg3);
}

/**
 * @brief  从ADS1220读取配置
 * @param  config: 配置结构体指针
 * @retval 无
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
 * @param  无
 * @retval true: 数据就绪, false: 数据未就绪
 */
bool ADS1220_IsDataReady(void)
{
    return (GPIO_ReadInputDataBit(ADS1220_DRDY_PORT, ADS1220_DRDY_PIN) == Bit_RESET);
}

/**
 * @brief  等待数据就绪
 * @param  timeout_ms: 超时时间(毫秒)
 * @retval true: 数据就绪, false: 超时
 */
bool ADS1220_WaitForData(uint32_t timeout_ms)
{
    uint32_t tick = 0;
    
    while (! ADS1220_IsDataReady())
    {
        Delay_ms(1);
        if (++tick >= timeout_ms)
            return false;
    }
    
    return true;
}

/**
 * @brief  读取24位ADC数据
 * @param  无
 * @retval 带符号的32位整数 (符号扩展)
 */
int32_t ADS1220_ReadData(void)
{
    int32_t data = 0;
    uint8_t buf[3];
    
    ADS1220_CS_Low();
    Delay_us(2);
    
    // 发送读数据命令
    ADS1220_SPI_TransferByte(ADS1220_CMD_RDATA);
    
    // 读取3字节数据 (MSB first)
    buf[0] = ADS1220_SPI_TransferByte(0xFF);
    buf[1] = ADS1220_SPI_TransferByte(0xFF);
    buf[2] = ADS1220_SPI_TransferByte(0xFF);
    
    Delay_us(2);
    ADS1220_CS_High();
    
    // 组合数据
    data = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2];
    
    // 符号扩展 (24位补码转32位)
    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }
    
    return data;
}

/**
 * @brief  读取电压值
 * @param  gain: 增益设置 (1, 2, 4, 8, 16, 32, 64, 128)
 * @param  vref: 参考电压 (V)
 * @retval 电压值 (V)
 */
float ADS1220_ReadVoltage(uint8_t gain, float vref)
{
    int32_t raw_data;
    float voltage;
    
    raw_data = ADS1220_ReadData();
    
    // 转换公式:  V = (raw_data / 2^23) * (Vref / Gain)
    voltage = ((float)raw_data / 8388608.0f) * (vref / (float)gain);
    
    return voltage;
}

/**
 * @brief  读取内部温度传感器
 * @param  无
 * @retval 温度值 (0.1°C为单位)
 * @note   需要先配置为温度传感器模式
 */
int16_t ADS1220_ReadTemperature(void)
{
    int32_t raw_data;
    int16_t temperature;
    
    raw_data = ADS1220_ReadData();
    
    // 温度转换公式: T(°C) = raw_data * 0.03125
    // 为了使用整数运算，返回值单位为0.1°C
    temperature = (int16_t)((raw_data * 3125) / 100000);
    
    return temperature;
}

/**
 * @brief  设置输入多路复用器
 * @param  mux: MUX配置 (使用ADS1220_MUX_xxx宏)
 * @retval 无
 */
void ADS1220_SetInputMux(uint8_t mux)
{
    uint8_t reg0 = ADS1220_ReadRegister(ADS1220_REG0);
    reg0 = (reg0 & 0x0F) | (mux & 0xF0);
    ADS1220_WriteRegister(ADS1220_REG0, reg0);
}

/**
 * @brief  设置增益
 * @param  gain: 增益配置 (使用ADS1220_GAIN_xxx宏)
 * @retval 无
 */
void ADS1220_SetGain(uint8_t gain)
{
    uint8_t reg0 = ADS1220_ReadRegister(ADS1220_REG0);
    reg0 = (reg0 & 0xF1) | (gain & 0x0E);
    ADS1220_WriteRegister(ADS1220_REG0, reg0);
}

/**
 * @brief  设置数据速率
 * @param  rate: 数据速率 (使用ADS1220_DR_xxx宏)
 * @retval 无
 */
void ADS1220_SetDataRate(uint8_t rate)
{
    uint8_t reg1 = ADS1220_ReadRegister(ADS1220_REG1);
    reg1 = (reg1 & 0x1F) | (rate & 0xE0);
    ADS1220_WriteRegister(ADS1220_REG1, reg1);
}

/**
 * @brief  设置转换模式
 * @param  mode: 转换模式 (ADS1220_CM_SINGLE 或 ADS1220_CM_CONTINUOUS)
 * @retval 无
 */
void ADS1220_SetConversionMode(uint8_t mode)
{
    uint8_t reg1 = ADS1220_ReadRegister(ADS1220_REG1);
    reg1 = (reg1 & 0xFB) | (mode & 0x04);
    ADS1220_WriteRegister(ADS1220_REG1, reg1);
}

/**
 * @brief  设置电压基准
 * @param  vref: 基准选择 (使用ADS1220_VREF_xxx宏)
 * @retval 无
 */
void ADS1220_SetVref(uint8_t vref)
{
    uint8_t reg2 = ADS1220_ReadRegister(ADS1220_REG2);
    reg2 = (reg2 & 0x3F) | (vref & 0xC0);
    ADS1220_WriteRegister(ADS1220_REG2, reg2);
}

/**
 * @brief  获取默认配置
 * @param  config: 配置结构体指针
 * @retval 无
 */
void ADS1220_GetDefaultConfig(ADS1220_Config_t *config)
{
    // 默认配置:   
    // - 输入:   AIN0-AIN1差分
    // - 增益:  1
    // - PGA使能
    // - 数据速率: 20 SPS
    // - 正常模式
    // - 单次转换
    // - 内部2. 048V基准
    // - 无FIR滤波
    
    config->reg0 = ADS1220_MUX_AIN0_AIN1 | ADS1220_GAIN_1 | ADS1220_PGA_ENABLED;
    config->reg1 = ADS1220_DR_20SPS | ADS1220_MODE_NORMAL | ADS1220_CM_SINGLE;
    config->reg2 = ADS1220_VREF_INT | ADS1220_FIR_NONE;
    config->reg3 = 0x00;
}
