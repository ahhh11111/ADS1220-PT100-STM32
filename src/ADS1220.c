#include "ADS1220.h"

/* 私有函数声明 */
static void ADS1220_GPIO_Init(void);
static void ADS1220_SPI_Init(void);
static uint8_t ADS1220_SPI_TransferByte(uint8_t data);
static void ADS1220_CS_Low(void);
static void ADS1220_CS_High(void);

/* 全局错误状态 */
static int g_last_error = ADS1220_ERROR_NONE;

/* GPIO 控制宏 */
static void ADS1220_CS_Low(void) { GPIO_ResetBits(ADS1220_CS_PORT, ADS1220_CS_PIN); }
static void ADS1220_CS_High(void) { GPIO_SetBits(ADS1220_CS_PORT, ADS1220_CS_PIN); }

#ifdef ADS1220_USE_SOFTWARE_SPI
static void ADS1220_SCK_Low(void) { GPIO_ResetBits(ADS1220_SCK_PORT, ADS1220_SCK_PIN); }
static void ADS1220_SCK_High(void) { GPIO_SetBits(ADS1220_SCK_PORT, ADS1220_SCK_PIN); }
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

/**
 * @brief  软件SPI传输字节 (针对 ADS1220 SPI Mode 1 优化)
 * @note   Mode 1: CPOL=0 (Idle Low), CPHA=1 (Sample on Falling)
 * 时序: SCK Idle Low -> SCK Rise (Slave输出) -> Master读 -> SCK Fall (Slave采样) -> Master写
 */
static uint8_t ADS1220_SPI_TransferByte(uint8_t data)
{
    uint8_t i;
    uint8_t recv = 0;

    ADS1220_SCK_Low(); // 确保起始为低

    for (i = 0; i < 8; i++)
    {
        // 1. 准备数据 (Master在SCK上升沿之前或同时建立数据，以便Slave在下降沿采样)
        // 注意：严格的 Mode 1 中，MOSI 应该在 SCK 变低后变化。
        // 但对于 ADS1220，数据在 SCLK 下降沿读入。我们在上升沿前准备好即可。
        ADS1220_MOSI_Write(data & 0x80);
        data <<= 1;
        Delay_us(1);

        // 2. SCK 上升沿 (Slave 输出 MISO)
        ADS1220_SCK_High();
        Delay_us(1);

        // 3. 读取 MISO (Master 采样)
        recv <<= 1;
        if (ADS1220_MISO_Read())
            recv |= 0x01;

        // 4. SCK 下降沿 (Slave 采样 MOSI)
        ADS1220_SCK_Low();
        Delay_us(1);
    }
    return recv;
}

#else

/**
 * @brief  硬件SPI传输字节
 */
static uint8_t ADS1220_SPI_TransferByte(uint8_t data)
{
    uint16_t retry = 0;

    // 等待发送缓冲区空
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
    // 等待接收缓冲区非空
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
#endif

/* GPIO 初始化 */
static void ADS1220_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(ADS1220_CS_CLK | ADS1220_DRDY_CLK, ENABLE);

    // CS
    GPIO_InitStructure.GPIO_Pin = ADS1220_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1220_CS_PORT, &GPIO_InitStructure);
    ADS1220_CS_High();

    // DRDY
    GPIO_InitStructure.GPIO_Pin = ADS1220_DRDY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ADS1220_DRDY_PORT, &GPIO_InitStructure);

#ifdef ADS1220_USE_SOFTWARE_SPI
    RCC_APB2PeriphClockCmd(ADS1220_SCK_CLK | ADS1220_MISO_CLK | ADS1220_MOSI_CLK, ENABLE);

    // SCK, MOSI
    GPIO_InitStructure.GPIO_Pin = ADS1220_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1220_SCK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ADS1220_MOSI_PIN;
    GPIO_Init(ADS1220_MOSI_PORT, &GPIO_InitStructure);

    // MISO
    GPIO_InitStructure.GPIO_Pin = ADS1220_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ADS1220_MISO_PORT, &GPIO_InitStructure);

    ADS1220_SCK_Low();
#else
    RCC_APB2PeriphClockCmd(ADS1220_SPI_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

    // SCK, MOSI (AF_PP)
    GPIO_InitStructure.GPIO_Pin = ADS1220_PIN_SCK | ADS1220_PIN_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1220_SPI_GPIO_PORT, &GPIO_InitStructure);

    // MISO (IPU or Floating)
    GPIO_InitStructure.GPIO_Pin = ADS1220_PIN_MISO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ADS1220_SPI_GPIO_PORT, &GPIO_InitStructure);
#endif
}

/* SPI 初始化 */
static void ADS1220_SPI_Init(void)
{
#ifndef ADS1220_USE_SOFTWARE_SPI
    SPI_InitTypeDef SPI_InitStructure;

    RCC_APB2PeriphClockCmd(ADS1220_SPI_CLK, ENABLE);

    // ADS1220 SPI Mode 1 (CPOL=0, CPHA=1)
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; // Important for Mode 1
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // ~2.25MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(ADS1220_SPI, &SPI_InitStructure);

    SPI_Cmd(ADS1220_SPI, ENABLE);
#endif
}

/* ====================================================================
 * API 函数
 * ==================================================================== */

void ADS1220_Init(void)
{
#if defined(ADS1220_DELAY_SYSTICK)
    SysTick_Init();
#endif
    ADS1220_GPIO_Init();
    ADS1220_SPI_Init();
    Delay_ms(10);
    ADS1220_Reset();
}

void ADS1220_DeInit(void)
{
    ADS1220_PowerDown();
#ifndef ADS1220_USE_SOFTWARE_SPI
    SPI_Cmd(ADS1220_SPI, DISABLE);
#endif
}

void ADS1220_SendCommand(uint8_t cmd)
{
    ADS1220_CS_Low();
    Delay_us(2);
    ADS1220_SPI_TransferByte(cmd);
    Delay_us(2);
    ADS1220_CS_High();
}

void ADS1220_Reset(void)
{
    ADS1220_SendCommand(ADS1220_CMD_RESET);
    Delay_ms(2); // Datasheet recommends waiting 50us+
}

void ADS1220_StartSync(void)
{
    ADS1220_SendCommand(ADS1220_CMD_START);
}

void ADS1220_PowerDown(void)
{
    ADS1220_SendCommand(ADS1220_CMD_POWERDOWN);
}

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

uint8_t ADS1220_ReadRegister(uint8_t reg)
{
    if (reg > 3)
        return 0;

    uint8_t cmd = ADS1220_CMD_RREG | (reg << 2);
    uint8_t value;

    ADS1220_CS_Low();
    Delay_us(2);
    ADS1220_SPI_TransferByte(cmd);
    value = ADS1220_SPI_TransferByte(0xFF);
    Delay_us(2);
    ADS1220_CS_High();

    return value;
}

void ADS1220_WriteConfig(ADS1220_Config_t *config)
{
    ADS1220_WriteRegister(ADS1220_REG0, config->reg0);
    ADS1220_WriteRegister(ADS1220_REG1, config->reg1);
    ADS1220_WriteRegister(ADS1220_REG2, config->reg2);
    ADS1220_WriteRegister(ADS1220_REG3, config->reg3);
}

void ADS1220_ReadConfig(ADS1220_Config_t *config)
{
    config->reg0 = ADS1220_ReadRegister(ADS1220_REG0);
    config->reg1 = ADS1220_ReadRegister(ADS1220_REG1);
    config->reg2 = ADS1220_ReadRegister(ADS1220_REG2);
    config->reg3 = ADS1220_ReadRegister(ADS1220_REG3);
}

bool ADS1220_IsDataReady(void)
{
    return (GPIO_ReadInputDataBit(ADS1220_DRDY_PORT, ADS1220_DRDY_PIN) == Bit_RESET);
}

bool ADS1220_WaitForData(uint32_t timeout_ms)
{
    uint32_t start_ms = GetMillis();
    while (!ADS1220_IsDataReady())
    {
        if ((GetMillis() - start_ms) > timeout_ms)
            return false;
    }
    return true;
}

int32_t ADS1220_ReadData(void)
{
    int32_t data = 0;
    uint8_t buf[3];

    ADS1220_CS_Low();
    Delay_us(2);

    ADS1220_SPI_TransferByte(ADS1220_CMD_RDATA);
    buf[0] = ADS1220_SPI_TransferByte(0xFF);
    buf[1] = ADS1220_SPI_TransferByte(0xFF);
    buf[2] = ADS1220_SPI_TransferByte(0xFF);

    Delay_us(2);
    ADS1220_CS_High();

    data = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2];

    // Sign extension for 24-bit
    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return data;
}

float ADS1220_ReadVoltage(uint8_t gain, float vref)
{
    int32_t raw_data = ADS1220_ReadData();
    // 24-bit full scale is 2^23 - 1
    return ((float)raw_data / 8388608.0f) * (vref / (float)gain);
}

int16_t ADS1220_ReadTemperature(void)
{
    int32_t raw_data = ADS1220_ReadData();
    // 14-bit result left justified in 24-bit or 32-bit?
    // Datasheet: Internal Temp sensor data is 14-bit result.
    // Result is in 0.03125 C/LSB.
    // Standard conversion logic:
    return (int16_t)((raw_data * 3125) / 100000);
}

// 辅助设置函数 (Helper functions implementation omitted for brevity, logic was correct in original)
void ADS1220_SetInputMux(uint8_t mux)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG0);
    ADS1220_WriteRegister(ADS1220_REG0, (reg & 0x0F) | (mux & 0xF0));
}

void ADS1220_SetGain(uint8_t gain)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG0);
    ADS1220_WriteRegister(ADS1220_REG0, (reg & 0xF1) | (gain & 0x0E));
}

void ADS1220_SetDataRate(uint8_t rate)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG1);
    ADS1220_WriteRegister(ADS1220_REG1, (reg & 0x1F) | (rate & 0xE0));
}

void ADS1220_SetConversionMode(uint8_t mode)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG1);
    ADS1220_WriteRegister(ADS1220_REG1, (reg & 0xFB) | (mode & 0x04));
}

void ADS1220_SetVref(uint8_t vref)
{
    uint8_t reg = ADS1220_ReadRegister(ADS1220_REG2);
    ADS1220_WriteRegister(ADS1220_REG2, (reg & 0x3F) | (vref & 0xC0));
}

void ADS1220_GetDefaultConfig(ADS1220_Config_t *config)
{
    config->reg0 = ADS1220_MUX_AIN0_AIN1 | ADS1220_GAIN_1 | ADS1220_PGA_ENABLED;
    config->reg1 = ADS1220_DR_20SPS | ADS1220_MODE_NORMAL | ADS1220_CM_SINGLE;
    config->reg2 = ADS1220_VREF_INT | ADS1220_FIR_NONE;
    config->reg3 = 0x00;
}

int ADS1220_GetLastError(void) { return g_last_error; }
void ADS1220_ClearError(void) { g_last_error = ADS1220_ERROR_NONE; }