#ifndef __ADS1220_H
#define __ADS1220_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

/* ====================================================================
 * 错误码定义
 * ==================================================================== */
#define ADS1220_ERROR_NONE      0       // 无错误
#define ADS1220_ERROR_TIMEOUT   -1      // SPI超时
#define ADS1220_ERROR_INVALID   -2      // 无效参数

/* ====================================================================
 * 全局错误状态
 * ==================================================================== */
static int g_last_error = ADS1220_ERROR_NONE;

/* ====================================================================
 * 延时函数配置：选择延时实现方式（只能选择一个）
 * ==================================================================== */
// 选项1: 使用外部定义的延时函数
// #define ADS1220_DELAY_EXTERNAL

// 选项2: 使用SysTick精确延时（推荐，需要72MHz系统时钟）
#define ADS1220_DELAY_SYSTICK

// 选项3: 使用简单循环延时（默认，精度较低）
// #define ADS1220_DELAY_SIMPLE

// 延时方式优先级检查
#if defined(ADS1220_DELAY_EXTERNAL)
    #warning "使用外部延时函数"
    extern void Delay_us(uint32_t us);
    extern void Delay_ms(uint32_t ms);
#elif defined(ADS1220_DELAY_SYSTICK)
    #warning "使用SysTick精确延时"
    void Delay_us(uint32_t us);
    void Delay_ms(uint32_t ms);
    void SysTick_Init(void);
    uint32_t GetMicros(void);
    uint32_t GetMillis(void);
#else
    #define ADS1220_DELAY_SIMPLE
    #warning "使用简单循环延时"
    void Delay_us(uint32_t us);
    void Delay_ms(uint32_t ms);
#endif

/* ====================================================================
 * SPI 模式选择：取消注释以使用软件SPI
 * ==================================================================== */
// #define ADS1220_USE_SOFTWARE_SPI

/* ====================================================================
 * 硬件引脚定义 (根据实际硬件修改)
 * ==================================================================== */
// CS 片选引脚
#define ADS1220_CS_PIN          GPIO_Pin_4
#define ADS1220_CS_PORT         GPIOA
#define ADS1220_CS_CLK          RCC_APB2Periph_GPIOA

// DRDY 数据就绪引脚
#define ADS1220_DRDY_PIN        GPIO_Pin_3
#define ADS1220_DRDY_PORT       GPIOA
#define ADS1220_DRDY_CLK        RCC_APB2Periph_GPIOA

#ifdef ADS1220_USE_SOFTWARE_SPI
    // 软件SPI引脚定义
    #define ADS1220_SCK_PIN     GPIO_Pin_5
    #define ADS1220_SCK_PORT    GPIOA
    #define ADS1220_SCK_CLK     RCC_APB2Periph_GPIOA
    
    #define ADS1220_MISO_PIN    GPIO_Pin_6
    #define ADS1220_MISO_PORT   GPIOA
    #define ADS1220_MISO_CLK    RCC_APB2Periph_GPIOA
    
    #define ADS1220_MOSI_PIN    GPIO_Pin_7
    #define ADS1220_MOSI_PORT   GPIOA
    #define ADS1220_MOSI_CLK    RCC_APB2Periph_GPIOA
#else
    // 硬件SPI定义 (SPI1)
    #define ADS1220_SPI             SPI1
    #define ADS1220_SPI_CLK         RCC_APB2Periph_SPI1
    #define ADS1220_SPI_GPIO_PORT   GPIOA
    #define ADS1220_SPI_GPIO_CLK    RCC_APB2Periph_GPIOA
    #define ADS1220_PIN_SCK         GPIO_Pin_5
    #define ADS1220_PIN_MISO        GPIO_Pin_6
    #define ADS1220_PIN_MOSI        GPIO_Pin_7
#endif

/* ====================================================================
 * ADS1220 命令定义
 * ==================================================================== */
#define ADS1220_CMD_RESET       0x06    // 复位
#define ADS1220_CMD_START       0x08    // 开始/同步转换
#define ADS1220_CMD_POWERDOWN   0x02    // 掉电
#define ADS1220_CMD_RDATA       0x10    // 读数据
#define ADS1220_CMD_RREG        0x20    // 读寄存器 (0x2r, r=寄存器地址)
#define ADS1220_CMD_WREG        0x40    // 写寄存器 (0x4r, r=寄存器地址)

/* ====================================================================
 * 寄存器地址定义
 * ==================================================================== */
#define ADS1220_REG0            0x00
#define ADS1220_REG1            0x01
#define ADS1220_REG2            0x02
#define ADS1220_REG3            0x03

/* ====================================================================
 * Register 0 配置位定义
 * ==================================================================== */
// MUX[3:0]:   输入多路复用器配置
#define ADS1220_MUX_AIN0_AIN1   0x00    // AINP=AIN0, AINN=AIN1 (默认)
#define ADS1220_MUX_AIN0_AIN2   0x10
#define ADS1220_MUX_AIN0_AIN3   0x20
#define ADS1220_MUX_AIN1_AIN2   0x30
#define ADS1220_MUX_AIN1_AIN3   0x40
#define ADS1220_MUX_AIN2_AIN3   0x50
#define ADS1220_MUX_AIN1_AIN0   0x60
#define ADS1220_MUX_AIN3_AIN2   0x70
#define ADS1220_MUX_AIN0_AVSS   0x80    // AINP=AIN0, AINN=AVSS
#define ADS1220_MUX_AIN1_AVSS   0x90
#define ADS1220_MUX_AIN2_AVSS   0xA0
#define ADS1220_MUX_AIN3_AVSS   0xB0
#define ADS1220_MUX_REFPX_REFNX 0xC0    // (REFP–REFN)/4监测
#define ADS1220_MUX_AVDD_AVSS   0xD0    // (AVDD–AVSS)/4监测
#define ADS1220_MUX_SHORTED     0xE0    // AINP和AINN短接至AVDD/2

// GAIN[3:0]:  增益配置
#define ADS1220_GAIN_1          0x00    // 增益=1 (默认)
#define ADS1220_GAIN_2          0x02
#define ADS1220_GAIN_4          0x04
#define ADS1220_GAIN_8          0x06
#define ADS1220_GAIN_16         0x08
#define ADS1220_GAIN_32         0x0A
#define ADS1220_GAIN_64         0x0C
#define ADS1220_GAIN_128        0x0E

// PGA_BYPASS:   PGA旁路
#define ADS1220_PGA_DISABLED    0x01    // PGA禁用并旁路
#define ADS1220_PGA_ENABLED     0x00    // PGA使能 (默认)

/* ====================================================================
 * Register 1 配置位定义
 * ==================================================================== */
// DR[2:0]: 数据速率
#define ADS1220_DR_20SPS        0x00    // 20 SPS
#define ADS1220_DR_45SPS        0x20    // 45 SPS
#define ADS1220_DR_90SPS        0x40    // 90 SPS
#define ADS1220_DR_175SPS       0x60    // 175 SPS
#define ADS1220_DR_330SPS       0x80    // 330 SPS
#define ADS1220_DR_600SPS       0xA0    // 600 SPS
#define ADS1220_DR_1000SPS      0xC0    // 1000 SPS (默认)

// MODE[1:0]: 工作模式
#define ADS1220_MODE_NORMAL     0x00    // 正常模式 (256kHz调制器时钟, 默认)
#define ADS1220_MODE_DUTY       0x08    // 占空比模式
#define ADS1220_MODE_TURBO      0x10    // Turbo模式
#define ADS1220_MODE_DCT        0x18    // DCT模式

// CM:   转换模式
#define ADS1220_CM_SINGLE       0x00    // 单次转换模式 (默认)
#define ADS1220_CM_CONTINUOUS   0x04    // 连续转换模式

// TS:  温度传感器模式
#define ADS1220_TS_DISABLED     0x00    // 禁用温度传感器 (默认)
#define ADS1220_TS_ENABLED      0x02    // 使能温度传感器

// BCS: 烧毁电流源
#define ADS1220_BCS_OFF         0x00    // 电流源关闭 (默认)
#define ADS1220_BCS_ON          0x01    // 电流源开启

/* ====================================================================
 * Register 2 配置位定义
 * ==================================================================== */
// VREF[1:0]: 电压基准选择
#define ADS1220_VREF_INT        0x00    // 内部2.048V基准 (默认)
#define ADS1220_VREF_EXT_REF0   0x40    // 外部基准REFP0和REFN0
#define ADS1220_VREF_EXT_AIN    0x80    // 外部基准AIN0/REFP1和AIN3/REFN1
#define ADS1220_VREF_AVDD       0xC0    // 模拟电源AVDD和AVSS作为基准

// FIR[1:0]: FIR滤波器配置
#define ADS1220_FIR_NONE        0x00    // 无滤波器 (默认)
#define ADS1220_FIR_50HZ_60HZ   0x10    // 同时抑制50Hz和60Hz
#define ADS1220_FIR_50HZ        0x20    // 仅抑制50Hz
#define ADS1220_FIR_60HZ        0x30    // 仅抑制60Hz

// PSW:   低侧功率开关配置
#define ADS1220_PSW_OPEN        0x00    // 开关始终打开 (默认)
#define ADS1220_PSW_AUTO        0x08    // 自动控制

// IDAC[2:0]:   IDAC电流设置
#define ADS1220_IDAC_OFF        0x00    // 关闭 (默认)
#define ADS1220_IDAC_10UA       0x01    // 10µA
#define ADS1220_IDAC_50UA       0x02    // 50µA
#define ADS1220_IDAC_100UA      0x03    // 100µA
#define ADS1220_IDAC_250UA      0x04    // 250µA
#define ADS1220_IDAC_500UA      0x05    // 500µA
#define ADS1220_IDAC_1000UA     0x06    // 1000µA
#define ADS1220_IDAC_1500UA     0x07    // 1500µA

/* ====================================================================
 * Register 3 配置位定义
 * ==================================================================== */
// I1MUX[2:0]:  IDAC1路由配置
#define ADS1220_I1MUX_DISABLED  0x00    // 禁用 (默认)
#define ADS1220_I1MUX_AIN0      0x20    // AIN0/REFP1
#define ADS1220_I1MUX_AIN1      0x40    // AIN1
#define ADS1220_I1MUX_AIN2      0x60    // AIN2
#define ADS1220_I1MUX_AIN3      0x80    // AIN3/REFN1
#define ADS1220_I1MUX_REFP0     0xA0    // REFP0
#define ADS1220_I1MUX_REFN0     0xC0    // REFN0

// I2MUX[2:0]:  IDAC2路由配置
#define ADS1220_I2MUX_DISABLED  0x00    // 禁用 (默认)
#define ADS1220_I2MUX_AIN0      0x04    // AIN0/REFP1
#define ADS1220_I2MUX_AIN1      0x08    // AIN1
#define ADS1220_I2MUX_AIN2      0x0C    // AIN2
#define ADS1220_I2MUX_AIN3      0x10    // AIN3/REFN1
#define ADS1220_I2MUX_REFP0     0x14    // REFP0
#define ADS1220_I2MUX_REFN0     0x18    // REFN0

// DRDYM[1:0]:   DRDY模式
#define ADS1220_DRDYM_DRDY_ONLY 0x00    // 仅DRDY (默认)
#define ADS1220_DRDYM_DOUT      0x02    // DRDY和DOUT同时

/* ====================================================================
 * 配置结构体
 * ==================================================================== */
typedef struct {
    uint8_t reg0;   // 寄存器0配置
    uint8_t reg1;   // 寄存器1配置
    uint8_t reg2;   // 寄存器2配置
    uint8_t reg3;   // 寄存器3配置
} ADS1220_Config_t;

/* ====================================================================
 * 函数声明
 * ==================================================================== */
// 基础初始化
void ADS1220_Init(void);
void ADS1220_DeInit(void);

// 命令函数
void ADS1220_Reset(void);
void ADS1220_StartSync(void);
void ADS1220_PowerDown(void);
void ADS1220_SendCommand(uint8_t cmd);

// 寄存器读写
void ADS1220_WriteRegister(uint8_t reg, uint8_t value);
uint8_t ADS1220_ReadRegister(uint8_t reg);
void ADS1220_WriteConfig(ADS1220_Config_t *config);
void ADS1220_ReadConfig(ADS1220_Config_t *config);

// 数据读取
int32_t ADS1220_ReadData(void);
bool ADS1220_IsDataReady(void);
bool ADS1220_WaitForData(uint32_t timeout_ms);

// 高级功能
float ADS1220_ReadVoltage(uint8_t gain, float vref);
int16_t ADS1220_ReadTemperature(void);

// 快速配置函数
void ADS1220_SetInputMux(uint8_t mux);
void ADS1220_SetGain(uint8_t gain);
void ADS1220_SetDataRate(uint8_t rate);
void ADS1220_SetConversionMode(uint8_t mode);
void ADS1220_SetVref(uint8_t vref);

// 默认配置
void ADS1220_GetDefaultConfig(ADS1220_Config_t *config);

// 错误处理
int ADS1220_GetLastError(void);
void ADS1220_ClearError(void);

#endif /* __ADS1220_H */
