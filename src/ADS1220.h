/**
 * @file    ADS1220.h
 * @brief   ADS1220 24位高精度ADC驱动库 - 头文件
 * @details TI ADS1220是一款24位、低功耗、delta-sigma ADC，内置PGA和基准电压源
 *          本驱动支持硬件/软件SPI、多种延时模式、完整寄存器配置
 * @version 1.1
 * @date    2024-01-11
 * @note    适配STM32F103系列(标准外设库SPL)，可移植到其他平台
 */

#ifndef __ADS1220_H
#define __ADS1220_H

/* ====================================================================
 * 头文件包含
 * ==================================================================== */
#include "stm32f10x.h"  // 如果使用F4，请改为 stm32f4xx.h
#include "Delay.h"      // 延时函数库
#include <stdint.h>
#include <stdbool.h>

/* ====================================================================
 * 错误码定义
 * ==================================================================== */
#define ADS1220_ERROR_NONE      0       /**< 无错误 */
#define ADS1220_ERROR_TIMEOUT   -1      /**< SPI通信超时 */
#define ADS1220_ERROR_INVALID   -2      /**< 无效参数 */

/* ====================================================================
 * 延时配置选择
 * 说明：三选一，推荐使用SysTick模式获得最佳精度
 * 注意：延时模式配置和函数声明已移至 Delay.h
 * ==================================================================== */
#define ADS1220_DELAY_SYSTICK           /**< SysTick定时器延时(推荐，精度±1μs) */
// #define ADS1220_DELAY_SIMPLE         /**< 简单循环延时(精度±10μs) */
// #define ADS1220_DELAY_EXTERNAL       /**< 外部自定义延时函数 */

/* ====================================================================
 * SPI 模式选择
 * 说明：默认使用硬件SPI(推荐)，取消注释下一行可切换到软件SPI
 * ==================================================================== */
// #define ADS1220_USE_SOFTWARE_SPI     /**< 启用软件SPI(IO口模拟) */

/* ====================================================================
 * 硬件引脚定义
 * 说明：请根据实际电路连接修改以下引脚定义
 * ==================================================================== */
/* CS片选引脚配置 */
#define ADS1220_CS_PIN          GPIO_Pin_4      /**< CS片选引脚号 */
#define ADS1220_CS_PORT         GPIOA           /**< CS片选GPIO端口 */
#define ADS1220_CS_CLK          RCC_APB2Periph_GPIOA  /**< CS时钟 */

/* DRDY数据就绪引脚配置 */
#define ADS1220_DRDY_PIN        GPIO_Pin_3      /**< DRDY引脚号 */
#define ADS1220_DRDY_PORT       GPIOA           /**< DRDY GPIO端口 */
#define ADS1220_DRDY_CLK        RCC_APB2Periph_GPIOA  /**< DRDY时钟 */

/* 软件SPI引脚配置 */
#ifdef ADS1220_USE_SOFTWARE_SPI
    #define ADS1220_SCK_PIN     GPIO_Pin_5      /**< SCK时钟引脚 */
    #define ADS1220_SCK_PORT    GPIOA           /**< SCK GPIO端口 */
    #define ADS1220_SCK_CLK     RCC_APB2Periph_GPIOA
    
    #define ADS1220_MISO_PIN    GPIO_Pin_6      /**< MISO数据输入引脚 */
    #define ADS1220_MISO_PORT   GPIOA           /**< MISO GPIO端口 */
    #define ADS1220_MISO_CLK    RCC_APB2Periph_GPIOA
    
    #define ADS1220_MOSI_PIN    GPIO_Pin_7      /**< MOSI数据输出引脚 */
    #define ADS1220_MOSI_PORT   GPIOA           /**< MOSI GPIO端口 */
    #define ADS1220_MOSI_CLK    RCC_APB2Periph_GPIOA

/* 硬件SPI配置 */
#else
    #define ADS1220_SPI             SPI1        /**< SPI外设选择 */
    #define ADS1220_SPI_CLK         RCC_APB2Periph_SPI1  /**< SPI时钟 */
    #define ADS1220_SPI_GPIO_PORT   GPIOA       /**< SPI GPIO端口 */
    #define ADS1220_SPI_GPIO_CLK    RCC_APB2Periph_GPIOA
    #define ADS1220_PIN_SCK         GPIO_Pin_5  /**< SCK引脚(复用功能) */
    #define ADS1220_PIN_MISO        GPIO_Pin_6  /**< MISO引脚(复用功能) */
    #define ADS1220_PIN_MOSI        GPIO_Pin_7  /**< MOSI引脚(复用功能) */
#endif

/* ====================================================================
 * ADS1220 命令定义
 * 说明：ADS1220支持的标准命令集
 * ==================================================================== */
#define ADS1220_CMD_RESET       0x06    /**< 复位命令 */
#define ADS1220_CMD_START       0x08    /**< 启动/同步转换命令 */
#define ADS1220_CMD_POWERDOWN   0x02    /**< 掉电命令 */
#define ADS1220_CMD_RDATA       0x10    /**< 读取数据命令 */
#define ADS1220_CMD_RREG        0x20    /**< 读取寄存器命令(基础值) */
#define ADS1220_CMD_WREG        0x40    /**< 写入寄存器命令(基础值) */

/* ====================================================================
 * ADS1220 寄存器地址定义
 * ==================================================================== */
#define ADS1220_REG0            0x00    /**< 配置寄存器0 */
#define ADS1220_REG1            0x01    /**< 配置寄存器1 */
#define ADS1220_REG2            0x02    /**< 配置寄存器2 */
#define ADS1220_REG3            0x03    /**< 配置寄存器3 */

/* ====================================================================
 * 配置寄存器0 (0x00) - 输入多路复用器和增益配置
 * ==================================================================== */
/* 输入多路复用器配置(MUX[3:0]) - 选择模拟输入通道 */
#define ADS1220_MUX_AIN0_AIN1   0x00    /**< 差分: AINp=AIN0, AINn=AIN1 (默认) */
#define ADS1220_MUX_AIN0_AIN2   0x10    /**< 差分: AINp=AIN0, AINn=AIN2 */
#define ADS1220_MUX_AIN0_AIN3   0x20    /**< 差分: AINp=AIN0, AINn=AIN3 */
#define ADS1220_MUX_AIN1_AIN2   0x30    /**< 差分: AINp=AIN1, AINn=AIN2 */
#define ADS1220_MUX_AIN1_AIN3   0x40    /**< 差分: AINp=AIN1, AINn=AIN3 */
#define ADS1220_MUX_AIN2_AIN3   0x50    /**< 差分: AINp=AIN2, AINn=AIN3 */
#define ADS1220_MUX_AIN1_AIN0   0x60    /**< 差分: AINp=AIN1, AINn=AIN0 (反向) */
#define ADS1220_MUX_AIN3_AIN2   0x70    /**< 差分: AINp=AIN3, AINn=AIN2 (反向) */
#define ADS1220_MUX_AIN0_AVSS   0x80    /**< 单端: AINp=AIN0, AINn=AVSS */
#define ADS1220_MUX_AIN1_AVSS   0x90    /**< 单端: AINp=AIN1, AINn=AVSS */
#define ADS1220_MUX_AIN2_AVSS   0xA0    /**< 单端: AINp=AIN2, AINn=AVSS */
#define ADS1220_MUX_AIN3_AVSS   0xB0    /**< 单端: AINp=AIN3, AINn=AVSS */
#define ADS1220_MUX_REFPX_REFNX 0xC0    /**< (REFPx-REFNx)/4 监测 */
#define ADS1220_MUX_AVDD_AVSS   0xD0    /**< (AVDD-AVSS)/4 监测 */
#define ADS1220_MUX_SHORTED     0xE0    /**< 输入短接，用于噪声测试 */

/* 增益配置(GAIN[2:0]) - PGA增益选择 */
#define ADS1220_GAIN_1          0x00    /**< 增益=1 (默认) */
#define ADS1220_GAIN_2          0x02    /**< 增益=2 */
#define ADS1220_GAIN_4          0x04    /**< 增益=4 */
#define ADS1220_GAIN_8          0x06    /**< 增益=8 */
#define ADS1220_GAIN_16         0x08    /**< 增益=16 */
#define ADS1220_GAIN_32         0x0A    /**< 增益=32 */
#define ADS1220_GAIN_64         0x0C    /**< 增益=64 */
#define ADS1220_GAIN_128        0x0E    /**< 增益=128 */

/* PGA使能控制 */
#define ADS1220_PGA_DISABLED    0x01    /**< 禁用PGA (旁路模式) */
#define ADS1220_PGA_ENABLED     0x00    /**< 使能PGA (默认) */

/* ====================================================================
 * 配置寄存器1 (0x01) - 数据速率和工作模式配置
 * ==================================================================== */
/* 数据速率配置(DR[2:0]) - 采样率选择 */
#define ADS1220_DR_20SPS        0x00    /**< 20 SPS (样本/秒) (默认) */
#define ADS1220_DR_45SPS        0x20    /**< 45 SPS */
#define ADS1220_DR_90SPS        0x40    /**< 90 SPS */
#define ADS1220_DR_175SPS       0x60    /**< 175 SPS */
#define ADS1220_DR_330SPS       0x80    /**< 330 SPS */
#define ADS1220_DR_600SPS       0xA0    /**< 600 SPS */
#define ADS1220_DR_1000SPS      0xC0    /**< 1000 SPS */

/* 工作模式配置(MODE[1:0]) */
#define ADS1220_MODE_NORMAL     0x00    /**< 正常模式 (默认) */
#define ADS1220_MODE_DUTY       0x08    /**< 占空比模式 (省电) */
#define ADS1220_MODE_TURBO      0x10    /**< Turbo模式 (高速) */
#define ADS1220_MODE_DCT        0x18    /**< DCT模式 (滤波器测试) */

/* 转换模式配置(CM) */
#define ADS1220_CM_SINGLE       0x00    /**< 单次转换模式 (默认) */
#define ADS1220_CM_CONTINUOUS   0x04    /**< 连续转换模式 */

/* 温度传感器配置(TS) */
#define ADS1220_TS_DISABLED     0x00    /**< 禁用内部温度传感器 (默认) */
#define ADS1220_TS_ENABLED      0x02    /**< 使能内部温度传感器 */

/* 烧毁电流源配置(BCS) */
#define ADS1220_BCS_OFF         0x00    /**< 关闭烧毁电流源 (默认) */
#define ADS1220_BCS_ON          0x01    /**< 打开烧毁电流源 (10μA) */

/* ====================================================================
 * 配置寄存器2 (0x02) - 基准电压和激励电流配置
 * ==================================================================== */
/* 基准电压源选择(VREF[1:0]) */
#define ADS1220_VREF_INT        0x00    /**< 内部2.048V基准 (默认) */
#define ADS1220_VREF_EXT_REF0   0x40    /**< 外部基准REFP0-REFN0 */
#define ADS1220_VREF_EXT_AIN    0x80    /**< 模拟电源AIN0/REFP1作为基准 */
#define ADS1220_VREF_AVDD       0xC0    /**< 模拟电源AVDD-AVSS作为基准 */

/* FIR滤波器配置(FIR[1:0]) - 同步滤波器选择 */
#define ADS1220_FIR_NONE        0x00    /**< 无滤波器 (默认) */
#define ADS1220_FIR_50HZ_60HZ   0x10    /**< 同时抑制50Hz和60Hz */
#define ADS1220_FIR_50HZ        0x20    /**< 抑制50Hz */
#define ADS1220_FIR_60HZ        0x30    /**< 抑制60Hz */

/* 低边电源开关配置(PSW) */
#define ADS1220_PSW_OPEN        0x00    /**< 开关始终打开 (默认) */
#define ADS1220_PSW_AUTO        0x08    /**< 自动控制开关 */

/* IDAC电流值设置(IDAC[2:0]) - 激励电流源幅值 */
#define ADS1220_IDAC_OFF        0x00    /**< 关闭IDAC (默认) */
#define ADS1220_IDAC_10UA       0x01    /**< 10 μA */
#define ADS1220_IDAC_50UA       0x02    /**< 50 μA */
#define ADS1220_IDAC_100UA      0x03    /**< 100 μA */
#define ADS1220_IDAC_250UA      0x04    /**< 250 μA */
#define ADS1220_IDAC_500UA      0x05    /**< 500 μA */
#define ADS1220_IDAC_1000UA     0x06    /**< 1000 μA (1 mA) */
#define ADS1220_IDAC_1500UA     0x07    /**< 1500 μA (1.5 mA) */

/* ====================================================================
 * 配置寄存器3 (0x03) - IDAC路由配置
 * ==================================================================== */
/* IDAC1路由配置(I1MUX[2:0]) - IDAC1输出引脚选择 */
#define ADS1220_I1MUX_DISABLED  0x00    /**< IDAC1禁用 (默认) */
#define ADS1220_I1MUX_AIN0      0x20    /**< IDAC1连接到AIN0 */
#define ADS1220_I1MUX_AIN1      0x40    /**< IDAC1连接到AIN1 */
#define ADS1220_I1MUX_AIN2      0x60    /**< IDAC1连接到AIN2 */
#define ADS1220_I1MUX_AIN3      0x80    /**< IDAC1连接到AIN3 */
#define ADS1220_I1MUX_REFP0     0xA0    /**< IDAC1连接到REFP0 */
#define ADS1220_I1MUX_REFN0     0xC0    /**< IDAC1连接到REFN0 */

/* IDAC2路由配置(I2MUX[2:0]) - IDAC2输出引脚选择 */
#define ADS1220_I2MUX_DISABLED  0x00    /**< IDAC2禁用 (默认) */
#define ADS1220_I2MUX_AIN0      0x04    /**< IDAC2连接到AIN0 */
#define ADS1220_I2MUX_AIN1      0x08    /**< IDAC2连接到AIN1 */
#define ADS1220_I2MUX_AIN2      0x0C    /**< IDAC2连接到AIN2 */
#define ADS1220_I2MUX_AIN3      0x10    /**< IDAC2连接到AIN3 */
#define ADS1220_I2MUX_REFP0     0x14    /**< IDAC2连接到REFP0 */
#define ADS1220_I2MUX_REFN0     0x18    /**< IDAC2连接到REFN0 */

/* DRDY模式配置(DRDYM) */
#define ADS1220_DRDYM_DRDY_ONLY 0x00    /**< DRDY引脚仅作为数据就绪输出 (默认) */
#define ADS1220_DRDYM_DOUT      0x02    /**< DRDY引脚复用为DOUT */

/* ====================================================================
 * 配置结构体定义
 * ==================================================================== */
/**
 * @brief ADS1220配置结构体
 * @note  包含4个配置寄存器的完整配置
 */
typedef struct {
    uint8_t reg0;   /**< 配置寄存器0: 输入MUX和增益 */
    uint8_t reg1;   /**< 配置寄存器1: 数据速率和工作模式 */
    uint8_t reg2;   /**< 配置寄存器2: 基准电压和IDAC */
    uint8_t reg3;   /**< 配置寄存器3: IDAC路由 */
} ADS1220_Config_t;

/* ====================================================================
 * API函数声明
 * ==================================================================== */

/* 初始化和控制函数 */
/**
 * @brief  初始化ADS1220
 * @note   包括GPIO、SPI和延时系统初始化，并执行复位
 * @param  无
 * @retval 无
 */
void ADS1220_Init(void);

/**
 * @brief  反初始化ADS1220
 * @note   使芯片进入掉电模式并禁用SPI
 * @param  无
 * @retval 无
 */
void ADS1220_DeInit(void);

/**
 * @brief  复位ADS1220
 * @note   发送复位命令，所有寄存器恢复默认值
 * @param  无
 * @retval 无
 */
void ADS1220_Reset(void);

/**
 * @brief  启动同步转换
 * @note   发送START/SYNC命令开始新的转换
 * @param  无
 * @retval 无
 */
void ADS1220_StartSync(void);

/**
 * @brief  进入掉电模式
 * @note   发送掉电命令，降低功耗
 * @param  无
 * @retval 无
 */
void ADS1220_PowerDown(void);

/**
 * @brief  发送命令字节
 * @param  cmd: 命令字节
 * @retval 无
 */
void ADS1220_SendCommand(uint8_t cmd);

/* 寄存器读写函数 */
/**
 * @brief  写入单个寄存器
 * @param  reg: 寄存器地址(0-3)
 * @param  value: 要写入的值
 * @retval 无
 */
void ADS1220_WriteRegister(uint8_t reg, uint8_t value);

/**
 * @brief  读取单个寄存器
 * @param  reg: 寄存器地址(0-3)
 * @retval 寄存器值
 */
uint8_t ADS1220_ReadRegister(uint8_t reg);

/**
 * @brief  写入完整配置
 * @param  config: 配置结构体指针
 * @retval 无
 */
void ADS1220_WriteConfig(ADS1220_Config_t *config);

/**
 * @brief  读取完整配置
 * @param  config: 配置结构体指针(输出)
 * @retval 无
 */
void ADS1220_ReadConfig(ADS1220_Config_t *config);

/* 数据采集函数 */
/**
 * @brief  读取24位ADC原始数据
 * @note   返回带符号扩展的32位整数
 * @param  无
 * @retval 24位ADC数据(符号扩展到32位)
 */
int32_t ADS1220_ReadData(void);

/**
 * @brief  检查数据是否就绪
 * @note   通过读取DRDY引脚状态判断
 * @param  无
 * @retval true=数据就绪, false=数据未就绪
 */
bool ADS1220_IsDataReady(void);

/**
 * @brief  等待数据就绪
 * @param  timeout_ms: 超时时间(毫秒)
 * @retval true=数据就绪, false=超时
 */
bool ADS1220_WaitForData(uint32_t timeout_ms);

/**
 * @brief  读取电压值
 * @param  gain: 增益值(1,2,4,8,16,32,64,128)
 * @param  vref: 基准电压(V)
 * @retval 电压值(V)
 */
float ADS1220_ReadVoltage(uint8_t gain, float vref);

/**
 * @brief  读取内部温度传感器值
 * @note   需要先配置TS=1使能温度传感器
 * @param  无
 * @retval 温度值(°C)
 */
int16_t ADS1220_ReadTemperature(void);

/* 快速配置函数 */
/**
 * @brief  设置输入多路复用器
 * @param  mux: 多路复用器配置值
 * @retval 无
 */
void ADS1220_SetInputMux(uint8_t mux);

/**
 * @brief  设置增益
 * @param  gain: 增益配置值
 * @retval 无
 */
void ADS1220_SetGain(uint8_t gain);

/**
 * @brief  设置数据速率
 * @param  rate: 数据速率配置值
 * @retval 无
 */
void ADS1220_SetDataRate(uint8_t rate);

/**
 * @brief  设置转换模式
 * @param  mode: 转换模式(单次/连续)
 * @retval 无
 */
void ADS1220_SetConversionMode(uint8_t mode);

/**
 * @brief  设置基准电压源
 * @param  vref: 基准电压源配置值
 * @retval 无
 */
void ADS1220_SetVref(uint8_t vref);

/* 辅助函数 */
/**
 * @brief  获取默认配置
 * @note   返回推荐的默认配置参数
 * @param  config: 配置结构体指针(输出)
 * @retval 无
 */
void ADS1220_GetDefaultConfig(ADS1220_Config_t *config);

/**
 * @brief  获取最后一次错误码
 * @param  无
 * @retval 错误码
 */
int ADS1220_GetLastError(void);

/**
 * @brief  清除错误状态
 * @param  无
 * @retval 无
 */
void ADS1220_ClearError(void);

#endif /* __ADS1220_H */