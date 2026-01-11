#ifndef __ADS1220_H
#define __ADS1220_H

#include "stm32f10x.h" // 如果使用F4，请改为 stm32f4xx.h
#include <stdint.h>
#include <stdbool.h>

/* ====================================================================
 * 错误码定义
 * ==================================================================== */
#define ADS1220_ERROR_NONE      0       // 无错误
#define ADS1220_ERROR_TIMEOUT   -1      // SPI超时
#define ADS1220_ERROR_INVALID   -2      // 无效参数

/* ====================================================================
 * 延时配置
 * ==================================================================== */
#define ADS1220_DELAY_SYSTICK    // 推荐使用SysTick

// 延时函数声明
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void SysTick_Init(void);
uint32_t GetMicros(void);
uint32_t GetMillis(void);

/* ====================================================================
 * SPI 模式选择
 * ==================================================================== */
// 取消注释以使用软件SPI，否则使用硬件SPI
// #define ADS1220_USE_SOFTWARE_SPI

/* ====================================================================
 * 硬件引脚定义 (请根据实际电路修改)
 * ==================================================================== */
#define ADS1220_CS_PIN          GPIO_Pin_4
#define ADS1220_CS_PORT         GPIOA
#define ADS1220_CS_CLK          RCC_APB2Periph_GPIOA

#define ADS1220_DRDY_PIN        GPIO_Pin_3
#define ADS1220_DRDY_PORT       GPIOA
#define ADS1220_DRDY_CLK        RCC_APB2Periph_GPIOA

#ifdef ADS1220_USE_SOFTWARE_SPI
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
    #define ADS1220_SPI             SPI1
    #define ADS1220_SPI_CLK         RCC_APB2Periph_SPI1
    #define ADS1220_SPI_GPIO_PORT   GPIOA
    #define ADS1220_SPI_GPIO_CLK    RCC_APB2Periph_GPIOA
    #define ADS1220_PIN_SCK         GPIO_Pin_5
    #define ADS1220_PIN_MISO        GPIO_Pin_6
    #define ADS1220_PIN_MOSI        GPIO_Pin_7
#endif

/* ====================================================================
 * ADS1220 命令与寄存器定义 (保持原样)
 * ==================================================================== */
#define ADS1220_CMD_RESET       0x06
#define ADS1220_CMD_START       0x08
#define ADS1220_CMD_POWERDOWN   0x02
#define ADS1220_CMD_RDATA       0x10
#define ADS1220_CMD_RREG        0x20
#define ADS1220_CMD_WREG        0x40

#define ADS1220_REG0            0x00
#define ADS1220_REG1            0x01
#define ADS1220_REG2            0x02
#define ADS1220_REG3            0x03

// Register 0
#define ADS1220_MUX_AIN0_AIN1   0x00
#define ADS1220_MUX_AIN0_AIN2   0x10
#define ADS1220_MUX_AIN0_AIN3   0x20
#define ADS1220_MUX_AIN1_AIN2   0x30
#define ADS1220_MUX_AIN1_AIN3   0x40
#define ADS1220_MUX_AIN2_AIN3   0x50
#define ADS1220_MUX_AIN1_AIN0   0x60
#define ADS1220_MUX_AIN3_AIN2   0x70
#define ADS1220_MUX_AIN0_AVSS   0x80
#define ADS1220_MUX_AIN1_AVSS   0x90
#define ADS1220_MUX_AIN2_AVSS   0xA0
#define ADS1220_MUX_AIN3_AVSS   0xB0
#define ADS1220_MUX_REFPX_REFNX 0xC0
#define ADS1220_MUX_AVDD_AVSS   0xD0
#define ADS1220_MUX_SHORTED     0xE0

#define ADS1220_GAIN_1          0x00
#define ADS1220_GAIN_2          0x02
#define ADS1220_GAIN_4          0x04
#define ADS1220_GAIN_8          0x06
#define ADS1220_GAIN_16         0x08
#define ADS1220_GAIN_32         0x0A
#define ADS1220_GAIN_64         0x0C
#define ADS1220_GAIN_128        0x0E

#define ADS1220_PGA_DISABLED    0x01
#define ADS1220_PGA_ENABLED     0x00

// Register 1
#define ADS1220_DR_20SPS        0x00
#define ADS1220_DR_45SPS        0x20
#define ADS1220_DR_90SPS        0x40
#define ADS1220_DR_175SPS       0x60
#define ADS1220_DR_330SPS       0x80
#define ADS1220_DR_600SPS       0xA0
#define ADS1220_DR_1000SPS      0xC0

#define ADS1220_MODE_NORMAL     0x00
#define ADS1220_MODE_DUTY       0x08
#define ADS1220_MODE_TURBO      0x10
#define ADS1220_MODE_DCT        0x18

#define ADS1220_CM_SINGLE       0x00
#define ADS1220_CM_CONTINUOUS   0x04

#define ADS1220_TS_DISABLED     0x00
#define ADS1220_TS_ENABLED      0x02
#define ADS1220_BCS_OFF         0x00
#define ADS1220_BCS_ON          0x01

// Register 2
#define ADS1220_VREF_INT        0x00
#define ADS1220_VREF_EXT_REF0   0x40
#define ADS1220_VREF_EXT_AIN    0x80
#define ADS1220_VREF_AVDD       0xC0

#define ADS1220_FIR_NONE        0x00
#define ADS1220_FIR_50HZ_60HZ   0x10
#define ADS1220_FIR_50HZ        0x20
#define ADS1220_FIR_60HZ        0x30

#define ADS1220_PSW_OPEN        0x00
#define ADS1220_PSW_AUTO        0x08

#define ADS1220_IDAC_OFF        0x00
#define ADS1220_IDAC_10UA       0x01
#define ADS1220_IDAC_50UA       0x02
#define ADS1220_IDAC_100UA      0x03
#define ADS1220_IDAC_250UA      0x04
#define ADS1220_IDAC_500UA      0x05
#define ADS1220_IDAC_1000UA     0x06
#define ADS1220_IDAC_1500UA     0x07

// Register 3
#define ADS1220_I1MUX_DISABLED  0x00
#define ADS1220_I1MUX_AIN0      0x20
#define ADS1220_I1MUX_AIN1      0x40
#define ADS1220_I1MUX_AIN2      0x60
#define ADS1220_I1MUX_AIN3      0x80
#define ADS1220_I1MUX_REFP0     0xA0
#define ADS1220_I1MUX_REFN0     0xC0

#define ADS1220_I2MUX_DISABLED  0x00
#define ADS1220_I2MUX_AIN0      0x04
#define ADS1220_I2MUX_AIN1      0x08
#define ADS1220_I2MUX_AIN2      0x0C
#define ADS1220_I2MUX_AIN3      0x10
#define ADS1220_I2MUX_REFP0     0x14
#define ADS1220_I2MUX_REFN0     0x18

#define ADS1220_DRDYM_DRDY_ONLY 0x00
#define ADS1220_DRDYM_DOUT      0x02

typedef struct {
    uint8_t reg0;
    uint8_t reg1;
    uint8_t reg2;
    uint8_t reg3;
} ADS1220_Config_t;

/* ====================================================================
 * 函数声明
 * ==================================================================== */
void ADS1220_Init(void);
void ADS1220_DeInit(void);
void ADS1220_Reset(void);
void ADS1220_StartSync(void);
void ADS1220_PowerDown(void);
void ADS1220_SendCommand(uint8_t cmd);

void ADS1220_WriteRegister(uint8_t reg, uint8_t value);
uint8_t ADS1220_ReadRegister(uint8_t reg);
void ADS1220_WriteConfig(ADS1220_Config_t *config);
void ADS1220_ReadConfig(ADS1220_Config_t *config);

int32_t ADS1220_ReadData(void);
bool ADS1220_IsDataReady(void);
bool ADS1220_WaitForData(uint32_t timeout_ms);

float ADS1220_ReadVoltage(uint8_t gain, float vref);
int16_t ADS1220_ReadTemperature(void);

void ADS1220_SetInputMux(uint8_t mux);
void ADS1220_SetGain(uint8_t gain);
void ADS1220_SetDataRate(uint8_t rate);
void ADS1220_SetConversionMode(uint8_t mode);
void ADS1220_SetVref(uint8_t vref);

void ADS1220_GetDefaultConfig(ADS1220_Config_t *config);
int ADS1220_GetLastError(void);
void ADS1220_ClearError(void);

#endif /* __ADS1220_H */