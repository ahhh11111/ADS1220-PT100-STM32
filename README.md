# ADS1220 + PT100 Driver for STM32F103

[![License:  MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/Platform-STM32F103-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html)
[![Library](https://img.shields.io/badge/Library-STM32%20SPL-green.svg)](https://www.st.com/en/embedded-software/stsw-stm32054.html)

🌡️ 高精度24位ADC ADS1220驱动库 + PT100/PT1000温度传感器测量库

适用于STM32F103系列微控制器，基于STM32标准外设库（SPL）开发。

---

## ✨ 特性

### ADS1220驱动
- ✅ **完整的寄存器配置** - 支持所有ADS1220功能
- ✅ **硬件/软件SPI** - 宏定义切换，灵活适配
- ✅ **三种延时模式** - SysTick精确延时 / 简单循环 / 外部函数
- ✅ **24位精度** - 完整的符号扩展和数据处理
- ✅ **多通道支持** - 差分/单端测量
- ✅ **可编程增益** - 1~128倍增益
- ✅ **内部温度传感器** - 芯片温度监测
- ✅ **低噪声滤波** - 50/60Hz陷波滤波器
- ✅ **硬件比例测量** - ADS1220内置比例测量功能

### PT100温度测量
- 🌡️ **PT100/PT1000支持** - 自动识别
- 🎯 **高精度算法** - 查表+线性插值，精度±0.1°C
- ⚡ **可配置激励电流** - 10μA ~ 1500μA
- 🔧 **灵活配置** - 增益、采样率、滤波器
- 📊 **直接温度输出** - 自动电阻-温度转换
- 🛠️ **校准功能** - 单点校准支持
- 🔢 **纯整数运算** - 适用于无FPU的MCU（如STM32F103）

---

## 🔧 硬件
- **MCU**: STM32F103C8T6
- **外设**: SPI1, GPIO, USART1 (调试)

---

## 🚀 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/yourusername/ADS1220-PT100-STM32.git
cd ADS1220-PT100-STM32
```

### 2. 添加到您的项目

#### Keil MDK
1. 将 `src/` 目录下的文件添加到项目
2. 添加头文件路径：`项目设置` -> `C/C++` -> `Include Paths`
3. 配置系统时钟为72MHz
4. 编译运行

#### STM32CubeIDE
1. 复制 `src/` 文件到 `Core/Src` 和 `Core/Inc`
2. 添加到编译列表
3. 配置时钟树
4. 构建项目

### 3. 基本使用

```c
#include "ADS1220.h"
#include "PT100.h"

int main(void)
{
    // 系统初始化
    SystemInit();
    
    // 初始化ADS1220
    ADS1220_Init();
    
    // 配置PT100测量
    PT100_Config_t pt100_config = {
        .type = PT100_TYPE,
        .idac = PT100_IDAC_250UA,
        .gain = 8,
        .vref_mv = 2048,  // 2048mV = 2.048V
        .input_p = ADS1220_MUX_AIN0_AIN1,
        .wire_mode = PT100_2WIRE
    };
    
    PT100_Init(&pt100_config);
    
    while(1)
    {
        // 读取温度（单位：0.01°C）
        int32_t temperature = PT100_ReadTemperature_Int(&pt100_config);
        
        // 输出格式转换：2500 -> 25.00°C
        printf("Temperature: %ld.%02ld °C\n", 
               (long)(temperature / 100), 
               (long)(temperature >= 0 ? temperature % 100 : (-temperature) % 100));
        
        Delay_ms(1000);
    }
}
```

---

## 🔌 硬件连接

### 引脚定义（默认配置）

| ADS1220 引脚 | STM32F103 引脚 | 功能 | 说明 |
|-------------|---------------|------|------|
| CS          | PA4           | 片选 | 可修改 |
| SCLK        | PA5           | SPI时钟 | SPI1_SCK |
| MISO (DOUT) | PA6           | SPI数据输出 | SPI1_MISO |
| MOSI (DIN)  | PA7           | SPI数据输入 | SPI1_MOSI |
| DRDY        | PA3           | 数据就绪 | 可修改 |
| AVDD        | 3.3V          | 模拟电源 | - |
| DGND        | GND           | 数字地 | - |
| AVSS        | GND           | 模拟地 | - |

### PT100连接（三线制）

```
                  STM32F103
                 ┌─────────┐
    PT100        │         │
   ┌─────┐       │  ADS1220│
   │     │───────┤ AIN0    │  (IDAC1 输出 + PT100正极)
   │     │       │         │
   │     │───────┤ AIN1    │  (PT100负极)
   └─────┘       │         │
                 │  REFP0  │──── 外部2. 5V基准(可选)
                 │  REFN0  │──── GND
                 └─────────┘
```

---

## ⚙️ 配置选项

### 1. SPI模式选择

在 `ADS1220.h` 中：

```c
// 使用硬件SPI (默认，推荐)
// #define ADS1220_USE_SOFTWARE_SPI

// 使用软件SPI (IO口模拟)
#define ADS1220_USE_SOFTWARE_SPI
```

### 2. 延时函数选择

```c
// 选项1: SysTick精确延时 (推荐)
#define ADS1220_DELAY_SYSTICK

// 选项2: 简单循环延时
// #define ADS1220_DELAY_SIMPLE

// 选项3: 外部延时函数
// #define ADS1220_DELAY_EXTERNAL
```

| 延时模式 | 精度 | 资源占用 | 推荐场景 |
|---------|------|---------|---------|
| SysTick | ±1μs | SysTick定时器 | 高精度测量 |
| 简单循环 | ±10μs | 无 | 资源受限 |
| 外部函数 | 自定义 | 自定义 | 已有延时系统 |

### 3. 引脚配置

修改 `ADS1220.h` 中的引脚定义：

```c
// CS 片选引脚
#define ADS1220_CS_PIN          GPIO_Pin_4
#define ADS1220_CS_PORT         GPIOA

// DRDY 数据就绪引脚
#define ADS1220_DRDY_PIN        GPIO_Pin_3
#define ADS1220_DRDY_PORT       GPIOA
```

---

## 📚 API文档

### ADS1220 核心函数

#### 初始化和配置

```c
void ADS1220_Init(void);
void ADS1220_DeInit(void);
void ADS1220_Reset(void);
void ADS1220_WriteConfig(ADS1220_Config_t *config);
void ADS1220_GetDefaultConfig(ADS1220_Config_t *config);
```

#### 数据读取

```c
int32_t ADS1220_ReadData(void);
int32_t ADS1220_ReadVoltage_Int(uint8_t gain, int32_t vref_unit);
uint8_t ADS1220_WaitForData(uint32_t timeout_ms);
```

#### 快速配置

```c
void ADS1220_SetInputMux(uint8_t mux);
void ADS1220_SetGain(uint8_t gain);
void ADS1220_SetDataRate(uint8_t rate);
void ADS1220_SetConversionMode(uint8_t mode);
```

### PT100 测量函数

```c
void PT100_Init(PT100_Config_t *config);
int32_t PT100_ReadResistance_Int(PT100_Config_t *config);   // 返回值单位: mΩ
int32_t PT100_ReadTemperature_Int(PT100_Config_t *config);  // 返回值单位: 0.01°C
int32_t PT100_ResistanceToTemperature_Int(int32_t resistance_mohm, PT100_Type_t type);
void PT100_Calibrate_Int(PT100_Config_t *config, int32_t known_temp_centideg, int32_t *offset_centideg);
```

**单位说明:**
- 电阻: mΩ（毫欧姆），例如 100000mΩ = 100Ω
- 温度: 0.01°C（百分之一摄氏度），例如 2500 = 25.00°C

---

## 💡 示例代码

### 示例1: 基本ADC读取

```c
#include "ADS1220.h"

int main(void)
{
    ADS1220_Init();
    
    ADS1220_Config_t config;
    ADS1220_GetDefaultConfig(&config);
    config.reg0 = ADS1220_MUX_AIN0_AIN1 | ADS1220_GAIN_1;
    ADS1220_WriteConfig(&config);
    
    while(1)
    {
        ADS1220_StartSync();
        if (ADS1220_WaitForData(1000))
        {
            // 读取电压 (单位: mV)
            int32_t voltage_mv = ADS1220_ReadVoltage_Int(1, 2048);
            printf("Voltage: %ld mV\n", (long)voltage_mv);
        }
        Delay_ms(100);
    }
}
```

### 示例2: PT100温度测量

```c
#include "PT100.h"

int main(void)
{
    ADS1220_Init();
    
    PT100_Config_t pt100 = {
        .type = PT100_TYPE,
        .idac = PT100_IDAC_250UA,
        .gain = 8,
        .vref_mv = 2048,
        .input_p = ADS1220_MUX_AIN0_AIN1,
        .wire_mode = PT100_2WIRE
    };
    
    PT100_Init(&pt100);
    
    while(1)
    {
        int32_t temp = PT100_ReadTemperature_Int(&pt100);  // 单位: 0.01°C
        int32_t res = PT100_ReadResistance_Int(&pt100);    // 单位: mΩ
        
        printf("Temperature: %ld.%02ld °C\n", 
               (long)(temp / 100), 
               (long)(temp >= 0 ? temp % 100 : (-temp) % 100));
        printf("Resistance: %ld.%03ld Ω\n", 
               (long)(res / 1000), (long)(res % 1000));
        
        Delay_ms(1000);
    }
}
```

### 示例3: 多通道扫描

```c
uint8_t channels[] = {
    ADS1220_MUX_AIN0_AVSS,
    ADS1220_MUX_AIN1_AVSS,
    ADS1220_MUX_AIN2_AVSS,
    ADS1220_MUX_AIN3_AVSS
};

for (int i = 0; i < 4; i++)
{
    ADS1220_SetInputMux(channels[i]);
    ADS1220_StartSync();
    ADS1220_WaitForData(1000);
    
    int32_t voltage_mv = ADS1220_ReadVoltage_Int(1, 2048);
    printf("CH%d: %ld mV\n", i, (long)voltage_mv);
}
```

---

## 📊 性能指标

| 参数 | 值 | 说明 |
|-----|----|----|
| ADC分辨率 | 24位 | 有效位数约20位 |
| 测量范围 | ±2.048V | 内部基准 |
| 增益范围 | 1 ~ 128 | 可编程 |
| 采样率 | 20 ~ 2000 SPS | 可配置 |
| SPI时钟 | 最高4MHz | 实际使用2.25MHz |
| 温度精度 | ±0.1°C | 使用查表+线性插值 |

---
