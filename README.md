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

### PT100温度测量
- 🌡️ **PT100/PT1000支持** - 自动识别
- 🎯 **高精度算法** - Callendar-Van Dusen方程
- ⚡ **可配置激励电流** - 10μA ~ 1500μA
- 🔧 **灵活配置** - 增益、采样率、滤波器
- 📊 **直接温度输出** - 自动电阻-温度转换
- 🛠️ **校准功能** - 单点/多点校准支持

---

## 📋 目录

- [硬件要求](#硬件要求)
- [快速开始](#快速开始)
- [硬件连接](#硬件连接)
- [配置选项](#配置选项)
- [API文档](#api文档)
- [示例代码](#示例代码)
- [常见问题](#常见问题)
- [许可证](#许可证)

---

## 🔧 硬件要求

### 最低要求
- **MCU**: STM32F103C8T6 或更高
- **Flash**: 16KB+
- **RAM**: 4KB+
- **外设**: SPI1, GPIO, USART1 (调试)

### 推荐硬件
- STM32F103C8T6开发板（蓝/黑丸）
- ADS1220 模块
- PT100温度传感器（三线制或四线制）
- ST-Link V2 调试器

### 支持的芯片
- ✅ STM32F103C8T6 (64KB Flash)
- ✅ STM32F103RBT6 (128KB Flash)
- ✅ STM32F103CBT6 (128KB Flash)
- ✅ STM32F103RCT6 (256KB Flash)
- ✅ STM32F103VET6 (512KB Flash)

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
        .vref = 2.048f,
        .input_p = ADS1220_MUX_AIN0_AIN1
    };
    
    PT100_Init(&pt100_config);
    
    while(1)
    {
        // 读取温度
        float temperature = PT100_ReadTemperature(&pt100_config);
        printf("Temperature: %.2f °C\n", temperature);
        
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

**详细连接说明**:  见 [docs/hardware-connection.md](docs/hardware-connection.md)

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

**完整配置指南**: [docs/configuration-guide.md](docs/configuration-guide.md)

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
float ADS1220_ReadVoltage(uint8_t gain, float vref);
bool ADS1220_WaitForData(uint32_t timeout_ms);
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
float PT100_ReadResistance(PT100_Config_t *config);
float PT100_ReadTemperature(PT100_Config_t *config);
float PT100_ResistanceToTemperature(float resistance, PT100_Type_t type);
void PT100_Calibrate(PT100_Config_t *config, float known_temp, float *offset);
```

**完整API参考**: [docs/api-reference.md](docs/api-reference.md)

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
            float voltage = ADS1220_ReadVoltage(1, 2.048);
            printf("Voltage: %. 6f V\n", voltage);
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
        .vref = 2.048f,
        .input_p = ADS1220_MUX_AIN0_AIN1
    };
    
    PT100_Init(&pt100);
    
    while(1)
    {
        float temp = PT100_ReadTemperature(&pt100);
        float res = PT100_ReadResistance(&pt100);
        
        printf("Temperature: %.2f °C\n", temp);
        printf("Resistance: %.3f Ω\n", res);
        
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
    
    float voltage = ADS1220_ReadVoltage(1, 2.048);
    printf("CH%d: %.6f V\n", i, voltage);
}
```

**更多示例**: [examples/](examples/)

---

## 📊 性能指标

| 参数 | 值 | 说明 |
|-----|----|----|
| ADC分辨率 | 24位 | 有效位数约20位 |
| 测量范围 | ±2. 048V | 内部基准 |
| 增益范围 | 1 ~ 128 | 可编程 |
| 采样率 | 20 ~ 2000 SPS | 可配置 |
| 温度精度 | ±0.1°C | PT100, -50~150°C |
| 电阻精度 | ±0.1Ω | PT100 |
| SPI时钟 | 最高4MHz | 实际使用2. 25MHz |

---

## ❓ 常见问题

### Q1: 读取数据始终为0或0xFFFFFF？
**A**: 检查：
1. SPI连接是否正确
2. CS和DRDY引脚定义是否匹配
3. 芯片供电是否正常（3.3V）
4. SPI模式是否正确（CPOL=0, CPHA=1）

### Q2: PT100温度读数不准确？
**A**: 
1. 确认IDAC电流设置（推荐250μA）
2. 检查PT100接线（三线制需抵消线阻）
3. 验证参考电压精度
4. 使用校准功能：`PT100_Calibrate()`

### Q3: 延时不准确？
**A**: 
- 使用SysTick延时模式
- 确保系统时钟为72MHz
- 检查编译器优化等级

### Q4: 编译错误：`undefined reference to Delay_us`？
**A**: 
- 确认延时模式配置
- 如果使用外部延时，需提供实现

### Q5: 数据噪声较大？
**A**: 
1. 启用50/60Hz滤波器
2. ���低采样率（20 SPS）
3. 多次采样求平均
4. 检查电源和地线质量

---

## 🗺️ 路线图

- [x] 基础ADS1220驱动
- [x] PT100/PT1000测量
- [x] 三种延时模式
- [x] 硬件/软件SPI
- [ ] DMA传输支持
- [ ] RTOS集成示例
- [ ] 热电偶测量支持
- [ ] 多点校准算法
- [ ] Python上位机工具
- [ ] HAL库版本

---

## 🤝 贡献

欢迎提交Issue和Pull Request！

### 贡献指南
1. Fork本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启Pull Request

---

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 👨‍💻 作者

**Your Name**
- GitHub: [@yourusername](https://github.com/yourusername)
- Email: your.email@example.com

---

## 🙏 致谢

- [Texas Instruments](https://www.ti.com/) - ADS1220数据手册
- [STMicroelectronics](https://www.st.com/) - STM32参考手册
- 所有贡献者和用户

---

## 📮 联系方式

- 提交Issue:  [GitHub Issues](https://github.com/yourusername/ADS1220-PT100-STM32/issues)
- 邮箱: your.email@example.com
- QQ群: 123456789

---

## 📖 相关资源

- [ADS1220数据手册](https://www.ti.com/lit/ds/symlink/ads1220.pdf)
- [PT100标准 IEC 60751](https://webstore.iec.ch/publication/3426)
- [STM32F103参考手册](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
- [项目Wiki](https://github.com/yourusername/ADS1220-PT100-STM32/wiki)

---

**⭐ 如果这个项目对您有帮助，请给个星标！**

---

*最后更新: 2025-01-11*
