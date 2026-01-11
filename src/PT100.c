/**
 * @file    PT100.c
 * @brief   PT100/PT1000温度传感器测量库 - 实现文件 (已修复)
 * @details 实现PT100温度测量的核心功能:
 * - 修复了函数嵌套导致的编译错误
 * - 补全了寄存器配置写入逻辑
 * - 电阻测量(绝对测量法/比例测量法)
 * - 温度转换(Callendar-Van Dusen方程)
 * @version 1.2
 * @date    2026-01-12
 */

#include "PT100.h"
#include <math.h>

/* ====================================================================
 * PT100常数定义
 * ==================================================================== */
#define PT100_R0 100.0f       /**< PT100在0°C时的标称电阻(Ω) */
#define PT100_ALPHA 0.00385f  /**< PT100温度系数(Ω/Ω/°C) */
#define PT1000_R0 1000.0f     /**< PT1000在0°C时的标称电阻(Ω) */
#define PT1000_ALPHA 0.00385f /**< PT1000温度系数(Ω/Ω/°C) */

/* Callendar-Van Dusen方程系数 */
#define CVD_A 3.9083e-3f /**< CVD方程系数A */
#define CVD_B -5.775e-7f /**< CVD方程系数B */

/* ====================================================================
 * 私有辅助函数 (已移出 Init 函数)
 * ==================================================================== */

/**
 * @brief  获取IDAC电流值
 * @param  idac_setting: IDAC设置值
 * @retval 电流值(μA)
 */
static float PT100_GetIDACCurrent(uint8_t idac_setting)
{
    switch (idac_setting & 0x07)
    {
    case ADS1220_IDAC_10UA:   return 10.0f;
    case ADS1220_IDAC_50UA:   return 50.0f;
    case ADS1220_IDAC_100UA:  return 100.0f;
    case ADS1220_IDAC_250UA:  return 250.0f;
    case ADS1220_IDAC_500UA:  return 500.0f;
    case ADS1220_IDAC_1000UA: return 1000.0f;
    case ADS1220_IDAC_1500UA: return 1500.0f;
    default: return 0.0f;
    }
}

/**
 * @brief  读取ADC原始值(内部辅助函数)
 * @retval ADC原始值，失败返回0x7FFFFFFF
 */
static int32_t PT100_ReadADCRaw(void)
{
    ADS1220_ClearError();
    ADS1220_StartSync();
    
    // 等待数据就绪 (20SPS约50ms，给予充足的超时时间)
    if (!ADS1220_WaitForData(100)) 
    {
        return 0x7FFFFFFF; 
    }
    return ADS1220_ReadData();
}

/**
 * @brief  绝对测量法读取PT100电阻
 * @param  config: PT100配置参数指针
 * @retval 电阻值(Ω)，失败返回-1.0
 * @note   公式: R = V / I = (ADC * Vref / Gain) / IDAC
 */
static float PT100_ReadResistance_Absolute(PT100_Config_t *config)
{
    int32_t raw = PT100_ReadADCRaw();
    if (raw == 0x7FFFFFFF)
        return -1.0f;

    /* 计算电压 V = (Raw / 2^23) * (Vref / Gain) */
    /* 注意：ADS1220_ReadData返回的是带符号的32位，满量程是2^23 */
    float voltage = ((float)raw / 8388608.0f) * (config->vref / (float)config->gain);

    /* 获取IDAC电流(μA) */
    float current_ua = PT100_GetIDACCurrent(config->idac);
    if (current_ua == 0.0f)
        return -1.0f;

    /* 计算电阻: R(Ω) = V / I = V / (I_μA * 1e-6) */
    return (voltage / (current_ua / 1000000.0f));
}

/**
 * @brief  软件比例测量法读取PT100电阻
 * @param  config: PT100配置参数指针
 * @retval 电阻值(Ω)，失败返回-1.0
 * @note   分别测量参考电阻和PT100，计算比值消除电流误差
 */
static float PT100_ReadResistance_Ratiometric(PT100_Config_t *config)
{
    if (config->ref_resistor <= 0.0f)
        return -1.0f;

    /* 步骤1: 测量参考电阻 (切换通道) */
    ADS1220_SetInputMux(config->ref_channel);
    
    // 重要：切换通道后丢弃前几个样本或延时，等待数字滤波器和信号稳定
    Delay_ms(60); // 20SPS下至少等待一个周期(50ms) + 裕量
    
    int32_t adc_ref = PT100_ReadADCRaw();
    if (adc_ref == 0x7FFFFFFF || adc_ref == 0)
        return -1.0f; 

    /* 步骤2: 测量PT100 (切换回主通道) */
    ADS1220_SetInputMux(config->input_p); // 注意这里假设input_p包含了MUX组合
    
    Delay_ms(60); // 等待稳定
    
    int32_t adc_pt100 = PT100_ReadADCRaw();
    if (adc_pt100 == 0x7FFFFFFF)
        return -1.0f;

    /* 计算电阻: R_pt100 = R_ref * (Code_pt100 / Code_ref) */
    return config->ref_resistor * ((float)adc_pt100 / (float)adc_ref);
}

/* ====================================================================
 * 公共API函数实现
 * ==================================================================== */

/**
 * @brief  初始化PT100测量
 * @param  config: PT100配置参数指针
 * @note   配置ADS1220的寄存器和IDAC路由
 */
void PT100_Init(PT100_Config_t *config)
{
    ADS1220_Config_t ads_config;
    uint8_t gain_reg;

    /* 1. 增益映射 */
    switch (config->gain)
    {
    case 1:   gain_reg = ADS1220_GAIN_1; break;
    case 2:   gain_reg = ADS1220_GAIN_2; break;
    case 4:   gain_reg = ADS1220_GAIN_4; break;
    case 8:   gain_reg = ADS1220_GAIN_8; break;
    case 16:  gain_reg = ADS1220_GAIN_16; break;
    case 32:  gain_reg = ADS1220_GAIN_32; break;
    case 64:  gain_reg = ADS1220_GAIN_64; break;
    case 128: gain_reg = ADS1220_GAIN_128; break;
    default:  gain_reg = ADS1220_GAIN_8; config->gain = 8; break;
    }

    /* 2. 构建寄存器配置 */
    
    /* Reg0: 输入MUX + 增益 + PGA使能 */
    // 使用 config->input_p 作为 MUX 设置 (假设用户传入的是 ADS1220_MUX_xxx 宏)
    ads_config.reg0 = (config->input_p & 0xF0) | gain_reg | ADS1220_PGA_ENABLED;

    /* Reg1: 数据速率 + 模式 */
    // 默认使用 20SPS (低噪声), 正常模式, 单次转换(即使是连续读取，也可通过Start控制)
    // 关闭内部温度传感器 (TS_DISABLED)
    ads_config.reg1 = ADS1220_DR_20SPS | ADS1220_MODE_NORMAL | ADS1220_CM_SINGLE | ADS1220_TS_DISABLED;

    /* Reg2: 基准电压 + FIR + IDAC电流 */
    // 默认使用内部2.048V基准 (软件比例法也依赖内部基准进行电压量化)
    // 启用 50Hz/60Hz 同时抑制滤波器
    ads_config.reg2 = ADS1220_VREF_INT | ADS1220_FIR_50HZ_60HZ | (config->idac & 0x07);

    /* Reg3: IDAC路由 */
    uint8_t i1mux = ADS1220_I1MUX_DISABLED;
    uint8_t i2mux = ADS1220_I2MUX_DISABLED;

    // 自动推断 IDAC1 的位置: 通常连接到 PT100 的正端 (AIN0)
    // 简单起见，我们根据 mux input_p 的高4位推断，或者默认 AIN0
    if ((config->input_p & 0xF0) == ADS1220_MUX_AIN0_AIN1) i1mux = ADS1220_I1MUX_AIN0;
    else if ((config->input_p & 0xF0) == ADS1220_MUX_AIN1_AIN2) i1mux = ADS1220_I1MUX_AIN1;
    else i1mux = ADS1220_I1MUX_AIN0; // 默认

    // 如果是3线制，需要配置 IDAC2
    if (config->wire_mode == PT100_3WIRE)
    {
        i2mux = config->idac2_pin; // 使用用户指定的引脚 (例如 ADS1220_I2MUX_AIN2)
    }
    
    ads_config.reg3 = i1mux | i2mux | ADS1220_DRDYM_DRDY_ONLY;

    /* 3. 写入硬件配置 */
    ADS1220_WriteConfig(&ads_config);
    Delay_ms(10); // 等待配置生效
}

/**
 * @brief  读取PT100电阻值
 * @param  config: PT100配置参数指针
 * @retval 电阻值(Ω)，失败返回负值
 * @note   根据use_ratiometric选择绝对测量或比例测量
 */
float PT100_ReadResistance(PT100_Config_t *config)
{
    if (config->use_ratiometric)
        return PT100_ReadResistance_Ratiometric(config);
    else
        return PT100_ReadResistance_Absolute(config);
}

/**
 * @brief  电阻值转换为温度值
 * @param  resistance: 电阻值(Ω)
 * @param  type: PT100类型(PT100或PT1000)
 * @retval 温度值(°C)
 * @note   使用Callendar-Van Dusen方程进行高精度转换
 */
float PT100_ResistanceToTemperature(float resistance, PT100_Type_t type)
{
    /* 获取0°C时的标称电阻 */
    float r0 = (type == PT100_TYPE) ? PT100_R0 : PT1000_R0;
    float rt_r0 = resistance / r0; // 电阻比值

    /* 正温区(Rt/R0 >= 1): 使用Callendar-Van Dusen方程 */
    /* Rt = R0 * (1 + A*t + B*t^2) -> B*t^2 + A*t + (1 - Rt/R0) = 0 */
    /* 解一元二次方程: t = [-A + sqrt(A^2 - 4B(1 - Rt/R0))] / 2B */
    if (rt_r0 >= 1.0f)
    {
        float discriminant = CVD_A * CVD_A - 4.0f * CVD_B * (1.0f - rt_r0);
        if (discriminant >= 0)
            return (-CVD_A + sqrtf(discriminant)) / (2.0f * CVD_B);
    }

    /* 负温区或简单线性近似: T = (Rt - R0) / (R0 * Alpha) */
    /* 注意：对于极低温度(-200~0)，CVD方程有C系数，此处简化为线性或沿用正温公式误差也较小 */
    float alpha = (type == PT100_TYPE) ? PT100_ALPHA : PT1000_ALPHA;
    return (resistance - r0) / (r0 * alpha);
}

/**
 * @brief  读取PT100温度值
 * @param  config: PT100配置参数指针
 * @retval 温度值(°C)，失败返回-999.0
 */
float PT100_ReadTemperature(PT100_Config_t *config)
{
    float r = PT100_ReadResistance(config);
    if (r < 0)
        return -999.0f; // 读取失败
    return PT100_ResistanceToTemperature(r, config->type);
}

/**
 * @brief  PT100单点校准
 * @param  config: PT100配置参数指针
 * @param  known_temp: 已知的标准温度(°C)
 * @param  offset: 输出的温度偏移量(°C)
 */
void PT100_Calibrate(PT100_Config_t *config, float known_temp, float *offset)
{
    float measured_temp = PT100_ReadTemperature(config);
    if (measured_temp > -900.0f)
    {
        *offset = known_temp - measured_temp;
    }
}