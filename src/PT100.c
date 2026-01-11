/**
 * @file    PT100.c
 * @brief   PT100/PT1000温度传感器测量库 - 实现文件
 * @details 实现PT100温度测量的核心功能:
 *          - 电阻测量(绝对测量法/比例测量法)
 *          - 温度转换(Callendar-Van Dusen方程)
 *          - 多种接线方式支持
 * @version 1.1
 * @date    2024-01-11
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
 * 私有函数
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
    case 0x01:
        return 10.0f; /**< 10μA */
    case 0x02:
        return 50.0f; /**< 50μA */
    case 0x03:
        return 100.0f; /**< 100μA */
    case 0x04:
        return 250.0f; /**< 250μA */
    case 0x05:
        return 500.0f; /**< 500μA */
    case 0x06:
        return 1000.0f; /**< 1mA */
    case 0x07:
        return 1500.0f; /**< 1.5mA */
    default:
        return 0.0f; /**< IDAC关闭 */
    }
}

/**
 * @brief  获取IDAC1路由配置
 * @param  mux_setting: 输入多路复用器设置
 * @retval IDAC1路由值
 * @note   简化实现: 通常IDAC1路由到正输入端
 */
static uint8_t PT100_GetIDAC1Routing(uint8_t mux_setting)
{
    /* 根据输入通道选择，将IDAC1路由到正输入端 */
    if ((mux_setting & 0xF0) <= 0x20)
        return ADS1220_I1MUX_AIN0; // 对于MUX=0x00/0x10/0x20
    return ADS1220_I1MUX_AIN0;     // 默认路由
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
    uint8_t gain_reg = ADS1220_GAIN_8; // Default

    // Gain Mapping... (omitted)
    switch (config->gain)
    {
    case 1:
        gain_reg = ADS1220_GAIN_1;
        break;
    // ...
    case 128:
        gain_reg = ADS1220_GAIN_128;
        break;
    default:
        gain_reg = ADS1220_GAIN_8;
        config->gain = 8;
        break;
    }

    /**
     * @brief  读取ADC原始值(内部辅助函数)
     * @retval ADC原始值，失败返回0x7FFFFFFF
     */
    static int32_t PT100_ReadADCRaw(void)
    {
        ADS1220_ClearError();
        ADS1220_StartSync();
        if (!ADS1220_WaitForData(200))
            return 0x7FFFFFFF; // 20SPS ~50ms
        return ADS1220_ReadData();
    }

    /**
     * @brief  绝对测量法读取PT100电阻
     * @param  config: PT100配置参数指针
     * @retval 电阻值(Ω)，失败返回-1.0
     * @note   公式: R = V / I = (ADC * Vref / Gain) / IDAC
     */
    static float PT100_ReadResistance_Absolute(PT100_Config_t * config)
    {
        int32_t raw = PT100_ReadADCRaw();
        if (raw == 0x7FFFFFFF)
            return -1.0f;

        /* 计算电压 */
        float voltage = ((float)raw / 8388608.0f) * (config->vref / (float)config->gain);

        /* 获取IDAC电流(μA) */
        float current = PT100_GetIDACCurrent(config->idac);
        if (current == 0.0f)
            return -1.0f;

        /* 计算电阻: R(Ω) = V / I = V / (I_μA * 1e-6) */
        return (voltage / current) * 1000000.0f;
    }

    /**
     * @brief  比例测量法读取PT100电阻(软件实现)
     * @param  config: PT100配置参数指针
     * @retval 电阻值(Ω)，失败返回-1.0
     * @note   注意：这不是ADC硬件比例测量(VREF=REFP0)
     *         这里是分别测量PT100电压和参考电阻电压，然后计算比值
     *         虽然能消除电流绝对误差，但受转换间隔期间的噪声影响
     *         公式: R_pt100 = R_ref * (ADC_pt100 / ADC_ref)
     */
    static float PT100_ReadResistance_Ratiometric(PT100_Config_t * config)
    {
        if (config->ref_resistor <= 0.0f)
            return -1.0f;

        /* 步骤1: 测量参考电阻(假设IDAC电流流经参考电阻) */
        ADS1220_SetInputMux(config->ref_channel);
        Delay_ms(10); // 等待滤波器稳定(重要!)

        int32_t adc_ref = PT100_ReadADCRaw();
        if (adc_ref == 0x7FFFFFFF || adc_ref == 0)
            return -1.0f; // 避免除零

        /* 步骤2: 测量PT100 */
        ADS1220_SetInputMux(config->input_p);
        Delay_ms(10); // 等待滤波器稳定

        int32_t adc_pt100 = PT100_ReadADCRaw();
        if (adc_pt100 == 0x7FFFFFFF)
            return -1.0f;

        /* 计算电阻: R_pt100 = R_ref * (Code_pt100 / Code_ref)
         * 假设: 增益相同、VREF相同、IDAC相同且稳定 */
        return config->ref_resistor * ((float)adc_pt100 / (float)adc_ref);
    }

    /**
     * @brief  读取PT100电阻值
     * @param  config: PT100配置参数指针
     * @retval 电阻值(Ω)，失败返回负值
     * @note   根据use_ratiometric选择绝对测量或比例测量
     */
    float PT100_ReadResistance(PT100_Config_t * config)
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
     *         正温区: T = (-A + sqrt(A^2 - 4B(1-Rt/R0))) / (2B)
     *         负温区: 使用线性近似
     */
    float PT100_ResistanceToTemperature(float resistance, PT100_Type_t type)
    {
        /* 获取0°C时的标称电阻 */
        float r0 = (type == PT100_TYPE) ? PT100_R0 : PT1000_R0;
        float rt_r0 = resistance / r0; // 电阻比值

        /* 正温区(Rt/R0 >= 1): 使用Callendar-Van Dusen方程 */
        if (rt_r0 >= 1.0f)
        {
            float discriminant = CVD_A * CVD_A - 4.0f * CVD_B * (1.0f - rt_r0);
            if (discriminant >= 0)
                return (-CVD_A + sqrtf(discriminant)) / (2.0f * CVD_B);
        }

        /* 负温区或异常情况: 使用线性近似 */
        float alpha = (type == PT100_TYPE) ? PT100_ALPHA : PT1000_ALPHA;
        return (resistance - r0) / (r0 * alpha);
    }

    /**
     * @brief  读取PT100温度值
     * @param  config: PT100配置参数指针
     * @retval 温度值(°C)，失败返回-999.0
     * @note   内部调用PT100_ReadResistance和PT100_ResistanceToTemperature
     */
    float PT100_ReadTemperature(PT100_Config_t * config)
    {
        float r = PT100_ReadResistance(config);
        if (r < 0)
            return -999.0f; // 读取失败
        return PT100_ResistanceToTemperature(r, config->type);
    }