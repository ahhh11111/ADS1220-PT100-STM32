#include "PT100.h"
#include <math.h>

/* ====================================================================
 * PT100 常数定义
 * ==================================================================== */
// PT100 标准参数 (IEC 60751)
#define PT100_R0            100.0f      // 0°C时的电阻值 (Ω)
#define PT100_ALPHA         0.00385f    // 温度系数 (Ω/Ω/°C)

// PT1000 标准参数
#define PT1000_R0           1000.0f     // 0°C时的电阻值 (Ω)
#define PT1000_ALPHA        0.00385f    // 温度系数

// Callendar-Van Dusen 系数 (用于高精度计算)
#define CVD_A               3.9083e-3f
#define CVD_B               -5.775e-7f
#define CVD_C               -4.183e-12f // 仅用于负温度

/* ====================================================================
 * IDAC电流值映射 (μA) - 修复索引对应关系
 * ==================================================================== */
static float PT100_GetIDACCurrent(uint8_t idac_setting)
{
    switch(idac_setting & 0x07)
    {
        case 0x00: return 0.0f;      // OFF
        case 0x01: return 10.0f;     // 10μA
        case 0x02: return 50.0f;     // 50μA
        case 0x03: return 100.0f;    // 100μA
        case 0x04: return 250.0f;    // 250μA
        case 0x05: return 500.0f;    // 500μA
        case 0x06: return 1000.0f;   // 1000μA
        case 0x07: return 1500.0f;   // 1500μA
        default:    return 0.0f;
    }
}

static uint8_t PT100_GetIDAC1Routing(uint8_t mux_setting)
{
    switch(mux_setting & 0xF0)
    {
        case ADS1220_MUX_AIN0_AIN1:
        case ADS1220_MUX_AIN0_AIN2:
        case ADS1220_MUX_AIN0_AIN3:
        case ADS1220_MUX_AIN0_AVSS:
            return ADS1220_I1MUX_AIN0;
        case ADS1220_MUX_AIN1_AIN0:
        case ADS1220_MUX_AIN1_AIN2:
        case ADS1220_MUX_AIN1_AIN3:
        case ADS1220_MUX_AIN1_AVSS:
            return ADS1220_I1MUX_AIN1;
        case ADS1220_MUX_AIN2_AIN3:
        case ADS1220_MUX_AIN2_AVSS:
            return ADS1220_I1MUX_AIN2;
        case ADS1220_MUX_AIN3_AIN2:
        case ADS1220_MUX_AIN3_AVSS:
            return ADS1220_I1MUX_AIN3;
        default:
            return ADS1220_I1MUX_AIN0;
    }
}

static uint8_t PT100_GetIDAC2Routing(uint8_t mux_setting)
{
    switch(mux_setting & 0xF0)
    {
        case ADS1220_MUX_AIN0_AIN1:
        case ADS1220_MUX_AIN0_AIN2:
        case ADS1220_MUX_AIN0_AIN3:
        case ADS1220_MUX_AIN0_AVSS:
            return ADS1220_I2MUX_AIN0;
        case ADS1220_MUX_AIN1_AIN0:
        case ADS1220_MUX_AIN1_AIN2:
        case ADS1220_MUX_AIN1_AIN3:
        case ADS1220_MUX_AIN1_AVSS:
            return ADS1220_I2MUX_AIN1;
        case ADS1220_MUX_AIN2_AIN3:
        case ADS1220_MUX_AIN2_AVSS:
            return ADS1220_I2MUX_AIN2;
        case ADS1220_MUX_AIN3_AIN2:
        case ADS1220_MUX_AIN3_AVSS:
            return ADS1220_I2MUX_AIN3;
        default:
            return ADS1220_I2MUX_AIN0;
    }
}

/**
 * @brief  初始化PT100测量
 * @param  config: PT100配置结构体指针
 * @retval 无
 * @note   3线制接线说明:
 *         - PT100一端连接IDAC1输出引脚
 *         - PT100另一端通过导线连接到差分输入负端
 *         - IDAC2输出到导线补偿引脚，消除导线电阻影响
 *         例如: IDAC1->AIN0 (PT100+), AIN1 (PT100-), IDAC2->AIN2 (导线补偿)
 */
void PT100_Init(PT100_Config_t *config)
{
    ADS1220_Config_t ads_config;
    uint8_t gain_reg;
    
    // 根据增益设置寄存器值
    switch(config->gain) {
        case 1:   gain_reg = ADS1220_GAIN_1; break;
        case 2:   gain_reg = ADS1220_GAIN_2; break;
        case 4:   gain_reg = ADS1220_GAIN_4; break;
        case 8:   gain_reg = ADS1220_GAIN_8; break;
        case 16:  gain_reg = ADS1220_GAIN_16; break;
        case 32:  gain_reg = ADS1220_GAIN_32; break;
        case 64:  gain_reg = ADS1220_GAIN_64; break;
        case 128: gain_reg = ADS1220_GAIN_128; break;
        default:   gain_reg = ADS1220_GAIN_8; config->gain = 8; break;
    }
    
    // 配置 Register 0: 差分输入 + 增益
    ads_config.reg0 = config->input_p | gain_reg | ADS1220_PGA_ENABLED;
    
    // 配置 Register 1: 20 SPS (低噪声) + 正常模式 + 单次转换
    ads_config.reg1 = ADS1220_DR_20SPS | 
                      ADS1220_MODE_NORMAL | 
                      ADS1220_CM_SINGLE |
                      ADS1220_TS_DISABLED |
                      ADS1220_BCS_OFF;
    
    // 配置 Register 2: 参考电压 + 50/60Hz滤波 + IDAC电流
    if (config->vref >= 2.0f && config->vref <= 2.1f) {
        // 使用内部2.048V基准
        ads_config.reg2 = ADS1220_VREF_INT | ADS1220_FIR_50HZ_60HZ | config->idac;
    } else {
        // 使用外部基准
        ads_config.reg2 = ADS1220_VREF_EXT_REF0 | ADS1220_FIR_50HZ_60HZ | config->idac;
    }
    
    // 配置 Register 3: IDAC1路由 + IDAC2路由(3线制)
    // 根据输入通道配置IDAC输出
    uint8_t idac1_routing;
    uint8_t idac2_routing = ADS1220_I2MUX_DISABLED; // 默认禁用IDAC2

    idac1_routing = PT100_GetIDAC1Routing(config->input_p);
    
    // 3线制配置: 启用IDAC2
    if (config->wire_mode == PT100_3WIRE) {
        idac2_routing = config->idac2_pin;
    } else if (config->use_ratiometric) {
        idac2_routing = PT100_GetIDAC2Routing(config->ref_channel);
    }
    
    ads_config.reg3 = idac1_routing | idac2_routing;
    
    // 写入配置
    ADS1220_WriteConfig(&ads_config);
    
    // 等待配置稳定
    Delay_ms(50);
}

/**
 * @brief  读取ADC原始数据
 * @param  无
 * @retval ADC原始值，错误返回0x7FFFFFFF
 */
static int32_t PT100_ReadADCRaw(void)
{
    // 清除之前的错误
    ADS1220_ClearError();
    
    // 启动转换
    ADS1220_StartSync();
    
    // 等待数据就绪 (20SPS需要约50ms)
    if (!ADS1220_WaitForData(1000)) {
        return 0x7FFFFFFF; // 超时错误
    }
    
    // 读取ADC原始值
    int32_t data = ADS1220_ReadData();
    
    // 检查SPI错误
    if (ADS1220_GetLastError() != ADS1220_ERROR_NONE) {
        return 0x7FFFFFFF; // SPI错误
    }
    
    return data;
}

/**
 * @brief  读取PT100电阻值 (绝对测量法)
 * @param  config: PT100配置结构体指针
 * @retval 电阻值 (Ω)，错误返回-1.0f
 */
static float PT100_ReadResistance_Absolute(PT100_Config_t *config)
{
    int32_t raw_data;
    float voltage;
    float resistance;
    float idac_current_uA;
    
    raw_data = PT100_ReadADCRaw();
    if (raw_data == 0x7FFFFFFF) {
        return -1.0f;
    }

    // 读取ADC值并转换为电压
    voltage = ((float)raw_data / 8388608.0f) * (config->vref / (float)config->gain);
    
    // 获取IDAC电流值
    idac_current_uA = PT100_GetIDACCurrent(config->idac);
    
    if (idac_current_uA == 0.0f) {
        return -1.0f; // IDAC未配置
    }
    
    // 计算电阻:  R = V / I
    // 电压单位:  V, 电流单位: μA
    // R(Ω) = V(V) / I(μA) * 1000000
    resistance = (voltage / idac_current_uA) * 1000000.0f;
    
    return resistance;
}

/**
 * @brief  读取PT100电阻值 (比例测量法)
 * @param  config: PT100配置结构体指针
 * @retval 电阻值 (Ω)，错误返回-1.0f
 * @note   消除IDAC电流源误差，提高测量精度
 */
static float PT100_ReadResistance_Ratiometric(PT100_Config_t *config)
{
    int32_t adc_ref, adc_pt100;
    float resistance;

    if (config->ref_resistor <= 0.0f) {
        return -1.0f;
    }
    
    // 1. 测量参考电阻
    ADS1220_SetInputMux(config->ref_channel);
    Delay_ms(10); // 等待配置稳定
    
    adc_ref = PT100_ReadADCRaw();
    if (adc_ref == 0x7FFFFFFF || adc_ref == 0) {
        return -1.0f; // 读取参考电阻失败
    }
    
    // 2. 测量PT100
    ADS1220_SetInputMux(config->input_p);
    Delay_ms(10); // 等待配置稳定
    
    adc_pt100 = PT100_ReadADCRaw();
    if (adc_pt100 == 0x7FFFFFFF) {
        return -1.0f; // 读取PT100失败
    }
    
    // 3. 计算PT100实际电阻 (比例法消除IDAC误差)
    // R_PT100 = R_ref × (ADC_PT100 / ADC_ref)
    resistance = config->ref_resistor * ((float)adc_pt100 / (float)adc_ref);
    
    return resistance;
}

/**
 * @brief  读取PT100电阻值
 * @param  config: PT100配置结构体指针
 * @retval 电阻值 (Ω)，错误返回-1.0f
 */
float PT100_ReadResistance(PT100_Config_t *config)
{
    if (config->use_ratiometric) {
        return PT100_ReadResistance_Ratiometric(config);
    } else {
        return PT100_ReadResistance_Absolute(config);
    }
}

/**
 * @brief  简化温度计算 (线性近似，适用于-50°C ~ 150°C)
 * @param  resistance: 电阻值 (Ω)
 * @param  type: PT100或PT1000类型
 * @retval 温度 (°C)
 * @note   使用线性公式:  T = (R - R0) / (R0 * α)
 */
static float PT100_SimpleTemperature(float resistance, PT100_Type_t type)
{
    float r0 = (type == PT100_TYPE) ? PT100_R0 :  PT1000_R0;
    float alpha = (type == PT100_TYPE) ? PT100_ALPHA : PT1000_ALPHA;
    
    return (resistance - r0) / (r0 * alpha);
}

/**
 * @brief  高精度温度计算 (Callendar-Van Dusen方程)
 * @param  resistance: 电阻值 (Ω)
 * @param  type: PT100或PT1000类型
 * @retval 温度 (°C)
 */
float PT100_ResistanceToTemperature(float resistance, PT100_Type_t type)
{
    float r0 = (type == PT100_TYPE) ? PT100_R0 :  PT1000_R0;
    float rt_r0 = resistance / r0;
    float temp;
    
    // 对于正温度 (0°C ~ 850°C):
    // Rt/R0 = 1 + A*t + B*t^2
    // 求解二次方程
    if (rt_r0 >= 1.0f) {
        // 使用二次公式:  t = (-A + sqrt(A^2 - 4*B*(1-Rt/R0))) / (2*B)
        float discriminant = CVD_A * CVD_A - 4.0f * CVD_B * (1.0f - rt_r0);
        
        if (discriminant >= 0) {
            temp = (-CVD_A + sqrtf(discriminant)) / (2.0f * CVD_B);
        } else {
            // 降级使用线性近似
            temp = PT100_SimpleTemperature(resistance, type);
        }
    }
    // 对于负温度 (-200°C ~ 0°C):
    else {
        // 使用线性近似（足够精确）
        temp = PT100_SimpleTemperature(resistance, type);
    }
    
    return temp;
}

/**
 * @brief  读取PT100温度
 * @param  config: PT100配置结构体指针
 * @retval 温度 (°C)，错误返回-999.0f
 */
float PT100_ReadTemperature(PT100_Config_t *config)
{
    float resistance;
    
    // 读取电阻
    resistance = PT100_ReadResistance(config);
    
    if (resistance < 0) {
        return -999.0f; // 错误标志
    }
    
    // 转换为温度
    return PT100_ResistanceToTemperature(resistance, config->type);
}

/**
 * @brief  校准PT100 (单点校准)
 * @param  config: PT100配置结构体指针
 * @param  known_temp: 已知温度 (°C)
 * @param  offset: 输出校准偏移量指针
 * @retval 无
 */
void PT100_Calibrate(PT100_Config_t *config, float known_temp, float *offset)
{
    float measured_temp;
    
    // 读取当前温度
    measured_temp = PT100_ReadTemperature(config);
    
    // 计算偏移
    *offset = known_temp - measured_temp;
}
