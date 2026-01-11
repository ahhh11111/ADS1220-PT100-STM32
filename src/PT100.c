#include "PT100.h"
#include <math.h>

#define PT100_R0            100.0f
#define PT100_ALPHA         0.00385f
#define PT1000_R0           1000.0f
#define PT1000_ALPHA        0.00385f
#define CVD_A               3.9083e-3f
#define CVD_B               -5.775e-7f

/* IDAC 电流查找表 */
static float PT100_GetIDACCurrent(uint8_t idac_setting) {
    switch(idac_setting & 0x07) {
        case 0x01: return 10.0f;
        case 0x02: return 50.0f;
        case 0x03: return 100.0f;
        case 0x04: return 250.0f;
        case 0x05: return 500.0f;
        case 0x06: return 1000.0f;
        case 0x07: return 1500.0f;
        default:   return 0.0f;
    }
}

// 辅助路由函数 (保持原样)
static uint8_t PT100_GetIDAC1Routing(uint8_t mux_setting) {
    // 简化：通常IDAC1路由到Positive Input
    if ((mux_setting & 0xF0) <= 0x20) return ADS1220_I1MUX_AIN0; // 0x00, 0x10, 0x20
    return ADS1220_I1MUX_AIN0; // Default fallback
}

/* 初始化函数保持大致不变，但在Ratiometric模式下添加注释 */
void PT100_Init(PT100_Config_t *config) {
    ADS1220_Config_t ads_config;
    uint8_t gain_reg = ADS1220_GAIN_8; // Default

    // Gain Mapping... (omitted)
    switch(config->gain) {
        case 1: gain_reg = ADS1220_GAIN_1; break;
        // ...
        case 128: gain_reg = ADS1220_GAIN_128; break;
        default: gain_reg = ADS1220_GAIN_8; config->gain = 8; break;
    }

    ads_config.reg0 = config->input_p | gain_reg | ADS1220_PGA_ENABLED;
    ads_config.reg1 = ADS1220_DR_20SPS | ADS1220_MODE_NORMAL | ADS1220_CM_SINGLE;
    
    // VREF配置
    if (config->vref >= 2.0f && config->vref <= 2.1f) {
        ads_config.reg2 = ADS1220_VREF_INT | ADS1220_FIR_50HZ_60HZ | config->idac;
    } else {
        ads_config.reg2 = ADS1220_VREF_EXT_REF0 | ADS1220_FIR_50HZ_60HZ | config->idac;
    }
    
    // IDAC Routing
    uint8_t idac1 = PT100_GetIDAC1Routing(config->input_p);
    uint8_t idac2 = ADS1220_I2MUX_DISABLED;
    
    if (config->wire_mode == PT100_3WIRE) {
        idac2 = config->idac2_pin;
    }
    // 注意：如果是“软件比例测量”，这里不需要启用IDAC2用于测量参考电阻
    // 除非电路需要IDAC流经参考电阻（通常是的）
    
    ads_config.reg3 = idac1 | idac2;
    
    ADS1220_WriteConfig(&ads_config);
    Delay_ms(50);
}

static int32_t PT100_ReadADCRaw(void) {
    ADS1220_ClearError();
    ADS1220_StartSync();
    if (!ADS1220_WaitForData(200)) return 0x7FFFFFFF; // 20SPS ~50ms
    return ADS1220_ReadData();
}

static float PT100_ReadResistance_Absolute(PT100_Config_t *config) {
    int32_t raw = PT100_ReadADCRaw();
    if (raw == 0x7FFFFFFF) return -1.0f;
    
    float voltage = ((float)raw / 8388608.0f) * (config->vref / (float)config->gain);
    float current = PT100_GetIDACCurrent(config->idac);
    if (current == 0.0f) return -1.0f;
    
    return (voltage / current) * 1000000.0f;
}

/**
 * @brief  软件实现的比例测量法 (伪比例)
 * @note   注意：这不是ADC硬件比例测量(VREF=REFP0)。
 * 这里是分别测量PT100电压和参考电阻电压，然后计算。
 * 虽然能消除电流绝对误差，但受转换间隔期间的噪声影响。
 */
static float PT100_ReadResistance_Ratiometric(PT100_Config_t *config) {
    if (config->ref_resistor <= 0.0f) return -1.0f;

    // 1. 测量参考电阻 (假设电流流经参考电阻)
    ADS1220_SetInputMux(config->ref_channel);
    Delay_ms(10); // 等待滤波器稳定 (重要!)
    
    int32_t adc_ref = PT100_ReadADCRaw();
    if (adc_ref == 0x7FFFFFFF || adc_ref == 0) return -1.0f; // 避免除0
    
    // 2. 测量 PT100
    ADS1220_SetInputMux(config->input_p);
    Delay_ms(10);
    
    int32_t adc_pt100 = PT100_ReadADCRaw();
    if (adc_pt100 == 0x7FFFFFFF) return -1.0f;
    
    // 计算: R_pt100 = R_ref * (Code_pt100 / Code_ref)
    // 假设增益相同，VREF相同，IDAC相同且稳定
    return config->ref_resistor * ((float)adc_pt100 / (float)adc_ref);
}

float PT100_ReadResistance(PT100_Config_t *config) {
    if (config->use_ratiometric) return PT100_ReadResistance_Ratiometric(config);
    else return PT100_ReadResistance_Absolute(config);
}

float PT100_ResistanceToTemperature(float resistance, PT100_Type_t type) {
    // 保持原算法，正确
    float r0 = (type == PT100_TYPE) ? PT100_R0 : PT1000_R0;
    float rt_r0 = resistance / r0;
    
    if (rt_r0 >= 1.0f) {
        float discriminant = CVD_A * CVD_A - 4.0f * CVD_B * (1.0f - rt_r0);
        if (discriminant >= 0) return (-CVD_A + sqrtf(discriminant)) / (2.0f * CVD_B);
    }
    // 简单的线性回退或负温处理
    return (resistance - r0) / (r0 * (type==PT100_TYPE ? PT100_ALPHA : PT1000_ALPHA));
}

float PT100_ReadTemperature(PT100_Config_t *config) {
    float r = PT100_ReadResistance(config);
    if (r < 0) return -999.0f;
    return PT100_ResistanceToTemperature(r, config->type);
}