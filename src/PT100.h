#ifndef __PT100_H
#define __PT100_H

#include "ADS1220.h"

/* ====================================================================
 * PT100 配置参数
 * ==================================================================== */
// IDAC激励电流选择 (单位:  μA)
typedef enum {
    PT100_IDAC_250UA  = ADS1220_IDAC_250UA,   // 250μA (推荐用于PT100)
    PT100_IDAC_500UA  = ADS1220_IDAC_500UA,   // 500μA
    PT100_IDAC_1000UA = ADS1220_IDAC_1000UA   // 1000μA (大电流，自热效应)
} PT100_IDAC_Current_t;

// PT100类型
typedef enum {
    PT100_TYPE = 0,     // PT100: 0°C时100Ω
    PT1000_TYPE = 1     // PT1000: 0°C时1000Ω
} PT100_Type_t;

// PT100配置结构体
typedef struct {
    PT100_Type_t type;              // PT100或PT1000
    PT100_IDAC_Current_t idac;      // 激励电流
    uint8_t gain;                   // 增益 (1, 2, 4, 8, 16, 32, 64, 128)
    float vref;                     // 参考电压 (V)
    uint8_t input_p;                // 正输入通道
    uint8_t input_n;                // 负输入通道
    // 比例测量法配置
    bool use_ratiometric;           // 是否使用比例测量法
    float ref_resistor;             // 参考电阻阻值 (Ω)，用于比例测量法
    uint8_t ref_channel;            // 参考电阻测量通道
} PT100_Config_t;

/* ====================================================================
 * 函数声明
 * ==================================================================== */
void PT100_Init(PT100_Config_t *config);
float PT100_ReadResistance(PT100_Config_t *config);
float PT100_ReadTemperature(PT100_Config_t *config);
float PT100_ResistanceToTemperature(float resistance, PT100_Type_t type);
void PT100_Calibrate(PT100_Config_t *config, float known_temp, float *offset);

#endif /* __PT100_H */
