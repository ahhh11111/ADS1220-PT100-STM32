/**
 * @file    PT100.h
 * @brief   PT100/PT1000温度传感器测量库 - 头文件
 * @details 基于ADS1220实现高精度PT100/PT1000温度测量
 *          支持多种接线方式、比例测量法、温度转换等功能
 * @version 1.1
 * @date    2024-01-11
 * @note    使用Callendar-Van Dusen方程进行温度转换，精度高
 */

#ifndef __PT100_H
#define __PT100_H

/* ====================================================================
 * 头文件包含
 * ==================================================================== */
#include "ADS1220.h"

/* ====================================================================
 * PT100 类型定义
 * ==================================================================== */

/**
 * @brief IDAC激励电流选择枚举
 * @note  PT100推荐使用250μA，较大电流会引起自热效应
 */
/**
 * @brief IDAC激励电流选择枚举
 * @note  PT100推荐使用250μA，较大电流会引起自热效应
 */
typedef enum
{
    PT100_IDAC_250UA = ADS1220_IDAC_250UA,  /**< 250μA (推荐用于PT100) */
    PT100_IDAC_500UA = ADS1220_IDAC_500UA,  /**< 500μA */
    PT100_IDAC_1000UA = ADS1220_IDAC_1000UA /**< 1000μA (大电流，注意自热效应) */
} PT100_IDAC_Current_t;

/**
 * @brief PT100传感器类型枚举
 */
typedef enum
{
    PT100_TYPE = 0, /**< PT100: 0°C时阻值为100Ω */
    PT1000_TYPE = 1 /**< PT1000: 0°C时阻值为1000Ω */
} PT100_Type_t;

/**
 * @brief PT100接线模式枚举
 */
typedef enum
{
    PT100_2WIRE = 0, /**< 2线制: 仅使用IDAC1，导线电阻影响大 */
    PT100_3WIRE = 1, /**< 3线制: IDAC1+IDAC2，可消除导线电阻影响 */
    PT100_4WIRE = 2  /**< 4线制: 仅IDAC1，测量精度最高 */
} PT100_WireMode_t;

/**
 * @brief PT100配置结构体
 * @details 包含PT100测量的所有配置参数
 */
typedef struct
{
    /* 基本配置 */
    PT100_Type_t type;         /**< PT100或PT1000类型 */
    PT100_IDAC_Current_t idac; /**< 激励电流大小 */
    uint8_t gain;              /**< PGA增益 (1, 2, 4, 8, 16, 32, 64, 128) */
    float vref;                /**< 基准电压 (V)，通常为2.048V */
    uint8_t input_p;           /**< 正输入通道(差分测量的正端) */
    uint8_t input_n;           /**< 负输入通道(差分测量的负端，保留) */

    /* 比例测量法配置 */
    uint8_t use_ratiometric; /**< 是否使用比例测量法 */
    float ref_resistor;      /**< 参考电阻阻值 (Ω)，用于比例测量法 */
    uint8_t ref_channel;     /**< 参考电阻测量通道 */

    /* 接线模式配置 */
    PT100_WireMode_t wire_mode; /**< 接线模式: 2线制/3线制/4线制 */
    uint8_t idac2_pin;          /**< IDAC2输出引脚 (用于3线制导线补偿) */
} PT100_Config_t;

/* ====================================================================
 * API函数声明
 * ==================================================================== */

/**
 * @brief  初始化PT100测量
 * @param  config: PT100配置参数指针
 * @note   根据配置参数设置ADS1220寄存器和IDAC路由
 * @retval 无
 */
void PT100_Init(PT100_Config_t *config);

/**
 * @brief  读取PT100电阻值
 * @param  config: PT100配置参数指针
 * @retval 电阻值(Ω)，失败返回负值
 * @note   根据use_ratiometric选择绝对测量或比例测量
 */
float PT100_ReadResistance(PT100_Config_t *config);

/**
 * @brief  读取PT100温度值
 * @param  config: PT100配置参数指针
 * @retval 温度值(°C)，失败返回-999.0
 * @note   内部调用PT100_ReadResistance和PT100_ResistanceToTemperature
 */
float PT100_ReadTemperature(PT100_Config_t *config);

/**
 * @brief  电阻值转换为温度值
 * @param  resistance: 电阻值(Ω)
 * @param  type: PT100类型(PT100或PT1000)
 * @retval 温度值(°C)
 * @note   使用Callendar-Van Dusen方程进行高精度转换
 *         正温区: T = (-A + sqrt(A^2 - 4B(1-Rt/R0))) / (2B)
 *         负温区: 使用线性近似
 */
float PT100_ResistanceToTemperature(float resistance, PT100_Type_t type);

/**
 * @brief  PT100校准
 * @param  config: PT100配置参数指针
 * @param  known_temp: 已知的标准温度(°C)
 * @param  offset: 输出的温度偏移量(°C)
 * @note   在已知温度环境下进行单点校准，计算偏移量
 * @retval 无
 */
void PT100_Calibrate(PT100_Config_t *config, float known_temp, float *offset);

#endif /* __PT100_H */
