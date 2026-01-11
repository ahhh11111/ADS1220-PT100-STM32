/**
 * @file    PT100.h
 * @brief   PT100/PT1000温度传感器测量库 - 头文件
 * @details 基于ADS1220实现高精度PT100/PT1000温度测量
 *          支持多种接线方式、比例测量法、温度转换等功能
 *          支持浮点和整数两种计算模式，整数模式适合无FPU的MCU
 * @version 1.2
 * @date    2024-01-11
 * @note    使用Callendar-Van Dusen方程进行温度转换，精度高
 *          整数模式使用查表+线性插值实现，精度优于±0.1°C
 */

#ifndef __PT100_H
#define __PT100_H

/* ====================================================================
 * 头文件包含
 * ==================================================================== */
#include "ADS1220.h"

/* ====================================================================
 * 配置选项 - 浮点/整数模式选择
 * ==================================================================== */
/**
 * @brief 定义此宏以使用纯整数运算（适用于无FPU的MCU如STM32F103）
 *        取消注释下一行或在编译器中定义此宏
 */
#define PT100_USE_INTEGER_MATH

/* ====================================================================
 * PT100 类型定义
 * ==================================================================== */

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

#ifdef PT100_USE_INTEGER_MATH
/**
 * @brief PT100配置结构体（整数模式）
 * @details 所有参数使用整数表示，避免浮点运算
 */
typedef struct
{
    /* 基本配置 */
    PT100_Type_t type;         /**< PT100或PT1000类型 */
    PT100_IDAC_Current_t idac; /**< 激励电流大小 */
    uint8_t gain;              /**< PGA增益 (1, 2, 4, 8, 16, 32, 64, 128) */
    uint16_t vref_mv;          /**< 基准电压 (mV)，通常为2048mV */
    uint8_t input_p;           /**< 正输入通道(差分测量的正端) */
    uint8_t input_n;           /**< 负输入通道(差分测量的负端，保留) */

    /* 比例测量法配置 */
    uint8_t use_ratiometric;  /**< 是否使用比例测量法 */
    uint32_t ref_resistor_mohm; /**< 参考电阻阻值 (mΩ毫欧)，用于比例测量法 */
    uint8_t ref_channel;      /**< 参考电阻测量通道 */

    /* 接线模式配置 */
    PT100_WireMode_t wire_mode; /**< 接线模式: 2线制/3线制/4线制 */
    uint8_t idac2_pin;          /**< IDAC2输出引脚 (用于3线制导线补偿) */
} PT100_Config_t;

#else
/**
 * @brief PT100配置结构体（浮点模式）
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
#endif

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

#ifdef PT100_USE_INTEGER_MATH

/* ====================================================================
 * 整数模式API - 适用于无FPU的MCU
 * 单位说明:
 *   - 电阻: mΩ（毫欧姆）
 *   - 温度: 0.01°C（百分之一摄氏度）
 * ==================================================================== */

/**
 * @brief  读取PT100电阻值（整数模式）
 * @param  config: PT100配置参数指针
 * @retval 电阻值(mΩ毫欧)，失败返回负值
 * @note   根据use_ratiometric选择绝对测量或比例测量
 */
int32_t PT100_ReadResistance_Int(PT100_Config_t *config);

/**
 * @brief  读取PT100温度值（整数模式）
 * @param  config: PT100配置参数指针
 * @retval 温度值(0.01°C)，失败返回-99900
 * @note   内部调用PT100_ReadResistance_Int和PT100_ResistanceToTemperature_Int
 * @example 返回值2350表示23.50°C
 */
int32_t PT100_ReadTemperature_Int(PT100_Config_t *config);

/**
 * @brief  电阻值转换为温度值（整数模式）
 * @param  resistance_mohm: 电阻值(mΩ毫欧)
 * @param  type: PT100类型(PT100或PT1000)
 * @retval 温度值(0.01°C)
 * @note   使用查表+线性插值实现Callendar-Van Dusen转换
 *         精度优于±0.1°C（即±10个单位）
 * @example 输入100000mΩ(100Ω)，返回0（表示0.00°C）
 */
int32_t PT100_ResistanceToTemperature_Int(int32_t resistance_mohm, PT100_Type_t type);

/**
 * @brief  PT100校准（整数模式）
 * @param  config: PT100配置参数指针
 * @param  known_temp_centideg: 已知的标准温度(0.01°C)
 * @param  offset_centideg: 输出的温度偏移量(0.01°C)
 * @note   在已知温度环境下进行单点校准，计算偏移量
 * @retval 无
 */
void PT100_Calibrate_Int(PT100_Config_t *config, int32_t known_temp_centideg, int32_t *offset_centideg);

/* 兼容性宏定义 - 将旧API映射到新API */
#define PT100_ReadResistance(cfg)       PT100_ReadResistance_Int(cfg)
#define PT100_ReadTemperature(cfg)      PT100_ReadTemperature_Int(cfg)
#define PT100_ResistanceToTemperature(r, t)  PT100_ResistanceToTemperature_Int(r, t)

#else

/* ====================================================================
 * 浮点模式API - 适用于有FPU的MCU
 * ==================================================================== */

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

#endif /* PT100_USE_INTEGER_MATH */

#endif /* __PT100_H */
