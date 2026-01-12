/**
 * @file    PT100.h
 * @brief   PT100/PT1000温度传感器测量库 - 头文件
 * @details 基于ADS1220实现高精度PT100/PT1000温度测量
 *          支持多种接线方式、温度转换等功能
 *          使用纯整数运算模式，适合无FPU的MCU如STM32F103
 * @version 2.0
 * @date    2026-01-12
 * @note    使用查表+线性插值实现温度转换，精度优于±0.1°C
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
    PT100_2WIRE = 0,            /**< 2线制: 仅使用IDAC1，导线电阻影响大 */
    PT100_3WIRE = 1,            /**< 3线制: IDAC1+IDAC2，可消除导线电阻影响 */
    PT100_4WIRE = 2,            /**< 4线制: 仅IDAC1，测量精度最高 */
    PT100_3WIRE_RATIOMETRIC = 3 /**< 3线制硬件比例测量: 使用外部参考电阻，精度最高 */
} PT100_WireMode_t;

/**
 * @brief PT100配置结构体
 * @details 所有参数使用整数表示，避免浮点运算
 */
typedef struct
{
    /* -------- PT100 本体 -------- */
    PT100_Type_t type;          /**< PT100 / PT1000 */
    PT100_WireMode_t wire_mode; /**< 2/3/4线，仅用于算法 */

    /* -------- ADC 输入 -------- */
    uint8_t mux;  /**< ADS1220_MUX_AINx_AINy */
    uint8_t gain; /**< PGA增益 */

    /* -------- IDAC 配置 -------- */
    PT100_IDAC_Current_t idac; /**< IDAC 电流大小 */
    uint8_t idac1_pin;         /**< IDAC1 路由引脚 */
    uint8_t idac2_pin;         /**< IDAC2 路由引脚（不用则 DISABLED） */

    /* -------- 参考与滤波（完全显式） -------- */
    uint8_t vref_sel; /**< ADS1220_VREF_INT / ADS1220_VREF_EXT_REF0 */
    uint8_t fir_mode; /**< ADS1220_FIR_xxx */

    /* -------- 参考配置 -------- */
    uint16_t vref_mv;   /**< 绝对测量用 */
    uint32_t rref_mohm; /**< 比例测量用 */

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

/* ====================================================================
 * 整数模式API - 适用于无FPU的MCU
 * 单位说明:
 *   - 电阻: mΩ（毫欧姆）
 *   - 温度: 0.01°C（百分之一摄氏度）
 * ==================================================================== */

/**
 * @brief  读取PT100电阻值
 * @param  config: PT100配置参数指针
 * @retval 电阻值(mΩ毫欧)，失败返回负值
 */
int32_t PT100_ReadResistance_Int(PT100_Config_t *config);

/**
 * @brief  读取PT100温度值
 * @param  config: PT100配置参数指针
 * @retval 温度值(0.01°C)，失败返回-99900
 * @note   内部调用PT100_ReadResistance_Int和PT100_ResistanceToTemperature_Int
 * @example 返回值2350表示23.50°C
 */
int32_t PT100_ReadTemperature_Int(PT100_Config_t *config);

/**
 * @brief  电阻值转换为温度值
 * @param  resistance_mohm: 电阻值(mΩ毫欧)
 * @param  type: PT100类型(PT100或PT1000)
 * @retval 温度值(0.01°C)
 * @note   使用查表+线性插值实现Callendar-Van Dusen转换
 *         精度优于±0.1°C（即±10个单位）
 * @example 输入100000mΩ(100Ω)，返回0（表示0.00°C）
 */
int32_t PT100_ResistanceToTemperature_Int(int32_t resistance_mohm, PT100_Type_t type);

/**
 * @brief  PT100校准
 * @param  config: PT100配置参数指针
 * @param  known_temp_centideg: 已知的标准温度(0.01°C)
 * @param  offset_centideg: 输出的温度偏移量(0.01°C)
 * @note   在已知温度环境下进行单点校准，计算偏移量
 * @retval 无
 */
void PT100_Calibrate_Int(PT100_Config_t *config, int32_t known_temp_centideg, int32_t *offset_centideg);

/**
 * @brief 便捷宏定义 - 提供简短的API名称
 * @note  - PT100_ReadResistance 返回 int32_t (mΩ)
 *        - PT100_ReadTemperature 返回 int32_t (0.01°C)
 *        - PT100_ResistanceToTemperature 参数和返回值均为 int32_t
 */
#define PT100_ReadResistance(cfg) PT100_ReadResistance_Int(cfg)
#define PT100_ReadTemperature(cfg) PT100_ReadTemperature_Int(cfg)
#define PT100_ResistanceToTemperature(r, t) PT100_ResistanceToTemperature_Int(r, t)

#endif /* __PT100_H */
