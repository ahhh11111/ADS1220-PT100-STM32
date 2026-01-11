/**
 * @file    Delay.h
 * @brief   延时函数库 - 头文件
 * @details 提供三种延时实现方式，通过宏定义选择:
 *          - ADS1220_DELAY_SYSTICK: SysTick定时器精确延时(推荐，精度±1μs)
 *          - ADS1220_DELAY_SIMPLE: 简单循环延时(精度±10μs)
 *          - ADS1220_DELAY_EXTERNAL: 外部自定义延时函数
 * @version 1.0
 * @date    2024-01-11
 * @note    适配STM32F103系列(标准外设库SPL)，可移植到其他平台
 */

#ifndef __DELAY_H
#define __DELAY_H

/* ====================================================================
 * 头文件包含
 * ==================================================================== */
#include "stm32f10x.h"
#include <stdint.h>

/* ====================================================================
 * 延时模式选择
 * 说明：三选一，推荐使用SysTick模式获得最佳精度
 * ==================================================================== */
#ifndef ADS1220_DELAY_SYSTICK
#ifndef ADS1220_DELAY_SIMPLE
#ifndef ADS1220_DELAY_EXTERNAL
    #define ADS1220_DELAY_SYSTICK           /**< 默认使用SysTick定时器延时 */
#endif
#endif
#endif

/* ====================================================================
 * 延时函数接口声明
 * ==================================================================== */

/**
 * @brief  微秒级延时
 * @param  us: 延时时间(微秒)
 * @retval 无
 */
void Delay_us(uint32_t us);

/**
 * @brief  毫秒级延时
 * @param  ms: 延时时间(毫秒)
 * @retval 无
 */
void Delay_ms(uint32_t ms);

/* 以下函数仅在SysTick模式下可用 */
#if defined(ADS1220_DELAY_SYSTICK)

/**
 * @brief  SysTick定时器初始化
 * @note   配置SysTick为1ms中断
 * @param  无
 * @retval 无
 */
void SysTick_Init(void);

/**
 * @brief  获取微秒时间戳
 * @note   基于SysTick计数器计算
 * @param  无
 * @retval 当前微秒时间戳
 */
uint32_t GetMicros(void);

/**
 * @brief  获取毫秒时间戳
 * @note   基于SysTick中断计数
 * @param  无
 * @retval 当前毫秒时间戳
 */
uint32_t GetMillis(void);

#endif /* ADS1220_DELAY_SYSTICK */

/* 以下函数在外部延时模式下需要用户实现 */
#if defined(ADS1220_DELAY_EXTERNAL)

/**
 * @brief  外部延时初始化函数(用户实现)
 * @note   在外部延时模式下，用户需要实现此函数
 * @param  无
 * @retval 无
 */
void Delay_Init(void);

/**
 * @brief  外部获取毫秒时间戳函数(用户实现)
 * @note   在外部延时模式下，用户需要实现此函数
 * @param  无
 * @retval 当前毫秒时间戳
 */
uint32_t GetMillis(void);

#endif /* ADS1220_DELAY_EXTERNAL */

#endif /* __DELAY_H */
