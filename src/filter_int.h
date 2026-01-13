/**
 * @file    filter_int.h
 * @brief   数字滤波器库 - 包含 5 种常用滤波算法
 *
 * 功能模块：
 *  1. AvgFilter_Int     - 简单平均滤波器（整数版本）
 *  2. MovAvgFilter_Int  - 滑动平均滤波器（整数版本）
 *  3. IIR1_LPF_Int      - 一阶 IIR 低通滤波器（整数版本）
 *
 * 定点数格式说明：
 *  - Q16:   16.16 定点数，范围 ±32768，精度 1/65536 ≈ 0.000015
 *  - Q15: 1.15 定点数，范围 ±1. 0，精度 1/32768 ≈ 0.00003
 *
 * 性能特点：
 *  - 纯整数运算，无浮点操作
 *  - 支持位移优化模式和直接除法模式
 *  - 适用于 Cortex-M0/M0+/M3 等无 FPU 的 MCU
 *  - 速度提升 5-10 倍（相比软件浮点）
 *
 * 模式选择：
 *  - 定义 USE_SHIFT_DIV：使用位移替代除法（需要 N 为 2 的幂）- 最快
 *  - 不定义：使用标准除法（支持任意 N）- 灵活但较慢
 *
 * @author  STM32 Embedded
 * @date    2026-01-13
 */

#ifndef FILTER_INT_H_
#define FILTER_INT_H_

#include <stdint. h>
#include <string.h>

/* ===================== 配置选项 ===================== */

/**
 * @brief  除法优化模式选择
 * 
 * 选项 1：定义 USE_SHIFT_DIV（推荐）
 *  - 使用位移替代除法：avg = sum >> log2(N)
 *  - 性能：1 周期
 *  - 约束：N 必须是 2 的幂（2, 4, 8, 16, 32, 64, ... ）
 *  - 适用：性能敏感应用
 * 
 * 选项 2：不定义 USE_SHIFT_DIV
 *  - 使用标准除法：avg = sum / N
 *  - 性能：~20 周期
 *  - 约束：N 可以是任意值
 *  - 适用：需要灵活窗口大小的应用
 * 
 * 使用方法：
 *  在工程中定义宏（推荐在编译选项中添加）：
 *  - Keil:  Options -> C/C++ -> Preprocessor Symbols 添加 USE_SHIFT_DIV
 *  - GCC:   CFLAGS += -DUSE_SHIFT_DIV
 *  - 或在本文件开头定义：#define USE_SHIFT_DIV
 */
// #define USE_SHIFT_DIV  /* 取消注释以启用位移优化 */

/* ===================== 定点数格式定义 ===================== */

/**
 * Q16 格式：16.16 定点数
 * - 范围:  -32768.0 ~ +32767.99998
 * - 精度:   1/65536 ≈ 0.000015
 * - 使用 int32_t 存储
 */
#define Q16_SHIFT       (16)                    /*!< Q16 格式小数位数 */
#define Q16_ONE         (1 << Q16_SHIFT)        /*!< Q16 格式的 1.0 = 65536 */
#define FLOAT_TO_Q16(x) ((int32_t)((x) * Q16_ONE + ((x) >= 0 ? 0.5f : -0.5f)))
#define Q16_TO_FLOAT(x) ((float)(x) / Q16_ONE)

/**
 * Q15 格式：1.15 定点数（符号位 + 15 位小数）
 * - 范围: -1.0 ~ +0.999969
 * - 精度:   1/32768 ≈ 0.00003
 * - 使用 int16_t 存储（节省内存）
 */
#define Q15_SHIFT       (15)                    /*!< Q15 格式小数位数 */
#define Q15_ONE         (1 << Q15_SHIFT)        /*!< Q15 格式的 1.0 = 32768 */
#define FLOAT_TO_Q15(x) ((int16_t)((x) * Q15_ONE + ((x) >= 0 ? 0.5f : -0.5f)))
#define Q15_TO_FLOAT(x) ((float)(x) / Q15_ONE)

/**
 * Q15 乘法（带溢出保护）
 * @note (a * b) >> 15，使用 int32_t 中间变量防止溢出
 */
static inline int16_t Q15_MUL(int16_t a, int16_t b)
{
    return (int16_t)(((int32_t)a * (int32_t)b) >> Q15_SHIFT);
}

/* ===================== 辅助函数：计算 log2(N) ===================== */

#ifdef USE_SHIFT_DIV
/**
 * @brief  计算整数 N 的 log2 值（仅支持 2 的幂次方）
 * @param  N 输入值（必须是 2 的幂：2, 4, 8, 16, 32, ...  ）
 * @return log2(N)，如果 N 不是 2 的幂则返回 0
 * @note   用于确定移位量：avg = sum >> log2(N)
 */
static inline uint8_t calc_log2(uint16_t N)
{
    uint8_t shift = 0;
    if (N == 0 || (N & (N - 1)) != 0)
        return 0; /* N 不是 2 的幂，返回 0 */
    
    while (N > 1)
    {
        N >>= 1;
        shift++;
    }
    return shift;
}

/**
 * @brief  检查 N 是否为 2 的幂
 * @param  N 输入值
 * @return 1 - 是 2 的幂；0 - 不是
 */
static inline uint8_t is_power_of_2(uint16_t N)
{
    return (N > 0) && ((N & (N - 1)) == 0);
}
#endif

/* ===================== 除法宏定义（兼容模式） ===================== */

#ifdef USE_SHIFT_DIV
    /* 位移模式：sum >> shift */
    #define INT_DIV(sum, N, shift)  ((sum) >> (shift))
    #define NEED_SHIFT_PARAM        1  /* 结构体需要 shift 成员 */
#else
    /* 直接除法模式：sum / N */
    #define INT_DIV(sum, N, shift)  ((sum) / (N))
    #define NEED_SHIFT_PARAM        0  /* 结构体不需要 shift 成员 */
#endif

/* ===================== 1.  简单平均滤波器（整数版本） =====================
 *
 * 功能：累积 N 个样本后输出一个平均值，然后重新开始
 * 
 * USE_SHIFT_DIV 模式：
 *  - 约束：N 必须是 2 的幂（2, 4, 8, 16, 32, ... ）
 *  - 优化：sum / N = sum >> log2(N)
 *  - 性能：1 周期
 * 
 * 直接除法模式：
 *  - 约束：N 可以是任意值（1-65535）
 *  - 计算：sum / N
 *  - 性能：~20 周期
 *
 * 使用示例：
 *  AvgFilter_Int_t f;
 *  AvgFilter_Int_Init(&f, 16);  // USE_SHIFT_DIV:  N=16(2^4); 否则：N=任意值
 *  int16_t out;
 *  if (AvgFilter_Int_Put(&f, adc_value, &out))
 *      printf("Average: %d\n", out);
 */

typedef struct
{
    int32_t sum;      /*!< 累加和（使用 32 位防止溢出） */
    uint16_t count;   /*! < 已累加次数 */
    uint16_t N;       /*!< 窗口大小 */
#if NEED_SHIFT_PARAM
    uint8_t shift;    /*! < log2(N)，用于右移代替除法（仅 USE_SHIFT_DIV 模式） */
#endif
} AvgFilter_Int_t;

/**
 * @brief  初始化简单平均滤波器（整数版本）
 * @param  f 滤波器结构体指针
 * @param  N 平均窗口大小
 *           - USE_SHIFT_DIV 模式：必须是 2 的幂（2, 4, 8, 16, 32, ...）
 *           - 直接除法模式：可以是任意值（1-65535）
 * @note   USE_SHIFT_DIV 模式下，如果 N 不是 2 的幂，会自动调整为 16
 */
void AvgFilter_Int_Init(AvgFilter_Int_t *f, uint16_t N);

/**
 * @brief  输入一个样本，累积 N 个样本后输出平均值
 * @param  f   滤波器结构体指针
 * @param  x   输入样本值（int16_t，如 ADC 原始值）
 * @param  out 输出均值指针（仅当返回 1 时有效）
 * @return 0 - 样本未满，继续累积；1 - 输出已准备好
 */
uint8_t AvgFilter_Int_Put(AvgFilter_Int_t *f, int16_t x, int16_t *out);

/* ===================== 2. 滑动平均滤波器（整数版本） =====================
 *
 * 功能：维持一个滑动窗口，每次输入一个新样本时立即输出当前窗口的平均值
 * 
 * USE_SHIFT_DIV 模式：
 *  - 约束：N 必须是 2 的幂（2, 4, 8, 16, 32, ...）
 *  - 优化：sum / N = sum >> log2(N)
 * 
 * 直接除法模式：
 *  - 约束：N 可以是任意值
 *
 * 使用示例：
 *  int16_t buf[32];  // 缓冲区必须由外部分配
 *  MovAvgFilter_Int_t f;
 *  MovAvgFilter_Int_Init(&f, buf, 32);
 *  int16_t avg = MovAvgFilter_Int_Put(&f, sensor_value);
 */

typedef struct
{
    int16_t *buf;     /*!< 外部分配的数据缓冲区指针（大小 N*sizeof(int16_t)） */
    int32_t sum;      /*! < 窗口内数据的累加和（32 位防止溢出） */
    uint16_t N;       /*!< 窗口大小 */
    uint16_t index;   /*!< 当前写入位置（0 到 N-1，循环） */
#if NEED_SHIFT_PARAM
    uint8_t shift;    /*!< log2(N)，用于右移代替除法（仅 USE_SHIFT_DIV 模式） */
#endif
} MovAvgFilter_Int_t;

/**
 * @brief  初始化滑动平均滤波器（整数版本）
 * @param  f   滤波器结构体指针
 * @param  buf 外部分配的缓冲区指针（存储 N 个 int16_t）
 * @param  N   滑动窗口大小
 *             - USE_SHIFT_DIV 模式：必须是 2 的幂（2, 4, 8, 16, 32, ...）
 *             - 直接除法模式：可以是任意值
 * @note   缓冲区由调用者分配和管理，不能释放
 */
void MovAvgFilter_Int_Init(MovAvgFilter_Int_t *f, int16_t *buf, uint16_t N);

/**
 * @brief  输入一个样本，返回 N 个样本的滑动平均值
 * @param  f 滤波器结构体指针
 * @param  x 输入样本值（int16_t）
 * @return 当前窗口内的平均值（int16_t）
 * @note   每次调用返回新的平均值，实现无延迟的流处理
 */
int16_t MovAvgFilter_Int_Put(MovAvgFilter_Int_t *f, int16_t x);

/* ===================== 3. 一阶 IIR 低��滤波器（整数版本） =====================
 *
 * 公式：y = y_prev + alpha * (x - y_prev)
 * 定点实现：使用 Q15 格式表示 alpha（0.0 到 1.0）
 *
 * Alpha 选择指南（Q15 格式）：
 *  - alpha = 655   (0.02)  - 极强平滑
 *  - alpha = 3277  (0.1)   - 强平滑（推荐）
 *  - alpha = 6554  (0.2)   - 中等平滑
 *  - alpha = 16384 (0.5)   - 中等响应
 *  - alpha = 26214 (0.8)   - 快速响应
 *
 * 使用示例：
 *  IIR1_LPF_Int_t f;
 *  IIR1_LPF_Int_Init(&f, 3277);  // alpha = 0.1
 *  int16_t filtered = IIR1_LPF_Int_Put(&f, adc_value);
 */

typedef struct
{
    int16_t y_prev;    /*!< 上一次的输出值（Q16 或原始格式） */
    int16_t alpha;     /*!< 平滑系数（Q15 格式：0 到 32767 对应 0.0 到 1.0） */
    uint8_t first_run; /*!< 首次运行标志 */
} IIR1_LPF_Int_t;

/**
 * @brief  初始化一阶 IIR 低通滤波器（整数版本）
 * @param  f     滤波器结构体指针
 * @param  alpha 平滑系数（Q15 格式：0-32767 对应 0.0-1.0）
 * @note   推荐值：3277 (0.1), 6554 (0.2), 16384 (0.5)
 */
void IIR1_LPF_Int_Init(IIR1_LPF_Int_t *f, int16_t alpha);

/**
 * @brief  使用浮点 alpha 初始化（自动转换为 Q15）
 * @param  f     滤波器结构体指针
 * @param  alpha 平滑系数（浮点：0.0-1.0）
 */
void IIR1_LPF_Int_Init_Float(IIR1_LPF_Int_t *f, float alpha);

/**
 * @brief  使用截止频率初始化一阶 IIR 低通滤波器
 * @param  f  滤波器结构体指针
 * @param  fs 采样频率（Hz）
 * @param  fc 截止频率（Hz）
 * @note   自动计算 alpha = (2πfc/fs) / (1 + 2πfc/fs)
 */
void IIR1_LPF_Int_Init_Fc(IIR1_LPF_Int_t *f, uint16_t fs, uint16_t fc);

/**
 * @brief  输入一个样本，返回低通滤波后的值
 * @param  f 滤波器结构体指针
 * @param  x 输入样本值（int16_t）
 * @return 滤波后的输出值（int16_t）
 * @note   y = y_prev + alpha * (x - y_prev)
 */
int16_t IIR1_LPF_Int_Put(IIR1_LPF_Int_t *f, int16_t x);

/* ===================== 辅助宏：快速 Alpha 常量定义 ===================== */

/* 预定义常用 alpha 值（Q15 格式） */
#define ALPHA_Q15_0_01  (328)    /*!< 0.01 - 极强平滑 */
#define ALPHA_Q15_0_02  (655)    /*!< 0.02 */
#define ALPHA_Q15_0_05  (1638)   /*!< 0.05 */
#define ALPHA_Q15_0_1   (3277)   /*!< 0.1 - 推荐值 */
#define ALPHA_Q15_0_2   (6554)   /*!< 0.2 */
#define ALPHA_Q15_0_3   (9830)   /*!< 0.3 */
#define ALPHA_Q15_0_5   (16384)  /*!< 0.5 */
#define ALPHA_Q15_0_8   (26214)  /*!< 0.8 */

/* ===================== 编译时配置信息输出 ===================== */

#ifdef USE_SHIFT_DIV
    #pragma message("Filter Mode:  SHIFT DIVISION (Optimized, N must be power of 2)")
#else
    #pragma message("Filter Mode:  DIRECT DIVISION (Flexible, N can be any value)")
#endif

#endif /* FILTER_INT_H_ */