#include "filter_int.h"
#include <math.h>  /* 仅用于初始化时的 alpha 计算 */

/*================================================================================================*/
/* 1. 简单平均滤波器（整数版本）                                                                  */
/*================================================================================================*/

/**
 * @brief  初始化简单平均滤波器（整数版本）
 * @param  f    滤波器结构体指针
 * @param  N    平均窗口大小
 */
void AvgFilter_Int_Init(AvgFilter_Int_t *f, uint16_t N)
{
#ifdef USE_SHIFT_DIV
    /* 位移模式：检查 N 是否为 2 的幂 */
    uint8_t shift = calc_log2(N);
    if (shift == 0)
    {
        /* N 不是 2 的幂，默认设置为 16 */
        N = 16;
        shift = 4;
    }
    f->shift = shift;
#else
    /* 直接除法模式：N 可以是任意值 */
    if (N == 0)
        N = 1;  /* 防止除零 */
#endif
    
    f->sum = 0;
    f->count = 0;
    f->N = N;
}

/**
 * @brief  输入一个样本，累积N个样本后输出平均值
 * @param  f    滤波器结构体指针
 * @param  x    输入样本值（int16_t）
 * @param  out  输出均值指针（仅当返回1时有效）
 * @return 0    样本未满，继续累积；1 - 输出已准备好
 */
uint8_t AvgFilter_Int_Put(AvgFilter_Int_t *f, int16_t x, int16_t *out)
{
    f->sum += x; /* 累加新样本 */
    if (++f->count < f->N)
        return 0; /* 样本数未达到N，继续累积 */

#ifdef USE_SHIFT_DIV
    /* 位移模式：sum >> shift */
    *out = (int16_t)(f->sum >> f->shift);
#else
    /* 直接除法模式：sum / N */
    *out = (int16_t)(f->sum / f->N);
#endif
    
    /* 重置状态 */
    f->sum = 0;
    f->count = 0;
    return 1; /* 输出有效 */
}

/*================================================================================================*/
/* 2. 滑动平均滤波器（整数版本）                                                                  */
/*================================================================================================*/

/**
 * @brief  初始化滑动平均滤波器（整数版本）
 * @param  f    滤波器结构体指针
 * @param  buf  外部分配的缓冲区指针（存储N个int16_t）
 * @param  N    滑动窗口大小
 * @note   缓冲区由调用者分配和管理，不能释放
 */
void MovAvgFilter_Int_Init(MovAvgFilter_Int_t *f, int16_t *buf, uint16_t N)
{
#ifdef USE_SHIFT_DIV
    /* 位移模式：检查 N 是否为 2 的幂 */
    uint8_t shift = calc_log2(N);
    if (shift == 0)
    {
        /* N 不是 2 的幂，默认设置为 16 */
        N = 16;
        shift = 4;
    }
    f->shift = shift;
#else
    /* 直接除法模式：N 可以是任意值 */
    if (N == 0)
        N = 1;  /* 防止除零 */
#endif
    
    memset(buf, 0, N * sizeof(int16_t)); /* 清空缓冲区 */
    f->buf = buf;
    f->sum = 0;
    f->N = N;
    f->index = 0;
}

/**
 * @brief  输入一个样本，返回N个样本的滑动平均值
 * @param  f 滤波器结构体指针
 * @param  x 输入样本值（int16_t）
 * @return 当前窗口内的平均值（int16_t）
 */
int16_t MovAvgFilter_Int_Put(MovAvgFilter_Int_t *f, int16_t x)
{
    uint16_t idx = f->index;
    
    /* 更新窗口和：加入新样本，移除被挤出的旧样本 */
    f->sum += x - f->buf[idx];
    f->buf[idx] = x; /* 存储新样本 */

    /* 环形缓冲区索引递进 */
    if (++idx >= f->N)
        idx = 0;
    f->index = idx;

#ifdef USE_SHIFT_DIV
    /* 位移模式：sum >> shift */
    return (int16_t)(f->sum >> f->shift);
#else
    /* 直接除法模式：sum / N */
    return (int16_t)(f->sum / f->N);
#endif
}

/*================================================================================================*/
/* 3. 一阶 IIR 低通滤波器（整数版本）                                                             */
/*================================================================================================*/

/**
 * @brief  初始化一阶 IIR 低通滤波器（整数版本）
 * @param  f     滤波器结构体指针
 * @param  alpha 平滑系数（Q15 格式：0-32767 对应 0.0-1.0）
 */
void IIR1_LPF_Int_Init(IIR1_LPF_Int_t *f, int16_t alpha)
{
    /* 限制 alpha 在有效范围内（0 到 Q15_MAX） */
    if (alpha > Q15_MAX)
        alpha = Q15_MAX;
    if (alpha < 0)
        alpha = 0;
    
    f->alpha = alpha;
    f->first_run = 1; /* 标记首次运行 */
}

/**
 * @brief  使用浮点 alpha 初始化（自动转换为 Q15）
 * @param  f     滤波器结构体指针
 * @param  alpha 平滑系数（浮点：0.0-1.0）
 */
void IIR1_LPF_Int_Init_Float(IIR1_LPF_Int_t *f, float alpha)
{
    /* 限制 alpha 范围 */
    if (alpha > 1.0f)
        alpha = 1.0f;
    if (alpha < 0.0f)
        alpha = 0.0f;
    
    int16_t alpha_q15 = FLOAT_TO_Q15(alpha);
    IIR1_LPF_Int_Init(f, alpha_q15);
}

/**
 * @brief  使用截止频率初始化一阶 IIR 低通滤波器
 * @param  f  滤波器结构体指针
 * @param  fs 采样频率（Hz）
 * @param  fc 截止频率（Hz）
 */
void IIR1_LPF_Int_Init_Fc(IIR1_LPF_Int_t *f, uint16_t fs, uint16_t fc)
{
    /* 计算 alpha = (2*π*fc/fs) / (1 + 2*π*fc/fs) */
    float dt = 1.0f / fs;
    float rc = 1.0f / (2.0f * 3.14159265f * fc);
    float alpha_f = dt / (rc + dt);
    
    IIR1_LPF_Int_Init_Float(f, alpha_f);
}

/**
 * @brief  输入一个样本，返回低通滤波后的值
 * @param  f 滤波器结构体指针
 * @param  x 输入样本值（int16_t）
 * @return 滤波后的输出值（int16_t）
 * @note   y = y_prev + alpha * (x - y_prev)
 */
int16_t IIR1_LPF_Int_Put(IIR1_LPF_Int_t *f, int16_t x)
{
    if (f->first_run)
    {
        /* 首次运行时用输入值初始化输出 */
        f->first_run = 0;
        f->y_prev = x;
        return x;
    }
    
    /* EWMA 公式：y = y_prev + alpha * (x - y_prev)
     * 定点实现：
     *  diff = x - y_prev
     *  delta = (alpha * diff) >> Q15_SHIFT
     *  y = y_prev + delta
     */
    int16_t diff = x - f->y_prev;
    int32_t delta = ((int32_t)f->alpha * (int32_t)diff) >> Q15_SHIFT;
    int16_t y = f->y_prev + (int16_t)delta;
    
    f->y_prev = y; /* 保存输出供下次使用 */
    return y;
}