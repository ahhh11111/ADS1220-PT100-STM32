#include "filter_int.h"
#include <math.h>  /* 仅用于初始化时的 alpha 计算 */

/*================================================================================================*/
/* 1. 简单平均滤波器（整数版本，支持 int32_t）                                                    */
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
 * @param  x    输入样本值（int32_t）
 * @param  out  输出均值指针（仅当返回1时有效）
 * @return 0    样本未满，继续累积；1 - 输出已准备好
 */
uint8_t AvgFilter_Int_Put(AvgFilter_Int_t *f, int32_t x, int32_t *out)
{
    f->sum += x; /* 累加新样本（int64_t 防止溢出） */
    if (++f->count < f->N)
        return 0; /* 样本数未达到N，继续累积 */

#ifdef USE_SHIFT_DIV
    /* 位移模式：sum >> shift */
    *out = (int32_t)(f->sum >> f->shift);
#else
    /* 直接除法模式：sum / N */
    *out = (int32_t)(f->sum / f->N);
#endif
    
    /* 重置状态 */
    f->sum = 0;
    f->count = 0;
    return 1; /* 输出有效 */
}

/*================================================================================================*/
/* 2. 一阶 IIR 低通滤波器（整数版本，支持 int32_t）                                               */
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
 * @param  x 输入样本值（int32_t）
 * @return 滤波后的输出值（int32_t）
 * @note   y = y_prev + alpha * (x - y_prev)
 */
int32_t IIR1_LPF_Int_Put(IIR1_LPF_Int_t *f, int32_t x)
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
     * 
     * 注意：需要使用 int64_t 防止中间乘法溢出
     *  alpha (int16_t) * diff (int32_t) 最大可达 ±70,000,000,000,000
     *  需要 int64_t 存储
     */
    int32_t diff = x - f->y_prev;
    int64_t delta = ((int64_t)f->alpha * (int64_t)diff) >> Q15_SHIFT;
    int32_t y = f->y_prev + (int32_t)delta;
    
    f->y_prev = y; /* 保存输出供下次使用 */
    return y;
}

/*================================================================================================*/
/* 3. 去极值平均滤波器（整数版本）                                                                */
/*================================================================================================*/

/* 冒泡排序保持不变 */
static void _int_sort(int32_t *arr, uint16_t len)
{
    uint16_t i, j;
    int32_t temp;
    for (i = 0; i < len - 1; i++)
    {
        for (j = 0; j < len - 1 - i; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

/**
 * @brief  初始化去极值滤波器
 * @param  f        滤波器结构体指针
 * @param  pBuffer  外部定义的缓存数组
 * @param  N        采样窗口大小
 * @param  remove   单侧去除的极值个数
 */
void TrimFilter_Int_Init(TrimFilter_Int_t *f, int32_t *pBuffer, uint16_t N, uint16_t remove)
{
    /* 1. 基本安全检查 */
    if (N == 0) N = 1;
    if (remove * 2 >= N)
    {
        remove = (N - 1) / 2; /* 至少保留 1 个数据 */
    }

    f->pBuffer = pBuffer;
    f->N = N;
    f->count = 0;

#ifdef USE_SHIFT_DIV
    /* * 位移模式优化逻辑：
     * 有效数据量 valid_count = N - 2 * remove
     * 这个 valid_count 必须是 2 的幂。
     * 如果不是，我们增加 remove 的数量，直到 valid_count 变成 2 的幂。
     */
    uint16_t valid_count = N - (remove * 2);

    /* 循环检查：如果不是2的幂，且还有数据可减，则增加去除量 */
    while ((!is_power_of_2(valid_count)) && (valid_count > 1))
    {
        remove++;          /* 多去掉一组极值 */
        valid_count -= 2;  /* 有效数据减少2个 */
    }

    /* 防止减过头（例如 N=10, remove原本是4，valid=2。如果再减变成0） */
    if (valid_count == 0)
    {
        /* 极端情况回退：不去除任何极值，强制 N 为 2 的幂 */
        /* 注意：这里为了不改写用户 N，通常建议用户一开始就设计好 */
        remove = 0; 
        valid_count = N; 
        /* 如果 N 也不是 2 的幂，计算 log2 会返回 0，除法变为 >>0 (除以1)，虽然不准但不会死机 */
    }

    f->remove = remove;
    f->shift = calc_log2(valid_count);
#else
    /* 普通除法模式：直接使用用户参数 */
    f->remove = remove;
#endif
}

/**
 * @brief  输入样本，积攒 N 个后排序并去极值平均
 * @return 0 - 正在积攒；1 - 计算完成
 */
uint8_t TrimFilter_Int_Put(TrimFilter_Int_t *f, int32_t x, int32_t *out)
{
    /* 1. 存入缓冲区 */
    if (f->count < f->N)
    {
        f->pBuffer[f->count] = x;
        f->count++;
    }

    /* 2. 检查是否存满 */
    if (f->count < f->N)
    {
        return 0; 
    }

    /* 3. 数据已满，开始处理 */
    _int_sort(f->pBuffer, f->N);

    /* 4. 求中间部分和 */
    int64_t sum = 0;
    uint16_t start_idx = f->remove;
    uint16_t end_idx = f->N - f->remove;
    uint16_t i;

    for (i = start_idx; i < end_idx; i++)
    {
        sum += f->pBuffer[i];
    }

    /* 5. 计算平均值（区分模式） */
#ifdef USE_SHIFT_DIV
    /* 位移模式：使用移位代替除法 */
    *out = (int32_t)(sum >> f->shift);
#else
    /* 除法模式：标准除法 */
    uint16_t valid_count = f->N - (f->remove * 2);
    if (valid_count == 0) valid_count = 1; // 防除零
    *out = (int32_t)(sum / valid_count);
#endif

    /* 6. 重置计数 */
    f->count = 0;
    return 1;
}