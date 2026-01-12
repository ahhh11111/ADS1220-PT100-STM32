/**
 * @file    PT100.c
 * @brief   PT100/PT1000温度传感器测量库 - 实现文件
 * @details 实现PT100温度测量的核心功能:
 * - 使用纯整数运算，适合无FPU的MCU如STM32F103
 * - 电阻测量（绝对测量法）
 * - 温度转换（查表+线性插值）
 * @version 2.0
 * @date    2026-01-12
 */

#include "PT100.h"

/* ====================================================================
 * PT100常数定义
 * ==================================================================== */
#define PT100_R0_MOHM 100000   /**< PT100在0°C时的标称电阻(mΩ) = 100Ω */
#define PT1000_R0_MOHM 1000000 /**< PT1000在0°C时的标称电阻(mΩ) = 1000Ω */

/* ====================================================================
 * PT100电阻-温度查找表 (扩展版)
 * 使用Callendar-Van Dusen方程预计算
 * 温度范围: -200°C ~ +850°C, 步进10°C
 * 电阻单位: mΩ（毫欧姆）
 * ==================================================================== */

/* PT100电阻值表 (mΩ)，对应温度从-200°C到850°C，步进10°C */
static const int32_t PT100_R_TABLE[] = {
    /* -200°C to -110°C */
    18520, /* -200°C: 18.520Ω (Callendar-Van Dusen) */
    22830, /* -190°C: 22.830Ω */
    27100, /* -180°C: 27.100Ω */
    31340, /* -170°C: 31.340Ω */
    35540, /* -160°C: 35.540Ω */
    39720, /* -150°C: 39.720Ω */
    43880, /* -140°C: 43.880Ω */
    48000, /* -130°C: 48.000Ω */
    52110, /* -120°C: 52.110Ω */
    56190, /* -110°C: 56.190Ω */
    /* -100°C to -10°C */
    60260, /* -100°C: 60.260Ω */
    64300, /*  -90°C: 64.300Ω */
    68330, /*  -80°C: 68.330Ω */
    72330, /*  -70°C: 72.330Ω */
    76330, /*  -60°C: 76.330Ω */
    80306, /*  -50°C: 80.306Ω */
    84271, /*  -40°C: 84.271Ω */
    88222, /*  -30°C: 88.222Ω */
    92160, /*  -20°C: 92.160Ω */
    96086, /*  -10°C: 96.086Ω */
    /* 0°C to 90°C */
    100000, /*   0°C: 100.000Ω */
    103903, /*  10°C: 103.903Ω */
    107794, /*  20°C: 107.794Ω */
    111673, /*  30°C: 111.673Ω */
    115541, /*  40°C: 115.541Ω */
    119397, /*  50°C: 119.397Ω */
    123242, /*  60°C: 123.242Ω */
    127075, /*  70°C: 127.075Ω */
    130897, /*  80°C: 130.897Ω */
    134707, /*  90°C: 134.707Ω */
    /* 100°C to 190°C */
    138506, /* 100°C: 138.506Ω */
    142293, /* 110°C: 142.293Ω */
    146068, /* 120°C: 146.068Ω */
    149832, /* 130°C: 149.832Ω */
    153584, /* 140°C: 153.584Ω */
    157325, /* 150°C: 157.325Ω */
    161054, /* 160°C: 161.054Ω */
    164772, /* 170°C: 164.772Ω */
    168478, /* 180°C: 168.478Ω */
    172173, /* 190°C: 172.173Ω */
    /* 200°C to 290°C */
    175856, /* 200°C: 175.856Ω */
    179528, /* 210°C: 179.528Ω */
    183188, /* 220°C: 183.188Ω */
    186836, /* 230°C: 186.836Ω */
    190473, /* 240°C: 190.473Ω */
    194098, /* 250°C: 194.098Ω */
    197712, /* 260°C: 197.712Ω */
    201314, /* 270°C: 201.314Ω */
    204904, /* 280°C: 204.904Ω */
    208483, /* 290°C: 208.483Ω */
    /* 300°C to 390°C */
    212050, /* 300°C: 212.050Ω */
    215606, /* 310°C: 215.606Ω */
    219150, /* 320°C: 219.150Ω */
    222682, /* 330°C: 222.682Ω */
    226203, /* 340°C: 226.203Ω */
    229712, /* 350°C: 229.712Ω */
    233209, /* 360°C: 233.209Ω */
    236695, /* 370°C: 236.695Ω */
    240169, /* 380°C: 240.169Ω */
    243631, /* 390°C: 243.631Ω */
    /* 400°C to 490°C */
    247082, /* 400°C: 247.082Ω */
    250521, /* 410°C: 250.521Ω */
    253948, /* 420°C: 253.948Ω */
    257364, /* 430°C: 257.364Ω */
    260768, /* 440°C: 260.768Ω */
    264160, /* 450°C: 264.160Ω */
    267540, /* 460°C: 267.540Ω */
    270909, /* 470°C: 270.909Ω */
    274266, /* 480°C: 274.266Ω */
    277611, /* 490°C: 277.611Ω */
    /* 500°C to 590°C */
    280944, /* 500°C: 280.944Ω */
    284266, /* 510°C: 284.266Ω */
    287576, /* 520°C: 287.576Ω */
    290874, /* 530°C: 290.874Ω */
    294160, /* 540°C: 294.160Ω */
    297434, /* 550°C: 297.434Ω */
    300697, /* 560°C: 300.697Ω */
    303947, /* 570°C: 303.947Ω */
    307186, /* 580°C: 307.186Ω */
    310413, /* 590°C: 310.413Ω */
    /* 600°C to 690°C */
    313628, /* 600°C: 313.628Ω */
    316831, /* 610°C: 316.831Ω */
    320022, /* 620°C: 320.022Ω */
    323202, /* 630°C: 323.202Ω */
    326370, /* 640°C: 326.370Ω */
    329526, /* 650°C: 329.526Ω */
    332670, /* 660°C: 332.670Ω */
    335802, /* 670°C: 335.802Ω */
    338922, /* 680°C: 338.922Ω */
    342030, /* 690°C: 342.030Ω */
    /* 700°C to 790°C */
    345126, /* 700°C: 345.126Ω */
    348210, /* 710°C: 348.210Ω */
    351282, /* 720°C: 351.282Ω */
    354342, /* 730°C: 354.342Ω */
    357390, /* 740°C: 357.390Ω */
    360426, /* 750°C: 360.426Ω */
    363450, /* 760°C: 363.450Ω */
    366462, /* 770°C: 366.462Ω */
    369462, /* 780°C: 369.462Ω */
    372450, /* 790°C: 372.450Ω */
    /* 800°C to 850°C */
    375426, /* 800°C: 375.426Ω */
    378390, /* 810°C: 378.390Ω */
    381342, /* 820°C: 381.342Ω */
    384282, /* 830°C: 384.282Ω */
    387210, /* 840°C: 387.210Ω */
    390126  /* 850°C: 390.126Ω */
};

#define PT100_TABLE_SIZE 106       /**< 查找表大小 (-200°C到850°C，步进10°C，共106个点) */
#define PT100_TABLE_T_MIN (-20000) /**< 最小温度 (0.01°C) = -200°C */
#define PT100_TABLE_T_MAX (85000)  /**< 最大温度 (0.01°C) = 850°C */
#define PT100_TABLE_T_STEP (1000)  /**< 温度步进 (0.01°C) = 10°C */

/* 线性外推使用的温度系数 (mΩ/°C) */
#define PT100_DRDT_AT_MINUS200 427 /**< -200°C处的dR/dT ≈ 0.427 Ω/°C = 427 mΩ/°C */
#define PT100_DRDT_AT_PLUS850 291  /**< 850°C处的dR/dT ≈ 0.291 Ω/°C = 291 mΩ/°C */

/* ====================================================================
 * 私有辅助函数
 * ==================================================================== */

/**
 * @brief  获取IDAC电流值
 * @param  idac_setting: IDAC设置值
 * @retval 电流值(μA)
 */
static uint16_t PT100_GetIDACCurrent(uint8_t idac_setting)
{
    switch (idac_setting & 0x07)
    {
    case ADS1220_IDAC_10UA:
        return 10;
    case ADS1220_IDAC_50UA:
        return 50;
    case ADS1220_IDAC_100UA:
        return 100;
    case ADS1220_IDAC_250UA:
        return 250;
    case ADS1220_IDAC_500UA:
        return 500;
    case ADS1220_IDAC_1000UA:
        return 1000;
    case ADS1220_IDAC_1500UA:
        return 1500;
    default:
        return 0;
    }
}

/**
 * @brief  读取ADC原始值(内部辅助函数)
 * @retval ADC原始值，失败返回0x7FFFFFFF
 */
static int32_t PT100_ReadADCRaw(void)
{
    ADS1220_ClearError();
    ADS1220_StartSync();

    // 等待数据就绪 (20SPS约50ms，给予充足的超时时间)
    if (!ADS1220_WaitForData(10000000))
    {
        return 0x7FFFFFFF;
    }
    return ADS1220_ReadData();
}

/* ====================================================================
 * 整数模式实现
 * ==================================================================== */

/**
 * @brief  读取PT100电阻（绝对测量法）
 * @param  config: PT100配置参数指针
 * @retval 电阻值(mΩ毫欧)，失败返回-1
 * @note   公式: R(mΩ) = V / I = (ADC * Vref_mv * 1000) / (2^23 * Gain * I_uA)
 */
static int32_t PT100_ReadResistance_Absolute(PT100_Config_t *config)
{
    int32_t raw = PT100_ReadADCRaw();
    if (raw == 0x7FFFFFFF)
        return -1;

    /* 获取IDAC电流(μA) */
    uint16_t current_ua = PT100_GetIDACCurrent(config->idac);
    if (current_ua == 0)
        return -1;

    /* 计算电阻: R(mΩ) = V / I
     * V = (raw / 2^23) * (Vref_mv / 1000) / Gain  (单位: V)
     * I = current_ua / 1000000  (单位: A)
     * R = V / I = (raw * Vref_mv * 1000000) / (2^23 * Gain * current_ua)  (单位: Ω)
     * R(mΩ) = R * 1000 = (raw * Vref_mv * 1000000 * 1000) / (8388608 * Gain * current_ua)
     *
     * 使用64位中间变量避免溢出
     */
    int64_t numerator = (int64_t)raw * (int64_t)config->vref_mv * 1000000LL;
    int64_t denominator = 8388608LL * (int64_t)config->gain * (int64_t)current_ua;

    return (int32_t)(numerator / denominator);
}

/**
 * @brief  读取PT100电阻（3线制硬件比例测量法）
 * @param  config: PT100配置参数指针
 * @retval 电阻值(mΩ毫欧)，失败返回-1
 *
 * @note   硬件比例测量原理:
 *
 *         比例计算:
 *           ADC_code / 2^23 = (I * Rpt100) / (I * Rref) = Rpt100 / Rref
 *           Rpt100 = ADC_code * Rref / 2^23
 *
 *         优点:
 *         1. 电流源漂移被消除 (比例测量)
 *         2. 参考电压精度不影响结果
 *         3. 3线制可消除导线电阻
 */
static int32_t PT100_ReadResistance_3Wire_Ratiometric(PT100_Config_t *config)
{
    int32_t raw = PT100_ReadADCRaw();
    if (raw == 0x7FFFFFFF)
        return -1;

    /* 检查参考电阻值是否有效 */
    if (config->rref_mohm == 0)
        return -1;

    /* 比例测量计算:
     *
     * ADC输入电压 = I × Rpt100 (经过PGA放大)
     * 参考电压 = I × Rref (不经过PGA)
     *
     * ADC_code / 2^23 = (I × Rpt100 × Gain) / (I × Rref)
     *
     * 因此: Rpt100(mΩ) = (ADC_code × Rref(mΩ)) / (2^23 × Gain)
     */
    int64_t numerator = (int64_t)raw * (int64_t)config->rref_mohm;
    int64_t denominator = 8388608LL * (int64_t)config->gain; /* 2^23 × Gain */

    return (int32_t)(numerator / denominator);
}

/**
 * @brief  读取PT100电阻值
 * @param  config: PT100配置参数指针
 * @retval 电阻值(mΩ毫欧)，失败返回负值
 */
int32_t PT100_ReadResistance_Int(PT100_Config_t *config)
{
    if (config->wire_mode == PT100_3WIRE_RATIOMETRIC)
    {
        return PT100_ReadResistance_3Wire_Ratiometric(config);
    }
    return PT100_ReadResistance_Absolute(config);
}

/**
 * @brief  电阻值转换为温度值
 * @param  resistance_mohm: 电阻值(mΩ毫欧)
 * @param  type: PT100类型(PT100或PT1000)
 * @retval 温度值(0.01°C)
 * @note   使用查表+线性插值实现，精度优于±0.1°C
 */
int32_t PT100_ResistanceToTemperature_Int(int32_t resistance_mohm, PT100_Type_t type)
{
    int32_t r_mohm = resistance_mohm;

    /* 对于PT1000，将电阻值缩小10倍，使用PT100查找表 */
    if (type == PT1000_TYPE)
    {
        r_mohm = resistance_mohm / 10;
    }

    /* 边界检查 */
    if (r_mohm <= PT100_R_TABLE[0])
    {
        /* 低于最小值，使用线性外推 */
        /* dT = (R - R_min) / dR_dT, 单位0.01°C */
        int32_t delta_r = r_mohm - PT100_R_TABLE[0];
        return PT100_TABLE_T_MIN + (delta_r * 100) / PT100_DRDT_AT_MINUS200;
    }

    if (r_mohm >= PT100_R_TABLE[PT100_TABLE_SIZE - 1])
    {
        /* 高于最大值，使用线性外推 */
        int32_t delta_r = r_mohm - PT100_R_TABLE[PT100_TABLE_SIZE - 1];
        return PT100_TABLE_T_MAX + (delta_r * 100) / PT100_DRDT_AT_PLUS850;
    }

    /* 二分查找找到电阻值所在的区间 */
    int32_t low = 0;
    int32_t high = PT100_TABLE_SIZE - 1;

    while (low < high - 1)
    {
        int32_t mid = (low + high) / 2;
        if (r_mohm < PT100_R_TABLE[mid])
            high = mid;
        else
            low = mid;
    }

    /* 线性插值计算温度 */
    /* T = T_low + (R - R_low) * (T_high - T_low) / (R_high - R_low) */
    int32_t r_low = PT100_R_TABLE[low];
    int32_t r_high = PT100_R_TABLE[high];
    int32_t t_low = PT100_TABLE_T_MIN + low * PT100_TABLE_T_STEP;

    /* 使用64位避免溢出 */
    int64_t delta_r = (int64_t)(r_mohm - r_low);
    int64_t range_r = (int64_t)(r_high - r_low);
    int32_t delta_t = (int32_t)((delta_r * PT100_TABLE_T_STEP) / range_r);

    return t_low + delta_t;
}

/**
 * @brief  读取PT100温度值
 * @param  config: PT100配置参数指针
 * @retval 温度值(0.01°C)，失败返回-99900
 */
int32_t PT100_ReadTemperature_Int(PT100_Config_t *config)
{
    int32_t r = PT100_ReadResistance_Int(config);
    if (r < 0)
        return -99900; // 读取失败
    return PT100_ResistanceToTemperature_Int(r, config->type);
}

/**
 * @brief  PT100单点校准
 * @param  config: PT100配置参数指针
 * @param  known_temp_centideg: 已知的标准温度(0.01°C)
 * @param  offset_centideg: 输出的温度偏移量(0.01°C)
 */
void PT100_Calibrate_Int(PT100_Config_t *config, int32_t known_temp_centideg, int32_t *offset_centideg)
{
    int32_t measured_temp = PT100_ReadTemperature_Int(config);
    if (measured_temp > -90000)
    {
        *offset_centideg = known_temp_centideg - measured_temp;
    }
}

/* ====================================================================
 * 公共API函数实现
 * ==================================================================== */

/**
 * @brief  PT100 + ADS1220 初始化（完全显式配置版）
 * @param  config  PT100 配置结构体指针
 *
 * @note
 *  - 本函数不包含任何“自动推断”逻辑
 *  - 所有硬件行为（MUX / IDAC / VREF / PGA）均由 config 明确指定
 *  - wire_mode 仅用于选择参考方式（比例 / 非比例），不参与引脚推理
 */
void PT100_Init(PT100_Config_t *config)
{
    ADS1220_Config_t ads; /* ADS1220 寄存器配置结构体 */
    uint8_t gain_reg;     /* PGA 增益对应的寄存器编码 */

    /* =====================================================================
     * 1. PGA 增益映射
     *    将用户传入的“物理倍数”转换为 ADS1220 寄存器定义
     * ===================================================================== */
    switch (config->gain)
    {
    case 1:
        gain_reg = ADS1220_GAIN_1;
        break; /* ×1  */
    case 2:
        gain_reg = ADS1220_GAIN_2;
        break; /* ×2  */
    case 4:
        gain_reg = ADS1220_GAIN_4;
        break; /* ×4  */
    case 8:
        gain_reg = ADS1220_GAIN_8;
        break; /* ×8  */
    case 16:
        gain_reg = ADS1220_GAIN_16;
        break; /* ×16 */
    case 32:
        gain_reg = ADS1220_GAIN_32;
        break; /* ×32 */
    case 64:
        gain_reg = ADS1220_GAIN_64;
        break; /* ×64 */
    case 128:
        gain_reg = ADS1220_GAIN_128;
        break; /* ×128 */

    default:
        /* 非法增益，回退到安全的 ×8（不修改 config 本身） */
        gain_reg = ADS1220_GAIN_8;
        break;
    }

    /* =====================================================================
     * 2. Reg0：输入多路复用 + PGA 配置
     *
     *  [7:4] MUX   : 差分输入通道选择 (AINx - AINy)
     *  [3:1] GAIN  : PGA 增益
     *  [0]   PGAEN : 使能 PGA
     * ===================================================================== */
    ads.reg0 = config->mux |        /* 差分输入通道选择 */
               gain_reg |           /* PGA 增益 */
               ADS1220_PGA_ENABLED; /* 启用 PGA 放大器 */

    /* =====================================================================
     * 3. Reg1：转换速率 & 工作模式
     *
     *  - 20SPS：低速低噪声，适合 RTD 测温
     *  - Normal mode：正常功耗模式
     *  - Single-shot：单次转换（由 START 或命令触发）
     *  - 禁用内部温度传感器
     * ===================================================================== */
    ads.reg1 = ADS1220_DR_20SPS |    /* 数据速率 20SPS */
               ADS1220_MODE_NORMAL | /* 正常工作模式 */
               ADS1220_CM_SINGLE |   /* 单次转换模式 */
               ADS1220_TS_DISABLED;  /* 关闭内部温度传感器 */

    /* =====================================================================
     * 4. Reg2：参考源 + 数字滤波 + IDAC 电流
     *
     * 所有位均由配置结构体直接给出：
     *  - vref_sel : 内部 / 外部参考
     *  - fir_mode : FIR 滤波类型
     *  - idac     : IDAC 电流档位
     * ===================================================================== */
    ads.reg2 = config->vref_sel |     /* 参考源选择 */
               config->fir_mode |     /* 数字滤波配置 */
               (config->idac & 0x07); /* IDAC 电流档位 */

    /* =====================================================================
     * 5. Reg3：IDAC 电流源路由
     *
     *  - IDAC1 / IDAC2 的输出引脚由配置结构体完全指定
     *  - 不进行任何自动判断或引脚推断
     *  - DRDY_MODE：仅在 DRDY 引脚输出就绪信号
     * ===================================================================== */
    ads.reg3 = config->idac1_pin |      /* IDAC1 路由引脚 */
               config->idac2_pin |      /* IDAC2 路由引脚（不用则 DISABLED） */
               ADS1220_DRDYM_DRDY_ONLY; /* DRDY 仅指示数据就绪 */

    /* =====================================================================
     * 6. 写入 ADS1220 寄存器
     * ===================================================================== */
    ADS1220_WriteConfig(&ads);

    /* 等待配置生效 / IDAC 稳定 */
    Delay_ms(10);
}
