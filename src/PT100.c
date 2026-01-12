/**
 * @file    PT100.c
 * @brief   PT100/PT1000温度传感器测量库 - 实现文件
 * @details 实现PT100温度测量的核心功能:
 * - 使用纯整数运算，适合无FPU的MCU如STM32F103
 * - 电阻测量（绝对测量法）
 * - 温度转换（查表+线性插值）
 * - 比例测量由ADS1220硬件实现
 * @version 2.0
 * @date    2026-01-12
 */

#include "PT100.h"

/* ====================================================================
 * PT100常数定义
 * ==================================================================== */
#define PT100_R0_MOHM 100000      /**< PT100在0°C时的标称电阻(mΩ) = 100Ω */
#define PT1000_R0_MOHM 1000000    /**< PT1000在0°C时的标称电阻(mΩ) = 1000Ω */

/* ====================================================================
 * PT100电阻-温度查找表
 * 使用Callendar-Van Dusen方程预计算
 * 温度范围: -50°C ~ +500°C, 步进10°C
 * 电阻单位: mΩ（毫欧姆）
 * ==================================================================== */

/* PT100电阻值表 (mΩ)，对应温度从-50°C到500°C，步进10°C */
static const int32_t PT100_R_TABLE[] = {
    /* -50°C to -10°C */
    80306,   /* -50°C: 80.306Ω */
    84271,   /* -40°C: 84.271Ω */
    88222,   /* -30°C: 88.222Ω */
    92160,   /* -20°C: 92.160Ω */
    96086,   /* -10°C: 96.086Ω */
    /* 0°C to 90°C */
    100000,  /*   0°C: 100.000Ω */
    103903,  /*  10°C: 103.903Ω */
    107794,  /*  20°C: 107.794Ω */
    111673,  /*  30°C: 111.673Ω */
    115541,  /*  40°C: 115.541Ω */
    119397,  /*  50°C: 119.397Ω */
    123242,  /*  60°C: 123.242Ω */
    127075,  /*  70°C: 127.075Ω */
    130897,  /*  80°C: 130.897Ω */
    134707,  /*  90°C: 134.707Ω */
    /* 100°C to 190°C */
    138506,  /* 100°C: 138.506Ω */
    142293,  /* 110°C: 142.293Ω */
    146068,  /* 120°C: 146.068Ω */
    149832,  /* 130°C: 149.832Ω */
    153584,  /* 140°C: 153.584Ω */
    157325,  /* 150°C: 157.325Ω */
    161054,  /* 160°C: 161.054Ω */
    164772,  /* 170°C: 164.772Ω */
    168478,  /* 180°C: 168.478Ω */
    172173,  /* 190°C: 172.173Ω */
    /* 200°C to 290°C */
    175856,  /* 200°C: 175.856Ω */
    179528,  /* 210°C: 179.528Ω */
    183188,  /* 220°C: 183.188Ω */
    186836,  /* 230°C: 186.836Ω */
    190473,  /* 240°C: 190.473Ω */
    194098,  /* 250°C: 194.098Ω */
    197712,  /* 260°C: 197.712Ω */
    201314,  /* 270°C: 201.314Ω */
    204904,  /* 280°C: 204.904Ω */
    208483,  /* 290°C: 208.483Ω */
    /* 300°C to 390°C */
    212050,  /* 300°C: 212.050Ω */
    215606,  /* 310°C: 215.606Ω */
    219150,  /* 320°C: 219.150Ω */
    222682,  /* 330°C: 222.682Ω */
    226203,  /* 340°C: 226.203Ω */
    229712,  /* 350°C: 229.712Ω */
    233209,  /* 360°C: 233.209Ω */
    236695,  /* 370°C: 236.695Ω */
    240169,  /* 380°C: 240.169Ω */
    243631,  /* 390°C: 243.631Ω */
    /* 400°C to 500°C */
    247082,  /* 400°C: 247.082Ω */
    250521,  /* 410°C: 250.521Ω */
    253948,  /* 420°C: 253.948Ω */
    257364,  /* 430°C: 257.364Ω */
    260768,  /* 440°C: 260.768Ω */
    264160,  /* 450°C: 264.160Ω */
    267540,  /* 460°C: 267.540Ω */
    270909,  /* 470°C: 270.909Ω */
    274266,  /* 480°C: 274.266Ω */
    277611,  /* 490°C: 277.611Ω */
    280944   /* 500°C: 280.944Ω */
};

#define PT100_TABLE_SIZE    56       /**< 查找表大小 */
#define PT100_TABLE_T_MIN   (-5000)  /**< 最小温度 (0.01°C) = -50°C */
#define PT100_TABLE_T_MAX   (50000)  /**< 最大温度 (0.01°C) = 500°C */
#define PT100_TABLE_T_STEP  (1000)   /**< 温度步进 (0.01°C) = 10°C */

/* 线性外推使用的温度系数 (mΩ/°C) */
#define PT100_DRDT_AT_MINUS50 396    /**< -50°C处的dR/dT ≈ 0.396 Ω/°C = 396 mΩ/°C */
#define PT100_DRDT_AT_PLUS500 333    /**< 500°C处的dR/dT ≈ 0.333 Ω/°C = 333 mΩ/°C */

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
    case ADS1220_IDAC_10UA:   return 10;
    case ADS1220_IDAC_50UA:   return 50;
    case ADS1220_IDAC_100UA:  return 100;
    case ADS1220_IDAC_250UA:  return 250;
    case ADS1220_IDAC_500UA:  return 500;
    case ADS1220_IDAC_1000UA: return 1000;
    case ADS1220_IDAC_1500UA: return 1500;
    default: return 0;
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
    if (!ADS1220_WaitForData(100)) 
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
 * @brief  读取PT100电阻值
 * @param  config: PT100配置参数指针
 * @retval 电阻值(mΩ毫欧)，失败返回负值
 */
int32_t PT100_ReadResistance_Int(PT100_Config_t *config)
{
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
        return PT100_TABLE_T_MIN + (delta_r * 100) / PT100_DRDT_AT_MINUS50;
    }
    
    if (r_mohm >= PT100_R_TABLE[PT100_TABLE_SIZE - 1])
    {
        /* 高于最大值，使用线性外推 */
        int32_t delta_r = r_mohm - PT100_R_TABLE[PT100_TABLE_SIZE - 1];
        return PT100_TABLE_T_MAX + (delta_r * 100) / PT100_DRDT_AT_PLUS500;
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
 * @brief  初始化PT100测量
 * @param  config: PT100配置参数指针
 * @note   配置ADS1220的寄存器和IDAC路由
 */
void PT100_Init(PT100_Config_t *config)
{
    ADS1220_Config_t ads_config;
    uint8_t gain_reg;

    /* 1. 增益映射 */
    switch (config->gain)
    {
    case 1:   gain_reg = ADS1220_GAIN_1; break;
    case 2:   gain_reg = ADS1220_GAIN_2; break;
    case 4:   gain_reg = ADS1220_GAIN_4; break;
    case 8:   gain_reg = ADS1220_GAIN_8; break;
    case 16:  gain_reg = ADS1220_GAIN_16; break;
    case 32:  gain_reg = ADS1220_GAIN_32; break;
    case 64:  gain_reg = ADS1220_GAIN_64; break;
    case 128: gain_reg = ADS1220_GAIN_128; break;
    default:  gain_reg = ADS1220_GAIN_8; config->gain = 8; break;
    }

    /* 2. 构建寄存器配置 */
    
    /* Reg0: 输入MUX + 增益 + PGA使能 */
    // 使用 config->input_p 作为 MUX 设置 (假设用户传入的是 ADS1220_MUX_xxx 宏)
    ads_config.reg0 = (config->input_p & 0xF0) | gain_reg | ADS1220_PGA_ENABLED;

    /* Reg1: 数据速率 + 模式 */
    // 默认使用 20SPS (低噪声), 正常模式, 单次转换(即使是连续读取，也可通过Start控制)
    // 关闭内部温度传感器 (TS_DISABLED)
    ads_config.reg1 = ADS1220_DR_20SPS | ADS1220_MODE_NORMAL | ADS1220_CM_SINGLE | ADS1220_TS_DISABLED;

    /* Reg2: 基准电压 + FIR + IDAC电流 */
    // 默认使用内部2.048V基准 (软件比例法也依赖内部基准进行电压量化)
    // 启用 50Hz/60Hz 同时抑制滤波器
    ads_config.reg2 = ADS1220_VREF_INT | ADS1220_FIR_50HZ_60HZ | (config->idac & 0x07);

    /* Reg3: IDAC路由 */
    uint8_t i1mux = ADS1220_I1MUX_DISABLED;
    uint8_t i2mux = ADS1220_I2MUX_DISABLED;

    // 自动推断 IDAC1 的位置: 通常连接到 PT100 的正端 (AIN0)
    // 简单起见，我们根据 mux input_p 的高4位推断，或者默认 AIN0
    if ((config->input_p & 0xF0) == ADS1220_MUX_AIN0_AIN1) i1mux = ADS1220_I1MUX_AIN0;
    else if ((config->input_p & 0xF0) == ADS1220_MUX_AIN1_AIN2) i1mux = ADS1220_I1MUX_AIN1;
    else i1mux = ADS1220_I1MUX_AIN0; // 默认

    // 如果是3线制，需要配置 IDAC2
    if (config->wire_mode == PT100_3WIRE)
    {
        i2mux = config->idac2_pin; // 使用用户指定的引脚 (例如 ADS1220_I2MUX_AIN2)
    }
    
    ads_config.reg3 = i1mux | i2mux | ADS1220_DRDYM_DRDY_ONLY;

    /* 3. 写入硬件配置 */
    ADS1220_WriteConfig(&ads_config);
    Delay_ms(10); // 等待配置生效
}