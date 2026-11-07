#include "imu.h"
#include "spi.h"
#include "robot_config.h"
#include <math.h>

/* 片选：低电平有效。这条必须自己拉，CubeMX 里 NSS 配的是 Disable */
#define MPU_CS_LOW()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_CS_HIGH()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

/* MPU6500 寄存器地址（数据手册里的，照抄即可） */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

/* ±2000 dps 量程下：16.4 LSB 对应 1 °/s */
#define GYRO_LSB_PER_DPS    16.4f
#define DEG_TO_RAD          0.017453293f

/* ±8g 量程下：4096 LSB 对应 1 g。
 * 为什么不用 ±2g（更精细）：车撞墙时云台加速度能到 3~5g，±2g 会削顶，
 * 削顶的加速度方向是错的 → Mahony 会把姿态拉歪。
 * 4096 LSB/g 的倾角分辨率约 0.014°，保持水平完全够用 */
#define ACCEL_LSB_PER_G     4096.0f
#define GRAVITY             9.80665f

static float   s_gyro[3];       /* 原始读数（未扣零偏），rad/s */
static float   s_acc[3];        /* 加速度，m/s²，机体系 */
static float   s_bias[3];       /* 零偏，IMU_CalibrateGyro() 里测出来 */
static float   s_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };  /* 姿态四元数，w x y z */
static float   s_gb[3] = { 0.0f, 0.0f, 0.0f };       /* Mahony 顺手估的陀螺零偏 */
static uint8_t s_online;

/** @brief 写一个寄存器。SPI 写的地址最高位是 0 */
static void MPU_Write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };

    MPU_CS_LOW();
    HAL_SPI_Transmit(&hspi5, tx, 2, 2);
    MPU_CS_HIGH();
}

/** @brief 连续读 len 个寄存器。SPI 读的地址最高位是 1 */
static void MPU_Read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t addr = (uint8_t)(reg | 0x80);

    MPU_CS_LOW();
    HAL_SPI_Transmit(&hspi5, &addr, 1, 2);   /* 先发地址 */
    HAL_SPI_Receive(&hspi5, buf, len, 2);    /* 再收数据 */
    MPU_CS_HIGH();
}

/* ==================== Mahony 姿态解算 ==================== */
/**
 * @brief 用"陀螺积分 + 重力方向校正"更新四元数。1ms 调一次
 *
 * 思路三句话：
 *   1. 陀螺仪积分算得准但会漂（零偏被积起来）；加速度计不漂但抖得厉害、
 *      而且只认"重力方向"，分不出绕重力轴的转动（yaw）。
 *   2. 拿加速度计量出来的重力方向，去和四元数推算出来的重力方向做叉积，
 *      叉积结果就是"姿态偏了多少"。用这个偏差去修陀螺读数 → 互补。
 *   3. 修完的角速度积分成四元数。云台只要 pitch，不需要磁力计定 yaw。
 *
 * @param gx,gy,gz 角速度 rad/s（已扣零偏）
 * @param ax,ay,az 加速度 m/s²（静止水平时约 0,0,+9.8）
 * @param dt       周期 s
 */
static void Mahony_Update(float gx, float gy, float gz,
                          float ax, float ay, float az, float dt)
{
    float recip;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* ---- 1. 加速度归一化：只留方向，丢掉大小（大小随振动乱跳） ---- */
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        recip = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recip;
        ay *= recip;
        az *= recip;

        /* ---- 2. 用当前四元数推算：重力在机体系里"应该"指向哪 ----
         * 这是旋转矩阵第三列的一半（省掉乘 2，后面叉积里会约掉） */
        halfvx = s_q[1] * s_q[3] - s_q[0] * s_q[2];
        halfvy = s_q[0] * s_q[1] + s_q[2] * s_q[3];
        halfvz = s_q[0] * s_q[0] - 0.5f + s_q[3] * s_q[3];

        /* ---- 3. 叉积 = "实测重力方向" × "估计重力方向" = 姿态偏差（弧度） ---- */
        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        /* ---- 4. 比例项：偏差直接拉回陀螺读数，把漂移按住 ---- */
        gx += MAHONY_KP * halfex;
        gy += MAHONY_KP * halfey;
        gz += MAHONY_KP * halfez;

        /* ---- 5. 积分项：把"一直存在、拉不平"的偏差记成零偏 ----
         * 好处是不用每次上电都手标定；给了 0 也能跑，只是稳态精度差一点 */
        if (MAHONY_KI > 0.0f)
        {
            s_gb[0] += MAHONY_KI * halfex * dt;
            s_gb[1] += MAHONY_KI * halfey * dt;
            s_gb[2] += MAHONY_KI * halfez * dt;

            gx += s_gb[0];
            gy += s_gb[1];
            gz += s_gb[2];
        }
    }

    /* ---- 6. 四元数一阶积分：q' = 0.5 * q ⊗ ω ---- */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = s_q[0];
    qb = s_q[1];
    qc = s_q[2];

    s_q[0] += (-qb * gx - qc * gy - s_q[3] * gz);
    s_q[1] += ( qa * gx + qc * gz - s_q[3] * gy);
    s_q[2] += ( qa * gy - qb * gz + s_q[3] * gx);
    s_q[3] += ( qa * gz + qb * gy - qc * gx);

    /* ---- 7. 必须归一化，否则数值会慢慢发散，最后全是 NaN ---- */
    recip = 1.0f / sqrtf(s_q[0] * s_q[0] + s_q[1] * s_q[1] +
                         s_q[2] * s_q[2] + s_q[3] * s_q[3]);

    s_q[0] *= recip;
    s_q[1] *= recip;
    s_q[2] *= recip;
    s_q[3] *= recip;
}

/**
 * @brief 上电时直接用加速度计"对齐"初始姿态（yaw 固定为 0）
 *
 * 不做这一步的话，四元数初值是"水平"，如果云台实际是歪的，
 * Mahony 要用两三秒才收敛，这段时间 IMU_GetPitch() 给的是错的。
 * 静止时加速度计算 pitch/roll 非常准，所以开局直接算出来最省事。
 */
static void IMU_AlignFromAccel(float ax, float ay, float az)
{
    float roll, pitch;      /* 绕 X 的角、绕 Y 的角 */
    float cr, sr, cp, sp;

    /* 静止时加速度计读的是"地面推回来的反作用力"，水平时指向机体 +Z。
     * 按 ZYX 欧拉角约定推出：
     *   sin(pitch) = ax / |a|      —— 机头抬起时 ax 变正
     *   sin(roll)  = -ay / |a|     —— 右倾时 ay 变负            */
    roll  = atan2f(-ay, sqrtf(ax * ax + az * az));
    pitch = atan2f( ax, az);

    cr = cosf(roll  * 0.5f);
    sr = sinf(roll  * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);

    /* q = q_pitch ⊗ q_roll（yaw = 0） */
    s_q[0] =  cr * cp;
    s_q[1] =  sr * cp;
    s_q[2] =  cr * sp;
    s_q[3] = -sr * sp;
}

void IMU_Init(void)
{
    uint8_t who = 0;

    HAL_Delay(100);                          /* 上电后等芯片内部起振 */

    MPU_Write(REG_PWR_MGMT_1, 0x80);         /* 软复位 */
    HAL_Delay(100);                          /* 复位要等，手册要求 100ms */

    MPU_Write(REG_PWR_MGMT_1, 0x01);         /* 退出睡眠，时钟源用 X 轴陀螺的 PLL（最稳） */
    MPU_Write(REG_SMPLRT_DIV, 0x00);         /* 采样率 = 1kHz / (1+0) = 1kHz */
    MPU_Write(REG_CONFIG, 0x03);             /* 数字低通 41Hz，滤掉电机振动的高频噪声 */
    MPU_Write(REG_GYRO_CONFIG, 0x18);        /* 量程 ±2000 °/s，不启用自检 */
    MPU_Write(REG_ACCEL_CONFIG, 0x10);       /* 量程 ±8g：Mahony 要用加速度，量程别给太小 */

    MPU_Read(REG_WHO_AM_I, &who, 1);

    /* 正常应该是 0x70（MPU6500）。读到 0 或 0xFF 说明 SPI 根本没通 */
    s_online = (who == 0x00 || who == 0xFF) ? 0 : 1;

    /* 零偏先清零。这样即使不调 IMU_CalibrateGyro()，读数和以前一样 */
    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;

    /* 四元数回到"水平"，等标定时再用加速度计对齐 */
    s_q[0] = 1.0f;
    s_q[1] = 0.0f;
    s_q[2] = 0.0f;
    s_q[3] = 0.0f;
    s_gb[0] = s_gb[1] = s_gb[2] = 0.0f;
}

/**
 * @brief 静止标定：连续采 500 次取平均，当作零偏，顺手对齐初始姿态。
 *        必须在上电、车完全静止时调用，否则会把"正在搬动"当成零偏。
 */
void IMU_CalibrateGyro(void)
{
    float sum[3] = { 0.0f, 0.0f, 0.0f };
    float asum[3] = { 0.0f, 0.0f, 0.0f };
    int i, k;

    for (i = 0; i < 500; i++)
    {
        IMU_Update();                   /* 直接读原始值，此时 s_gyro 还没扣偏 */
        sum[0] += s_gyro[0];
        sum[1] += s_gyro[1];
        sum[2] += s_gyro[2];

        asum[0] += s_acc[0];
        asum[1] += s_acc[1];
        asum[2] += s_acc[2];

        HAL_Delay(1);
    }

    for (k = 0; k < 3; k++) s_bias[k] = sum[k] / 500.0f;

    /* 静止 0.5s 采到的平均加速度方向 = 重力方向 → 直接算出当前水平偏差 */
    IMU_AlignFromAccel(asum[0], asum[1], asum[2]);
}

void IMU_Update(void)
{
    uint8_t b[14];
    int16_t raw[7];

    /* 一次把 加速度(6) + 温度(2) + 陀螺(6) = 14 字节全读出来。
     * 分两次读的话中间隔了几十微秒，等于加速度和角速度不同时刻采样，
     * 快速转动时会给姿态解算引入额外误差 */
    MPU_Read(REG_ACCEL_XOUT_H, b, 14);

    for (int k = 0; k < 7; k++)
    {
        raw[k] = (int16_t)((b[2 * k] << 8) | b[2 * k + 1]);
    }

    /* raw[0..2] = 加速度 X/Y/Z，raw[3] = 温度（不用），raw[4..6] = 陀螺 X/Y/Z */
    s_acc[0] = (float)raw[0] / ACCEL_LSB_PER_G * GRAVITY;
    s_acc[1] = (float)raw[1] / ACCEL_LSB_PER_G * GRAVITY;
    s_acc[2] = (float)raw[2] / ACCEL_LSB_PER_G * GRAVITY;

    /* 原始值 → °/s → rad/s */
    s_gyro[0] = (float)raw[4] / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    s_gyro[1] = (float)raw[5] / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    s_gyro[2] = (float)raw[6] / GYRO_LSB_PER_DPS * DEG_TO_RAD;

    /* 姿态解算用的是"扣了零偏"的角速度，否则积分会一直往一边漂 */
    Mahony_Update(s_gyro[0] - s_bias[0],
                  s_gyro[1] - s_bias[1],
                  s_gyro[2] - s_bias[2],
                  s_acc[0], s_acc[1], s_acc[2], CTRL_DT);
}

float   IMU_GetGyroX(void) { return s_gyro[0] - s_bias[0]; }
float   IMU_GetGyroY(void) { return s_gyro[1] - s_bias[1]; }
float   IMU_GetGyroZ(void) { return s_gyro[2] - s_bias[2]; }
uint8_t IMU_IsOnline(void) { return s_online; }

/**
 * @brief 俯仰角，rad。正值 = 机头（A 板 X 轴方向）抬起，0 = 水平
 *
 * 从四元数按 ZYX 欧拉角取：pitch = asin(2*(q0*q2 - q1*q3))
 * 和 IMU_AlignFromAccel() 里 atan2f(ax, az) 是同一个约定，可以互相印证。
 */
float IMU_GetPitch(void)
{
    float s = 2.0f * (s_q[0] * s_q[2] - s_q[3] * s_q[1]);

    /* 钳到 [-1,1]，否则 asinf 会返回 NaN。
     * NaN 一路传进 PID，再变成 int16 电压 → 电机瞬间满转 */
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;

    return asinf(s);
}
