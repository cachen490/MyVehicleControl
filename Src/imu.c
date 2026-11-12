#include "imu.h"
#include "spi.h"
#include "robot_config.h"
#include <math.h>

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

#define GYRO_LSB_PER_DPS    16.4f
#define DEG_TO_RAD          0.017453293f

#define ACCEL_LSB_PER_G     4096.0f
#define GRAVITY             9.80665f

static float   s_gyro[3];
static float   s_acc[3];
static float   s_bias[3];
static float   s_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static float   s_gb[3] = { 0.0f, 0.0f, 0.0f };
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
/*
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

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        recip = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recip;
        ay *= recip;
        az *= recip;

        halfvx = s_q[1] * s_q[3] - s_q[0] * s_q[2];
        halfvy = s_q[0] * s_q[1] + s_q[2] * s_q[3];
        halfvz = s_q[0] * s_q[0] - 0.5f + s_q[3] * s_q[3];

        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        gx += MAHONY_KP * halfex;
        gy += MAHONY_KP * halfey;
        gz += MAHONY_KP * halfez;

        if (MAHONY_KI > 0.0f)
        {
            s_gb[0] += MAHONY_KI * halfex * dt;
            s_gb[1] += MAHONY_KI * halfey * dt;
            s_gb[2] += MAHONY_KI * halfez * dt;

            /* ★ 必须限幅。这个量名义上是"陀螺零偏"，真实零偏不会超过 0.02rad/s；
             *   但只要有持续的姿态误差(底盘加速被当成倾斜、A板支架被顶弯、加速度计有零偏)，
             *   它就会一直往一个方向累加、**没有上限**。累加到 0.05rad/s 就意味着
             *   解算出来的角度每秒漂 3°，云台会慢慢歪掉 —— 这正是"调着调着就
             *   越来越不水平、重启又好了"的那类灵异现象。
             *   0.05rad/s 已经比任何真实零偏大一个数量级，限在这里不会损失精度。 */
            for (int k = 0; k < 3; k++)
            {
                if (s_gb[k] >  0.05f) s_gb[k] =  0.05f;
                if (s_gb[k] < -0.05f) s_gb[k] = -0.05f;
            }

            gx += s_gb[0];
            gy += s_gb[1];
            gz += s_gb[2];
        }
    }

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

    recip = 1.0f / sqrtf(s_q[0] * s_q[0] + s_q[1] * s_q[1] +s_q[2] * s_q[2] + s_q[3] * s_q[3]);

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

    HAL_Delay(100);

    MPU_Write(REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    MPU_Write(REG_PWR_MGMT_1, 0x01);
    MPU_Write(REG_SMPLRT_DIV, 0x00);
    MPU_Write(REG_CONFIG, 0x03);
    MPU_Write(REG_GYRO_CONFIG, 0x18);
    MPU_Write(REG_ACCEL_CONFIG, 0x10);

    MPU_Read(REG_WHO_AM_I, &who, 1);

    s_online = (who == 0x00 || who == 0xFF) ? 0 : 1;

    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;

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
        IMU_Update();
        sum[0] += s_gyro[0];
        sum[1] += s_gyro[1];
        sum[2] += s_gyro[2];

        asum[0] += s_acc[0];
        asum[1] += s_acc[1];
        asum[2] += s_acc[2];

        HAL_Delay(1);
    }

    for (k = 0; k < 3; k++) s_bias[k] = sum[k] / 500.0f;

    IMU_AlignFromAccel(asum[0], asum[1], asum[2]);
}

void IMU_Update(void)
{
    uint8_t b[14];
    int16_t raw[7];

    MPU_Read(REG_ACCEL_XOUT_H, b, 14);

    for (int k = 0; k < 7; k++)
    {
        raw[k] = (int16_t)((b[2 * k] << 8) | b[2 * k + 1]);
    }

    s_acc[0] = (float)raw[0] / ACCEL_LSB_PER_G * GRAVITY;
    s_acc[1] = (float)raw[1] / ACCEL_LSB_PER_G * GRAVITY;
    s_acc[2] = (float)raw[2] / ACCEL_LSB_PER_G * GRAVITY;

    s_gyro[0] = (float)raw[4] / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    s_gyro[1] = (float)raw[5] / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    s_gyro[2] = (float)raw[6] / GYRO_LSB_PER_DPS * DEG_TO_RAD;

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

    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;

    return asinf(s);
}

/**
 * @brief 横滚角，rad。绕 A 板 X 轴的倾角，0 = 水平
 *
 * 从四元数按 ZYX 欧拉角取：roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1² + q2²))
 * 和 IMU_GetPitch() 是同一套约定，只是取的轴不同。
 *
 * ★★★ 为什么 pitch 环要用它：
 *   实测本机的云台 pitch 关节转轴对应 MPU 的 **roll 轴**（A 板相对机构转了 90°）。
 *   证据：把摆臂从下挡块推到上挡块，IMU_GetPitch() 只变化 0.0361 rad（2.07°），
 *   而机构真实行程是几十度 —— 说明 pitch 轴只吃到关节转角的残余投影。
 *   换成 roll 之后，读数变化量应当 ≈ 真实行程，这才是能闭环的反馈。
 *
 * ★ 用 atan2 而不是 asin：asin 在 ±90° 附近不可微，且这里的分母天然非负，
 *   atan2 版本没有额外代价，反而在姿态接近侧倾 90° 时更稳。
 */
float IMU_GetRoll(void)
{
    float y = 2.0f * (s_q[0] * s_q[1] + s_q[2] * s_q[3]);
    float x = 1.0f - 2.0f * (s_q[1] * s_q[1] + s_q[2] * s_q[2]);

    return atan2f(y, x);
}
