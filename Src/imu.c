#include "imu.h"
#include "spi.h"
#include "robot_config.h"
#include <math.h>

#define MPU_CS_LOW()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_CS_HIGH()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

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

static float   s_gyro[3];       /* 陀螺原始角速度，rad/s */
static float   s_acc[3];        /* 加速度，m/s² */
static float   s_bias[3];       /* 陀螺零偏（上电静止 500 拍取均值） */
static float   s_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };   /* 姿态四元数 q0..q3 */
static float   s_gb[3] = { 0.0f, 0.0f, 0.0f };        /* Mahony 积分估出的零偏 */
static uint8_t s_online;        /* 1=WHO_AM_I 读到有效值 */

/* 写一个寄存器 */
static void MPU_Write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    MPU_CS_LOW();
    HAL_SPI_Transmit(&hspi5, tx, 2, 2);
    MPU_CS_HIGH();
}

/* 连续读寄存器，最高位置 1 表示读 */
static void MPU_Read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t addr = (uint8_t)(reg | 0x80);
    MPU_CS_LOW();
    HAL_SPI_Transmit(&hspi5, &addr, 1, 2);
    HAL_SPI_Receive(&hspi5, buf, len, 2);
    MPU_CS_HIGH();
}

/* Mahony 互补滤波：陀螺积分 + 加速度计修正，输出四元数 */
static void Mahony_Update(float gx, float gy, float gz,
                          float ax, float ay, float az, float dt)
{
    float recip;
    float anorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    anorm = sqrtf(ax * ax + ay * ay + az * az);

    /* 只在 |a| 接近 1g 时才信加速度计，挡住抬车/磕地的冲击 */
    if (fabsf(anorm - GRAVITY) < MAHONY_ACC_TOL)
    {
        recip = 1.0f / anorm;
        ax *= recip;
        ay *= recip;
        az *= recip;

        /* 由四元数推算重力方向（只取一半，后面统一乘 Kp） */
        halfvx = s_q[1] * s_q[3] - s_q[0] * s_q[2];
        halfvy = s_q[0] * s_q[1] + s_q[2] * s_q[3];
        halfvz = s_q[0] * s_q[0] - 0.5f + s_q[3] * s_q[3];

        /* 实测重力方向与推算方向的叉积 = 误差 */
        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        gx += MAHONY_KP * halfex;
        gy += MAHONY_KP * halfey;
        gz += MAHONY_KP * halfez;

        /* 积分项顺手估陀螺零偏 */
        s_gb[0] += MAHONY_KI * halfex * dt;
        s_gb[1] += MAHONY_KI * halfey * dt;
        s_gb[2] += MAHONY_KI * halfez * dt;

        for (int k = 0; k < 3; k++)
        {
            if (s_gb[k] >  0.05f) s_gb[k] =  0.05f;
            if (s_gb[k] < -0.05f) s_gb[k] = -0.05f;
        }

        gx += s_gb[0];
        gy += s_gb[1];
        gz += s_gb[2];
    }

    /* 一阶龙格库塔更新四元数 */
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

    /* 归一化 */
    recip = 1.0f / sqrtf(s_q[0] * s_q[0] + s_q[1] * s_q[1] +s_q[2] * s_q[2] + s_q[3] * s_q[3]);

    s_q[0] *= recip;
    s_q[1] *= recip;
    s_q[2] *= recip;
    s_q[3] *= recip;
}

/* 上电用加速度计直接定初始 roll/pitch（yaw 不可观测，取 0） */
static void IMU_AlignFromAccel(float ax, float ay, float az)
{
    float roll, pitch;
    float cr, sr, cp, sp;

    roll  = atan2f(-ay, sqrtf(ax * ax + az * az));
    pitch = atan2f( ax, az);

    cr = cosf(roll  * 0.5f);
    sr = sinf(roll  * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);

    s_q[0] =  cr * cp;
    s_q[1] =  sr * cp;
    s_q[2] =  cr * sp;
    s_q[3] = -sr * sp;
}

/* 复位 → 唤醒 → 配采样率/滤波/量程 → 读 WHO_AM_I 判在线 */
void IMU_Init(void)
{
    uint8_t who = 0;

    HAL_Delay(100);

    MPU_Write(REG_PWR_MGMT_1, 0x80);    /* 软复位 */
    HAL_Delay(100);

    MPU_Write(REG_PWR_MGMT_1, 0x01);    /* 用 X 轴陀螺时钟 */
    MPU_Write(REG_SMPLRT_DIV, 0x00);    /* 1kHz 不分频 */
    MPU_Write(REG_CONFIG, 0x03);        /* DLPF 44Hz */
    MPU_Write(REG_GYRO_CONFIG, 0x18);   /* ±2000dps */
    MPU_Write(REG_ACCEL_CONFIG, 0x10);  /* ±8g */

    MPU_Read(REG_WHO_AM_I, &who, 1);

    s_online = (who == 0x00 || who == 0xFF) ? 0 : 1;

    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;

    s_q[0] = 1.0f;
    s_q[1] = 0.0f;
    s_q[2] = 0.0f;
    s_q[3] = 0.0f;
    s_gb[0] = s_gb[1] = s_gb[2] = 0.0f;
}

/* 静止 500ms 取陀螺均值当零偏，同时用加速度计定初始姿态 */
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

/* 读 14 字节：加速度 + 温度 + 陀螺，转成物理量后喂 Mahony */
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
float   IMU_GetGyroZ(void) { return s_gyro[2] - s_bias[2]; }
uint8_t IMU_IsOnline(void) { return s_online; }

/* 四元数 → 欧拉角，单位 rad */
float IMU_GetRoll(void)
{
    float y = 2.0f * (s_q[0] * s_q[1] + s_q[2] * s_q[3]);
    float x = 1.0f - 2.0f * (s_q[1] * s_q[1] + s_q[2] * s_q[2]);

    return atan2f(y, x);
}

float IMU_GetYaw(void)
{
    float y = 2.0f * (s_q[0] * s_q[3] + s_q[1] * s_q[2]);
    float x = 1.0f - 2.0f * (s_q[2] * s_q[2] + s_q[3] * s_q[3]);

    return atan2f(y, x);
}
