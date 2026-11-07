#include "imu.h"
#include "spi.h"

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

static float   s_gyro[3];       /* 原始读数（未扣零偏） */
static float   s_bias[3];       /* 零偏，IMU_CalibrateGyro() 里测出来 */
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
    MPU_Write(REG_ACCEL_CONFIG, 0x18);       /* 加速度计量程 ±16g（本工程不用，顺手配上） */

    MPU_Read(REG_WHO_AM_I, &who, 1);

    /* 正常应该是 0x70（MPU6500）。读到 0 或 0xFF 说明 SPI 根本没通 */
    s_online = (who == 0x00 || who == 0xFF) ? 0 : 1;

    /* 零偏先清零。这样即使不调 IMU_CalibrateGyro()，读数和以前一样 */
    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;
}

/**
 * @brief 静止标定：连续采 500 次取平均，当作零偏。
 *        必须在上电、车完全静止时调用，否则会把"正在搬动"当成零偏。
 */
void IMU_CalibrateGyro(void)
{
    float sum[3] = { 0.0f, 0.0f, 0.0f };
    int i, k;

    for (i = 0; i < 500; i++)
    {
        IMU_Update();                   /* 直接读原始值，此时 s_gyro 还没扣偏 */
        sum[0] += s_gyro[0];
        sum[1] += s_gyro[1];
        sum[2] += s_gyro[2];
        HAL_Delay(1);
    }

    for (k = 0; k < 3; k++) s_bias[k] = sum[k] / 500.0f;
}

void IMU_Update(void)
{
    uint8_t b[6];
    int16_t raw[3];

    MPU_Read(REG_ACCEL_XOUT_H + 8, b, 6);    /* 0x3B+8 = 0x43，陀螺 X/Y/Z 各 2 字节 */

    raw[0] = (int16_t)((b[0] << 8) | b[1]);
    raw[1] = (int16_t)((b[2] << 8) | b[3]);
    raw[2] = (int16_t)((b[4] << 8) | b[5]);

    /* 原始值 → °/s → rad/s */
    s_gyro[0] = (float)raw[0] / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    s_gyro[1] = (float)raw[1] / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    s_gyro[2] = (float)raw[2] / GYRO_LSB_PER_DPS * DEG_TO_RAD;

    s_online = 1;                            /* 能读到数据就算在线 */
}

float   IMU_GetGyroX(void) { return s_gyro[0] - s_bias[0]; }
float   IMU_GetGyroY(void) { return s_gyro[1] - s_bias[1]; }
float   IMU_GetGyroZ(void) { return s_gyro[2] - s_bias[2]; }
uint8_t IMU_IsOnline(void) { return s_online; }
