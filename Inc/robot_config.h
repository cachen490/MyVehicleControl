#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ==================== 机械参数（换车就要重新量） ==================== */
#define WHEEL_RADIUS        0.076f          /* 麦轮半径，m。3508 麦轮一般 76mm */
#define HALF_TRACK_X        0.24f           /* 左右两轮中心距的一半，m */
#define HALF_TRACK_Y        0.24f           /* 前后两轮中心距的一半，m */
#define GEAR_RATIO_3508     (3591.0f / 187.0f)  /* 约 19.2，减速箱减速比 */
#define COS45               0.70710678f     /* 辊子 45 度，投影 = 1/√2 */

/* ==================== 周期与限幅 ==================== */
#define CTRL_DT             0.001f          /* 控制周期 1ms，单位 s */
#define CHASSIS_V_MAX       2.0f            /* 摇杆推满时的平移速度，m/s */
#define CHASSIS_W_MAX       6.0f            /* 拨轮推满时的自转速度，rad/s */
#define CHASSIS_CUR_MAX     8.0f            /* 单轮电流上限，A */

/* ==================== 3508 速度环（Step 3 调出来的值填进来） ==================== */
#define CHASSIS_SPEED_KP    0.35f
#define CHASSIS_SPEED_KI    0.05f

/* ==================== 云台 ==================== */
#define GIMBAL_RATE_MAX     6.0f            /* 摇杆推满时的转动角速度，rad/s */
#define GIMBAL_VOLT_MAX     20.0f           /* 6020 电压上限，V */
#define PITCH_ANGLE_MIN     (-0.35f)        /* 低头限位，rad */
#define PITCH_ANGLE_MAX     (0.70f)         /* 抬头限位，rad */

/* 陀螺仪角速度一阶低通系数 α。越小滤得越狠、相位滞后越大。
 * 0.30 对应截止频率约 57Hz（1kHz 采样）。电机啸叫就往下调 0.2，响应发木就往上调 0.5 */
#define GYRO_LPF_ALPHA      0.30f

/* 电机 ID：按 Step 1.0 实测结果填。改了这里，解码和下标会自动跟着走。
 * 6020 只能填 1 / 2：Motor_6020_SetVoltage() 只往 0x1FF 的前两个槽位写 */
#define ID_YAW_6020         1               /* yaw 轴 6020 的 ID */
#define ID_PITCH_6020       2               /* pitch 轴 6020 的 ID */

/* MPU6500 是否跟着 yaw 轴一起转（也就是 A 板装在云台随动部分上）。
 * 1 = 跟着转（这台车）：yaw / pitch 两轴都直接用陀螺仪角速度做反馈
 * 0 = 不跟着转：yaw 仍用 wz，但 pitch 只能用 6020 的转速反馈（见 Step 6.4.5）
 * 判别方法见 Step 6.4.4 的 10 秒测试，别靠猜 */
#define GYRO_ON_YAW_PLATFORM 1

/* ==================== 安全阈值 ==================== */
#define REMOTE_LOST_MS      50              /* 遥控 50ms 没新帧就算掉线 */
#define MOTOR_LOST_MS       50              /* 电机 50ms 没反馈就算掉线 */

/* ==================== 标定符号（装反了就改成 -1.0f） ==================== */
#define SIGN_FL             (+1.0f)         /* 左前轮 */
#define SIGN_FR             (+1.0f)         /* 右前轮 */
#define SIGN_RL             (+1.0f)         /* 左后轮 */
#define SIGN_RR             (+1.0f)         /* 右后轮 */
#define SIGN_GX             (+1.0f)         /* 陀螺 X 轴 */
#define SIGN_GY             (+1.0f)         /* 陀螺 Y 轴 */
#define SIGN_GZ             (+1.0f)         /* 陀螺 Z 轴 */
#define SIGN_YAW            (+1.0f)         /* 云台 Yaw 电机 */
#define SIGN_PIT            (+1.0f)         /* 云台 Pitch 电机 */

#endif /* ROBOT_CONFIG_H */
