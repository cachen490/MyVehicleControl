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
#define GIMBAL_RATE_MAX     6.0f            /* 摇杆推满时的转动角速度，rad/s（yaw 用） */
#define GIMBAL_PITCH_RATE   3.0f            /* pitch 摇杆推满时"目标角"的变化速度，rad/s */
#define GIMBAL_VOLT_MAX     20.0f           /* 6020 电压上限，V */
#define PITCH_ANGLE_MIN     (-0.35f)        /* 目标角下限（低头），rad = -20° */
#define PITCH_ANGLE_MAX     (0.70f)         /* 目标角上限（抬头），rad = +40° */

/* ==================== Mahony 姿态解算（响应速度 vs 抖动） ==================== */
/* 1kHz 采样下起调值。抖动/啸叫往下减，跟不住重力方向往上加 */
#define MAHONY_KP           0.50f
#define MAHONY_KI           0.005f          /* 顺手估陀螺零偏；给 0 也能跑 */

/* ==================== 云台 pitch 角度环（外环） ==================== */
/* 单位 rad/s 每 rad 误差：给 2.0 表示偏 10° 就要 20°/s 的修正速度 */
#define GIMBAL_PITCH_KP     2.0f

/* A 板装在 pitch 臂上时，陀螺哪根轴对应 pitch 转轴：0=X  1=Y  2=Z
 * 下面填 1 就是"取陀螺 Y 轴"，gimbal.c 里写死调 IMU_GetGyroY()；
 * 如果你实物是绕 X 转的，就要把 gimbal.c 里那处改成 IMU_GetGyroX()。
 * 验证方法：手抬 pitch 臂，看 IMU_GetGyroX/Y/Z 哪个变化最大 */
#define GYRO_PITCH_AXIS     1
#define SIGN_GYRO_PITCH     (+1.0f)         /* 该轴读数反了就改 -1 */

/* IMU_GetPitch() 的符号：手把云台往上抬时，这个值应该变大。
 * 变小就改 -1。它必须和 SIGN_GYRO_PITCH、SIGN_PIT 同号，
 * 否则 pitch 的速度内环会变成正反馈，一上电就往一边顶死 */
#define SIGN_PITCH_ABS      (+1.0f)

/* 陀螺仪角速度一阶低通系数 α。越小滤得越狠、相位滞后越大。
 * 0.30 对应截止频率约 57Hz（1kHz 采样）。电机啸叫就往下调 0.2，响应发木就往上调 0.5 */
#define GYRO_LPF_ALPHA      0.30f

/* 电机 ID：按 Step 1.0 实测结果填。改了这里，解码和下标会自动跟着走。
 * 6020 只能填 1 / 2：Motor_6020_SetVoltage() 只往 0x1FF 的前两个槽位写 */
#define ID_YAW_6020         1               /* yaw 轴 6020 的 ID */
#define ID_PITCH_6020       2               /* pitch 轴 6020 的 ID */

/* 【已废弃，保留仅供参考】MPU6500 是否跟着 yaw 轴一起转。
 * 改成 Mahony + pitch 角度环之后，gimbal.c 不再读这个宏。
 * 1 = 跟着转：yaw / pitch 两轴都直接用陀螺仪角速度做反馈
 * 0 = 不跟着转：yaw 仍用 wz，但 pitch 只能用 6020 的转速反馈（见文档 6.4.5） */
#define GYRO_ON_YAW_PLATFORM 1

/* ==================== 安全阈值 ==================== */
#define REMOTE_LOST_MS      50              /* 遥控 50ms 没新帧就算掉线 */
#define MOTOR_LOST_MS       50              /* 电机 50ms 没反馈就算掉线 */

/* ==================== 标定符号（装反了就改成 -1.0f） ==================== */
#define SIGN_FL             (+1.0f)         /* 左前轮 */
#define SIGN_FR             (+1.0f)         /* 右前轮 */
#define SIGN_RL             (+1.0f)         /* 左后轮 */
#define SIGN_RR             (+1.0f)         /* 右后轮 */
#define SIGN_GX             (+1.0f)         /* 陀螺 X 轴（旧方案的投影公式用，现不用） */
#define SIGN_GY             (+1.0f)         /* 陀螺 Y 轴（同上；pitch 现在用 SIGN_GYRO_PITCH） */
#define SIGN_GZ             (+1.0f)         /* 陀螺 Z 轴 */
#define SIGN_YAW            (+1.0f)         /* 云台 Yaw 电机 */
#define SIGN_PIT            (+1.0f)         /* 云台 Pitch 电机 */

#endif /* ROBOT_CONFIG_H */
