#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ==================== 机械参数（换车就要重新量） ==================== */
#define WHEEL_RADIUS        0.076f          /* 麦轮半径，m。3508 麦轮一般 76mm */
#define HALF_TRACK_X        0.24f           /* 左右两轮中心距的一半，m */
#define HALF_TRACK_Y        0.24f           /* 前后两轮中心距的一半，m */
#define GEAR_RATIO_3508     (3591.0f / 187.0f)  /* 约 19.2，减速箱减速比 */

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

/* yaw / pitch 速度环的积分项。默认 0：纯 P 已经够用，积分调不好会慢慢来回荡。
 * 如果 yaw"松手后还以几度/分钟慢慢漂"，把 GIMBAL_RATE_KI 改成 0.02f 试试。
 * ★ i_max 是积分项的电压上限，必须是正数 —— 传 0 会把积分直接钳成 0，
 *   光改 ki 不改 i_max 是没有任何效果的 */
#define GIMBAL_RATE_KI      0.0f
#define GIMBAL_RATE_I_MAX   2.0f

/* ==================== Mahony 姿态解算（响应速度 vs 抖动） ==================== */
/* 1kHz 采样下起调值。抖动/啸叫往下减，跟不住重力方向往上加 */
#define MAHONY_KP           0.50f
#define MAHONY_KI           0.005f          /* 顺手估陀螺零偏；给 0 也能跑 */

/* ==================== 云台 pitch 角度环（外环） ==================== */
/* 单位 rad/s 每 rad 误差：给 2.0 表示偏 10° 就要 20°/s 的修正速度 */
#define GIMBAL_PITCH_KP     2.0f

/* ==================== 云台 yaw：锁大地 / 跟随底盘 ==================== */
/* 右拨杆在上 = 锁大地（云台不跟底盘转），在下 = 跟随底盘。
 * 两档共用同一个速度内环（反馈仍是陀螺 s_wz_f），区别只在"目标角速度"怎么来：
 *   锁大地 → 没有外环，目标角速度 = 摇杆 * GIMBAL_RATE_MAX
 *   跟随   → 加一个外环 P，反馈是 6020 编码器给的"云台相对底盘角" */

/* 相对角误差(rad) → 目标角速度(rad/s)。和 pitch 外环同量纲，先给 3.0。
 * 注意跟随是有稳态误差的：e = 车体角速度 / YAW_FOLLOW_KP，
 * 车体以 1 rad/s 转时偏 1/3 rad ≈ 19°。想减小先加内环 kp，别硬加这个 */
#define YAW_FOLLOW_KP       3.0f

/* 切进跟随时，目标角取哪个：
 * 0 = 取当前相对角 → 切过去原地接管，云台不跳变
 * 1 = 取车头方向   → 切过去会自己转回车头，这才是通常说的"底盘跟随"
 *
 * ★ 运行中这个值由【左拨杆】接管，不是这个宏：
 *   左拨杆上   → 1（转回车头）
 *   左拨杆中/下 → 0（原地接管）
 *   逻辑在 app.c 的 Robot_GetYawRealign()，每个控制周期都会写一次。
 *   这里的宏只是"遥控还没连上时"的兜底初值，正常跑起来就被覆盖了。
 *   想改默认行为，请改 app.c 里的档位映射，不要改这里。 */
#define YAW_FOLLOW_REALIGN  0

/* "车头方向"对应的云台相对角，rad。实测：手把云台转到正对车头，
 * 读 g_motor6020[0].ecd，乘 6.28318531/8192 填进来（ecd=2048 → 1.5708）
 *
 * ★★★ 0.0f 是占位值，不是实测值！在把它换成实测值之前，
 *      【不要把左拨杆打到上】—— 那时"转回车头"会停在一个跟车头无关的角度，
 *      而且松手后云台会一直朝那个错误方向转，看着像失控。
 *      上电默认（左拨杆在中位）是原地接管，不受这个值影响，可以放心调车。 */
#define YAW_ENC_HEAD        0.0f

/* yaw 编码器方向：手拨云台往 SIGN_YAW 的正方向转时 ecd 增大填 +1，减小填 -1。
 * ★ 填反的后果：外环变正反馈，一进跟随就往一边顶死 */
#define SIGN_ENC_YAW        (+1.0f)

/* 目标角速度的加速度上限，rad/s^2。防止模式切换、猛推摇杆时给内环一个阶跃。
 * 200 表示 0→6 rad/s 要约 30ms，手上察觉不到 */
#define YAW_RATE_SLEW       200.0f

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


/* ==================== 右拨杆三个档位的数值 ==================== */
/* DBUS 拨杆上报的是 1/2/3，但哪个数对应哪一档各家遥控器不一样。
 * 用上位机看 g_rc.s[0] 的实际值再填。含义见 app.c 的 Robot_GetMode()：
 *   上 = 锁大地（云台不跟底盘）   中 = 停车   下 = 跟随底盘
 * ★ 填错的安全方向是"什么都不动"：档位对不上就会被当成中位停车 */
#define SW_UP_VAL           1
#define SW_MID_VAL          2
#define SW_DOWN_VAL         3

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
