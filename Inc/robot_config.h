#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ==================== 角度单位 ==================== */
#define DEG2RAD             0.017453293f    /* 1° 对应的弧度 */

/* ==================== 机械参数（换车要重新量） ==================== */
#define WHEEL_RADIUS        0.076f          /* 麦轮半径，m */
#define HALF_TRACK_X        0.40f           /* 左右两轮中心距的一半，m */
#define HALF_TRACK_Y        0.43f           /* 前后两轮中心距的一半，m */
#define GEAR_RATIO_3508     (3591.0f / 187.0f)  /* 3508 减速比 ≈19.2 */

/* ==================== 周期与限幅 ==================== */
#define CTRL_DT             0.001f          /* 控制周期，s（1ms） */
#define CHASSIS_V_MAX       2.0f            /* 摇杆推满时的平移速度，m/s */
#define CHASSIS_W_MAX       1.2f            /* 拨轮推满时的自转速度，rad/s（手动/锁大地档） */
#define CHASSIS_CUR_MAX     8.0f            /* 单轮电流上限，A */

/* ==================== 3508 速度环 ==================== */
#define CHASSIS_SPEED_KP    0.35f           /* 速度环 P */
#define CHASSIS_SPEED_KI    0.05f           /* 速度环 I */

/* ==================== 底盘跟随云台 ==================== */
#define CHASSIS_FOLLOW_KP    5.0f           /* 位置误差(rad) → 底盘角速度(rad/s) */
#define CHASSIS_FOLLOW_KD    0.8f           /* 速度阻尼：(轮速-前馈)，稳态为 0 */
#define CHASSIS_FOLLOW_FF    1.0f           /* 前馈系数：1.0 = 完全按云台角速度补偿 */
#define CHASSIS_FOLLOW_KI      6.0f         /* 位置积分增益，(rad/s)/(rad·s) */
#define CHASSIS_FOLLOW_I_MAX   3.0f         /* 积分项绝对上限，rad/s */
#define CHASSIS_FOLLOW_I_RATE  2.5f         /* 积分上限 = 本值 × |云台转速指令| */
#define CHASSIS_FOLLOW_DB    (0.5f * DEG2RAD)        /* 位置误差死区，rad */
#define CHASSIS_FOLLOW_QUIET_W  0.05f       /* 静止闸门：误差在死区且前馈转速低于它 → 输出 0 */
#define CHASSIS_FOLLOW_SS_T   0.12f         /* 进档软起步时间，s（P/D 从 0 抬到 1） */
#define CHASSIS_FOLLOW_SS_SKIP  5.0f        /* 进档偏差小于此角(°)就跳过软起步 */
#define SIGN_CHASSIS_FOLLOW  (-1.0f)        /* 跟随方向，填错是正反馈 */
#define CHASSIS_FOLLOW_W_MAX  5.0f          /* 跟随环输出角速度上限，rad/s */

/* ==================== 云台：转速与电压 ==================== */
#define GIMBAL_RATE_MAX     6.0f            /* 锁大地档满杆 yaw 角速度，rad/s */
#define GIMBAL_YAW_RATE_FOLLOW 2.4f         /* 跟随档里云台自己的转速上限，rad/s */
#define GIMBAL_VOLT_MAX     20.0f           /* 6020 电压上限，V */

/* ---- pitch 行程与目标 ---- */
#define PITCH_ANGLE_CENTER  (0.0f * DEG2RAD)    /* 自稳保持的绝对角，0=水平 */
#define PITCH_MOUNT_OFFSET  (0.027f)            /* 安装零点 = 机构水平处的 IMU_GetRoll() */
#define SIGN_GYRO_PITCH     (+1.0f)             /* 陀螺 X 符号，必须与 SIGN_PITCH_ABS 同号 */
#define SIGN_PITCH_ABS      (+1.0f)             /* 角度反馈（IMU_GetRoll）符号 */
#define SIGN_PIT            (-1.0f)             /* pitch 电机符号，由接线决定 */

/* ---- pitch 单环 PID（输出直接是电压 V） ---- */
#define PITCH_KP            8.0f                /* P：角度 → 电压，V/rad */
#define PITCH_KI            12.0f               /* I：回中时间常数 τ = KP/KI = 2s */
#define PITCH_I_MAX         4.0f                /* 积分项电压上限，V（须 < KP×行程） */
#define PITCH_KD            1.5f                /* D：角速度 → 电压，V/(rad/s)，取陀螺 */
#define PITCH_D_MAX         5.0f                /* D 项电压硬限幅，V */

/* ---- yaw 摇杆方向 ---- */
#define SIGN_RC_YAW_LOCK    (-1.0f)             /* 锁大地档摇杆方向 */
#define SIGN_RC_VY          (-1.0f)             /* 左摇杆左右（平移 vy）方向 */

/* ---- yaw 速度内环 ---- */
#define YAW_RATE_KP         4.0f                /* P，V/(rad/s) */
#define GIMBAL_RATE_KI      3.0f                /* I，V/(rad·s) */
#define GIMBAL_RATE_I_MAX   4.0f                /* 积分电压上限，V（决定能锁住多快的底盘） */
#define GYRO_LPF_ALPHA      0.15f               /* 陀螺一阶低通系数 α（≈27Hz） */

/* ---- Mahony 姿态解算 ---- */
#define MAHONY_KP           0.50f               /* 姿态修正 P */
#define MAHONY_KI           0.005f              /* 顺手估陀螺零偏 */
#define MAHONY_ACC_TOL      1.0f                /* 加速度计可信门限，m/s² */

/* ---- yaw 机械限位（全部用 ecd 计数填，换算由下面的 _RAD 宏完成） ---- */
#define YAW_MECH_OFFSET_ECD   2349.0f           /* 限位A（ecd 小的那一侧） */
#define YAW_MECH_SPAN_ECD     4189.0f           /* 两侧挡块之间的跨度 = 184.09° */
#define YAW_ENC_HEAD_ECD      4444.0f           /* 车头方向的 ecd（行程中点） */
#define YAW_MECH_OFFSET_RAD   (YAW_MECH_OFFSET_ECD * ECD_TO_RAD)
#define YAW_MECH_SPAN_RAD     (YAW_MECH_SPAN_ECD   * ECD_TO_RAD)
#define YAW_SOFT_MARGIN       (10.0f * DEG2RAD) /* 软限位余量：离挡块这么远就不许再往外转 */
#define SIGN_ENC_YAW        (-1.0f)             /* yaw 编码器方向：ecd 增大填 +1 */
#define YAW_ENC_HEAD  (YAW_ENC_HEAD_ECD * ECD_TO_RAD * SIGN_ENC_YAW)    /* 车头对应的相对角，rad */
#define YAW_RATE_SLEW       600.0f              /* 目标角速度的加速度上限，rad/s² */

/* ---- yaw 锁大地位置环（速度环外面再套一个大地系目标角 θt） ---- */
#define YAW_LOCK_KP         6.0f                /* P，1/s */
#define YAW_LOCK_DB         (0.3f * DEG2RAD)    /* 回位死区，rad（就是回位残差下限） */
#define YAW_LOCK_KI         4.0f                /* I，1/s² */
#define YAW_LOCK_I_MAX      0.6f                /* 积分上限，rad/s */
#define YAW_LOCK_I_TAU      0.05f               /* 进死区后积分漏回 0 的时间常数，s */

/* ---- 堵转（被外力按住）时的积分保护 ---- */
#define YAW_STALL_CMD       0.3f    /* rad/s：指令超过它才算"想转但转不动" */
#define YAW_STALL_W         0.05f   /* rad/s：实际角速度低于它算没动 */
#define YAW_STALL_MS        150     /* 连续满足多少毫秒才判定 */
#define YAW_STALL_I_TAU     0.25f   /* 判定后积分漏回 0 的时间常数，s */

/* ---- 松手防"顺着摇杆方向继续走" ---- */
#define YAW_RELEASE_SNAP    1                   /* 摇杆回中那一拍把 θt 对齐到 θh */
#define YAW_TGT_LAG_MAX     (10.0f * DEG2RAD)   /* θt 超前量上限（单向，只在推杆侧） */

/* ---- yaw 限位墙（编码器域，速度规划式） ---- */
#define YAW_LIM_DB          (1.0f * DEG2RAD)    /* 限速死区，rad */
#define YAW_LIM_ACC         20.0f               /* 规划减速度，rad/s² */
#define YAW_EDGE_ENTER      (3.0f * DEG2RAD)    /* 余量小于它就切底盘系保持 */
#define YAW_EDGE_DB         (1.0f * DEG2RAD)    /* ②b 死区：决定停点位置 */
#define YAW_LIM_EXIT        (6.0f * DEG2RAD)    /* 余量回到这么多才切回锁大地（回差） */
#define YAW_LIM_DIR_FORCE   (-1)                /* 限位方向：+1/−1 编译期钉死（无在线自学习） */

/* ---- 大地角估计 θh ---- */
#define YAW_ANCHOR_ENABLE   1                   /* 底盘静止时用编码器锚定 θh */
#define YAW_ANCHOR_K        1.0f                /* 锚定增益，1/s */
#define YAW_ANCHOR_CH_W     0.05f               /* rad/s：底盘自转低于它算静止 */
#define YAW_ANCHOR_G_W      0.15f               /* rad/s：云台自转超过它暂停锚定 */
#define YAW_ANCHOR_JUMP     (11.0f * DEG2RAD)   /* 编码器单拍跳变门限，rad */

/* ==================== 电机 ID ==================== */
#define ID_YAW_6020         1               /* yaw 轴 6020 的 ID */
#define ID_PITCH_6020       2               /* pitch 轴 6020 的 ID */

/* ==================== 右拨杆三档 ==================== */
#define SW_UP_VAL           1               /* 上 = 锁大地 */
#define SW_DOWN_VAL         3               /* 下 = 跟随底盘 */

/* ==================== 安全阈值 ==================== */
#define REMOTE_LOST_MS      50              /* 遥控 50ms 没新帧算掉线 */
#define MOTOR_LOST_MS       50              /* 电机 50ms 没反馈算掉线 */

/* ==================== 轮子方向与轴符号 ==================== */
#define SIGN_FL             (+1.0f)         /* 左前轮 */
#define SIGN_FR             (-1.0f)         /* 右前轮 */
#define SIGN_RL             (+1.0f)         /* 左后轮 */
#define SIGN_RR             (-1.0f)         /* 右后轮 */
#define SIGN_GZ             (+1.0f)         /* 陀螺 Z 轴（yaw 速度环用） */
#define SIGN_YAW            (+1.0f)         /* 云台 yaw 电机 */

#endif /* ROBOT_CONFIG_H */
