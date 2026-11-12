#include "gimbal.h"
#include "robot_config.h"
#include "motor_dji.h"
#include "imu.h"
#include "pid.h"

static PID_t s_pid_yaw;
static PID_t s_pid_pitch;

static float s_wz_f;            /* yaw 角速度（陀螺 Z，已滤波） */
static float s_wp_f;            /* pitch 关节角速度（陀螺 X，已滤波）
                                 * ★ 名字里的 p 是 pitch，不是"绕 Y"。
                                 *   2026-11-11 换轴后它取的是 IMU_GetGyroX()，
                                 *   因为角度反馈用的是 roll（绕 X）—— 角度轴和
                                 *   角速度轴必须是物理同一根，否则 D 项失效。 */

static float s_pitch_target;    /* 绝对角目标 rad（自稳模式下恒为 PITCH_ANGLE_CENTER） */
static float s_pitch_t_cmd;     /* 限速之后的实际目标角，喂给 PID 的是它 */

/* ★ 积分记忆（重力前馈）。停机时把"扛住重力那份电压"存这里，
 *   下次从停机切回运行时预载回积分，不用重走 τ = KP/KI 的爬升。 */
static float   s_pitch_i_hold;
static uint8_t s_pitch_active;  /* 上一拍 pitch 环有没有在跑（用来找上升沿/下降沿） */

#define TWO_PI  6.28318531f

static uint8_t s_yaw_follow;    /* 0 = 锁大地，1 = 跟随底盘 */
static float   s_yaw_rel_tgt;   /* 跟随模式下"云台相对底盘"的目标角，rad */
static float   s_yaw_t_cmd;     /* 限速之后的目标角速度，rad/s */

/**
 * @brief 云台相对底盘的角度，0 ~ 2π
 *
 * 用 ecd（单圈绝对值 0~8191）而不是 angle（上电起累计）：
 * ecd 由 6020 内部霍尔决定，跟"上电时云台停在哪"无关，每次开机都一致；
 * angle 每次上电都从 0 开始，做不了绝对的位置环。
 */
static float Yaw_RelAngle(void)
{
    return (float)g_motor6020[M_YAW].ecd * ECD_TO_RAD * SIGN_ENC_YAW;
}

/** @brief 把角度差折到 (-π, π]，否则经过 0/2π 交界时会突然跳 360° */
static float Angle_Wrap(float d)
{
    while (d >  TWO_PI * 0.5f) d -= TWO_PI;
    while (d < -TWO_PI * 0.5f) d += TWO_PI;
    return d;
}

/* ==================== 给底盘跟随环用的两个只读出口 ====================
 *
 * 为什么要有它们：底盘跟随云台需要两样东西 ——
 *   ① rel（云台相对底盘角），做**反馈**；
 *   ② ω_云台（云台相对大地的角速度），做**前馈**。
 * 这两样都在本模块里（一个是编码器算的，一个是陀螺滤出来的），
 * chassis.c 拿不到 static，只能从这里开口。
 *
 * ★★★ 两者必须是**同一个坐标**（都以 yaw_t 的正方向为正），否则
 *   前馈和反馈会互相打架：前馈说"底盘正转"，反馈说"底盘反转"，
 *   底盘就在那儿抖。这里能保证同号，是因为：
 *     • Yaw_RelAngle() 乘的 SIGN_ENC_YAW，定义就是"云台往 +yaw_t 方向转时 rel 增大"；
 *     • s_wz_f 是速度环的反馈，环要稳定就必须满足"目标为正 → 实际测到为正"，
 *       所以 s_wz_f 为正 ⇔ 云台在往 +yaw_t 方向转。
 *   两条定义指向同一个物理方向，所以可以直接相加减。
 *
 * ★ s_wz_f 只在 Gimbal_Task() 里刷新，而 app.c 是先跑底盘再跑云台，
 *   所以前馈用的是**上一毫秒**的角速度，滞后 1ms。对几 Hz 的跟随环没有影响。 */
float Gimbal_GetYawRel(void)
{
    return Yaw_RelAngle();
}

float Gimbal_GetYawRate(void)
{
    return s_wz_f;
}

/* ==================== yaw 机械限位用的坐标变换 ====================
 *
 * 这里有两条独立的坐标，别再混：
 *
 *   ① "相对角" Yaw_RelAngle() = ecd·(2π/8192)·SIGN_ENC_YAW
 *      乘过符号，给**闭环**用（外环反馈、目标）。
 *      ★ 它会随 SIGN_ENC_YAW 镜像 —— 符号填反了它整个翻过来。
 *
 *   ② "机械偏移" Yaw_MechOffset() = ecd·(2π/8192) − OFFSET
 *      **不乘任何符号**，给**限位**用。
 *      限位是挡块的位置，是机构属性，绝不会因为电机接反就跑到另一边去。
 *      行程内返回值 ∈ [0, YAW_MECH_SPAN_RAD]。
 *
 * ★ 前提：YAW_MECH_SPAN_RAD ≤ π。因为相对角被 Angle_Wrap 折进了 (-π, π]，
 *   跨度超过 π 就分不清两端了。
 *
 *   ★★ 本机实测跨度 184.09° = 3.2129 rad，**已经踩破这条前提**。后果分档：
 *      • YAW_FOLLOW_IMPL = 0（底盘追云台，当前用的）：**没有影响**。
 *        跟随目标恒在车头，离两端各 92°，最大相位差 92° < 180°，
 *        折角永远折不到反面。
 *      • YAW_FOLLOW_IMPL = 1（云台追底盘）：有一处退化 —— 目标贴着一侧挡块、
 *        实际贴着另一侧时，真实差 184° 会被折成 −176°，方向判断反了，
 *        云台会朝"更远的那一边"走。所以真要切到 impl 1，必须先换掉 rel 这条
 *        链路（改成 2π 展开的累计角），或者把行程限到 180° 以内。
 *   180° 正好卡在边界上时没问题；超过就属于"设计上不该出现"，这次是实测
 *   出来才发现，记在这里免得以后忘。
 */

/** @brief 云台当前机械角，相对"限位A"的偏移，归一化到 [0, 2π) */
static float Yaw_MechOffset(void)
{
    float m = (float)g_motor6020[M_YAW].ecd * ECD_TO_RAD - YAW_MECH_OFFSET_RAD;

    while (m <  0.0f)   m += TWO_PI;
    while (m >= TWO_PI) m -= TWO_PI;

    return m;
}

/** @brief 把"相对角"换回"机械偏移"。注意是乘 SIGN_ENC_YAW，不是除 —— 1/(±1)=±1 */
static float Yaw_RelToMechOff(float rel)
{
    float m = rel * SIGN_ENC_YAW - YAW_MECH_OFFSET_RAD;

    while (m <  0.0f)   m += TWO_PI;
    while (m >= TWO_PI) m -= TWO_PI;

    return m;
}

/** @brief 机械偏移是否落在允许行程内（含硬兜底余量）。
 *
 *  已归一到 [0,2π)，所以"比限位A 还靠前一点点"会表现为 2π 附近的巨大值，
 *  两头都得判，不能只判上界。 */
static uint8_t Yaw_MechInRange(float off)
{
    if (off <= YAW_MECH_SPAN_RAD + YAW_GUARD_MARGIN) return 1;
    if (off >= TWO_PI - YAW_GUARD_MARGIN)            return 1;
    return 0;
}

void Gimbal_SetYawFollow(uint8_t on)
{
    if (on && !s_yaw_follow)                 /* 只在"进跟随"的那一拍做事 */
    {
        /* 目标角取当前相对角：原地接管，云台不跳变。 */
        s_yaw_rel_tgt = Yaw_RelAngle();

        PID_Reset(&s_pid_yaw);               /* 别把上一个模式的积分带过来 */
    }

    s_yaw_follow = on;
}

void Gimbal_Init(void)
{
    /* ---- yaw 速度环：反馈是角速度(rad/s)、输出是电压(V)，*/
    PID_Init(&s_pid_yaw,   YAW_RATE_KP, GIMBAL_RATE_KI, 0.0f, GIMBAL_RATE_I_MAX, GIMBAL_VOLT_MAX);

    /* ---- pitch 单环：输出直接是电压，所以 kd 传 0 ----*/
    PID_Init(&s_pid_pitch, PITCH_KP, PITCH_KI, 0.0f, PITCH_I_MAX, GIMBAL_VOLT_MAX);

    s_wz_f = 0.0f;
    s_wp_f = 0.0f;

    s_pitch_target = PITCH_ANGLE_CENTER;  /* 自稳保持角，默认水平 */
    s_pitch_t_cmd  = PITCH_ANGLE_CENTER;

    /* 上电第一次进模式时没有可预载的东西，从 0 开始 —— 这一次还是得等 τ。
     * 但之后每一次切档 / 掉帧 / 断反馈再回来，都是"接着上次的力气"，不用再等。 */
    s_pitch_i_hold = 0.0f;
    s_pitch_active = 0;

    s_yaw_follow  = 0;
    s_yaw_rel_tgt = 0.0f;
    s_yaw_t_cmd   = 0.0f;
}

void Gimbal_Stop(void)
{
    /* ★★★ 积分记忆：停机前把"扛住重力那份电压"抄下来。
     *
     *   ★ 必须加 s_pitch_active 守卫。Gimbal_Stop() 在停车档里是**每 1ms 被调
     *     一次**的（app.c 的 MODE_STOP 分支），不加守卫的话第一拍存下 3.5V、
     *     第二拍 PID_Reset 已经把 i_term 清零了，于是把 0 覆盖进去 —— 记忆直接失效，
     *     而且现象是"偶尔生效偶尔不生效"，极难查。
     *
     *   ★ 存的东西就是重力前馈：稳态下积分项单独顶住的就是这份静力矩。 */
    if (s_pitch_active)
    {
        s_pitch_i_hold = s_pid_pitch.i_term;
        if (s_pitch_i_hold >  PITCH_I_MAX) s_pitch_i_hold =  PITCH_I_MAX;
        if (s_pitch_i_hold < -PITCH_I_MAX) s_pitch_i_hold = -PITCH_I_MAX;
        s_pitch_active = 0;
    }

    PID_Reset(&s_pid_yaw);
    PID_Reset(&s_pid_pitch);
    s_wz_f = 0.0f;
    s_wp_f = 0.0f;
    s_pitch_target = PITCH_ANGLE_CENTER;   /* 下次恢复先回中位角，别接着上次的角度冲过去 */
    s_pitch_t_cmd  = PITCH_ANGLE_CENTER;

    s_yaw_follow  = 0;          /* 掉线恢复后一律从"锁大地"重新开始， */
    s_yaw_t_cmd   = 0.0f;       /* 下次进跟随时会重新同步目标角，不会拿着旧目标暴冲 */

    Motor_6020_Stop();
}

/**
 * @brief 云台 1ms 任务
 *
 * yaw  : 两种模式，共用同一个速度环（反馈都是陀螺 Z 轴 s_wz_f）：
 *        锁大地 → 没有外环，目标角速度 = 摇杆，松杆为 0 = 相对大地不转；
 *        跟随   → 加一个外环 P，反馈是 6020 编码器给的"云台相对底盘角"。
 *        换模式只是换"目标从哪来"，内环一个字都不用改。
 * pitch: 单环 PID，**目标角恒定 = PITCH_ANGLE_CENTER（默认水平）**。
 *        不做摇杆控制，只做姿态保持：外力把臂推开 → pit_abs 偏离目标 →
 *        PID 出力把它拉回来。这就是"抗外力"。
 *        反馈用 Mahony 解出来的绝对角（相对大地，不是相对底盘），
 *        D 项直接取陀螺当角速度。
 *
 *        ★ 为什么不用参考工程那套"编码器位置锁定"：
 *          编码器锁的是"转子相对定子"，也就是云台相对底盘。底盘歪了云台跟着歪，
 *          那不叫自稳。要抗"底盘姿态变化"必须用 IMU 的绝对角。
 *          两者的抗外力手感可以调得一样，但坐标系完全不同。
 *
 * ★ 符号约束（完整推导见 robot_config.h 里"SIGN_GYRO_PITCH"上方那一段）：
 *
 *   【必须同号的一对】SIGN_GYRO_PITCH ≡ SIGN_PITCH_ABS
 *     这两个是同源量 —— 角度取 roll（IMU_GetRoll，四元数算出），角速度取
 *     gyroX，而四元数就是同一组 gx,gy,gz 积分出来的，所以 roll 和 gyroX 的
 *     符号必然一起翻。（换轴前的说法是 pitch 与 gyroY，结论同理。）
 *     填反的后果只有一个：P 环正常、D 环变**负阻尼** → 能站住但一碰就抖。
 *
 *   【独立的一个】SIGN_PIT
 *     它的正确值只取决于电机接线（P 环稳定要求 SIGN_PIT = sign(Kt)），
 *     跟 IMU 一点关系都没有 —— 但这不等于"可以随便取反"：
 *     ABS 一旦按定义填对了，PIT 就唯一确定了，填反照样顶死。
 *     所以千万别为了"凑三个同号"去动 PIT。
 *     ★ 2026-11-11 实测 ABS = GYRO = +1；PIT 要用 PITCH_OPENLOOP_TEST
 *       单独量（给已知电压看臂往哪走），不要从"往哪顶"反推。
 *
 *   这正是"顶死"和"抖动"两种故障的原因不同，不要把三者绑成一个符号：
 *     PIT 反 → 刚度项为负 → 单调发散，一上电就顶死；
 *     GYRO 反 → 阻尼项为负 → 能站住但一碰就抖，越加增益越糟。
 */
void Gimbal_Task(const Remote_t *rc)
{
    float v[2];
    float yaw_t, pit_abs;

    /* ================= yaw：两种模式共用一个速度环 ================= */
    s_wz_f += GYRO_LPF_ALPHA * (IMU_GetGyroZ() * SIGN_GZ - s_wz_f);

    if (!s_yaw_follow)
    {
        /* ---- 锁大地：没有外环。
         * 陀螺测的是惯性系角速度，所以目标给 0 等于"相对大地不转"。
         * 底盘怎么转都会被速度环当成扰动顶回去，这就是"不跟随"。---- */
        yaw_t = rc->rx * SIGN_RC_YAW_LOCK * GIMBAL_RATE_MAX;
    }
    else
    {
        /* ---- 跟随底盘：外环 P，反馈换成编码器的"相对底盘角" ----
         * 摇杆改的是"目标相对角"（和 pitch 外环同一个套路，不是改角速度） */
        float next = Angle_Wrap(s_yaw_rel_tgt
                                + rc->rx * SIGN_RC_YAW_FOLLOW * GIMBAL_RATE_MAX * CTRL_DT);

        /* ★ 硬限位（第一道）：目标角一旦越出行程就**冻住不动**，
         *   而不是夹到边界上。夹到边界会有黏滞感 —— 摇杆推到超界以后，
         *   往回拉的那一小段完全没反应（目标还贴在边界上），手感像卡住。
         *   冻住 = "超出的那部分增量直接丢弃"，往回拉立刻响应。 */
        {
            float off = Yaw_RelToMechOff(next);

            if (off <= YAW_MECH_SPAN_RAD - YAW_SOFT_MARGIN || off >= TWO_PI - YAW_SOFT_MARGIN)
            {
                s_yaw_rel_tgt = next;
            }
        }

        yaw_t = YAW_FOLLOW_KP * Angle_Wrap(s_yaw_rel_tgt - Yaw_RelAngle());

        /* 外环输出就是内环的目标角速度，必须限制在机构能做到的范围里，
         * 否则一个 90° 的偏差会算出 12 rad/s 的目标，内环一直顶在限幅上 */
        if (yaw_t >  GIMBAL_RATE_MAX) yaw_t =  GIMBAL_RATE_MAX;
        if (yaw_t < -GIMBAL_RATE_MAX) yaw_t = -GIMBAL_RATE_MAX;
    }

    /* 目标角速度限速：模式切换、猛推摇杆都会让上面的 yaw_t 阶跃，
     * 直接喂给内环会听到"咚"一下，也会让 6020 瞬间吃到很大的力矩 */
    {
        float step = YAW_RATE_SLEW * CTRL_DT;
        float dy   = yaw_t - s_yaw_t_cmd;

        if (dy >  step) dy =  step;
        if (dy < -step) dy = -step;

        s_yaw_t_cmd += dy;
    }

    /* ============ yaw 软限位（两种模式通用）============
     *
     * 只做一件事：**往挡块外面推的方向，直接不给转**；往里的一律放行。
     *
     * ★ 位置很关键：必须钳在【限速之后】的 s_yaw_t_cmd 上，不能钳上面的 yaw_t。
     *   钳 yaw_t 只是"不再加速"，而 s_yaw_t_cmd 已经从 6rad/s 掉到 0 要走
     *   YAW_RATE_SLEW = 200rad/s² 的限速，需要 30ms，这期间还要多转约 5° ——
     *   3° 的软限位余量根本兜不住。钳 cmd 才是当拍就停。
     *
     * 锁大地档为什么也要它：那个档位没有外环，云台相对底盘的角度是
     * "底盘转出来的"，控制层根本管不到目标。底盘一直往一个方向转，
     * 相对角迟早撞块 —— 这时唯一的救法就是检测到"已经在边界附近、
     * 而转速还朝外"就把转速归零，让速度环先把云台刹住。
     *
     * ★ 这个档位下云台**不能长时间单向锁大地**：180° 行程就是物理上限，
     *   底盘转够 90° 之后必然顶到挡块。代码只能保证"顶到时不出力"，
     *   不能凭空变出行程。真要锁大地转圈，得用能无限旋转的机构。
     *
     * ★ 方向判断靠 SIGN_YAW_MECH_DIR：机械角变大的方向对应 yaw_t 的正负。 */
    {
        float off = Yaw_MechOffset();
        /* ★ 两个"贴近边界"的判断必须**互斥**，否则会出大事：
         *   off 在 2π 附近（刚越过限位A 一点点）时，
         *   "off > SPAN − SOFT" 和 "off > 2π − SOFT" 会同时成立，
         *   于是两个 if 分别把正、负两个方向的转速都置 0 —— 云台彻底动不了。
         *   所以 at_pos 必须**同时**限定在上界附近（不超过 SPAN + 硬兜底余量）。
         *   更远的行程外区间由下面的硬兜底接管，不归软限位管。 */
        float at_pos = ((off >  YAW_MECH_SPAN_RAD - YAW_SOFT_MARGIN) &&
                        (off <= YAW_MECH_SPAN_RAD + YAW_GUARD_MARGIN)) ? 1.0f : 0.0f;
        float at_neg = ((off <  YAW_SOFT_MARGIN) ||
                        (off >  TWO_PI - YAW_SOFT_MARGIN)) ? 1.0f : 0.0f;
        if (YAW_LIMIT_ENABLE)
        {
            if (at_pos && (s_yaw_t_cmd * SIGN_YAW_MECH_DIR > 0.0f)) s_yaw_t_cmd = 0.0f;
            if (at_neg && (s_yaw_t_cmd * SIGN_YAW_MECH_DIR < 0.0f)) s_yaw_t_cmd = 0.0f;
        }
        /* 关掉限位时什么都不做。用 if 而不是 #if：常量条件会被优化掉，
         * 代码只有一份，Yaw_MechInRange 也不会报 unused 警告。 */
    }

    v[M_YAW] = PID_Calc(&s_pid_yaw, s_yaw_t_cmd, s_wz_f, CTRL_DT) * SIGN_YAW;

    /* ============ pitch：姿态保持（自稳 / 抗外力）============
     *
     * 两种取目标的来源，由 robot_config.h 的 PITCH_RC_ENABLE 二选一：
     *   0（当前）= 目标钉死在 PITCH_ANGLE_CENTER，摇杆不参与，纯抗外力；
     *   1         = 摇杆改"目标绝对角"，松杆回水平，新角度上继续自稳。
     * 无论哪种，反馈都是 IMU 的绝对角，抗外力这条环一直存在。
     *
     * ★ 目标恒定（当前模式）时，下面那段 PITCH_TGT_SLEW 限速是直通的
     *   （s_pitch_target 恒等于 s_pitch_t_cmd，d 恒为 0）。切到 1 之后它
     *   立刻恢复作用：那时松杆那一拍目标会阶跃，不限速云台会"咣"地弹回去。 */
#if PITCH_RC_ENABLE
    /* ---- 摇杆可调自稳方向（任务书第 2 条那个模式）----
     * 摇杆改的是"目标绝对角"，不是电压：松杆回到 CENTER（水平），
     * 推杆到一个新角度后**继续自稳**。所以抗外力一直在，只是基准会跟着杆走。
     * 绝对映射的好处：目标永远被夹在机构行程内，不会出现"目标攒到 170°、
     * 松手后一直顶着限位"。 */
    {
        float ry = rc->ry * SIGN_RC_PITCH;

        if (ry >= 0.0f)
            s_pitch_target = PITCH_ANGLE_CENTER + ry * (PITCH_ANGLE_MAX - PITCH_ANGLE_CENTER);
        else
            s_pitch_target = PITCH_ANGLE_CENTER + ry * (PITCH_ANGLE_CENTER - PITCH_ANGLE_MIN);
    }
#else
    /* ---- 纯自稳：目标钉死在 CENTER，摇杆完全不参与 ----
     * 外力推开多少、PID 就往回拉多少。要恢复摇杆控制，
     * 把 robot_config.h 的 PITCH_RC_ENABLE 改成 1 重新编译即可。 */
    s_pitch_target = PITCH_ANGLE_CENTER;
#endif

    if (s_pitch_target > PITCH_ANGLE_MAX) s_pitch_target = PITCH_ANGLE_MAX;
    if (s_pitch_target < PITCH_ANGLE_MIN) s_pitch_target = PITCH_ANGLE_MIN;

    /* 目标角限速。★ 自稳模式下目标恒定，这块是直通（d 恒为 0），
     * 留着是为了恢复摇杆控制时不用再写一遍：那时松手那一拍目标会阶跃，
     * 不限速云台会"咣"地弹回去。和上面 yaw 的 s_yaw_t_cmd 是同一个套路 */
    {
        float step = PITCH_TGT_SLEW * CTRL_DT;
        float d    = s_pitch_target - s_pitch_t_cmd;

        if (d >  step) d =  step;
        if (d < -step) d = -step;

        s_pitch_t_cmd += d;
    }

    /* 角度反馈。本机 A 板相对机构转了 90°，pitch 关节的转轴对应的是 MPU 的
     * **roll 轴**，所以取 IMU_GetRoll()，不是 IMU_GetPitch()。
     * （实测：摆臂走完全行程，IMU_GetPitch() 只变 2.07°，是残余投影不是反馈量。）
     * 减掉安装偏移，让 pit_abs = 0 真的对应"机构水平"。 */
    pit_abs = IMU_GetRoll() * SIGN_PITCH_ABS - PITCH_MOUNT_OFFSET;

    /* D 项直接用陀螺角速度，不用对 pit_abs 求差分（无微分噪声放大）。
     * ★ 角度取 roll（绕 X），角速度就必须取陀螺 X —— 必须是物理同一根轴，
     *   否则 D 项阻尼不了 P 环，现象是"能站住但一碰就抖"。 */
    s_wp_f += GYRO_LPF_ALPHA * (IMU_GetGyroX() * SIGN_GYRO_PITCH - s_wp_f);

    /* 积分记忆：停机→运行的那一拍，把上次扛重力的那份电压预载回积分。
     * 不预载的话，每次切档/掉帧都会重新爬一遍 τ = KP/KI，表现为"回中力时有时无"。 */
    if (!s_pitch_active)
    {
        s_pid_pitch.i_term = s_pitch_i_hold;
    }

    {
        float err  = s_pitch_t_cmd - pit_abs;
        float duty;
        float d_term;   /* D 项单独算出来、单独钳，理由见下面钳位那段 */

        /* pitch 的 PID_Init 里 kd 传的是 0，所以 PID_Calc 内部就是
         *     out = kp*err + i_term
         * 我们自己再算一遍这个式子，就能知道它内部那个钳位到底有没有生效 ——
         * 这是"反算抗饱和"能不能做对的前提。 */
        PID_Calc(&s_pid_pitch, s_pitch_t_cmd, pit_abs, CTRL_DT);

        {
            float pi = s_pid_pitch.kp * err + s_pid_pitch.i_term;

            if (pi > GIMBAL_VOLT_MAX || pi < -GIMBAL_VOLT_MAX)
            {
                /* ★ 反算抗饱和（back-calculation）：P+I 已经撞限幅，说明
                 *   再攒积分也变不成电压，只会变成"松开时的那一下猛冲"。
                 *   把积分**直接改写**成"刚好让输出等于限幅"的值：
                 *       i_term = 限幅 − kp·err
                 *   这样被机构堵住多久，积分都不会灌到 I_MAX 去。
                 *
                 * ★ 为什么 I_MAX 从 3.2 提到 10 之后必须要它：
                 *   旧值 3.2V 太小，本来也攒不出多大冲量；10V 不设防的话，
                 *   手按住臂 3 秒就能灌满，一松手臂会"咣"地甩。
                 *   代价：堵转时不再"憋力"，这是对的 —— 憋的那份力本来也用不上。 */
                float lim = (pi > 0.0f) ? GIMBAL_VOLT_MAX : -GIMBAL_VOLT_MAX;

                s_pid_pitch.i_term = lim - s_pid_pitch.kp * err;
                if (s_pid_pitch.i_term >  PITCH_I_MAX) s_pid_pitch.i_term =  PITCH_I_MAX;
                if (s_pid_pitch.i_term < -PITCH_I_MAX) s_pid_pitch.i_term = -PITCH_I_MAX;

                pi = lim;
            }

            /* D 项是在 PID_Calc 外面减的，它内部的钳位管不到，这里再钳一次：
             * 一个猛甩能让 D 单独把电压顶出上限 */
            /* ★★★ 2026-11-11：这里原来只有"总输出"钳位，没有"D 项自己"的钳位。
             *   实测 duty_pp=40V / sat_pct=62% 才暴露出来：KD=8 时
             *   D 项峰值 = 8 × 12.5 rad/s ≈ ±100V，是 ±20V 限幅的 5 倍 ——
             *   它一个人就把放大器推成继电器，P 和 I 完全没话语权。
             *   所以必须先单独钳 D，再钳总输出。顺序不能反。 */
            d_term = PITCH_KD * s_wp_f;
            if (d_term >  PITCH_D_MAX) d_term =  PITCH_D_MAX;
            if (d_term < -PITCH_D_MAX) d_term = -PITCH_D_MAX;

            duty = pi - d_term;

            if (duty >  GIMBAL_VOLT_MAX) duty =  GIMBAL_VOLT_MAX;
            if (duty < -GIMBAL_VOLT_MAX) duty = -GIMBAL_VOLT_MAX;
        }

        v[M_PITCH] = duty * SIGN_PIT;
    }

    /* ============ 安全兜底 ============
     * 防的是"实际角"跑飞：云台被手硬掰过行程、或者 Mahony 发散。
     *
     * ★★★ 余量必须用 PITCH_GUARD_MARGIN（1.5°），不能写死一个大的弧度值。
     *   原来这里是 0.25f = 14.3°，门限被抬到 ±34°，而机构行程只有 ±20° ——
     *   云台顶在限位上时实际角最多 20° 出头，**永远够不到门限**，
     *   于是电机就这么一直堵着出力发烫，保护一次都不会动作。
     *
     * ★ 触发时**不把目标贴到当前位置**（原来那句 s_pitch_t_cmd = pit_abs 已删）。
     *   贴当前角等于"放弃回水平、就地躺平"，那是跟随语义；
     *   自稳要求外力推超限后、一松手就自己拉回来，所以目标必须保持 CENTER。 */
    if (pit_abs < PITCH_ANGLE_MIN - PITCH_GUARD_MARGIN ||
        pit_abs > PITCH_ANGLE_MAX + PITCH_GUARD_MARGIN)
    {
        v[M_PITCH] = 0.0f;
        PID_Reset(&s_pid_pitch);
        s_pitch_t_cmd = PITCH_ANGLE_CENTER;
    }

    /* ============ yaw 硬兜底（最后一道）============
     * 软限位管的是"正常该转但不该往那边转"，这里管的是"已经到界外了"：
     * 挡块被顶变形、机构根本没装挡块、编码器读数异常。
     *
     * 越界就卸力，但不冻结目标 —— 一旦读数回到范围内，立刻恢复正常闭环，
     * 云台能自己走出来。这和 pitch 兜底的处理是同一个思路。
     *
     * ★★★ 这一段的"越界"是按 YAW_MECH_OFFSET/SPAN 算出来的，所以**参数填错
     *   就等于整条 yaw 被静默关掉**：云台手一拨就是松的、推杆毫无反应、
     *   而 pitch 照常工作。这个现象很好认 —— 它就是"另一段弧"（方案B）
     *   的典型症状，因为方案B 下大部分行程正好落在这个"界外"区间里。 */
    if (YAW_LIMIT_ENABLE && !Yaw_MechInRange(Yaw_MechOffset()))
    {
        v[M_YAW] = 0.0f;
        PID_Reset(&s_pid_yaw);
        s_yaw_t_cmd = 0.0f;
    }

    Motor_6020_SetVoltage(v);

    s_pitch_active = 1;   /* 必须在最后：前面任何一条提前 return 都不该被当成"跑过" */
}
