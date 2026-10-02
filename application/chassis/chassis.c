#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "buzzer.h"
#include "power_meter.h"
#include "bmi088_diag.h"
#include <limits.h>
#include <math.h>

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD

static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Subscriber_t *chassis_power_sub;         // 用于订阅底盘的反馈数据
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Power_Data_s chassis_power_recv;
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据


static DJIMotorInstance *motor_l, *motor_r; // left right forward back

static PIDInstance chassis_follow_to_yaw_pid;
static BuzzzerInstance *buzzerc;
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息
float v_debug=4000.0;

#define CHASSIS_MOTOR_LEFT_ID 1u
#define CHASSIS_MOTOR_RIGHT_ID 2u
#define DJI_ENCODER_COUNTS_PER_REV 8192.0f
#define CHASSIS_STRAIGHT_W_EPSILON 5.0f
#define CHASSIS_MOTION_REF_EPSILON 50.0f
#define CHASSIS_SYNC_GAIN_PER_S 1.2f
#define CHASSIS_SYNC_MAX_RATIO 0.06f
#define CHASSIS_SYNC_MAX_REF 500.0f
#define CHASSIS_STILL_SPEED_EPSILON 20.0f
#define CHASSIS_IMU_MAX_AGE_MS 80U

static uint8_t straight_tracking;
static int8_t straight_direction;
static int32_t straight_start_left;
static int32_t straight_start_right;
static uint8_t chassis_motors_stopped = 1U;
#if CHASSIS_USE_BMI088_YAW_HOLD
static uint8_t heading_tracking;
static int8_t heading_direction;
static float heading_ref_deg;
static float heading_i_term;
static float heading_last_error;
static float heading_last_correction;
static uint32_t heading_last_ms;
#endif

static uint8_t ChassisMotorsStill(void)
{
    return fabsf(motor_l->measure.speed_aps) < CHASSIS_STILL_SPEED_EPSILON &&
           fabsf(motor_r->measure.speed_aps) < CHASSIS_STILL_SPEED_EPSILON;
}

static int32_t ChassisMotorCount(const DJIMotorInstance *motor, int32_t direction)
{
    int64_t count = ((int64_t)motor->measure.total_round * 8192LL +
                     (int64_t)motor->measure.ecd) * direction;
    uint32_t wrapped = (uint32_t)count;

    if (wrapped <= (uint32_t)INT32_MAX)
        return (int32_t)wrapped;
    return (int32_t)((int64_t)wrapped - 4294967296LL);
}

static int32_t ChassisCountDelta(int32_t current, int32_t start)
{
    uint32_t difference = (uint32_t)current - (uint32_t)start;

    if (difference <= (uint32_t)INT32_MAX)
        return (int32_t)difference;
    return (int32_t)((int64_t)difference - 4294967296LL);
}

static float ChassisClamp(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static float ChassisWrapDeg(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static void ChassisReadWheelCounts(int32_t *left, int32_t *right)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *left = ChassisMotorCount(motor_l, -1);
    *right = ChassisMotorCount(motor_r, 1);
    if (primask == 0U)
        __enable_irq();
}

static void ChassisResetSpeedPid(PIDInstance *pid)
{
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->ERRORHandler.ERRORCount = 0U;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
}

static void ChassisResetWheelSpeedPids(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ChassisResetSpeedPid(&motor_l->motor_controller.speed_PID);
    ChassisResetSpeedPid(&motor_r->motor_controller.speed_PID);
    if (primask == 0U)
        __enable_irq();
}

static void ChassisResetStraightControl(void)
{
    straight_tracking = 0U;
    straight_direction = 0;
#if CHASSIS_USE_BMI088_YAW_HOLD
    heading_tracking = 0U;
    heading_direction = 0;
    heading_i_term = 0.0f;
    heading_last_error = 0.0f;
    heading_last_correction = 0.0f;
    heading_last_ms = 0U;
#endif
}

#if CHASSIS_USE_BMI088_YAW_HOLD
static float ChassisApplyHeadingAssist(float base_v, float base_w)
{
    int8_t direction;
    BMI088DiagState imu;
    float yaw_error;
    float correction;
    float correction_limit;
    float i_limit;
    float dt;
    uint32_t now;

    if (fabsf(base_v) < CHASSIS_MOTION_REF_EPSILON ||
        fabsf(base_w) > CHASSIS_STRAIGHT_W_EPSILON ||
        !BMI088DiagGetState(&imu) ||
        !(imu.status & BMI088_DIAG_YAW_VALID) ||
        HAL_GetTick() - imu.last_update_ms > CHASSIS_IMU_MAX_AGE_MS)
    {
        heading_tracking = 0U;
        heading_direction = 0;
        heading_i_term = 0.0f;
        heading_last_error = 0.0f;
        heading_last_correction = 0.0f;
        heading_last_ms = 0U;
        return base_w;
    }

    direction = base_v > 0.0f ? 1 : -1;
    if (!heading_tracking || direction != heading_direction)
    {
        heading_ref_deg = imu.yaw_deg;
        heading_direction = direction;
        heading_tracking = 1U;
        heading_i_term = 0.0f;
        heading_last_error = 0.0f;
        heading_last_correction = 0.0f;
        heading_last_ms = HAL_GetTick();
        return base_w;
    }

    yaw_error = ChassisWrapDeg(heading_ref_deg - imu.yaw_deg);
    if (fabsf(yaw_error) < CHASSIS_YAW_HOLD_DEADBAND)
        yaw_error = 0.0f;
    heading_last_error = yaw_error;

    correction_limit = fabsf(base_v) * CHASSIS_YAW_HOLD_MAX_RATIO;
    if (correction_limit > CHASSIS_YAW_HOLD_MAX_REF)
        correction_limit = CHASSIS_YAW_HOLD_MAX_REF;
    i_limit = fabsf(base_v) * CHASSIS_YAW_HOLD_I_MAX_RATIO;
    if (i_limit > CHASSIS_YAW_HOLD_I_MAX_REF)
        i_limit = CHASSIS_YAW_HOLD_I_MAX_REF;

    now = HAL_GetTick();
    dt = (float)(now - heading_last_ms) * 0.001f;
    heading_last_ms = now;
    if (dt <= 0.0f || dt > 0.1f || !isfinite(dt))
        dt = 0.01f;

    heading_i_term += yaw_error * CHASSIS_YAW_HOLD_KI * dt;
    heading_i_term = ChassisClamp(heading_i_term, i_limit);

    correction = CHASSIS_YAW_HOLD_DIR *
                 (CHASSIS_YAW_HOLD_KP * yaw_error +
                  heading_i_term -
                  CHASSIS_YAW_HOLD_KD * imu.gyro_z_lpf_dps);
    correction = ChassisClamp(correction, correction_limit);
    heading_last_correction = correction;

    return base_w + correction;
}
#endif

static void ChassisApplyStraightSync(float base_v, float base_w,
                                     float *left_ref, float *right_ref)
{
    int32_t left_count;
    int32_t right_count;
    int8_t direction;
    int32_t left_delta;
    int32_t right_delta;
    int64_t phase_error_count;
    float correction;
    float correction_limit;

    if (fabsf(base_v) < CHASSIS_MOTION_REF_EPSILON ||
        fabsf(base_w) > CHASSIS_STRAIGHT_W_EPSILON)
    {
        ChassisResetStraightControl();
        return;
    }

    direction = base_v > 0.0f ? 1 : -1;
    ChassisReadWheelCounts(&left_count, &right_count);
    if (!straight_tracking || direction != straight_direction)
    {
        straight_start_left = left_count;
        straight_start_right = right_count;
        straight_direction = direction;
        straight_tracking = 1U;
        return;
    }

    left_delta = ChassisCountDelta(left_count, straight_start_left);
    right_delta = ChassisCountDelta(right_count, straight_start_right);
    phase_error_count = (int64_t)right_delta - (int64_t)left_delta;
    correction = (float)phase_error_count *
        (360.0f / DJI_ENCODER_COUNTS_PER_REV) * CHASSIS_SYNC_GAIN_PER_S;

    correction_limit = fabsf(base_v) * CHASSIS_SYNC_MAX_RATIO;
    if (correction_limit > CHASSIS_SYNC_MAX_REF)
        correction_limit = CHASSIS_SYNC_MAX_REF;
    correction = ChassisClamp(correction, correction_limit);

    *left_ref += correction;
    *right_ref -= correction;
}

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = CHASSIS_MOTOR_LEFT_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 1000, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 10000,
                .MaxOut_ = -10000
            },
            .speed_PID = {
                .Kp = 0.6f,
                .Ki = 0.02f,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 2500,
                .MaxOut = 7000,
                .MaxOut_ = -7000
            },
            .current_PID = {
                .Kp = 0.5f,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_IMPROVE_NONE,
                .IntegralLimit = 0,
                .MaxOut = 10000,
                .MaxOut_ = -10000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            /* DJI C610/M2006 accepts a current command directly over CAN.
             * Keeping the generic software current loop here halves the usable
             * startup torque with the current PID below, so the speed loop is
             * the chassis actuator loop.
             */
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    // 实际电调 ID：左轮 1，右轮 2。两台电机共用 0x200，分别使用第 1、2 个电流槽位。
    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_LEFT_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_l = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_RIGHT_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_r = DJIMotorInit(&chassis_motor_config);

    ChassisResetStraightControl();
    chassis_motors_stopped = 1U;

    Buzzer_config_s buzzer = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .loudness = 0,
        .octave = OCTAVE_2,
    };
    buzzerc = BuzzerRegister(&buzzer);
#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

#endif // ONE_BOARD
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    float left_ref;
    float right_ref;
    float w_ref;
    uint8_t should_stop;

    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif

    should_stop = fabsf(chassis_cmd_recv.v) < CHASSIS_MOTION_REF_EPSILON &&
                  fabsf(chassis_cmd_recv.w) < CHASSIS_MOTION_REF_EPSILON;
#if CHASSIS_USE_BMI088_YAW_HOLD
    {
        BMI088DiagState imu_state;
        uint8_t bias_ready;
        uint8_t imu_stationary;

        BMI088DiagGetState(&imu_state);
        bias_ready = (imu_state.status & BMI088_DIAG_BIAS_VALID) != 0U;
        /* Startup bias calibration should not be blocked by encoder speed
         * noise while the host command is zero.  After bias is ready, keep the
         * stricter wheel-still gate for online bias tracking.
         */
        imu_stationary = should_stop && (!bias_ready || ChassisMotorsStill());
        BMI088DiagUpdate(imu_stationary);
    }
#endif

    if (should_stop)
    {
        if (!chassis_motors_stopped)
            ChassisResetWheelSpeedPids();
        chassis_motors_stopped = 1U;
        ChassisResetStraightControl();
        DJIMotorStop(motor_l);
        DJIMotorStop(motor_r);
    }
    else
    {
        if (chassis_motors_stopped)
            ChassisResetWheelSpeedPids();
        chassis_motors_stopped = 0U;
        DJIMotorEnable(motor_l);
        DJIMotorEnable(motor_r);
    }

    w_ref = chassis_cmd_recv.w;
#if CHASSIS_USE_BMI088_YAW_HOLD
    if (!should_stop)
        w_ref = ChassisApplyHeadingAssist(chassis_cmd_recv.v, chassis_cmd_recv.w);
#endif

    left_ref = chassis_cmd_recv.v - w_ref;
    right_ref = chassis_cmd_recv.v + w_ref;
    if (!should_stop)
    {
        ChassisApplyStraightSync(chassis_cmd_recv.v, chassis_cmd_recv.w,
                                 &left_ref, &right_ref);
    }

    DJIMotorSetRef(motor_l, left_ref);
    DJIMotorSetRef(motor_r, right_ref);
}

void ChassisGetHostFeedback(Chassis_Host_Feedback_s *feedback)
{
    const float degps_to_mps = PERIMETER_WHEEL /
        (1000.0f * 360.0f * REDUCTION_RATIO_WHEEL);
    uint32_t primask;

    if (feedback == NULL)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    feedback->left_total_count = ChassisMotorCount(motor_l, -1);
    feedback->right_total_count = ChassisMotorCount(motor_r, 1);
    feedback->target_left_mps = motor_l->motor_controller.pid_ref * degps_to_mps;
    feedback->target_right_mps = motor_r->motor_controller.pid_ref * degps_to_mps;
    feedback->measured_left_mps = -motor_l->measure.speed_aps * degps_to_mps;
    feedback->measured_right_mps = motor_r->measure.speed_aps * degps_to_mps;
    feedback->left_output = (int16_t)-motor_l->set_value;
    feedback->right_output = motor_r->set_value;
#if CHASSIS_USE_BMI088_YAW_HOLD
    feedback->heading_active = heading_tracking;
    feedback->heading_ref_deg = heading_ref_deg;
    feedback->heading_error_deg = heading_last_error;
    feedback->heading_correction_ref = heading_last_correction;
#else
    feedback->heading_active = 0U;
    feedback->heading_ref_deg = 0.0f;
    feedback->heading_error_deg = 0.0f;
    feedback->heading_correction_ref = 0.0f;
#endif
    if (primask == 0U)
        __enable_irq();
}
