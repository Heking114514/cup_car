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

#define CHASSIS_MOTOR_LEFT_ID 2u
#define CHASSIS_MOTOR_RIGHT_ID 3u
#define CHASSIS_STRAIGHT_W_EPSILON 5.0f
#define CHASSIS_MOTION_REF_EPSILON 50.0f
#define CHASSIS_HEADING_FUSION_TAU_S 1.5f
#define CHASSIS_STILL_SPEED_EPSILON 20.0f
#define CHASSIS_IMU_MAX_AGE_MS 80U
#define CHASSIS_WHEEL_REF_LIMIT 13000.0f
#define CHASSIS_INNER_WHEEL_MIN_REF 1000.0f
#define CHASSIS_STRAIGHT_LEFT_TRIM 1.0f
#define CHASSIS_STRAIGHT_RIGHT_TRIM 1.0f
#define CHASSIS_JLINK_TEST_ONLY 0U
#define CHASSIS_JLINK_OVERRIDE_REF_LIMIT 7000.0f
#define CHASSIS_JLINK_OVERRIDE_TIMEOUT_MS 800U
#define CHASSIS_LEFT_SPEED_KP 0.18f
#define CHASSIS_LEFT_SPEED_KI 0.12f
#define CHASSIS_LEFT_SPEED_MAX_OUT 1700.0f
#define CHASSIS_LEFT_SPEED_I_LIMIT 1200.0f
#define CHASSIS_RIGHT_SPEED_KP 0.22f
#define CHASSIS_RIGHT_SPEED_KI 0.06f
#define CHASSIS_RIGHT_SPEED_MAX_OUT 1500.0f
#define CHASSIS_RIGHT_SPEED_I_LIMIT 1000.0f
#define CHASSIS_FRICTION_START_SPEED_DPS 350.0f
#define CHASSIS_FRICTION_RESTART_DELAY_MS 80U
#define CHASSIS_FRICTION_START_DECAY_MS 200U
#define CHASSIS_LEFT_NEGATIVE_START_FF 650.0f
#define CHASSIS_LEFT_NEGATIVE_RUN_FF 180.0f
#define CHASSIS_LEFT_POSITIVE_START_FF 250.0f
#define CHASSIS_LEFT_POSITIVE_RUN_FF 180.0f
#define CHASSIS_RIGHT_NEGATIVE_START_FF 150.0f
#define CHASSIS_RIGHT_POSITIVE_START_FF 250.0f
#define CHASSIS_RIGHT_RUN_FF 140.0f
#define CHASSIS_START_REF_MIN 1800.0f
#define CHASSIS_START_SPEED_MIN 350.0f
#define CHASSIS_START_HOLD_REF 700.0f
#define CHASSIS_START_BOOST_REF 4300.0f
#define CHASSIS_START_BOOST_DELAY_MS 120U
#define CHASSIS_START_TIMEOUT_MS 2500U
#define CHASSIS_DRIVE_START_MISMATCH 0x01U
#define CHASSIS_DRIVE_START_FAULT 0x02U
#define CHASSIS_DRIVE_START_BOOST 0x04U

static uint8_t straight_tracking;
static int8_t straight_direction;
static int32_t straight_start_left;
static int32_t straight_start_right;
static float straight_last_phase_error_deg;
static float straight_last_correction;
static uint8_t chassis_motors_stopped = 1U;
static uint32_t wheel_start_wait_ms;
static uint8_t wheel_start_fault;
static uint8_t wheel_drive_status;
typedef struct
{
    int8_t direction;
    uint8_t started;
    uint32_t low_speed_start_ms;
    uint32_t started_time_ms;
} Chassis_Friction_State_s;
static Chassis_Friction_State_s left_friction_state;
static Chassis_Friction_State_s right_friction_state;
static float chassis_left_current_ff;
static float chassis_right_current_ff;
#if ROBOT_USE_REMOTE_TEST_MODE && CHASSIS_JLINK_TEST_ONLY
volatile uint32_t chassis_jlink_override_enable;
volatile float chassis_jlink_left_ref;
volatile float chassis_jlink_right_ref;
static uint32_t chassis_jlink_override_start_ms;
#endif
#if CHASSIS_USE_BMI088_YAW_HOLD
static uint8_t heading_tracking;
static int8_t heading_direction;
static float heading_ref_deg;
static float heading_i_term;
static float heading_fused_error_deg;
static float heading_imu_error_deg;
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

static float ChassisLimitMagnitude(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static void ChassisLimitInsideWheel(float base_v, float *turn_ref)
{
    float max_turn;
    float speed_abs = fabsf(base_v);

    /* For a pivot turn there is no inside wheel.  For a moving turn, keep
     * enough reference on the inside wheel for its speed loop to overcome
     * static friction instead of silently becoming a stopped wheel. */
    if (speed_abs < CHASSIS_INNER_WHEEL_MIN_REF ||
        fabsf(*turn_ref) < CHASSIS_STRAIGHT_W_EPSILON)
        return;

    max_turn = speed_abs - CHASSIS_INNER_WHEEL_MIN_REF;
    if (max_turn < 0.0f)
        max_turn = 0.0f;
    *turn_ref = ChassisClamp(*turn_ref, max_turn);
}

static void ChassisLimitWheelRefs(float *left_ref, float *right_ref)
{
    float left_abs = fabsf(*left_ref);
    float right_abs = fabsf(*right_ref);
    float max_abs = left_abs > right_abs ? left_abs : right_abs;
    float scale;

    if (max_abs <= CHASSIS_WHEEL_REF_LIMIT)
        return;

    scale = CHASSIS_WHEEL_REF_LIMIT / max_abs;
    *left_ref *= scale;
    *right_ref *= scale;
}

static float ChassisApplyMinimumMagnitude(float value, float minimum)
{
    if (value > 0.0f && value < minimum)
        return minimum;
    if (value < 0.0f && value > -minimum)
        return -minimum;
    return value;
}

static float ChassisFrictionMagnitude(float ref, float speed,
                                      float negative_start,
                                      float negative_run,
                                      float positive_start,
                                      float positive_run,
                                      Chassis_Friction_State_s *state)
{
    int8_t direction;
    uint32_t now;

    if (fabsf(ref) < CHASSIS_MOTION_REF_EPSILON)
    {
        memset(state, 0, sizeof(*state));
        return 0.0f;
    }

    direction = ref > 0.0f ? 1 : -1;
    now = HAL_GetTick();
    if (state->direction != direction)
    {
        state->direction = direction;
        state->started = 0U;
        state->low_speed_start_ms = now;
    }

    if (speed * ref > 0.0f &&
        fabsf(speed) >= CHASSIS_FRICTION_START_SPEED_DPS)
    {
        state->started = 1U;
        state->low_speed_start_ms = 0U;
    }
    else if (state->started)
    {
        if (state->low_speed_start_ms == 0U)
            state->low_speed_start_ms = now;
        else if (now - state->low_speed_start_ms >=
                 CHASSIS_FRICTION_RESTART_DELAY_MS)
            state->started = 0U;
    }

    if (direction < 0)
        return state->started ? negative_run : negative_start;
    return state->started ? positive_run : positive_start;
}

static void ChassisUpdateFrictionCompensation(float left_ref, float right_ref,
                                               uint8_t stopped)
{
    float left_magnitude;
    float right_magnitude;

    if (stopped)
    {
        memset(&left_friction_state, 0, sizeof(left_friction_state));
        memset(&right_friction_state, 0, sizeof(right_friction_state));
        chassis_left_current_ff = 0.0f;
        chassis_right_current_ff = 0.0f;
        return;
    }

    left_magnitude = ChassisFrictionMagnitude(
        left_ref, -motor_l->measure.speed_aps,
        CHASSIS_LEFT_NEGATIVE_START_FF, CHASSIS_LEFT_NEGATIVE_RUN_FF,
        CHASSIS_LEFT_POSITIVE_START_FF, CHASSIS_LEFT_POSITIVE_RUN_FF,
        &left_friction_state);
    right_magnitude = ChassisFrictionMagnitude(
        right_ref, motor_r->measure.speed_aps,
        CHASSIS_RIGHT_NEGATIVE_START_FF, CHASSIS_RIGHT_RUN_FF,
        CHASSIS_RIGHT_POSITIVE_START_FF, CHASSIS_RIGHT_RUN_FF,
        &right_friction_state);

    /* Feedforward is expressed in each motor's raw electrical direction. */
    chassis_left_current_ff = left_ref > 0.0f
        ? -left_magnitude
        : (left_ref < 0.0f ? left_magnitude : 0.0f);
    chassis_right_current_ff = right_ref > 0.0f
        ? right_magnitude
        : (right_ref < 0.0f ? -right_magnitude : 0.0f);
}

static uint8_t ChassisApplyWheelStartSafety(float base_v, float base_w,
                                            uint8_t command_stopped,
                                            float *left_ref, float *right_ref)
{
    float left_speed = -motor_l->measure.speed_aps;
    float right_speed = motor_r->measure.speed_aps;
    uint8_t left_started;
    uint8_t right_started;
    uint32_t now;

    if (command_stopped)
    {
        wheel_start_wait_ms = 0U;
        wheel_start_fault = 0U;
        wheel_drive_status = 0U;
        return 0U;
    }

    if (wheel_start_fault)
    {
        wheel_drive_status = CHASSIS_DRIVE_START_FAULT;
        *left_ref = 0.0f;
        *right_ref = 0.0f;
        return 1U;
    }

    if (fabsf(base_v) < CHASSIS_START_REF_MIN ||
        fabsf(base_w) > CHASSIS_STRAIGHT_W_EPSILON ||
        *left_ref * *right_ref <= 0.0f ||
        fabsf(*left_ref) < CHASSIS_START_REF_MIN ||
        fabsf(*right_ref) < CHASSIS_START_REF_MIN)
    {
        wheel_start_wait_ms = 0U;
        wheel_drive_status = 0U;
        return 0U;
    }

    left_started = left_speed * *left_ref > 0.0f &&
                   fabsf(left_speed) >= CHASSIS_START_SPEED_MIN;
    right_started = right_speed * *right_ref > 0.0f &&
                    fabsf(right_speed) >= CHASSIS_START_SPEED_MIN;
    if (left_started && right_started)
    {
        wheel_start_wait_ms = 0U;
        wheel_drive_status = 0U;
        return 0U;
    }

    now = HAL_GetTick();
    if (wheel_start_wait_ms == 0U)
        wheel_start_wait_ms = now;
    wheel_drive_status = CHASSIS_DRIVE_START_MISMATCH;

    if (now - wheel_start_wait_ms >= CHASSIS_START_BOOST_DELAY_MS)
    {
        wheel_drive_status |= CHASSIS_DRIVE_START_BOOST;
        if (!left_started)
            *left_ref = ChassisApplyMinimumMagnitude(
                *left_ref, CHASSIS_START_BOOST_REF);
        if (!right_started)
            *right_ref = ChassisApplyMinimumMagnitude(
                *right_ref, CHASSIS_START_BOOST_REF);
    }

    if (left_started && !right_started)
        *left_ref = ChassisLimitMagnitude(*left_ref, CHASSIS_START_HOLD_REF);
    else if (right_started && !left_started)
        *right_ref = ChassisLimitMagnitude(*right_ref, CHASSIS_START_HOLD_REF);

    if (now - wheel_start_wait_ms < CHASSIS_START_TIMEOUT_MS)
        return 0U;

    wheel_start_fault = 1U;
    wheel_drive_status = CHASSIS_DRIVE_START_FAULT;
    *left_ref = 0.0f;
    *right_ref = 0.0f;
    return 1U;
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
    /* Rebase the controller clock as well as its state.  Otherwise a debugger
     * halt or a long stop interval is integrated as one enormous control step. */
    pid->DWT_CNT = DWT->CYCCNT;
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
    pid->dt = 0.0f;
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
    straight_last_phase_error_deg = 0.0f;
    straight_last_correction = 0.0f;
#if CHASSIS_USE_BMI088_YAW_HOLD
    heading_tracking = 0U;
    heading_direction = 0;
    heading_i_term = 0.0f;
    heading_fused_error_deg = 0.0f;
    heading_last_error = 0.0f;
    heading_last_correction = 0.0f;
    heading_last_ms = 0U;
#endif
}

static void ChassisSetWheelSpeedPidConfig(Motor_Init_Config_s *config,
                                          float kp, float ki,
                                          float max_out, float integral_limit)
{
    config->controller_param_init_config.speed_PID.Kp = kp;
    config->controller_param_init_config.speed_PID.Ki = ki;
    config->controller_param_init_config.speed_PID.Improve =
        PID_Integral_Limit | PID_Trapezoid_Intergral;
    config->controller_param_init_config.speed_PID.IntegralLimit = integral_limit;
    config->controller_param_init_config.speed_PID.MaxOut = max_out;
    config->controller_param_init_config.speed_PID.MaxOut_ = -max_out;
}

#if CHASSIS_SPEED_LOOP_TUNING_MODE
static int16_t ChassisJLinkTuneToInt16(float value)
{
    if (value >= (float)INT16_MAX)
        return INT16_MAX;
    if (value <= (float)INT16_MIN)
        return INT16_MIN;
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static uint16_t ChassisJLinkTuneAge(uint32_t now, const DJIMotorInstance *motor)
{
    uint32_t age = motor->feedback_count != 0U
        ? now - motor->last_feedback_ms
        : UINT32_MAX;

    return age > UINT16_MAX ? UINT16_MAX : (uint16_t)age;
}

static uint32_t ChassisJLinkTuneDtUs(float dt)
{
    double dt_us;

    if (!isfinite(dt) || dt <= 0.0f)
        return 0U;
    dt_us = (double)dt * 1000000.0;
    if (dt_us >= (double)UINT32_MAX)
        return UINT32_MAX;
    return (uint32_t)(dt_us + 0.5);
}

static uint8_t ChassisJLinkTuneMotorOnline(uint32_t now,
                                           const DJIMotorInstance *motor)
{
    return motor->feedback_count != 0U &&
           now - motor->last_feedback_ms <=
               CHASSIS_JLINK_TUNE_FEEDBACK_TIMEOUT_MS &&
           DaemonIsOnline(motor->daemon);
}

static void ChassisJLinkTuneFinish(uint8_t state, uint8_t reason)
{
    CAN_Diagnostic_s can_diagnostic;

    CANGetDiagnostic(&hcan1, &can_diagnostic);
    chassis_jlink_tune_active = 0U;
    chassis_jlink_tune_capture.end_time_ms = HAL_GetTick();
    chassis_jlink_tune_capture.left_feedback_end = motor_l->feedback_count;
    chassis_jlink_tune_capture.right_feedback_end = motor_r->feedback_count;
    chassis_jlink_tune_capture.can_tx_failure_end =
        can_diagnostic.tx_failure_count;
    chassis_jlink_tune_capture.can_hal_error_code =
        can_diagnostic.hal_error_code;
    chassis_jlink_tune_capture.can_protocol_last_error_code =
        can_diagnostic.protocol_last_error_code;
    chassis_jlink_tune_capture.can_bus_off = can_diagnostic.bus_off;
    chassis_jlink_tune_capture.can_error_passive =
        can_diagnostic.error_passive;
    chassis_jlink_tune_capture.stop_reason = reason;
    chassis_jlink_tune_capture.state = state;
}

static void ChassisJLinkTuneReject(uint32_t request_seq, uint8_t reason)
{
    uint32_t capture_seq = chassis_jlink_tune_capture.capture_seq + 1U;

    memset((void *)&chassis_jlink_tune_capture, 0,
           sizeof(chassis_jlink_tune_capture));
    chassis_jlink_tune_capture.capture_seq = capture_seq;
    chassis_jlink_tune_capture.request_seq = request_seq;
    chassis_jlink_tune_capture.sample_capacity =
        CHASSIS_JLINK_TUNE_SAMPLE_CAPACITY;
    chassis_jlink_tune_capture.sample_period_ms =
        CHASSIS_JLINK_TUNE_SAMPLE_PERIOD_MS;
    chassis_jlink_tune_capture.state = CHASSIS_JLINK_TUNE_REJECTED;
    chassis_jlink_tune_capture.stop_reason = reason;
    chassis_jlink_tune_command.arm_key = 0U;
    chassis_jlink_tune_active = 0U;
}

static void ChassisJLinkTuneStart(uint32_t request_seq,
                                  int32_t left_ref, int32_t right_ref,
                                  uint32_t duration_ms,
                                  int32_t left_kp_milli,
                                  int32_t left_ki_milli,
                                  int32_t right_kp_milli,
                                  int32_t right_ki_milli,
                                  uint32_t control_mode,
                                  int32_t left_current_kp_milli,
                                  int32_t right_current_kp_milli)
{
    CAN_Diagnostic_s can_diagnostic;
    uint32_t capture_seq = chassis_jlink_tune_capture.capture_seq + 1U;

    ChassisResetWheelSpeedPids();
    motor_l->motor_controller.speed_PID.Kp = (float)left_kp_milli * 0.001f;
    motor_l->motor_controller.speed_PID.Ki = (float)left_ki_milli * 0.001f;
    motor_r->motor_controller.speed_PID.Kp = (float)right_kp_milli * 0.001f;
    motor_r->motor_controller.speed_PID.Ki = (float)right_ki_milli * 0.001f;
    motor_l->motor_controller.current_PID.Kp =
        (float)left_current_kp_milli * 0.001f;
    motor_r->motor_controller.current_PID.Kp =
        (float)right_current_kp_milli * 0.001f;
    motor_l->motor_controller.current_PID.Ki = 0.0f;
    motor_r->motor_controller.current_PID.Ki = 0.0f;
    motor_l->motor_controller.current_PID.MaxOut =
        CHASSIS_LEFT_SPEED_MAX_OUT;
    motor_l->motor_controller.current_PID.MaxOut_ =
        -CHASSIS_LEFT_SPEED_MAX_OUT;
    motor_r->motor_controller.current_PID.MaxOut =
        CHASSIS_RIGHT_SPEED_MAX_OUT;
    motor_r->motor_controller.current_PID.MaxOut_ =
        -CHASSIS_RIGHT_SPEED_MAX_OUT;
    motor_l->motor_settings.close_loop_type =
        control_mode == CHASSIS_JLINK_TUNE_SPEED_CURRENT
            ? SPEED_AND_CURRENT_LOOP
            : SPEED_LOOP;
    motor_r->motor_settings.close_loop_type =
        control_mode == CHASSIS_JLINK_TUNE_SPEED_CURRENT
            ? SPEED_AND_CURRENT_LOOP
            : SPEED_LOOP;

    CANGetDiagnostic(&hcan1, &can_diagnostic);
    memset((void *)&chassis_jlink_tune_capture, 0,
           sizeof(chassis_jlink_tune_capture));
    chassis_jlink_tune_capture.capture_seq = capture_seq;
    chassis_jlink_tune_capture.request_seq = request_seq;
    chassis_jlink_tune_capture.start_time_ms = HAL_GetTick();
    chassis_jlink_tune_capture.requested_duration_ms = duration_ms;
    chassis_jlink_tune_capture.control_mode = control_mode;
    chassis_jlink_tune_capture.left_ref_dps = left_ref;
    chassis_jlink_tune_capture.right_ref_dps = right_ref;
    chassis_jlink_tune_capture.left_kp_milli = left_kp_milli;
    chassis_jlink_tune_capture.left_ki_milli = left_ki_milli;
    chassis_jlink_tune_capture.right_kp_milli = right_kp_milli;
    chassis_jlink_tune_capture.right_ki_milli = right_ki_milli;
    chassis_jlink_tune_capture.left_current_kp_milli =
        left_current_kp_milli;
    chassis_jlink_tune_capture.right_current_kp_milli =
        right_current_kp_milli;
    chassis_jlink_tune_capture.left_feedback_start = motor_l->feedback_count;
    chassis_jlink_tune_capture.right_feedback_start = motor_r->feedback_count;
    chassis_jlink_tune_capture.can_tx_failure_start =
        can_diagnostic.tx_failure_count;
    chassis_jlink_tune_capture.sample_capacity =
        CHASSIS_JLINK_TUNE_SAMPLE_CAPACITY;
    chassis_jlink_tune_capture.sample_period_ms =
        CHASSIS_JLINK_TUNE_SAMPLE_PERIOD_MS;
    chassis_jlink_tune_capture.state = CHASSIS_JLINK_TUNE_ACTIVE;
    chassis_jlink_tune_capture.stop_reason = CHASSIS_JLINK_TUNE_STOP_NONE;

    chassis_jlink_tune_start_ms = chassis_jlink_tune_capture.start_time_ms;
    chassis_jlink_tune_last_sample_ms = chassis_jlink_tune_start_ms -
        CHASSIS_JLINK_TUNE_SAMPLE_PERIOD_MS;
    chassis_jlink_tune_active = 1U;
    chassis_jlink_tune_command.arm_key = 0U;
}

static uint8_t ChassisJLinkTuneUpdate(float *left_ref, float *right_ref)
{
    CAN_Diagnostic_s can_diagnostic;
    uint32_t request_seq = chassis_jlink_tune_command.request_seq;
    uint32_t now = HAL_GetTick();

    if (request_seq != 0U &&
        request_seq != chassis_jlink_tune_last_request_seq)
    {
        uint32_t arm_key = chassis_jlink_tune_command.arm_key;
        int32_t requested_left = chassis_jlink_tune_command.left_ref_dps;
        int32_t requested_right = chassis_jlink_tune_command.right_ref_dps;
        uint32_t duration_ms = chassis_jlink_tune_command.duration_ms;
        int32_t left_kp_milli = chassis_jlink_tune_command.left_kp_milli;
        int32_t left_ki_milli = chassis_jlink_tune_command.left_ki_milli;
        int32_t right_kp_milli = chassis_jlink_tune_command.right_kp_milli;
        int32_t right_ki_milli = chassis_jlink_tune_command.right_ki_milli;
        uint32_t control_mode = chassis_jlink_tune_command.control_mode;
        int32_t left_current_kp_milli =
            chassis_jlink_tune_command.left_current_kp_milli;
        int32_t right_current_kp_milli =
            chassis_jlink_tune_command.right_current_kp_milli;
        uint8_t left_online = ChassisJLinkTuneMotorOnline(now, motor_l);
        uint8_t right_online = ChassisJLinkTuneMotorOnline(now, motor_r);

        chassis_jlink_tune_last_request_seq = request_seq;
        if (arm_key != CHASSIS_JLINK_TUNE_ARM_KEY)
        {
            ChassisJLinkTuneReject(request_seq,
                                   CHASSIS_JLINK_TUNE_STOP_BAD_KEY);
        }
        else if (requested_left == 0 && requested_right == 0)
        {
            if (chassis_jlink_tune_active)
                ChassisJLinkTuneFinish(CHASSIS_JLINK_TUNE_COMPLETE,
                                       CHASSIS_JLINK_TUNE_STOP_HOST);
            chassis_jlink_tune_command.arm_key = 0U;
        }
        else if (duration_ms < CHASSIS_JLINK_TUNE_MIN_DURATION_MS ||
                 duration_ms > CHASSIS_JLINK_TUNE_MAX_DURATION_MS ||
                 requested_left > CHASSIS_JLINK_TUNE_MAX_REF_DPS ||
                 requested_left < -CHASSIS_JLINK_TUNE_MAX_REF_DPS ||
                 requested_right > CHASSIS_JLINK_TUNE_MAX_REF_DPS ||
                 requested_right < -CHASSIS_JLINK_TUNE_MAX_REF_DPS ||
                 left_kp_milli < 0 ||
                 left_kp_milli > CHASSIS_JLINK_TUNE_MAX_KP_MILLI ||
                 left_ki_milli < 0 ||
                 left_ki_milli > CHASSIS_JLINK_TUNE_MAX_KI_MILLI ||
                 right_kp_milli < 0 ||
                 right_kp_milli > CHASSIS_JLINK_TUNE_MAX_KP_MILLI ||
                 right_ki_milli < 0 ||
                 right_ki_milli > CHASSIS_JLINK_TUNE_MAX_KI_MILLI ||
                 control_mode > CHASSIS_JLINK_TUNE_SPEED_CURRENT ||
                 left_current_kp_milli < 0 ||
                 left_current_kp_milli >
                     CHASSIS_JLINK_TUNE_MAX_CURRENT_KP_MILLI ||
                 right_current_kp_milli < 0 ||
                 right_current_kp_milli >
                     CHASSIS_JLINK_TUNE_MAX_CURRENT_KP_MILLI ||
                 (control_mode == CHASSIS_JLINK_TUNE_SPEED_CURRENT &&
                  (left_current_kp_milli == 0 ||
                   right_current_kp_milli == 0)))
        {
            ChassisJLinkTuneReject(request_seq,
                                   CHASSIS_JLINK_TUNE_STOP_BAD_REQUEST);
        }
        else if ((requested_left != 0 && !left_online) ||
                 (requested_right != 0 && !right_online))
        {
            ChassisJLinkTuneReject(request_seq,
                                   CHASSIS_JLINK_TUNE_STOP_FEEDBACK_LOST);
        }
        else if ((requested_left != 0 &&
                  motor_l->measure.temperature >=
                      CHASSIS_JLINK_TUNE_MAX_TEMPERATURE_C) ||
                 (requested_right != 0 &&
                  motor_r->measure.temperature >=
                      CHASSIS_JLINK_TUNE_MAX_TEMPERATURE_C))
        {
            ChassisJLinkTuneReject(request_seq,
                                   CHASSIS_JLINK_TUNE_STOP_OVERTEMPERATURE);
        }
        else
        {
            CANGetDiagnostic(&hcan1, &can_diagnostic);
            if (can_diagnostic.bus_off || can_diagnostic.error_passive)
                ChassisJLinkTuneReject(request_seq,
                                       CHASSIS_JLINK_TUNE_STOP_CAN_FAULT);
            else
                ChassisJLinkTuneStart(request_seq,
                                      requested_left, requested_right,
                                      duration_ms,
                                      left_kp_milli, left_ki_milli,
                                      right_kp_milli, right_ki_milli,
                                      control_mode,
                                      left_current_kp_milli,
                                      right_current_kp_milli);
        }
    }

    if (!chassis_jlink_tune_active)
    {
        *left_ref = 0.0f;
        *right_ref = 0.0f;
        return 0U;
    }

    now = HAL_GetTick();
    if (now - chassis_jlink_tune_start_ms >=
        chassis_jlink_tune_capture.requested_duration_ms)
    {
        ChassisJLinkTuneFinish(CHASSIS_JLINK_TUNE_COMPLETE,
                               CHASSIS_JLINK_TUNE_STOP_DURATION);
    }
    else if ((chassis_jlink_tune_capture.left_ref_dps != 0 &&
              !ChassisJLinkTuneMotorOnline(now, motor_l)) ||
             (chassis_jlink_tune_capture.right_ref_dps != 0 &&
              !ChassisJLinkTuneMotorOnline(now, motor_r)))
    {
        ChassisJLinkTuneFinish(CHASSIS_JLINK_TUNE_ABORTED,
                               CHASSIS_JLINK_TUNE_STOP_FEEDBACK_LOST);
    }
    else
    {
        CANGetDiagnostic(&hcan1, &can_diagnostic);
        if (can_diagnostic.bus_off || can_diagnostic.error_passive)
            ChassisJLinkTuneFinish(CHASSIS_JLINK_TUNE_ABORTED,
                                   CHASSIS_JLINK_TUNE_STOP_CAN_FAULT);
        else if ((chassis_jlink_tune_capture.left_ref_dps != 0 &&
                  motor_l->measure.temperature >=
                      CHASSIS_JLINK_TUNE_MAX_TEMPERATURE_C) ||
                 (chassis_jlink_tune_capture.right_ref_dps != 0 &&
                  motor_r->measure.temperature >=
                      CHASSIS_JLINK_TUNE_MAX_TEMPERATURE_C))
            ChassisJLinkTuneFinish(CHASSIS_JLINK_TUNE_ABORTED,
                                   CHASSIS_JLINK_TUNE_STOP_OVERTEMPERATURE);
        else if ((chassis_jlink_tune_capture.left_ref_dps != 0 &&
                  fabsf(motor_l->measure.speed_aps) >
                      CHASSIS_JLINK_TUNE_MAX_SPEED_DPS) ||
                 (chassis_jlink_tune_capture.right_ref_dps != 0 &&
                  fabsf(motor_r->measure.speed_aps) >
                      CHASSIS_JLINK_TUNE_MAX_SPEED_DPS))
            ChassisJLinkTuneFinish(CHASSIS_JLINK_TUNE_ABORTED,
                                   CHASSIS_JLINK_TUNE_STOP_OVERSPEED);
    }

    if (!chassis_jlink_tune_active)
    {
        *left_ref = 0.0f;
        *right_ref = 0.0f;
        return 0U;
    }

    *left_ref = (float)chassis_jlink_tune_capture.left_ref_dps;
    *right_ref = (float)chassis_jlink_tune_capture.right_ref_dps;
    return 1U;
}

static void ChassisJLinkTuneCapture(void)
{
    Chassis_JLink_Tune_Sample_s *sample;
    CAN_Diagnostic_s can_diagnostic;
    uint32_t now;
    uint16_t sample_index;
    float left_speed;
    float right_speed;

    if (!chassis_jlink_tune_active)
        return;

    now = HAL_GetTick();
    if (now - chassis_jlink_tune_last_sample_ms <
        CHASSIS_JLINK_TUNE_SAMPLE_PERIOD_MS)
        return;

    sample_index = chassis_jlink_tune_capture.sample_count;
    if (sample_index >= CHASSIS_JLINK_TUNE_SAMPLE_CAPACITY)
    {
        ChassisJLinkTuneFinish(CHASSIS_JLINK_TUNE_COMPLETE,
                               CHASSIS_JLINK_TUNE_STOP_DURATION);
        return;
    }

    chassis_jlink_tune_last_sample_ms = now;
    sample = (Chassis_JLink_Tune_Sample_s *)&
        chassis_jlink_tune_capture.samples[sample_index];
    left_speed = -motor_l->measure.speed_aps;
    right_speed = motor_r->measure.speed_aps;
    CANGetDiagnostic(&hcan1, &can_diagnostic);

    sample->time_ms = now - chassis_jlink_tune_start_ms;
    sample->left_pid_dt_us = ChassisJLinkTuneDtUs(
        motor_l->motor_controller.speed_PID.dt);
    sample->right_pid_dt_us = ChassisJLinkTuneDtUs(
        motor_r->motor_controller.speed_PID.dt);
    sample->left_ref_dps = (int16_t)chassis_jlink_tune_capture.left_ref_dps;
    sample->right_ref_dps = (int16_t)chassis_jlink_tune_capture.right_ref_dps;
    sample->left_speed_dps = ChassisJLinkTuneToInt16(left_speed);
    sample->right_speed_dps = ChassisJLinkTuneToInt16(right_speed);
    sample->left_error_dps = ChassisJLinkTuneToInt16(
        (float)sample->left_ref_dps - left_speed);
    sample->right_error_dps = ChassisJLinkTuneToInt16(
        (float)sample->right_ref_dps - right_speed);
    sample->left_p_output = ChassisJLinkTuneToInt16(
        -motor_l->motor_controller.speed_PID.Pout);
    sample->right_p_output = ChassisJLinkTuneToInt16(
        motor_r->motor_controller.speed_PID.Pout);
    sample->left_i_output = ChassisJLinkTuneToInt16(
        -motor_l->motor_controller.speed_PID.Iout);
    sample->right_i_output = ChassisJLinkTuneToInt16(
        motor_r->motor_controller.speed_PID.Iout);
    sample->left_command = (int16_t)-motor_l->set_value;
    sample->right_command = motor_r->set_value;
    sample->left_current = (int16_t)-motor_l->measure.real_current;
    sample->right_current = motor_r->measure.real_current;
    sample->left_feedback_age_ms = ChassisJLinkTuneAge(now, motor_l);
    sample->right_feedback_age_ms = ChassisJLinkTuneAge(now, motor_r);
    sample->left_online = ChassisJLinkTuneMotorOnline(now, motor_l);
    sample->right_online = ChassisJLinkTuneMotorOnline(now, motor_r);
    sample->drive_status = wheel_drive_status;
    sample->can_flags = (can_diagnostic.bus_off ? 0x01U : 0U) |
                        (can_diagnostic.error_passive ? 0x02U : 0U);
    sample->can_last_error_code =
        (uint8_t)can_diagnostic.protocol_last_error_code;
    chassis_jlink_tune_capture.sample_count = sample_index + 1U;
}
#endif

#if ROBOT_USE_REMOTE_TEST_MODE && CHASSIS_JLINK_TEST_ONLY
static uint8_t ChassisApplyJLinkOverride(float *left_ref, float *right_ref)
{
    uint32_t now;

    if (!chassis_jlink_override_enable)
    {
        chassis_jlink_override_start_ms = 0U;
        return 0U;
    }

    now = HAL_GetTick();
    if (chassis_jlink_override_start_ms == 0U)
        chassis_jlink_override_start_ms = now;

    if (now - chassis_jlink_override_start_ms > CHASSIS_JLINK_OVERRIDE_TIMEOUT_MS)
    {
        chassis_jlink_override_enable = 0U;
        chassis_jlink_left_ref = 0.0f;
        chassis_jlink_right_ref = 0.0f;
        chassis_jlink_override_start_ms = 0U;
        return 0U;
    }

    *left_ref = ChassisClamp(chassis_jlink_left_ref,
                             CHASSIS_JLINK_OVERRIDE_REF_LIMIT);
    *right_ref = ChassisClamp(chassis_jlink_right_ref,
                              CHASSIS_JLINK_OVERRIDE_REF_LIMIT);
    return 1U;
}
#endif

#if CHASSIS_USE_BMI088_YAW_HOLD
static float ChassisApplyHeadingAssist(float base_v, float base_w)
{
    int8_t direction;
    BMI088DiagState imu;
    float yaw_error;
    float encoder_assist_error;
    float encoder_yaw_error;
    float fusion_alpha;
    float correction;
    float correction_limit;
    float gyro_z_corrected_dps;
    float predicted_error;
    float dt;
    uint32_t now;
    uint8_t required_status = BMI088_DIAG_YAW_VALID |
                              BMI088_DIAG_BIAS_VALID;

    if (fabsf(base_v) < CHASSIS_MOTION_REF_EPSILON ||
        fabsf(base_w) > CHASSIS_STRAIGHT_W_EPSILON ||
        !straight_tracking ||
        !BMI088DiagGetState(&imu) ||
        (imu.status & required_status) != required_status ||
        (imu.status & (BMI088_DIAG_SATURATED |
                       BMI088_DIAG_SAMPLE_TIMEOUT)) != 0U ||
        HAL_GetTick() - imu.last_update_ms > CHASSIS_IMU_MAX_AGE_MS)
    {
        heading_tracking = 0U;
        heading_direction = 0;
        heading_i_term = 0.0f;
        heading_fused_error_deg = 0.0f;
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
        heading_fused_error_deg = 0.0f;
        heading_last_error = 0.0f;
        heading_last_correction = 0.0f;
        heading_last_ms = HAL_GetTick();
        return base_w;
    }

    now = HAL_GetTick();
    dt = (float)(now - heading_last_ms) * 0.001f;
    heading_last_ms = now;
    if (dt <= 0.0f || dt > 0.1f || !isfinite(dt))
        dt = 0.01f;

    gyro_z_corrected_dps =
        imu.gyro_z_lpf_dps - imu.gyro_z_bias_dps;
    encoder_yaw_error = -straight_last_phase_error_deg *
                        RADIUS_WHEEL / TRACK_WIDTH;
    predicted_error = ChassisWrapDeg(
        heading_fused_error_deg - gyro_z_corrected_dps * dt);
    fusion_alpha = CHASSIS_HEADING_FUSION_TAU_S /
                   (CHASSIS_HEADING_FUSION_TAU_S + dt);
    heading_fused_error_deg = ChassisWrapDeg(
        fusion_alpha * predicted_error +
        (1.0f - fusion_alpha) * encoder_yaw_error);
    /* Keep BMI088 as the main heading measurement.  A bounded fraction of
     * the slow encoder phase estimate corrects residual gyro-bias drift that
     * the IMU alone cannot observe during a long straight run. */
    encoder_assist_error = ChassisClamp(
        heading_fused_error_deg,
        CHASSIS_YAW_HOLD_ENCODER_ASSIST_MAX_DEG);
    encoder_assist_error *= CHASSIS_YAW_HOLD_ENCODER_ASSIST_GAIN;
    yaw_error = heading_imu_error_deg + encoder_assist_error;
    if (fabsf(yaw_error) < CHASSIS_YAW_HOLD_DEADBAND)
        yaw_error = 0.0f;
    heading_last_error = yaw_error;

    correction_limit = fabsf(base_v) * CHASSIS_YAW_HOLD_MAX_RATIO;
    if (correction_limit > CHASSIS_YAW_HOLD_MAX_REF)
        correction_limit = CHASSIS_YAW_HOLD_MAX_REF;
    {
        float i_limit = fabsf(base_v) * CHASSIS_YAW_HOLD_I_MAX_RATIO;
        float i_delta;
        float candidate_i_term;
        float candidate_correction;

        if (i_limit > CHASSIS_YAW_HOLD_I_MAX_REF)
            i_limit = CHASSIS_YAW_HOLD_I_MAX_REF;
        /* The IMU yaw error contains gyro bias and vibration.  Integrating
         * it can build a correction opposite to the measured wheel phase.
         * Use the slow encoder residual for the integral term instead. */
        i_delta = encoder_assist_error * CHASSIS_YAW_HOLD_KI * dt;
        candidate_i_term = ChassisClamp(heading_i_term + i_delta,
                                        i_limit);
        candidate_correction = CHASSIS_YAW_HOLD_DIR *
            (CHASSIS_YAW_HOLD_KP * yaw_error +
             candidate_i_term -
             CHASSIS_YAW_HOLD_KD * gyro_z_corrected_dps);

        /* Stop integrating while the correction is saturated in the same
         * direction as the error. Keep integrating when it pulls saturation
         * back toward the allowed range. */
        if (fabsf(candidate_correction) <= correction_limit ||
            candidate_correction *
                (CHASSIS_YAW_HOLD_DIR * i_delta) < 0.0f)
            heading_i_term = candidate_i_term;
    }
    correction = CHASSIS_YAW_HOLD_DIR *
                 (CHASSIS_YAW_HOLD_KP * yaw_error +
                  heading_i_term -
                  CHASSIS_YAW_HOLD_KD * gyro_z_corrected_dps);
    correction = ChassisClamp(correction, correction_limit);
    heading_last_correction = correction;
    straight_last_correction = 0.0f;

    return base_w + correction;
}
#endif

static void ChassisUpdateStraightPhase(float base_v, float base_w)
{
    int32_t left_count;
    int32_t right_count;
    int8_t direction;
    int32_t left_delta;
    int32_t right_delta;
    double left_wheel_angle_deg;
    double right_wheel_angle_deg;
    double phase_error_wheel_deg;

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
        straight_last_phase_error_deg = 0.0f;
        straight_last_correction = 0.0f;
        return;
    }

    left_delta = ChassisCountDelta(left_count, straight_start_left);
    right_delta = ChassisCountDelta(right_count, straight_start_right);
    left_wheel_angle_deg = (double)left_delta * 360.0 /
                           ((double)LEFT_ENCODER_COUNTS_PER_WHEEL_REV *
                            (double)CHASSIS_STRAIGHT_LEFT_TRIM);
    right_wheel_angle_deg = (double)right_delta * 360.0 /
                            ((double)RIGHT_ENCODER_COUNTS_PER_WHEEL_REV *
                             (double)CHASSIS_STRAIGHT_RIGHT_TRIM);
    phase_error_wheel_deg = right_wheel_angle_deg - left_wheel_angle_deg;
    straight_last_phase_error_deg = (float)phase_error_wheel_deg;
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
                .Kp = CHASSIS_RIGHT_SPEED_KP,
                .Ki = CHASSIS_RIGHT_SPEED_KI,
                .Kd = 0,
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
                .IntegralLimit = CHASSIS_RIGHT_SPEED_I_LIMIT,
                .MaxOut = CHASSIS_RIGHT_SPEED_MAX_OUT,
                .MaxOut_ = -CHASSIS_RIGHT_SPEED_MAX_OUT
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
            .current_feedforward_ptr = &chassis_left_current_ff,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            /* DJI C610/M2006 accepts a current command directly over CAN.
             * Its internal current regulator closes the electrical loop, so the
             * software speed PI directly outputs the requested current.
             */
            .close_loop_type = SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    // 按微信参考版映射：左轮 2，右轮 3。两台电机共用 0x200，分别使用第 2、3 个电流槽位。
    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_LEFT_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    ChassisSetWheelSpeedPidConfig(&chassis_motor_config,
                                  CHASSIS_LEFT_SPEED_KP,
                                  CHASSIS_LEFT_SPEED_KI,
                                  CHASSIS_LEFT_SPEED_MAX_OUT,
                                  CHASSIS_LEFT_SPEED_I_LIMIT);
    motor_l = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_RIGHT_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr =
        &chassis_right_current_ff;
    ChassisSetWheelSpeedPidConfig(&chassis_motor_config,
                                  CHASSIS_RIGHT_SPEED_KP,
                                  CHASSIS_RIGHT_SPEED_KI,
                                  CHASSIS_RIGHT_SPEED_MAX_OUT,
                                  CHASSIS_RIGHT_SPEED_I_LIMIT);
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
#if ROBOT_USE_REMOTE_TEST_MODE && CHASSIS_JLINK_TEST_ONLY
    uint8_t jlink_override_active;
#endif

    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif

#if ROBOT_USE_REMOTE_TEST_MODE && CHASSIS_JLINK_TEST_ONLY
    left_ref = 0.0f;
    right_ref = 0.0f;
    jlink_override_active = ChassisApplyJLinkOverride(&left_ref, &right_ref);
    should_stop = !jlink_override_active;
#else
    should_stop = fabsf(chassis_cmd_recv.v) < CHASSIS_MOTION_REF_EPSILON &&
                  fabsf(chassis_cmd_recv.w) < CHASSIS_MOTION_REF_EPSILON;
#endif
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

#if !(ROBOT_USE_REMOTE_TEST_MODE && CHASSIS_JLINK_TEST_ONLY)
    if (!should_stop)
        ChassisUpdateStraightPhase(chassis_cmd_recv.v, chassis_cmd_recv.w);
#endif

    w_ref = chassis_cmd_recv.w;
#if CHASSIS_USE_BMI088_YAW_HOLD
    if (!should_stop)
        w_ref = ChassisApplyHeadingAssist(chassis_cmd_recv.v, chassis_cmd_recv.w);
#endif

    if (!should_stop)
        ChassisLimitInsideWheel(chassis_cmd_recv.v, &w_ref);

#if !(ROBOT_USE_REMOTE_TEST_MODE && CHASSIS_JLINK_TEST_ONLY)
    left_ref = chassis_cmd_recv.v - w_ref;
    right_ref = chassis_cmd_recv.v + w_ref;
    if (fabsf(chassis_cmd_recv.w) <= CHASSIS_STRAIGHT_W_EPSILON)
    {
        left_ref *= CHASSIS_STRAIGHT_LEFT_TRIM;
        right_ref *= CHASSIS_STRAIGHT_RIGHT_TRIM;
    }
#endif

    ChassisLimitWheelRefs(&left_ref, &right_ref);

    /* Apply only bounded low-speed friction feedforward.  This is added by
     * DJI motor control after the speed PI and is therefore a current command,
     * not a second speed reference. */
    ChassisUpdateFrictionCompensation(left_ref, right_ref, should_stop);

    DJIMotorSetRef(motor_l, left_ref);
    DJIMotorSetRef(motor_r, right_ref);
}

void ChassisGetHostFeedback(Chassis_Host_Feedback_s *feedback)
{
    const float left_degps_to_mps = PERIMETER_WHEEL /
        (1000.0f * 360.0f * LEFT_REDUCTION_RATIO_WHEEL);
    const float right_degps_to_mps = PERIMETER_WHEEL /
        (1000.0f * 360.0f * RIGHT_REDUCTION_RATIO_WHEEL);
    uint32_t primask;
    uint32_t now;
    CAN_Diagnostic_s can_diagnostic;

    if (feedback == NULL)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();
    CANGetDiagnostic(&hcan1, &can_diagnostic);
    feedback->left_total_count = ChassisMotorCount(motor_l, -1);
    feedback->right_total_count = ChassisMotorCount(motor_r, 1);
    feedback->target_left_mps =
        motor_l->motor_controller.pid_ref * left_degps_to_mps;
    feedback->target_right_mps =
        motor_r->motor_controller.pid_ref * right_degps_to_mps;
    feedback->measured_left_mps =
        -motor_l->measure.speed_aps * left_degps_to_mps;
    feedback->measured_right_mps =
        motor_r->measure.speed_aps * right_degps_to_mps;
    feedback->left_output = (int16_t)-motor_l->set_value;
    feedback->right_output = motor_r->set_value;
    feedback->left_stop_flag = motor_l->stop_flag;
    feedback->right_stop_flag = motor_r->stop_flag;
    feedback->left_current = (int16_t)-motor_l->measure.real_current;
    feedback->right_current = motor_r->measure.real_current;
    feedback->left_integral_output = (int16_t)-motor_l->motor_controller.speed_PID.Iout;
    feedback->right_integral_output = (int16_t)motor_r->motor_controller.speed_PID.Iout;
    feedback->left_temperature = motor_l->measure.temperature;
    feedback->right_temperature = motor_r->measure.temperature;
    feedback->drive_status = wheel_drive_status;
    feedback->left_feedback_count = motor_l->feedback_count;
    feedback->right_feedback_count = motor_r->feedback_count;
    feedback->left_feedback_age_ms = motor_l->feedback_count != 0U
        ? now - motor_l->last_feedback_ms : UINT32_MAX;
    feedback->right_feedback_age_ms = motor_r->feedback_count != 0U
        ? now - motor_r->last_feedback_ms : UINT32_MAX;
    feedback->left_feedback_online = motor_l->feedback_count != 0U &&
        feedback->left_feedback_age_ms <= 80U && DaemonIsOnline(motor_l->daemon);
    feedback->right_feedback_online = motor_r->feedback_count != 0U &&
        feedback->right_feedback_age_ms <= 80U && DaemonIsOnline(motor_r->daemon);
    feedback->can_tx_success_count = can_diagnostic.tx_success_count;
    feedback->can_tx_failure_count = can_diagnostic.tx_failure_count;
    feedback->can_last_tx_failure_age_ms = can_diagnostic.last_tx_failure_age_ms;
    feedback->can_hal_error_code = can_diagnostic.hal_error_code;
    feedback->can_protocol_last_error_code = can_diagnostic.protocol_last_error_code;
    feedback->can_state = can_diagnostic.state;
    feedback->can_bus_off = can_diagnostic.bus_off;
    feedback->can_error_passive = can_diagnostic.error_passive;
    feedback->can_tx_fifo_free_level = can_diagnostic.tx_fifo_free_level;
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
    feedback->sync_active = straight_tracking;
    feedback->sync_phase_error_deg = straight_last_phase_error_deg;
    feedback->sync_correction_ref = straight_last_correction;
    if (primask == 0U)
        __enable_irq();
}
