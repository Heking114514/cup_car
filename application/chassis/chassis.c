#include "chassis.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088_diag.h"
#include "buzzer.h"
#include <limits.h>
#include <math.h>
#include <string.h>

#define CHASSIS_MOTOR_RIGHT_ID 1u
#define CHASSIS_MOTOR_LEFT_ID 2u
/* The installed motors are mirror images: raw left positive and raw
 * right negative produce vehicle-forward motion.  Keep this convention
 * consistent for output, feedback, encoder counts, and synchronization. */
#define CHASSIS_LEFT_DIRECTION 1
#define CHASSIS_RIGHT_DIRECTION (-1)
#define CHASSIS_STRAIGHT_W_EPSILON (5.0f * CHASSIS_REFERENCE_SCALE)
#define CHASSIS_MOTION_REF_EPSILON (50.0f * CHASSIS_REFERENCE_SCALE)
/* Only a zero command stops the chassis.  The larger threshold above is
 * for straight-line assist engagement, not minimum commanded wheel speed. */
#define CHASSIS_STOP_REF_EPSILON 1.0e-5f
#define CHASSIS_HEADING_FUSION_TAU_S 1.5f
#define CHASSIS_STILL_SPEED_EPSILON 0.05f
#define CHASSIS_IMU_MAX_AGE_MS 80U
#define CHASSIS_WHEEL_REF_LIMIT (17500.0f * CHASSIS_REFERENCE_SCALE)
#define CHASSIS_STRAIGHT_LEFT_TRIM 1.0f
#define CHASSIS_STRAIGHT_RIGHT_TRIM 0.992f

#if CHASSIS_JLINK_TUNING_MODE
#define CHASSIS_JLINK_OVERRIDE_TIMEOUT_MS 800U
volatile uint32_t chassis_jlink_override_enable;
volatile float chassis_jlink_left_ref;
volatile float chassis_jlink_right_ref;
static uint32_t chassis_jlink_override_start_ms;
#endif

static DMMotorInstance *motor_l, *motor_r;
static Subscriber_t *chassis_sub;
static Publisher_t *chassis_pub;
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
static Chassis_Upload_Data_s chassis_feedback_data;

typedef enum {
    CHASSIS_DRIVE_DISABLED = 0,
    CHASSIS_DRIVE_MOVING,
    CHASSIS_DRIVE_BRAKING,
} ChassisDriveState;
static ChassisDriveState drive_state;
static uint32_t brake_start_ms;
static uint32_t brake_still_since_ms;
static uint8_t brake_still_tracking;

static uint8_t straight_tracking;
static int8_t straight_direction;
static int32_t straight_start_left;
static int32_t straight_start_right;
static float straight_last_phase_error_deg;
static float straight_last_correction;
static float wheel_sync_i_term;
static uint32_t wheel_sync_last_ms;
static float wheel_sync_last_phase_error_deg;
static uint8_t wheel_sync_phase_valid;
#if CHASSIS_USE_BMI088_YAW_HOLD
static uint8_t heading_tracking;
static int8_t heading_direction;
static float heading_ref_deg;
static float heading_i_term;
static float heading_fused_error_deg;
static float heading_last_error;
static float heading_last_correction;
static uint32_t heading_last_ms;
#endif

static uint8_t ChassisMotorsStill(void)
{
    return motor_l != NULL && motor_r != NULL &&
           fabsf(motor_l->measure.velocity) < CHASSIS_STILL_SPEED_EPSILON &&
           fabsf(motor_r->measure.velocity) < CHASSIS_STILL_SPEED_EPSILON;
}

static uint8_t ChassisMotorFeedbackFresh(const DMMotorInstance *motor)
{
    return motor != NULL && motor->feedback_online != 0U &&
           motor->feedback_count != 0U &&
           HAL_GetTick() - motor->last_feedback_ms <=
               DM_MOTOR_FEEDBACK_TIMEOUT_MS;
}

/* A normal stop must reach the motor's velocity loop before disabling it.
 * Only brake after motion, so startup/idle zero commands never enable motors.
 * Invalid input and stale feedback retain immediate disable semantics. */
static void ChassisStopMotors(uint8_t allow_brake)
{
    DMMotorSetRef(motor_l, 0.0f);
    DMMotorSetRef(motor_r, 0.0f);
    if (allow_brake && drive_state != CHASSIS_DRIVE_DISABLED &&
        ChassisMotorFeedbackFresh(motor_l) &&
        ChassisMotorFeedbackFresh(motor_r))
    {
        uint32_t now = HAL_GetTick();
        if (drive_state == CHASSIS_DRIVE_MOVING)
        {
            drive_state = CHASSIS_DRIVE_BRAKING;
            brake_start_ms = now;
            brake_still_tracking = 0U;
        }
        if (ChassisMotorsStill())
        {
            if (!brake_still_tracking)
            {
                brake_still_since_ms = now;
                brake_still_tracking = 1U;
            }
        }
        else
        {
            brake_still_tracking = 0U;
        }
        if (now - brake_start_ms < CHASSIS_STOP_BRAKE_TIMEOUT_MS &&
            (!brake_still_tracking ||
             now - brake_still_since_ms < CHASSIS_STOP_SETTLE_MS))
        {
            DMMotorEnable(motor_l);
            DMMotorEnable(motor_r);
            return;
        }
    }
    drive_state = CHASSIS_DRIVE_DISABLED;
    brake_still_tracking = 0U;
    DMMotorDisable(motor_l);
    DMMotorDisable(motor_r);
}

static int32_t ChassisMotorCount(const DMMotorInstance *motor, int32_t direction)
{
    if (motor == NULL)
        return 0;
    int64_t count = (int64_t)(motor->measure.total_position *
        MOTOR_ENCODER_COUNTS_PER_REV / (2.0 * PI)) * direction;
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

#if CHASSIS_USE_BMI088_YAW_HOLD
static float ChassisClamp(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}
#endif

static void ChassisLimitWheelRefs(float *left_ref, float *right_ref)
{
    float max_abs = fmaxf(fabsf(*left_ref), fabsf(*right_ref));
    float limit = fminf(CHASSIS_WHEEL_REF_LIMIT,
                        fminf(motor_l->limits.v_max, motor_r->limits.v_max));
    if (max_abs > limit)
    {
        float scale = limit / max_abs;
        *left_ref *= scale;
        *right_ref *= scale;
    }
}

#if CHASSIS_USE_BMI088_YAW_HOLD
static float ChassisWrapDeg(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}
#endif

static void ChassisReadWheelCounts(int32_t *left, int32_t *right)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *left = ChassisMotorCount(motor_l, CHASSIS_LEFT_DIRECTION);
    *right = ChassisMotorCount(motor_r, CHASSIS_RIGHT_DIRECTION);
    if (primask == 0U)
        __enable_irq();
}

static void ChassisResetStraightControl(void)
{
    straight_tracking = 0U;
    straight_direction = 0;
    straight_last_phase_error_deg = 0.0f;
    straight_last_correction = 0.0f;
    wheel_sync_i_term = 0.0f;
    wheel_sync_last_ms = 0U;
    wheel_sync_last_phase_error_deg = 0.0f;
    wheel_sync_phase_valid = 0U;
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
    float sync_correction;
    float correction_limit;
    float gyro_z_corrected_dps;
    float predicted_error;
    float dt;
    uint32_t now;
    uint8_t required_status = BMI088_DIAG_YAW_VALID |
                              BMI088_DIAG_BIAS_VALID;

    if (fabsf(base_v) < CHASSIS_MOTION_REF_EPSILON ||
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
    encoder_yaw_error = ChassisClamp(
        encoder_yaw_error, CHASSIS_YAW_HOLD_ENCODER_ASSIST_MAX_DEG);
    encoder_yaw_error *= CHASSIS_YAW_HOLD_ENCODER_ASSIST_GAIN;
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
    /* Keep the integrated IMU heading as the primary heading measurement;
     * the wheel phase is only a bounded drift assist, since wheel slip can
     * make encoder-derived yaw inaccurate at speed. */
    yaw_error = ChassisWrapDeg(heading_ref_deg - imu.yaw_deg) +
                encoder_assist_error;
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
        /* Integrate the fused heading error so a persistent mechanical bias
         * is removed.  The anti-windup check below prevents the integral
         * term from growing while the correction is at its limit. */
        i_delta = yaw_error * CHASSIS_YAW_HOLD_KI * dt;
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

    /* The wheel phase loop is useful for small left/right speed mismatch,
     * but its long-term phase estimate can demand a turn opposite to the
     * BMI088 heading loop.  Do not let that secondary loop cancel the
     * heading correction that is actively reducing yaw error. */
    sync_correction = base_w;
    if (correction != 0.0f && sync_correction * correction < 0.0f)
    {
        float opposing_limit =
            fabsf(correction) * CHASSIS_YAW_HOLD_OPPOSING_SYNC_RATIO;
        if (fabsf(sync_correction) > opposing_limit)
            sync_correction = copysignf(opposing_limit, sync_correction);
    }
    return sync_correction + correction;
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

static float ChassisApplyWheelSync(float base_v, float base_w)
{
    float phase_rate_error;
    float phase_position_error;
    float error;
    float correction_limit;
    float dt;
    uint32_t now;

    if (fabsf(base_v) < CHASSIS_MOTION_REF_EPSILON ||
        fabsf(base_w) > CHASSIS_STRAIGHT_W_EPSILON ||
        motor_l == NULL || motor_r == NULL ||
        !ChassisMotorFeedbackFresh(motor_l) ||
        !ChassisMotorFeedbackFresh(motor_r))
    {
        wheel_sync_i_term = 0.0f;
        wheel_sync_last_ms = 0U;
        wheel_sync_last_phase_error_deg = 0.0f;
        wheel_sync_phase_valid = 0U;
        straight_last_correction = 0.0f;
        return base_w;
    }

    now = HAL_GetTick();
    dt = wheel_sync_last_ms == 0U ? 0.01f :
        (float)(now - wheel_sync_last_ms) * 0.001f;
    wheel_sync_last_ms = now;
    if (dt <= 0.0f || dt > 0.1f || !isfinite(dt))
        dt = 0.01f;

    /* SYN reports right_angle - left_angle.  With the signed wheel feedback
     * used here, a positive phase error means the left wheel has travelled
     * farther in the commanded chassis direction.  The stabilizing
     * differential correction is therefore negative for a positive phase
     * error, for both travel directions. */
    if (wheel_sync_phase_valid)
    {
        phase_rate_error =
            -(straight_last_phase_error_deg - wheel_sync_last_phase_error_deg) *
            (float)PI / (180.0f * dt);
    }
    else
    {
        phase_rate_error = 0.0f;
        wheel_sync_phase_valid = 1U;
    }
    wheel_sync_last_phase_error_deg = straight_last_phase_error_deg;
    phase_position_error = -straight_last_phase_error_deg *
                           (float)PI / 180.0f;
    error = phase_rate_error +
            CHASSIS_WHEEL_SYNC_PHASE_POSITION_KP * phase_position_error;

    correction_limit = fabsf(base_v) * CHASSIS_WHEEL_SYNC_MAX_RATIO;
    wheel_sync_i_term += CHASSIS_WHEEL_SYNC_KI * error * dt;
    if (wheel_sync_i_term > correction_limit)
        wheel_sync_i_term = correction_limit;
    else if (wheel_sync_i_term < -correction_limit)
        wheel_sync_i_term = -correction_limit;
    error = base_w + CHASSIS_WHEEL_SYNC_KP * error + wheel_sync_i_term;
    if (error > correction_limit)
        error = correction_limit;
    else if (error < -correction_limit)
        error = -correction_limit;
    /* Report and apply the same differential correction in SYN telemetry. */
    straight_last_correction = error;
    return error;
}


void ChassisInit(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = CHASSIS_MOTOR_RIGHT_ID,
            .rx_id = 0u,
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    motor_r = DMMotorInit(&config);
    DMMotorSetDMType(motor_r, DM2325);
    DMMotorSetControlMode(motor_r, DM_CTRL_VEL);
    DMMotorDisable(motor_r);

    config.can_init_config.tx_id = CHASSIS_MOTOR_LEFT_ID;
    motor_l = DMMotorInit(&config);
    DMMotorSetDMType(motor_l, DM2325);
    DMMotorSetControlMode(motor_l, DM_CTRL_VEL);
    DMMotorDisable(motor_l);

    ChassisResetStraightControl();
    Buzzer_config_s buzzer = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .loudness = 0,
        .octave = OCTAVE_2,
    };
    BuzzerRegister(&buzzer);
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

void ChassisTask(void)
{
    float left_ref = 0.0f;
    float right_ref = 0.0f;
    float w_ref;
    uint8_t should_stop;
#if CHASSIS_JLINK_TUNING_MODE
    uint8_t jlink_override_active = 0U;
#endif

    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#if CHASSIS_JLINK_TUNING_MODE
    if (chassis_jlink_override_enable != 0U)
    {
        uint32_t now = HAL_GetTick();
        if (chassis_jlink_override_start_ms == 0U)
            chassis_jlink_override_start_ms = now;
        if (now - chassis_jlink_override_start_ms <=
            CHASSIS_JLINK_OVERRIDE_TIMEOUT_MS)
        {
            left_ref = chassis_jlink_left_ref;
            right_ref = chassis_jlink_right_ref;
            jlink_override_active = isfinite(left_ref) && isfinite(right_ref);
        }
        else
        {
            chassis_jlink_override_enable = 0U;
            chassis_jlink_override_start_ms = 0U;
        }
    }
    else
    {
        chassis_jlink_override_start_ms = 0U;
    }
#endif
    should_stop = !isfinite(chassis_cmd_recv.v) || !isfinite(chassis_cmd_recv.w) ||
        (fabsf(chassis_cmd_recv.v) < CHASSIS_STOP_REF_EPSILON &&
         fabsf(chassis_cmd_recv.w) < CHASSIS_STOP_REF_EPSILON) ||
        motor_l == NULL || motor_r == NULL;
#if CHASSIS_JLINK_TUNING_MODE
    if (jlink_override_active)
        should_stop = 0U;
#endif
#if CHASSIS_USE_BMI088_YAW_HOLD
    {
        BMI088DiagState imu_state;
        BMI088DiagGetState(&imu_state);
        uint8_t bias_ready = (imu_state.status & BMI088_DIAG_BIAS_VALID) != 0U;
        BMI088DiagUpdate(should_stop && (!bias_ready || ChassisMotorsStill()));
    }
#endif

    if (should_stop)
    {
        ChassisResetStraightControl();
        ChassisStopMotors(isfinite(chassis_cmd_recv.v) &&
                          isfinite(chassis_cmd_recv.w));
    }
    else
    {
        uint8_t feedback_ready = ChassisMotorFeedbackFresh(motor_l) &&
                                  ChassisMotorFeedbackFresh(motor_r);

        /* Never let one wheel run alone.  During startup and whenever either
         * DM feedback times out, keep both drivers enabled at zero speed so
         * feedback can recover, then resume only after both are online. */
        if (!feedback_ready)
        {
            ChassisResetStraightControl();
            DMMotorSetRef(motor_l, 0.0f);
            DMMotorSetRef(motor_r, 0.0f);
            DMMotorEnable(motor_l);
            DMMotorEnable(motor_r);
            goto publish_feedback;
        }
        drive_state = CHASSIS_DRIVE_MOVING;
        brake_still_tracking = 0U;
#if CHASSIS_JLINK_TUNING_MODE
        if (!jlink_override_active)
        {
#endif
        ChassisUpdateStraightPhase(chassis_cmd_recv.v, chassis_cmd_recv.w);
        w_ref = chassis_cmd_recv.w;
        w_ref = ChassisApplyWheelSync(chassis_cmd_recv.v, w_ref);
#if CHASSIS_USE_BMI088_YAW_HOLD
        if (fabsf(chassis_cmd_recv.w) <= CHASSIS_STRAIGHT_W_EPSILON)
            w_ref = ChassisApplyHeadingAssist(chassis_cmd_recv.v, w_ref);
#endif
        left_ref = chassis_cmd_recv.v - w_ref;
        right_ref = chassis_cmd_recv.v + w_ref;
        if (fabsf(chassis_cmd_recv.w) <= CHASSIS_STRAIGHT_W_EPSILON)
        {
            left_ref *= CHASSIS_STRAIGHT_LEFT_TRIM;
            right_ref *= CHASSIS_STRAIGHT_RIGHT_TRIM;
        }
#if CHASSIS_JLINK_TUNING_MODE
        }
#endif
        ChassisLimitWheelRefs(&left_ref, &right_ref);
        /* Apply each wheel's installation direction to output-shaft rad/s;
           both driver instances use NORMAL. */
        DMMotorSetRef(motor_l, left_ref * CHASSIS_LEFT_DIRECTION);
        DMMotorSetRef(motor_r, right_ref * CHASSIS_RIGHT_DIRECTION);
        DMMotorEnable(motor_l);
        DMMotorEnable(motor_r);
    }

publish_feedback:
    chassis_feedback_data.motor_speed[0] = motor_l != NULL
        ? motor_l->measure.velocity * CHASSIS_LEFT_DIRECTION : 0.0f;
    chassis_feedback_data.motor_speed[1] = motor_r != NULL
        ? motor_r->measure.velocity * CHASSIS_RIGHT_DIRECTION : 0.0f;
    PubPushMessage(chassis_pub, &chassis_feedback_data);
}

static void ChassisFillMotorFeedback(const DMMotorInstance *motor, int32_t direction,
                                     float ratio, int32_t *count,
                                     float *target_mps, float *measured_mps,
                                     uint8_t *temperature,
                                     uint8_t *stop_flag, uint8_t *online,
                                     uint32_t *age_ms, uint32_t *feedback_count)
{
    if (motor == NULL)
    {
        *age_ms = UINT32_MAX;
        return;
    }
    float radps_to_mps = RADIUS_WHEEL / (1000.0f * ratio);
    *count = ChassisMotorCount(motor, direction);
    *target_mps = motor->stop_flag == MOTOR_STOP
        ? 0.0f : direction * motor->pid_ref * radps_to_mps;
    *measured_mps = direction * motor->measure.velocity * radps_to_mps;
    *temperature = (uint8_t)fmaxf(motor->measure.T_Mos, motor->measure.T_Rotor);
    *stop_flag = motor->stop_flag;
    *online = ChassisMotorFeedbackFresh(motor);
    *age_ms = motor->feedback_count != 0U
        ? HAL_GetTick() - motor->last_feedback_ms : UINT32_MAX;
    *feedback_count = motor->feedback_count;
}

void ChassisGetHostFeedback(Chassis_Host_Feedback_s *feedback)
{
    if (feedback == NULL)
        return;
    CAN_Diagnostic_s diagnostic;
    CANGetDiagnostic(&hcan1, &diagnostic);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset(feedback, 0, sizeof(*feedback));
    ChassisFillMotorFeedback(motor_l, CHASSIS_LEFT_DIRECTION,
        LEFT_REDUCTION_RATIO_WHEEL, &feedback->left_total_count,
        &feedback->target_left_mps, &feedback->measured_left_mps,
        &feedback->left_temperature,
        &feedback->left_stop_flag, &feedback->left_feedback_online,
        &feedback->left_feedback_age_ms, &feedback->left_feedback_count);
    ChassisFillMotorFeedback(motor_r, CHASSIS_RIGHT_DIRECTION,
        RIGHT_REDUCTION_RATIO_WHEEL, &feedback->right_total_count,
        &feedback->target_right_mps, &feedback->measured_right_mps,
        &feedback->right_temperature,
        &feedback->right_stop_flag, &feedback->right_feedback_online,
        &feedback->right_feedback_age_ms, &feedback->right_feedback_count);
    /* DM velocity mode has no external current/PI output. Preserve the wire
       format with zero output/current/PI fields; torque remains in measure. */
    feedback->can_tx_success_count = diagnostic.tx_success_count;
    feedback->can_tx_failure_count = diagnostic.tx_failure_count;
    feedback->can_last_tx_failure_age_ms = diagnostic.last_tx_failure_age_ms;
    feedback->can_hal_error_code = diagnostic.hal_error_code;
    feedback->can_protocol_last_error_code = diagnostic.protocol_last_error_code;
    feedback->can_state = diagnostic.state;
    feedback->can_bus_off = diagnostic.bus_off;
    feedback->can_error_passive = diagnostic.error_passive;
    feedback->can_tx_fifo_free_level = diagnostic.tx_fifo_free_level;
#if CHASSIS_USE_BMI088_YAW_HOLD
    feedback->heading_active = heading_tracking;
    feedback->heading_ref_deg = heading_ref_deg;
    feedback->heading_error_deg = heading_last_error;
    feedback->heading_correction_ref = heading_last_correction;
    feedback->heading_i_term_ref = heading_i_term;
    feedback->heading_fused_error_deg = heading_fused_error_deg;
#endif
    feedback->sync_active = straight_tracking;
    feedback->sync_phase_error_deg = straight_last_phase_error_deg;
    feedback->sync_correction_ref = straight_last_correction;
    if (primask == 0U)
        __enable_irq();
}
