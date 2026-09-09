#include "chassis.h"

#include "encoder.h"
#include "motor.h"
#include "wheel_speed_controller.h"

#define CHASSIS_LEFT_DISTANCE_SCALE     1.000f
#define CHASSIS_RIGHT_DISTANCE_SCALE    1.000f
#define CHASSIS_LEFT_FEEDFORWARD_SCALE  0.930f
#define CHASSIS_RIGHT_FEEDFORWARD_SCALE 0.915f
#define CHASSIS_COMMAND_WHEEL_LIMIT_MPS 0.60f
#define CHASSIS_PI_KP                   800.0f
#define CHASSIS_PI_KI                   900.0f
#define CHASSIS_SPEED_FILTER_ALPHA      0.35f
#define CHASSIS_INTEGRAL_LIMIT          1.00f
#define CHASSIS_SYNC_GAIN_PER_S         3.00f
#define CHASSIS_SYNC_MAX_MPS            0.08f
#define CHASSIS_STRAIGHT_AZ_EPSILON     0.001f
#define CHASSIS_MOVING_VX_EPSILON       0.01f
#define CHASSIS_DEFAULT_SAMPLE_S        0.010f
#define CHASSIS_MIN_SAMPLE_S            0.005f
#define CHASSIS_MAX_SAMPLE_S            0.050f
#define CHASSIS_PI                      3.14159265f

static wheel_speed_controller_t left_controller;
static wheel_speed_controller_t right_controller;
static float target_left_mps;
static float target_right_mps;
static bool straight_tracking;
static int32_t straight_start_left;
static int32_t straight_start_right;
static uint32_t last_sample_time_ms;
static uint32_t last_sample_sequence;
static chassis_feedback_t chassis_feedback;

static float chassis_clamp(float value, float limit)
{
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

static int16_t chassis_speed_to_pwm(float speed_mps, float feedforward_scale)
{
  float command = chassis_clamp(speed_mps, CHASSIS_MAX_WHEEL_SPEED_MPS);
  float magnitude;
  float pwm_float;
  int16_t pwm;

  if (command > -0.001f && command < 0.001f) {
    return 0;
  }

  magnitude = command >= 0.0f ? command : -command;
  pwm_float = ((float)MOTOR_PWM_MIN +
               magnitude * (float)(MOTOR_PWM_MAX - MOTOR_PWM_MIN) /
               CHASSIS_MAX_WHEEL_SPEED_MPS) * feedforward_scale;
  if (pwm_float > (float)MOTOR_PWM_MAX) {
    pwm_float = (float)MOTOR_PWM_MAX;
  }
  pwm = (int16_t)(pwm_float + 0.5f);
  return command >= 0.0f ? pwm : -pwm;
}

static float chassis_abs(float value)
{
  return value >= 0.0f ? value : -value;
}

static void chassis_limit_wheel_speeds(float *left_mps, float *right_mps)
{
  float left_magnitude = chassis_abs(*left_mps);
  float right_magnitude = chassis_abs(*right_mps);
  float maximum_magnitude = left_magnitude > right_magnitude
                              ? left_magnitude
                              : right_magnitude;

  if (maximum_magnitude > CHASSIS_COMMAND_WHEEL_LIMIT_MPS) {
    float scale = CHASSIS_COMMAND_WHEEL_LIMIT_MPS / maximum_magnitude;
    *left_mps *= scale;
    *right_mps *= scale;
  }
}

static float chassis_counts_to_distance(int64_t counts, float counts_per_rev,
                                        float distance_scale)
{
  return (float)counts * 2.0f * CHASSIS_PI * CHASSIS_WHEEL_RADIUS_M /
         counts_per_rev * distance_scale;
}

static float chassis_pwm_to_speed(uint16_t pwm)
{
  uint16_t limited_pwm = pwm > MOTOR_PWM_MAX ? MOTOR_PWM_MAX : pwm;
  return (float)limited_pwm * CHASSIS_MAX_WHEEL_SPEED_MPS / (float)MOTOR_PWM_MAX;
}

static void chassis_reset_control(void)
{
  target_left_mps = 0.0f;
  target_right_mps = 0.0f;
  straight_tracking = false;
  wheel_speed_controller_reset(&left_controller);
  wheel_speed_controller_reset(&right_controller);
  chassis_feedback.target_left_mps = 0.0f;
  chassis_feedback.target_right_mps = 0.0f;
  chassis_feedback.left_pwm = 0;
  chassis_feedback.right_pwm = 0;
  chassis_feedback.sync_error_m = 0.0f;
}

void chassis_init(void)
{
  const encoder_data_t *encoder = encoder_get_data();

  motor_init();
  wheel_speed_controller_init(
    &left_controller, CHASSIS_PI_KP, CHASSIS_PI_KI,
    CHASSIS_SPEED_FILTER_ALPHA, CHASSIS_INTEGRAL_LIMIT);
  wheel_speed_controller_init(
    &right_controller, CHASSIS_PI_KP, CHASSIS_PI_KI,
    CHASSIS_SPEED_FILTER_ALPHA, CHASSIS_INTEGRAL_LIMIT);
  last_sample_time_ms = encoder->sample_time_ms;
  last_sample_sequence = encoder->sample_sequence;
  chassis_reset_control();
}

void chassis_forward(uint16_t pwm)
{
  chassis_set_velocity(chassis_pwm_to_speed(pwm), 0.0f);
}

void chassis_backward(uint16_t pwm)
{
  chassis_set_velocity(-chassis_pwm_to_speed(pwm), 0.0f);
}

void chassis_turn_left(uint16_t pwm)
{
  float wheel_speed = chassis_pwm_to_speed(pwm);
  chassis_set_velocity(0.0f, 2.0f * wheel_speed / CHASSIS_TRACK_WIDTH_M);
}

void chassis_turn_right(uint16_t pwm)
{
  float wheel_speed = chassis_pwm_to_speed(pwm);
  chassis_set_velocity(0.0f, -2.0f * wheel_speed / CHASSIS_TRACK_WIDTH_M);
}

void chassis_stop(void)
{
  chassis_reset_control();
  motor_stop();
}

bool chassis_command_is_straight(float vx_mps, float az_radps)
{
  return chassis_abs(az_radps) <= CHASSIS_STRAIGHT_AZ_EPSILON &&
         chassis_abs(vx_mps) >= CHASSIS_MOVING_VX_EPSILON;
}

void chassis_set_velocity(float vx_mps, float az_radps)
{
  const encoder_data_t *encoder = encoder_get_data();
  float left_speed = vx_mps - az_radps * CHASSIS_TRACK_WIDTH_M * 0.5f;
  float right_speed = vx_mps + az_radps * CHASSIS_TRACK_WIDTH_M * 0.5f;
  bool straight_requested = chassis_command_is_straight(vx_mps, az_radps);

  chassis_limit_wheel_speeds(&left_speed, &right_speed);
  target_left_mps = left_speed;
  target_right_mps = right_speed;

  if (chassis_abs(target_left_mps) < 0.001f &&
      chassis_abs(target_right_mps) < 0.001f) {
    chassis_stop();
    return;
  }

  if (straight_requested && !straight_tracking) {
    straight_start_left = encoder->left_total;
    straight_start_right = encoder->right_total;
    straight_tracking = true;
  } else if (!straight_requested) {
    straight_tracking = false;
  }
}

void chassis_process(void)
{
  const encoder_data_t *encoder = encoder_get_data();
  float dt_s;
  float measured_left_mps;
  float measured_right_mps;
  float controlled_left_mps = target_left_mps;
  float controlled_right_mps = target_right_mps;
  float sync_error_m = 0.0f;
  int16_t left_pwm;
  int16_t right_pwm;

  if (encoder->sample_sequence == last_sample_sequence) {
    return;
  }

  dt_s = (float)(encoder->sample_time_ms - last_sample_time_ms) * 0.001f;
  last_sample_time_ms = encoder->sample_time_ms;
  last_sample_sequence = encoder->sample_sequence;
  if (dt_s < CHASSIS_MIN_SAMPLE_S || dt_s > CHASSIS_MAX_SAMPLE_S) {
    dt_s = CHASSIS_DEFAULT_SAMPLE_S;
    wheel_speed_controller_reset(&left_controller);
    wheel_speed_controller_reset(&right_controller);
  }

  measured_left_mps = chassis_counts_to_distance(
    encoder->left_delta, CHASSIS_LEFT_ENCODER_COUNTS_PER_REV,
    CHASSIS_LEFT_DISTANCE_SCALE) / dt_s;
  measured_right_mps = chassis_counts_to_distance(
    encoder->right_delta, CHASSIS_RIGHT_ENCODER_COUNTS_PER_REV,
    CHASSIS_RIGHT_DISTANCE_SCALE) / dt_s;

  chassis_feedback.sample_time_ms = encoder->sample_time_ms;
  chassis_feedback.sample_sequence = encoder->sample_sequence;
  chassis_feedback.left_delta = encoder->left_delta;
  chassis_feedback.right_delta = encoder->right_delta;

  if (chassis_abs(target_left_mps) < 0.001f &&
      chassis_abs(target_right_mps) < 0.001f) {
    chassis_feedback.target_left_mps = 0.0f;
    chassis_feedback.target_right_mps = 0.0f;
    chassis_feedback.measured_left_mps = measured_left_mps;
    chassis_feedback.measured_right_mps = measured_right_mps;
    chassis_feedback.left_pwm = 0;
    chassis_feedback.right_pwm = 0;
    chassis_feedback.sync_error_m = 0.0f;
    motor_stop();
    return;
  }

  if (straight_tracking) {
    int64_t left_counts = (int64_t)encoder->left_total - straight_start_left;
    int64_t right_counts = (int64_t)encoder->right_total - straight_start_right;
    float distance_error =
      chassis_counts_to_distance(
        right_counts, CHASSIS_RIGHT_ENCODER_COUNTS_PER_REV,
        CHASSIS_RIGHT_DISTANCE_SCALE) -
      chassis_counts_to_distance(
        left_counts, CHASSIS_LEFT_ENCODER_COUNTS_PER_REV,
        CHASSIS_LEFT_DISTANCE_SCALE);
    float correction_limit = chassis_abs(target_left_mps) * 0.5f;
    float correction;

    if (correction_limit > CHASSIS_SYNC_MAX_MPS) {
      correction_limit = CHASSIS_SYNC_MAX_MPS;
    }
    correction = chassis_clamp(
      distance_error * CHASSIS_SYNC_GAIN_PER_S, correction_limit);
    sync_error_m = distance_error;
    controlled_left_mps = chassis_clamp(
      target_left_mps + correction, CHASSIS_MAX_WHEEL_SPEED_MPS);
    controlled_right_mps = chassis_clamp(
      target_right_mps - correction, CHASSIS_MAX_WHEEL_SPEED_MPS);
  }

  left_pwm = wheel_speed_controller_update(
    &left_controller, controlled_left_mps, measured_left_mps, dt_s,
    chassis_speed_to_pwm(controlled_left_mps, CHASSIS_LEFT_FEEDFORWARD_SCALE),
    MOTOR_PWM_MAX);
  right_pwm = wheel_speed_controller_update(
    &right_controller, controlled_right_mps, measured_right_mps, dt_s,
    chassis_speed_to_pwm(controlled_right_mps, CHASSIS_RIGHT_FEEDFORWARD_SCALE),
    MOTOR_PWM_MAX);

  chassis_feedback.target_left_mps = controlled_left_mps;
  chassis_feedback.target_right_mps = controlled_right_mps;
  chassis_feedback.measured_left_mps = left_controller.filtered_speed;
  chassis_feedback.measured_right_mps = right_controller.filtered_speed;
  chassis_feedback.left_pwm = left_pwm;
  chassis_feedback.right_pwm = right_pwm;
  chassis_feedback.sync_error_m = sync_error_m;

  motor_set(left_pwm, right_pwm);
}

const chassis_feedback_t *chassis_get_feedback(void)
{
  return &chassis_feedback;
}
