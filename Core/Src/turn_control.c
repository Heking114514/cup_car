#include "turn_control.h"

#include <math.h>
#include <string.h>

#define TURN_ANGLE_KP                         1.80f
#define TURN_ANGLE_KI                         0.12f
#define TURN_ANGLE_KD                         0.18f
#define TURN_ANGLE_INTEGRAL_ZONE_RAD          0.17453293f
#define TURN_ANGLE_MAX_I_OUTPUT_RADPS         0.03f
#define TURN_MAX_RATE_RADPS                   0.40000000f
#define TURN_STOPPING_DECEL_RADPS2            0.30000000f
#define TURN_MAX_ACCEL_RADPS2                 2.61799388f
#define TURN_RATE_FILTER_ALPHA                0.35f

#define TURN_RATE_KP_PWM                      220.0f
#define TURN_RATE_KI_PWM                      600.0f
#define TURN_RATE_INTEGRAL_LIMIT_RAD          0.60f
#define TURN_BREAKAWAY_PWM                    180.0f
#define TURN_MAX_DRIVE_PWM                    360.0f
#define TURN_BREAKAWAY_TIME_MS                20U
#define TURN_BREAKAWAY_RATE_RADPS             0.03490659f
#define TURN_OVERSPEED_MARGIN_RADPS           0.13962634f
#define TURN_OVERSPEED_RELEASE_MARGIN_RADPS   0.06981317f
#define TURN_ABSOLUTE_RATE_LIMIT_RADPS         1.20000000f
#define TURN_MAX_REVERSALS                    1U

#define TURN_ANGLE_CAPTURE_RAD                0.01745329252f
#define TURN_ANGLE_HOLD_RAD                   0.02617994f
#define TURN_SETTLED_RATE_RADPS               0.05235988f
#define TURN_BRAKE_RELEASE_RATE_RADPS         0.08726646f
#define TURN_SETTLED_TIME_MS                  300U
#define TURN_REENTER_TIME_MS                  100U
#define TURN_DEFAULT_SAMPLE_S                 0.010f
#define TURN_MIN_SAMPLE_S                     0.005f
#define TURN_MAX_SAMPLE_S                     0.050f
#define TURN_MIN_TIMEOUT_MS                   1000U
#define TURN_MAX_TIMEOUT_MS                   60000U
#define TURN_PI                               3.14159265f
#define TURN_TWO_PI                           6.28318531f

static float turn_abs(float value)
{
  return value >= 0.0f ? value : -value;
}

static float turn_clamp(float value, float minimum, float maximum)
{
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}

static float turn_wrap_pi(float angle_rad)
{
  while (angle_rad > TURN_PI) {
    angle_rad -= TURN_TWO_PI;
  }
  while (angle_rad < -TURN_PI) {
    angle_rad += TURN_TWO_PI;
  }
  return angle_rad;
}

static int8_t turn_sign(float value)
{
  if (value > 0.0f) {
    return 1;
  }
  if (value < 0.0f) {
    return -1;
  }
  return 0;
}

static int16_t turn_round_pwm(float value)
{
  return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static void turn_clear_runtime(turn_controller_t *controller)
{
  controller->settled_since_ms = 0U;
  controller->reenter_since_ms = 0U;
  controller->drive_started_ms = 0U;
  controller->action = TURN_CONTROL_ACTION_COAST;
  controller->brake_reason = TURN_CONTROL_BRAKE_NONE;
  controller->error_rad = 0.0f;
  controller->measured_rate_radps = 0.0f;
  controller->filtered_rate_radps = 0.0f;
  controller->requested_rate_radps = 0.0f;
  controller->target_rate_radps = 0.0f;
  controller->angle_integral_rad_s = 0.0f;
  controller->rate_integral_rad = 0.0f;
  controller->p_term_radps = 0.0f;
  controller->i_term_radps = 0.0f;
  controller->d_term_radps = 0.0f;
  controller->rate_p_term_pwm = 0.0f;
  controller->rate_i_term_pwm = 0.0f;
  controller->pivot_pwm = 0;
  controller->drive_direction = 0;
  controller->reversal_count = 0U;
  controller->timing_valid = false;
  controller->rate_filter_valid = false;
  controller->output_saturated = false;
  controller->breakaway_armed = false;
}

static void turn_enter_braking(turn_controller_t *controller,
                               turn_control_brake_reason_t reason)
{
  controller->state = TURN_CONTROL_STATE_BRAKING;
  controller->action = TURN_CONTROL_ACTION_BRAKE;
  controller->brake_reason = reason;
  controller->target_rate_radps = 0.0f;
  controller->requested_rate_radps = 0.0f;
  controller->rate_integral_rad = 0.0f;
  controller->rate_p_term_pwm = 0.0f;
  controller->rate_i_term_pwm = 0.0f;
  controller->pivot_pwm = 0;
  controller->breakaway_armed = false;
}

static void turn_enter_settling(turn_controller_t *controller,
                                uint32_t now_ms)
{
  controller->state = TURN_CONTROL_STATE_SETTLING;
  controller->action = TURN_CONTROL_ACTION_BRAKE;
  controller->brake_reason = TURN_CONTROL_BRAKE_TARGET_ZONE;
  controller->settled_since_ms = 0U;
  controller->reenter_since_ms = 0U;
  controller->target_rate_radps = 0.0f;
  controller->requested_rate_radps = 0.0f;
  controller->angle_integral_rad_s = 0.0f;
  controller->rate_integral_rad = 0.0f;
  controller->pivot_pwm = 0;
  controller->breakaway_armed = false;
  controller->drive_started_ms = now_ms;
}

static void turn_set_terminal(turn_controller_t *controller,
                              turn_control_state_t state,
                              turn_control_action_t action)
{
  controller->state = state;
  controller->action = action;
  if (state == TURN_CONTROL_STATE_TIMEOUT) {
    controller->brake_reason = TURN_CONTROL_BRAKE_TIMEOUT;
  } else if (state == TURN_CONTROL_STATE_IMU_FAULT) {
    controller->brake_reason = TURN_CONTROL_BRAKE_IMU_FAULT;
  } else if (state == TURN_CONTROL_STATE_REVERSAL_LIMIT) {
    controller->brake_reason = TURN_CONTROL_BRAKE_REVERSAL_LIMIT;
  } else if (state == TURN_CONTROL_STATE_RATE_FAULT) {
    controller->brake_reason = TURN_CONTROL_BRAKE_RATE_FAULT;
  }
  controller->requested_rate_radps = 0.0f;
  controller->target_rate_radps = 0.0f;
  controller->angle_integral_rad_s = 0.0f;
  controller->rate_integral_rad = 0.0f;
  controller->rate_p_term_pwm = 0.0f;
  controller->rate_i_term_pwm = 0.0f;
  controller->pivot_pwm = 0;
  controller->output_saturated = false;
  controller->breakaway_armed = false;
}

static bool turn_accept_reversal(turn_controller_t *controller)
{
  if (controller->reversal_count >= TURN_MAX_REVERSALS) {
    turn_set_terminal(
      controller, TURN_CONTROL_STATE_REVERSAL_LIMIT,
      TURN_CONTROL_ACTION_BRAKE);
    return false;
  }
  controller->reversal_count++;
  return true;
}

void turn_control_init(turn_controller_t *controller)
{
  memset(controller, 0, sizeof(*controller));
  controller->state = TURN_CONTROL_STATE_IDLE;
  controller->action = TURN_CONTROL_ACTION_COAST;
}

bool turn_control_start(turn_controller_t *controller,
                        uint32_t sequence,
                        float relative_angle_rad,
                        uint32_t timeout_ms,
                        float current_yaw_rad,
                        uint32_t now_ms)
{
  bool was_active = turn_control_is_active(controller);
  float previous_filtered_rate = controller->filtered_rate_radps;
  int8_t previous_direction = controller->drive_direction;
  int8_t requested_direction = turn_sign(relative_angle_rad);
  bool reversing_command = was_active && previous_direction != 0 &&
                           requested_direction != previous_direction;
  bool opposing_motion = was_active &&
                         previous_filtered_rate * (float)requested_direction <
                           -TURN_BRAKE_RELEASE_RATE_RADPS;

  if (!isfinite(relative_angle_rad) || !isfinite(current_yaw_rad) ||
      turn_abs(relative_angle_rad) <= TURN_ANGLE_CAPTURE_RAD ||
      turn_abs(relative_angle_rad) > TURN_TWO_PI ||
      timeout_ms < TURN_MIN_TIMEOUT_MS || timeout_ms > TURN_MAX_TIMEOUT_MS) {
    return false;
  }

  turn_clear_runtime(controller);
  controller->sequence = sequence;
  controller->start_time_ms = now_ms;
  controller->timeout_ms = timeout_ms;
  controller->sample_time_ms = now_ms;
  controller->requested_angle_rad = relative_angle_rad;
  controller->start_yaw_rad = current_yaw_rad;
  controller->target_yaw_rad = turn_wrap_pi(current_yaw_rad + relative_angle_rad);
  controller->measured_yaw_rad = current_yaw_rad;
  controller->previous_yaw_rad = current_yaw_rad;
  controller->target_yaw_unwrapped_rad = current_yaw_rad + relative_angle_rad;
  controller->measured_yaw_unwrapped_rad = current_yaw_rad;
  controller->error_rad = relative_angle_rad;
  controller->drive_direction = turn_sign(controller->error_rad);
  if (was_active && previous_direction != 0) {
    controller->drive_direction = previous_direction;
  }
  controller->drive_started_ms = now_ms;
  controller->measured_rate_radps = previous_filtered_rate;
  controller->filtered_rate_radps = previous_filtered_rate;
  controller->rate_filter_valid = was_active;
  if (was_active && turn_abs(previous_filtered_rate) >
                    TURN_BRAKE_RELEASE_RATE_RADPS) {
    controller->state = TURN_CONTROL_STATE_BRAKING;
    controller->action = TURN_CONTROL_ACTION_BRAKE;
    controller->brake_reason = reversing_command || opposing_motion
                                 ? TURN_CONTROL_BRAKE_REVERSAL
                                 : TURN_CONTROL_BRAKE_OVERSPEED;
  } else {
    controller->state = TURN_CONTROL_STATE_RUNNING;
    controller->action = TURN_CONTROL_ACTION_BRAKE;
    controller->breakaway_armed = !was_active;
  }
  return true;
}

void turn_control_cancel(turn_controller_t *controller)
{
  if (turn_control_is_active(controller)) {
    turn_set_terminal(
      controller, TURN_CONTROL_STATE_CANCELED, TURN_CONTROL_ACTION_COAST);
    controller->brake_reason = TURN_CONTROL_BRAKE_NONE;
  }
}

void turn_control_watchdog_fault(turn_controller_t *controller)
{
  if (turn_control_is_active(controller)) {
    turn_set_terminal(
      controller, TURN_CONTROL_STATE_CANCELED, TURN_CONTROL_ACTION_BRAKE);
    controller->brake_reason = TURN_CONTROL_BRAKE_WATCHDOG;
  }
}

void turn_control_imu_fault(turn_controller_t *controller)
{
  if (turn_control_is_active(controller)) {
    turn_set_terminal(
      controller, TURN_CONTROL_STATE_IMU_FAULT, TURN_CONTROL_ACTION_BRAKE);
  }
}

bool turn_control_is_active(const turn_controller_t *controller)
{
  return controller->state == TURN_CONTROL_STATE_RUNNING ||
         controller->state == TURN_CONTROL_STATE_BRAKING ||
         controller->state == TURN_CONTROL_STATE_SETTLING;
}

void turn_control_update(turn_controller_t *controller,
                         float yaw_rad,
                         float yaw_rate_radps,
                         uint32_t sample_time_ms,
                         uint32_t now_ms)
{
  float dt_s;
  float abs_error;
  float abs_rate;
  int8_t requested_direction;

  if (!turn_control_is_active(controller)) {
    return;
  }

  if (!isfinite(yaw_rad) || !isfinite(yaw_rate_radps)) {
    turn_set_terminal(
      controller, TURN_CONTROL_STATE_IMU_FAULT, TURN_CONTROL_ACTION_BRAKE);
    return;
  }

  controller->measured_rate_radps = yaw_rate_radps;

  if (now_ms - controller->start_time_ms >= controller->timeout_ms) {
    turn_set_terminal(
      controller, TURN_CONTROL_STATE_TIMEOUT, TURN_CONTROL_ACTION_BRAKE);
    return;
  }

  if (controller->breakaway_armed &&
      now_ms - controller->drive_started_ms >= TURN_BREAKAWAY_TIME_MS) {
    controller->breakaway_armed = false;
    controller->pivot_pwm = 0;
    controller->action = TURN_CONTROL_ACTION_BRAKE;
    controller->brake_reason = TURN_CONTROL_BRAKE_NONE;
  }

  if (controller->timing_valid && sample_time_ms == controller->sample_time_ms) {
    return;
  }

  if (controller->timing_valid) {
    dt_s = (float)(sample_time_ms - controller->sample_time_ms) * 0.001f;
    if (dt_s < TURN_MIN_SAMPLE_S || dt_s > TURN_MAX_SAMPLE_S) {
      dt_s = TURN_DEFAULT_SAMPLE_S;
      controller->angle_integral_rad_s = 0.0f;
      controller->rate_integral_rad = 0.0f;
    }
  } else {
    dt_s = TURN_DEFAULT_SAMPLE_S;
    controller->timing_valid = true;
  }
  controller->sample_time_ms = sample_time_ms;

  controller->measured_yaw_rad = yaw_rad;
  controller->measured_yaw_unwrapped_rad +=
    turn_wrap_pi(yaw_rad - controller->previous_yaw_rad);
  controller->previous_yaw_rad = yaw_rad;

  if (!controller->rate_filter_valid) {
    controller->filtered_rate_radps = yaw_rate_radps;
    controller->rate_filter_valid = true;
  } else {
    controller->filtered_rate_radps += TURN_RATE_FILTER_ALPHA *
      (yaw_rate_radps - controller->filtered_rate_radps);
  }

  controller->error_rad = controller->target_yaw_unwrapped_rad -
    controller->measured_yaw_unwrapped_rad;
  abs_error = turn_abs(controller->error_rad);
  abs_rate = turn_abs(controller->filtered_rate_radps);

  if (turn_abs(yaw_rate_radps) > TURN_ABSOLUTE_RATE_LIMIT_RADPS) {
    turn_set_terminal(
      controller, TURN_CONTROL_STATE_RATE_FAULT,
      TURN_CONTROL_ACTION_BRAKE);
    return;
  }

  if (controller->state == TURN_CONTROL_STATE_SETTLING) {
    controller->action = TURN_CONTROL_ACTION_BRAKE;
    controller->pivot_pwm = 0;
    if (abs_error > TURN_ANGLE_HOLD_RAD &&
        abs_rate <= TURN_BRAKE_RELEASE_RATE_RADPS) {
      if (controller->reenter_since_ms == 0U) {
        controller->reenter_since_ms = sample_time_ms;
      } else if (sample_time_ms - controller->reenter_since_ms >=
                 TURN_REENTER_TIME_MS) {
        controller->state = TURN_CONTROL_STATE_RUNNING;
        controller->settled_since_ms = 0U;
        controller->reenter_since_ms = 0U;
        if (controller->drive_direction != 0 &&
            turn_sign(controller->error_rad) != controller->drive_direction) {
          if (!turn_accept_reversal(controller)) {
            return;
          }
        }
        controller->drive_direction = turn_sign(controller->error_rad);
        controller->drive_started_ms = now_ms;
        controller->breakaway_armed = false;
        controller->brake_reason = TURN_CONTROL_BRAKE_NONE;
        controller->timing_valid = false;
        return;
      }
    } else {
      controller->reenter_since_ms = 0U;
    }
    if (abs_error <= TURN_ANGLE_HOLD_RAD &&
        abs_rate <= TURN_SETTLED_RATE_RADPS) {
      if (controller->settled_since_ms == 0U) {
        controller->settled_since_ms = sample_time_ms;
      } else if (sample_time_ms - controller->settled_since_ms >=
                 TURN_SETTLED_TIME_MS) {
        turn_set_terminal(
          controller, TURN_CONTROL_STATE_COMPLETE, TURN_CONTROL_ACTION_COAST);
      }
    } else {
      controller->settled_since_ms = 0U;
    }
    return;
  }

  if (controller->state == TURN_CONTROL_STATE_BRAKING) {
    controller->action = TURN_CONTROL_ACTION_BRAKE;
    controller->pivot_pwm = 0;
    if (abs_rate <= TURN_BRAKE_RELEASE_RATE_RADPS) {
      if (abs_error <= TURN_ANGLE_CAPTURE_RAD) {
        turn_enter_settling(controller, now_ms);
      } else {
        bool direction_changed;

        requested_direction = turn_sign(controller->error_rad);
        direction_changed = controller->drive_direction != 0 &&
                            requested_direction != controller->drive_direction;
        if (direction_changed) {
          if (!turn_accept_reversal(controller)) {
            return;
          }
          controller->angle_integral_rad_s = 0.0f;
        }
        controller->breakaway_armed = false;
        controller->drive_direction = requested_direction;
        controller->drive_started_ms = now_ms;
        controller->target_rate_radps = 0.0f;
        controller->rate_integral_rad = 0.0f;
        controller->state = TURN_CONTROL_STATE_RUNNING;
        controller->brake_reason = TURN_CONTROL_BRAKE_NONE;
      }
    }
    return;
  }

  if (abs_error <= TURN_ANGLE_CAPTURE_RAD) {
    turn_enter_settling(controller, now_ms);
    return;
  }

  requested_direction = turn_sign(controller->error_rad);
  if (controller->filtered_rate_radps * (float)requested_direction <
      -TURN_BRAKE_RELEASE_RATE_RADPS) {
    turn_enter_braking(controller, TURN_CONTROL_BRAKE_REVERSAL);
    return;
  }
  if (controller->drive_direction != 0 &&
      requested_direction != controller->drive_direction) {
    if (abs_rate > TURN_BRAKE_RELEASE_RATE_RADPS) {
      turn_enter_braking(controller, TURN_CONTROL_BRAKE_REVERSAL);
      return;
    }
    if (!turn_accept_reversal(controller)) {
      return;
    }
    controller->angle_integral_rad_s = 0.0f;
    controller->rate_integral_rad = 0.0f;
    controller->target_rate_radps = 0.0f;
    controller->drive_started_ms = now_ms;
    controller->breakaway_armed = false;
  }
  controller->drive_direction = requested_direction;

  {
    float candidate_integral;
    float unsaturated_rate;
    float saturated_rate;
    float rate_limit;
    float stopping_error = abs_error - TURN_ANGLE_CAPTURE_RAD;
    float maximum_integral =
      TURN_ANGLE_MAX_I_OUTPUT_RADPS / TURN_ANGLE_KI;
    bool accept_integral;

    if (stopping_error < 0.0f) {
      stopping_error = 0.0f;
    }
    rate_limit = sqrtf(
      2.0f * TURN_STOPPING_DECEL_RADPS2 * stopping_error);
    if (rate_limit > TURN_MAX_RATE_RADPS) {
      rate_limit = TURN_MAX_RATE_RADPS;
    }

    controller->p_term_radps = TURN_ANGLE_KP * controller->error_rad;
    controller->d_term_radps =
      -TURN_ANGLE_KD * controller->filtered_rate_radps;

    if (abs_error <= TURN_ANGLE_INTEGRAL_ZONE_RAD) {
      candidate_integral = turn_clamp(
        controller->angle_integral_rad_s + controller->error_rad * dt_s,
        -maximum_integral, maximum_integral);
    } else {
      controller->angle_integral_rad_s = 0.0f;
      candidate_integral = 0.0f;
    }

    unsaturated_rate = controller->p_term_radps +
      TURN_ANGLE_KI * candidate_integral + controller->d_term_radps;
    saturated_rate = turn_clamp(
      unsaturated_rate, -rate_limit, rate_limit);
    accept_integral =
      unsaturated_rate == saturated_rate ||
      (unsaturated_rate > rate_limit && controller->error_rad < 0.0f) ||
      (unsaturated_rate < -rate_limit && controller->error_rad > 0.0f);
    if (accept_integral) {
      controller->angle_integral_rad_s = candidate_integral;
    }
    controller->i_term_radps =
      TURN_ANGLE_KI * controller->angle_integral_rad_s;
    unsaturated_rate = controller->p_term_radps +
      controller->i_term_radps + controller->d_term_radps;
    controller->requested_rate_radps = turn_clamp(
      unsaturated_rate, -rate_limit, rate_limit);
    controller->output_saturated =
      controller->requested_rate_radps != unsaturated_rate;

    if (controller->requested_rate_radps * (float)requested_direction < 0.0f) {
      controller->requested_rate_radps = 0.0f;
      controller->output_saturated = true;
    }
  }

  {
    float maximum_step = TURN_MAX_ACCEL_RADPS2 * dt_s;
    float rate_step =
      controller->requested_rate_radps - controller->target_rate_radps;

    rate_step = turn_clamp(rate_step, -maximum_step, maximum_step);
    controller->target_rate_radps += rate_step;
    if (controller->target_rate_radps * (float)requested_direction < 0.0f) {
      controller->target_rate_radps = 0.0f;
    }
  }

  {
    float target_magnitude =
      turn_abs(controller->target_rate_radps);
    float measured_along =
      controller->filtered_rate_radps * (float)requested_direction;
    float raw_measured_along =
      yaw_rate_radps * (float)requested_direction;
    float overspeed_rate = measured_along;
    bool overspeed_braking =
      controller->action == TURN_CONTROL_ACTION_BRAKE &&
      controller->brake_reason == TURN_CONTROL_BRAKE_OVERSPEED;

    if (raw_measured_along > overspeed_rate) {
      overspeed_rate = raw_measured_along;
    }

    if (target_magnitude <= 0.001f) {
      turn_enter_braking(controller, TURN_CONTROL_BRAKE_TARGET_ZONE);
      return;
    }

    {
      float rate_error = target_magnitude - measured_along;
      float candidate_integral = turn_clamp(
        controller->rate_integral_rad + rate_error * dt_s,
        -TURN_RATE_INTEGRAL_LIMIT_RAD, TURN_RATE_INTEGRAL_LIMIT_RAD);
      float unsaturated_pwm = TURN_RATE_KP_PWM * rate_error +
        TURN_RATE_KI_PWM * candidate_integral;
      float output_pwm = turn_clamp(
        unsaturated_pwm, 0.0f, TURN_MAX_DRIVE_PWM);
      bool accept_integral =
        unsaturated_pwm == output_pwm ||
        (unsaturated_pwm > TURN_MAX_DRIVE_PWM && rate_error < 0.0f) ||
        (unsaturated_pwm < 0.0f && rate_error > 0.0f);

      if (accept_integral) {
        controller->rate_integral_rad = candidate_integral;
      }
      controller->rate_p_term_pwm = TURN_RATE_KP_PWM * rate_error;
      controller->rate_i_term_pwm =
        TURN_RATE_KI_PWM * controller->rate_integral_rad;
      unsaturated_pwm = controller->rate_p_term_pwm +
        controller->rate_i_term_pwm;
      output_pwm = turn_clamp(
        unsaturated_pwm, 0.0f, TURN_MAX_DRIVE_PWM);

      if (controller->breakaway_armed &&
          now_ms - controller->drive_started_ms < TURN_BREAKAWAY_TIME_MS &&
          raw_measured_along < TURN_BREAKAWAY_RATE_RADPS &&
          output_pwm < TURN_BREAKAWAY_PWM) {
        output_pwm = TURN_BREAKAWAY_PWM;
      }
      if (now_ms - controller->drive_started_ms >= TURN_BREAKAWAY_TIME_MS ||
          raw_measured_along >= TURN_BREAKAWAY_RATE_RADPS) {
        controller->breakaway_armed = false;
      }

      controller->output_saturated = controller->output_saturated ||
        output_pwm != unsaturated_pwm;
      controller->state = TURN_CONTROL_STATE_RUNNING;
      if (overspeed_rate > target_magnitude +
          (overspeed_braking
             ? TURN_OVERSPEED_RELEASE_MARGIN_RADPS
             : TURN_OVERSPEED_MARGIN_RADPS)) {
        controller->pivot_pwm = 0;
        controller->action = TURN_CONTROL_ACTION_BRAKE;
        controller->brake_reason = TURN_CONTROL_BRAKE_OVERSPEED;
        controller->breakaway_armed = false;
        controller->output_saturated = true;
      } else if (!controller->breakaway_armed && output_pwm < 0.5f) {
        controller->pivot_pwm = 0;
        controller->action = TURN_CONTROL_ACTION_BRAKE;
        controller->brake_reason = TURN_CONTROL_BRAKE_NONE;
      } else {
        controller->pivot_pwm = turn_round_pwm(
          output_pwm * (float)requested_direction);
        controller->action = TURN_CONTROL_ACTION_DRIVE;
        controller->brake_reason = TURN_CONTROL_BRAKE_NONE;
      }
    }
  }
}

const turn_controller_t *turn_control_get_feedback(
  const turn_controller_t *controller)
{
  return controller;
}
