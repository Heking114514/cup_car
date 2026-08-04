#include "wheel_speed_controller.h"

#define WHEEL_SPEED_ZERO_MPS 0.001f

static float wheel_speed_clamp(float value, float limit)
{
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

static int16_t wheel_speed_round_to_int16(float value)
{
  return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

void wheel_speed_controller_init(
  wheel_speed_controller_t *controller,
  float kp,
  float ki,
  float filter_alpha,
  float integral_limit)
{
  controller->kp = kp;
  controller->ki = ki;
  controller->filter_alpha = filter_alpha;
  controller->integral_limit = integral_limit;
  wheel_speed_controller_reset(controller);
}

void wheel_speed_controller_reset(wheel_speed_controller_t *controller)
{
  controller->integral = 0.0f;
  controller->filtered_speed = 0.0f;
  controller->previous_target = 0.0f;
  controller->filter_initialized = false;
}

int16_t wheel_speed_controller_update(
  wheel_speed_controller_t *controller,
  float target_speed_mps,
  float measured_speed_mps,
  float dt_s,
  int16_t feedforward_pwm,
  int16_t pwm_limit)
{
  float error;
  float candidate_integral;
  float unsaturated_output;
  float output;
  float output_min;
  float output_max;
  bool changed_direction;
  bool accept_integral;

  if (target_speed_mps > -WHEEL_SPEED_ZERO_MPS &&
      target_speed_mps < WHEEL_SPEED_ZERO_MPS) {
    wheel_speed_controller_reset(controller);
    return 0;
  }

  changed_direction = controller->previous_target * target_speed_mps < 0.0f;
  if (changed_direction) {
    controller->integral = 0.0f;
    controller->filter_initialized = false;
  }
  controller->previous_target = target_speed_mps;

  if (!controller->filter_initialized) {
    controller->filtered_speed = measured_speed_mps;
    controller->filter_initialized = true;
  } else {
    controller->filtered_speed += controller->filter_alpha *
      (measured_speed_mps - controller->filtered_speed);
  }

  error = target_speed_mps - controller->filtered_speed;
  output_min = target_speed_mps > 0.0f ? 0.0f : -(float)pwm_limit;
  output_max = target_speed_mps > 0.0f ? (float)pwm_limit : 0.0f;
  candidate_integral = wheel_speed_clamp(
    controller->integral + error * dt_s, controller->integral_limit);
  unsaturated_output = (float)feedforward_pwm + controller->kp * error +
    controller->ki * candidate_integral;

  accept_integral =
    (unsaturated_output <= output_max && unsaturated_output >= output_min) ||
    (unsaturated_output > output_max && error < 0.0f) ||
    (unsaturated_output < output_min && error > 0.0f);
  if (accept_integral) {
    controller->integral = candidate_integral;
  }

  output = (float)feedforward_pwm + controller->kp * error +
    controller->ki * controller->integral;
  if (output > output_max) {
    output = output_max;
  } else if (output < output_min) {
    output = output_min;
  }
  return wheel_speed_round_to_int16(output);
}
