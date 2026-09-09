#include "heading_control.h"

#define HEADING_CONTROL_KP                         3.00f
#define HEADING_CONTROL_KI                         0.60f
#define HEADING_CONTROL_KD                         0.10f
#define HEADING_CONTROL_MAX_CORRECTION_RADPS       0.45f
#define HEADING_CONTROL_MAX_INTEGRAL_OUTPUT_RADPS  0.08f
#define HEADING_CONTROL_YAW_DEADBAND_RAD           0.001745329f
#define HEADING_CONTROL_INTEGRAL_DEADBAND_RAD      0.000523599f
#define HEADING_CONTROL_INTEGRAL_ZONE_RAD          0.034906585f
#define HEADING_CONTROL_DEFAULT_SAMPLE_S           0.010f
#define HEADING_CONTROL_MAX_SAMPLE_S               0.050f
#define HEADING_CONTROL_PI                   3.14159265f
#define HEADING_CONTROL_TWO_PI               6.28318531f

static float heading_control_abs(float value)
{
  return value >= 0.0f ? value : -value;
}

static float heading_control_clamp(float value, float limit)
{
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

static float heading_control_integral_limit(void)
{
  return HEADING_CONTROL_MAX_INTEGRAL_OUTPUT_RADPS / HEADING_CONTROL_KI;
}

static float heading_control_wrap_pi(float angle_rad)
{
  while (angle_rad > HEADING_CONTROL_PI) {
    angle_rad -= HEADING_CONTROL_TWO_PI;
  }
  while (angle_rad < -HEADING_CONTROL_PI) {
    angle_rad += HEADING_CONTROL_TWO_PI;
  }
  return angle_rad;
}

void heading_control_init(heading_controller_t *controller)
{
  controller->target_yaw_rad = 0.0f;
  controller->integral_error_rad_s = 0.0f;
  controller->error_rad = 0.0f;
  controller->correction_radps = 0.0f;
  controller->last_sample_time_ms = 0U;
  controller->timing_valid = false;
  controller->has_target = false;
}

void heading_control_start(heading_controller_t *controller, float target_yaw_rad)
{
  controller->target_yaw_rad = heading_control_wrap_pi(target_yaw_rad);
  controller->integral_error_rad_s = 0.0f;
  controller->error_rad = 0.0f;
  controller->correction_radps = 0.0f;
  controller->timing_valid = false;
  controller->has_target = true;
}

void heading_control_stop(heading_controller_t *controller)
{
  controller->has_target = false;
  controller->integral_error_rad_s = 0.0f;
  controller->error_rad = 0.0f;
  controller->correction_radps = 0.0f;
  controller->timing_valid = false;
}

bool heading_control_has_target(const heading_controller_t *controller)
{
  return controller->has_target;
}

float heading_control_update(heading_controller_t *controller,
                             float yaw_rad,
                             float yaw_rate_radps,
                             uint32_t sample_time_ms)
{
  float control_error;
  float integral_error;
  float dt_s = 0.0f;
  float integral_limit;
  float yaw_error;

  if (!controller->has_target) {
    return 0.0f;
  }

  yaw_error = heading_control_wrap_pi(controller->target_yaw_rad - yaw_rad);
  controller->error_rad = yaw_error;
  control_error = heading_control_abs(yaw_error) <=
                    HEADING_CONTROL_YAW_DEADBAND_RAD ? 0.0f : yaw_error;
  integral_error = heading_control_abs(yaw_error) <=
                     HEADING_CONTROL_INTEGRAL_DEADBAND_RAD ? 0.0f : yaw_error;

  if (controller->timing_valid) {
    uint32_t elapsed_ms = sample_time_ms - controller->last_sample_time_ms;

    if (elapsed_ms == 0U) {
      return controller->correction_radps;
    }
    dt_s = (float)elapsed_ms * 0.001f;
    if (dt_s > HEADING_CONTROL_MAX_SAMPLE_S) {
      dt_s = HEADING_CONTROL_DEFAULT_SAMPLE_S;
    }
  } else {
    controller->timing_valid = true;
  }
  controller->last_sample_time_ms = sample_time_ms;

  if (heading_control_abs(yaw_error) <= HEADING_CONTROL_INTEGRAL_ZONE_RAD) {
    controller->integral_error_rad_s += integral_error * dt_s;
  } else {
    controller->integral_error_rad_s = 0.0f;
  }
  integral_limit = heading_control_integral_limit();
  controller->integral_error_rad_s = heading_control_clamp(
    controller->integral_error_rad_s, integral_limit);
  controller->correction_radps = heading_control_clamp(
    HEADING_CONTROL_KP * control_error +
    HEADING_CONTROL_KI * controller->integral_error_rad_s -
    HEADING_CONTROL_KD * yaw_rate_radps,
    HEADING_CONTROL_MAX_CORRECTION_RADPS);

  return controller->correction_radps;
}
