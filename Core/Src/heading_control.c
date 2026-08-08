#include "heading_control.h"

#define HEADING_CONTROL_KP                   2.00f
#define HEADING_CONTROL_MAX_CORRECTION_RADPS 0.50f
#define HEADING_CONTROL_YAW_DEADBAND_RAD     0.005f
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
  controller->has_target = false;
}

void heading_control_start(heading_controller_t *controller, float target_yaw_rad)
{
  controller->target_yaw_rad = heading_control_wrap_pi(target_yaw_rad);
  controller->has_target = true;
}

void heading_control_stop(heading_controller_t *controller)
{
  controller->has_target = false;
}

bool heading_control_has_target(const heading_controller_t *controller)
{
  return controller->has_target;
}

float heading_control_update(const heading_controller_t *controller, float yaw_rad)
{
  float yaw_error;

  if (!controller->has_target) {
    return 0.0f;
  }

  yaw_error = heading_control_wrap_pi(controller->target_yaw_rad - yaw_rad);
  if (heading_control_abs(yaw_error) <= HEADING_CONTROL_YAW_DEADBAND_RAD) {
    return 0.0f;
  }

  return heading_control_clamp(
    HEADING_CONTROL_KP * yaw_error, HEADING_CONTROL_MAX_CORRECTION_RADPS);
}
