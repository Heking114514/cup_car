#ifndef WHEEL_SPEED_CONTROLLER_H
#define WHEEL_SPEED_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float filter_alpha;
  float integral_limit;
  float integral;
  float filtered_speed;
  float previous_target;
  bool filter_initialized;
} wheel_speed_controller_t;

void wheel_speed_controller_init(
  wheel_speed_controller_t *controller,
  float kp,
  float ki,
  float filter_alpha,
  float integral_limit);
void wheel_speed_controller_reset(wheel_speed_controller_t *controller);
int16_t wheel_speed_controller_update(
  wheel_speed_controller_t *controller,
  float target_speed_mps,
  float measured_speed_mps,
  float dt_s,
  int16_t feedforward_pwm,
  int16_t pwm_limit);

#endif
