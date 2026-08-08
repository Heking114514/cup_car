#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

/* Tune these two values after measuring the real chassis. */
#define CHASSIS_TRACK_WIDTH_M       0.254f
#define CHASSIS_MAX_WHEEL_SPEED_MPS 1.000f

typedef struct {
  uint32_t sample_time_ms;
  uint32_t sample_sequence;
  int16_t left_delta;
  int16_t right_delta;
  float target_left_mps;
  float target_right_mps;
  float measured_left_mps;
  float measured_right_mps;
  int16_t left_pwm;
  int16_t right_pwm;
  float sync_error_m;
} chassis_feedback_t;

void chassis_init(void);
void chassis_process(void);
void chassis_forward(uint16_t pwm);
void chassis_backward(uint16_t pwm);
void chassis_turn_left(uint16_t pwm);
void chassis_turn_right(uint16_t pwm);
void chassis_stop(void);
bool chassis_command_is_straight(float vx_mps, float az_radps);
void chassis_set_velocity(float vx_mps, float az_radps);
const chassis_feedback_t *chassis_get_feedback(void);

#endif
