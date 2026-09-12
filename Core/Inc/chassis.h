#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

/* Measured chassis geometry and pooled independent 10-revolution calibration. */
#define CHASSIS_WHEEL_RADIUS_M                  0.0238f
#define CHASSIS_TRACK_WIDTH_M                   0.1247f
#define CHASSIS_LEFT_ENCODER_COUNTS_PER_REV     1060.1667f
#define CHASSIS_RIGHT_ENCODER_COUNTS_PER_REV    1060.9333f
#define CHASSIS_MAX_WHEEL_SPEED_MPS             1.000f

typedef enum {
  CHASSIS_OUTPUT_STOP = 0,
  CHASSIS_OUTPUT_SPEED,
  CHASSIS_OUTPUT_PIVOT_PWM,
  CHASSIS_OUTPUT_BRAKE
} chassis_output_mode_t;

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
  chassis_output_mode_t output_mode;
} chassis_feedback_t;

void chassis_init(void);
void chassis_process(void);
void chassis_forward(uint16_t pwm);
void chassis_backward(uint16_t pwm);
void chassis_turn_left(uint16_t pwm);
void chassis_turn_right(uint16_t pwm);
void chassis_stop(void);
void chassis_brake(void);
void chassis_set_pivot_pwm(int16_t yaw_pwm);
bool chassis_command_is_straight(float vx_mps, float az_radps);
void chassis_set_velocity(float vx_mps, float az_radps);
const chassis_feedback_t *chassis_get_feedback(void);

#endif
