#ifndef TURN_CONTROL_H
#define TURN_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  TURN_CONTROL_STATE_IDLE = 0,
  TURN_CONTROL_STATE_RUNNING,
  TURN_CONTROL_STATE_BRAKING,
  TURN_CONTROL_STATE_SETTLING,
  TURN_CONTROL_STATE_COMPLETE,
  TURN_CONTROL_STATE_TIMEOUT,
  TURN_CONTROL_STATE_CANCELED,
  TURN_CONTROL_STATE_IMU_FAULT,
  TURN_CONTROL_STATE_REVERSAL_LIMIT,
  TURN_CONTROL_STATE_RATE_FAULT
} turn_control_state_t;

typedef enum {
  TURN_CONTROL_ACTION_COAST = 0,
  TURN_CONTROL_ACTION_DRIVE,
  TURN_CONTROL_ACTION_BRAKE
} turn_control_action_t;

typedef enum {
  TURN_CONTROL_BRAKE_NONE = 0,
  TURN_CONTROL_BRAKE_TARGET_ZONE,
  TURN_CONTROL_BRAKE_OVERSPEED,
  TURN_CONTROL_BRAKE_REVERSAL,
  TURN_CONTROL_BRAKE_TIMEOUT,
  TURN_CONTROL_BRAKE_WATCHDOG,
  TURN_CONTROL_BRAKE_IMU_FAULT,
  TURN_CONTROL_BRAKE_REVERSAL_LIMIT,
  TURN_CONTROL_BRAKE_RATE_FAULT
} turn_control_brake_reason_t;

typedef struct {
  uint32_t sequence;
  uint32_t start_time_ms;
  uint32_t timeout_ms;
  uint32_t sample_time_ms;
  uint32_t settled_since_ms;
  uint32_t reenter_since_ms;
  uint32_t drive_started_ms;
  turn_control_state_t state;
  turn_control_action_t action;
  turn_control_brake_reason_t brake_reason;
  float requested_angle_rad;
  float start_yaw_rad;
  float target_yaw_rad;
  float measured_yaw_rad;
  float previous_yaw_rad;
  float target_yaw_unwrapped_rad;
  float measured_yaw_unwrapped_rad;
  float error_rad;
  float measured_rate_radps;
  float filtered_rate_radps;
  float requested_rate_radps;
  float target_rate_radps;
  float angle_integral_rad_s;
  float rate_integral_rad;
  float p_term_radps;
  float i_term_radps;
  float d_term_radps;
  float rate_p_term_pwm;
  float rate_i_term_pwm;
  int16_t pivot_pwm;
  int8_t drive_direction;
  uint8_t reversal_count;
  bool timing_valid;
  bool rate_filter_valid;
  bool output_saturated;
  bool breakaway_armed;
} turn_controller_t;

void turn_control_init(turn_controller_t *controller);
bool turn_control_start(turn_controller_t *controller,
                        uint32_t sequence,
                        float relative_angle_rad,
                        uint32_t timeout_ms,
                        float current_yaw_rad,
                        uint32_t now_ms);
void turn_control_cancel(turn_controller_t *controller);
void turn_control_watchdog_fault(turn_controller_t *controller);
void turn_control_imu_fault(turn_controller_t *controller);
void turn_control_update(turn_controller_t *controller,
                         float yaw_rad,
                         float yaw_rate_radps,
                         uint32_t sample_time_ms,
                         uint32_t now_ms);
void turn_control_follow_rate(turn_controller_t *controller,
                              float requested_rate_radps,
                              float yaw_rate_radps,
                              uint32_t sample_time_ms,
                              uint32_t now_ms);
bool turn_control_is_active(const turn_controller_t *controller);
const turn_controller_t *turn_control_get_feedback(
  const turn_controller_t *controller);

#endif
