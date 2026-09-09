#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  CAR_MODE_REMOTE = 0,
  CAR_MODE_NAVIGATION
} car_mode_t;

typedef struct {
  uint32_t sample_time_ms;
  bool active;
  bool imu_valid;
  float target_yaw_rad;
  float measured_yaw_rad;
  float error_rad;
  float correction_radps;
  float requested_az_radps;
  float controlled_az_radps;
} car_heading_feedback_t;

void car_control_init(void);
void car_control_process(void);
car_mode_t car_control_get_mode(void);
bool car_control_emergency_stopped(void);
bool car_control_heading_active(void);
const car_heading_feedback_t *car_control_get_heading_feedback(void);

#endif
