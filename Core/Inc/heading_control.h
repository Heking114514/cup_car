#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float target_yaw_rad;
  float integral_error_rad_s;
  float error_rad;
  float correction_radps;
  uint32_t last_sample_time_ms;
  bool timing_valid;
  bool has_target;
} heading_controller_t;

void heading_control_init(heading_controller_t *controller);
void heading_control_start(heading_controller_t *controller, float target_yaw_rad);
void heading_control_stop(heading_controller_t *controller);
bool heading_control_has_target(const heading_controller_t *controller);
float heading_control_update(heading_controller_t *controller,
                             float yaw_rad,
                             float yaw_rate_radps,
                             uint32_t sample_time_ms);

#endif
