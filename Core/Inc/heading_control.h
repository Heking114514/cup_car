#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdbool.h>

typedef struct {
  float target_yaw_rad;
  bool has_target;
} heading_controller_t;

void heading_control_init(heading_controller_t *controller);
void heading_control_start(heading_controller_t *controller, float target_yaw_rad);
void heading_control_stop(heading_controller_t *controller);
bool heading_control_has_target(const heading_controller_t *controller);
float heading_control_update(const heading_controller_t *controller, float yaw_rad);

#endif
