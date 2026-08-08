#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>

typedef enum {
  CAR_MODE_REMOTE = 0,
  CAR_MODE_NAVIGATION
} car_mode_t;

void car_control_init(void);
void car_control_process(void);
car_mode_t car_control_get_mode(void);
bool car_control_emergency_stopped(void);
bool car_control_heading_active(void);

#endif
