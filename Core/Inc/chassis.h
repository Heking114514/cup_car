#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* Tune these two values after measuring the real chassis. */
#define CHASSIS_TRACK_WIDTH_M       0.254f
#define CHASSIS_MAX_WHEEL_SPEED_MPS 1.000f

void chassis_init(void);
void chassis_forward(uint16_t pwm);
void chassis_backward(uint16_t pwm);
void chassis_turn_left(uint16_t pwm);
void chassis_turn_right(uint16_t pwm);
void chassis_stop(void);
void chassis_set_velocity(float vx_mps, float az_radps);

#endif
