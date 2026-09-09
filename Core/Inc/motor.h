#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_PWM_MAX 1200
#define MOTOR_PWM_MIN 600

void motor_init(void);
void motor_set(int16_t left, int16_t right);
void motor_stop(void);
void motor_brake(void);

#endif
