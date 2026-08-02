#include "chassis.h"

#include "motor.h"

static float chassis_clamp(float value, float limit)
{
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

static int16_t chassis_speed_to_pwm(float speed_mps)
{
  float command = chassis_clamp(speed_mps, CHASSIS_MAX_WHEEL_SPEED_MPS);
  float magnitude;
  int16_t pwm;

  if (command > -0.001f && command < 0.001f) {
    return 0;
  }

  magnitude = command >= 0.0f ? command : -command;
  pwm = (int16_t)((float)MOTOR_PWM_MIN +
                  magnitude * (float)(MOTOR_PWM_MAX - MOTOR_PWM_MIN) /
                  CHASSIS_MAX_WHEEL_SPEED_MPS);
  return command >= 0.0f ? pwm : -pwm;
}

static uint16_t chassis_clamp_pwm(uint16_t pwm)
{
  return pwm > MOTOR_PWM_MAX ? MOTOR_PWM_MAX : pwm;
}

void chassis_init(void)
{
  motor_init();
}

void chassis_forward(uint16_t pwm)
{
  int16_t command = (int16_t)chassis_clamp_pwm(pwm);
  motor_set(command, command);
}

void chassis_backward(uint16_t pwm)
{
  int16_t command = -(int16_t)chassis_clamp_pwm(pwm);
  motor_set(command, command);
}

void chassis_turn_left(uint16_t pwm)
{
  int16_t command = (int16_t)chassis_clamp_pwm(pwm);
  motor_set(-command, command);
}

void chassis_turn_right(uint16_t pwm)
{
  int16_t command = (int16_t)chassis_clamp_pwm(pwm);
  motor_set(command, -command);
}

void chassis_stop(void)
{
  motor_stop();
}

void chassis_set_velocity(float vx_mps, float az_radps)
{
  float left_speed = vx_mps - az_radps * CHASSIS_TRACK_WIDTH_M * 0.5f;
  float right_speed = vx_mps + az_radps * CHASSIS_TRACK_WIDTH_M * 0.5f;

  motor_set(chassis_speed_to_pwm(left_speed), chassis_speed_to_pwm(right_speed));
}
