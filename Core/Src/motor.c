#include "motor.h"

#include "main.h"
#include "tim.h"

#define MOTOR_LEFT_REVERSED  0
#define MOTOR_RIGHT_REVERSED 1

static int16_t motor_clamp(int16_t value)
{
  if (value > MOTOR_PWM_MAX) {
    return MOTOR_PWM_MAX;
  }
  if (value < -MOTOR_PWM_MAX) {
    return -MOTOR_PWM_MAX;
  }
  return value;
}

static void motor_set_side(GPIO_TypeDef *port, uint16_t in1, uint16_t in2,
                           uint32_t channel, int16_t command, uint8_t reversed)
{
  uint16_t duty;
  uint8_t forward = command >= 0;

  if (reversed) {
    forward = !forward;
  }

  HAL_GPIO_WritePin(port, in1, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(port, in2, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);

  duty = (uint16_t)(command >= 0 ? command : -command);
  __HAL_TIM_SET_COMPARE(&htim4, channel, duty);
}

void motor_init(void)
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  motor_stop();
}

void motor_set(int16_t left, int16_t right)
{
  left = motor_clamp(left);
  right = motor_clamp(right);

  motor_set_side(GPIOB, L_IN1_Pin, L_IN2_Pin, TIM_CHANNEL_3, left, MOTOR_LEFT_REVERSED);
  motor_set_side(GPIOB, R_IN1_Pin, R_IN2_Pin, TIM_CHANNEL_4, right, MOTOR_RIGHT_REVERSED);
}

void motor_stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  HAL_GPIO_WritePin(GPIOB, L_IN1_Pin | L_IN2_Pin | R_IN1_Pin | R_IN2_Pin, GPIO_PIN_RESET);
}

void motor_brake(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  HAL_GPIO_WritePin(GPIOB, L_IN1_Pin | L_IN2_Pin | R_IN1_Pin | R_IN2_Pin, GPIO_PIN_SET);
}
