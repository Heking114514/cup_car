#include "encoder.h"

#include "main.h"
#include "tim.h"

#define ENCODER_LEFT_REVERSED  1
#define ENCODER_RIGHT_REVERSED 0

static encoder_data_t encoder_data;
static uint16_t last_left;
static uint16_t last_right;
static uint32_t last_activity_ms;
static bool activity_seen;

void encoder_init(void)
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  encoder_reset();
}

void encoder_reset(void)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  last_left = 0;
  last_right = 0;
  encoder_data.left_delta = 0;
  encoder_data.right_delta = 0;
  encoder_data.left_total = 0;
  encoder_data.right_total = 0;
  encoder_data.sample_time_ms = HAL_GetTick();
  encoder_data.sample_sequence = 0U;
  last_activity_ms = 0U;
  activity_seen = false;
}

void encoder_update(void)
{
  uint16_t now_left = __HAL_TIM_GET_COUNTER(&htim2);
  uint16_t now_right = __HAL_TIM_GET_COUNTER(&htim3);
  int16_t left_delta = (int16_t)(now_left - last_left);
  int16_t right_delta = (int16_t)(now_right - last_right);

  last_left = now_left;
  last_right = now_right;

  if (ENCODER_LEFT_REVERSED) {
    left_delta = -left_delta;
  }
  if (ENCODER_RIGHT_REVERSED) {
    right_delta = -right_delta;
  }

  encoder_data.left_delta = left_delta;
  encoder_data.right_delta = right_delta;
  encoder_data.left_total += left_delta;
  encoder_data.right_total += right_delta;
  encoder_data.sample_time_ms = HAL_GetTick();
  encoder_data.sample_sequence++;

  if (left_delta != 0 || right_delta != 0) {
    last_activity_ms = HAL_GetTick();
    activity_seen = true;
  }
}

const encoder_data_t *encoder_get_data(void)
{
  return &encoder_data;
}

bool encoder_has_recent_activity(uint32_t timeout_ms)
{
  return activity_seen && HAL_GetTick() - last_activity_ms <= timeout_ms;
}
