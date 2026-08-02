#include "ps2.h"

#include "main.h"

#define PS2_HALF_CLOCK_US 16U

static void ps2_delay_us(uint32_t microseconds)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = microseconds * (SystemCoreClock / 1000000U);

  while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
  }
}

static uint8_t ps2_transfer(uint8_t command)
{
  uint8_t response = 0;

  for (uint8_t bit = 0; bit < 8U; bit++) {
    HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin,
                      (command & (1U << bit)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET);
    ps2_delay_us(PS2_HALF_CLOCK_US);

    if (HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port, PS2_DAT_Pin) == GPIO_PIN_SET) {
      response |= (uint8_t)(1U << bit);
    }

    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
    ps2_delay_us(PS2_HALF_CLOCK_US);
  }

  return response;
}

void ps2_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);
}

bool ps2_read(ps2_state_t *state)
{
  uint8_t response[9];

  HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET);
  ps2_delay_us(PS2_HALF_CLOCK_US);

  response[0] = ps2_transfer(0x01U);
  response[1] = ps2_transfer(0x42U);
  for (uint8_t index = 2; index < 9U; index++) {
    response[index] = ps2_transfer(0x00U);
  }

  HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);

  state->mode = response[1];
  state->connected = response[1] == 0x41U || response[1] == 0x73U || response[1] == 0x79U;
  state->buttons = (uint16_t)response[3] | ((uint16_t)response[4] << 8U);
  state->right_x = response[5];
  state->right_y = response[6];
  state->left_x = response[7];
  state->left_y = response[8];

  return state->connected;
}

bool ps2_button_down(const ps2_state_t *state, uint16_t button)
{
  return state->connected && (state->buttons & button) == 0U;
}
