#include "remote_control.h"

#include "main.h"
#include "ps2.h"

#define REMOTE_UPDATE_PERIOD_MS 10U
#define REMOTE_AXIS_DEADZONE    12
#define REMOTE_MAX_VX_MPS       0.60f
#define REMOTE_MAX_AZ_RADPS     2.50f

static ps2_state_t ps2_state;
static uint32_t last_update_ms;
static bool remote_active;
static float remote_vx_mps;
static float remote_az_radps;

static float remote_axis_to_unit(uint8_t raw, bool invert)
{
  int16_t centered = (int16_t)raw - 128;
  float value;

  if (centered > REMOTE_AXIS_DEADZONE) {
    value = (float)(centered - REMOTE_AXIS_DEADZONE) /
            (float)(127 - REMOTE_AXIS_DEADZONE);
  } else if (centered < -REMOTE_AXIS_DEADZONE) {
    value = (float)(centered + REMOTE_AXIS_DEADZONE) /
            (float)(128 - REMOTE_AXIS_DEADZONE);
  } else {
    value = 0.0f;
  }

  return invert ? -value : value;
}

void remote_control_init(void)
{
  ps2_init();
  remote_active = false;
  remote_vx_mps = 0.0f;
  remote_az_radps = 0.0f;
  last_update_ms = HAL_GetTick();
}

void remote_control_process(void)
{
  uint32_t now = HAL_GetTick();
  float vx;
  float az;

  if (now - last_update_ms < REMOTE_UPDATE_PERIOD_MS) {
    return;
  }
  last_update_ms = now;

  ps2_read(&ps2_state);
  remote_active = ps2_state.connected &&
                  (ps2_state.mode == 0x73U || ps2_state.mode == 0x79U);

  if (!remote_active) {
    remote_vx_mps = 0.0f;
    remote_az_radps = 0.0f;
    return;
  }

  /* Match navigation coordinates: forward vx and counter-clockwise az are positive. */
  vx = remote_axis_to_unit(ps2_state.left_y, true) * REMOTE_MAX_VX_MPS;
  az = remote_axis_to_unit(ps2_state.right_x, true) * REMOTE_MAX_AZ_RADPS;
  remote_vx_mps = vx;
  remote_az_radps = az;
}

bool remote_control_get_command(float *vx_mps, float *az_radps)
{
  if (!remote_active) {
    return false;
  }

  *vx_mps = remote_vx_mps;
  *az_radps = remote_az_radps;
  return true;
}

bool remote_control_button_down(uint16_t button)
{
  return ps2_button_down(&ps2_state, button);
}
