#include "telemetry.h"

#include <stdint.h>

#include "car_control.h"
#include "chassis.h"
#include "communication.h"
#include "main.h"
#include "serial.h"

#define TELEMETRY_CONTROL_PERIOD_MS 100U
#define TELEMETRY_MILLI_SCALE       1000.0f
#define TELEMETRY_INT32_MAX_FLOAT   2147483.0f
#define TELEMETRY_INT32_MIN_FLOAT  -2147483.0f

static uint32_t last_control_report_ms;

static int32_t telemetry_to_milli_units(float value)
{
  float scaled;

  if (value >= TELEMETRY_INT32_MAX_FLOAT) {
    return INT32_MAX;
  }
  if (value <= TELEMETRY_INT32_MIN_FLOAT) {
    return INT32_MIN;
  }

  scaled = value * TELEMETRY_MILLI_SCALE;
  return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void telemetry_send_control(serial_port_t port)
{
  communication_command_status_t command;
  const chassis_feedback_t *chassis = chassis_get_feedback();

  communication_get_command_status(&command);
  serial_printf(
    port,
    "CTL,%lu,%lu,%u,%u,%u,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d\r\n",
    (unsigned long)chassis->sample_time_ms,
    (unsigned long)chassis->sample_sequence,
    (unsigned int)car_control_get_mode(),
    car_control_emergency_stopped() ? 1U : 0U,
    command.valid ? 1U : 0U,
    (unsigned long)command.age_ms,
    (long)telemetry_to_milli_units(command.vx_mps),
    (long)telemetry_to_milli_units(command.az_radps),
    (long)telemetry_to_milli_units(chassis->target_left_mps),
    (long)telemetry_to_milli_units(chassis->target_right_mps),
    (long)telemetry_to_milli_units(chassis->measured_left_mps),
    (long)telemetry_to_milli_units(chassis->measured_right_mps),
    (int)chassis->left_pwm,
    (int)chassis->right_pwm);
}

void telemetry_init(void)
{
  last_control_report_ms = HAL_GetTick();
}

void telemetry_process(void)
{
  uint32_t now = HAL_GetTick();

  if (now - last_control_report_ms < TELEMETRY_CONTROL_PERIOD_MS) {
    return;
  }
  last_control_report_ms = now;

  telemetry_send_control(SERIAL_DEBUG);
  telemetry_send_control(SERIAL_AUX);
}
