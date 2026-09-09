#include "telemetry.h"

#include <stdint.h>

#include "car_control.h"
#include "chassis.h"
#include "communication.h"
#include "main.h"
#include "mpu6050.h"
#include "serial.h"

#define TELEMETRY_CONTROL_PERIOD_MS 100U
#define TELEMETRY_IMU_PERIOD_MS      50U
#define TELEMETRY_MILLI_SCALE       1000.0f
#define TELEMETRY_RAD_TO_DEG         57.2957795f
#define TELEMETRY_INT32_MAX_FLOAT   2147483.0f
#define TELEMETRY_INT32_MIN_FLOAT  -2147483.0f

static uint32_t last_control_report_ms;
static uint32_t last_imu_report_ms;
static uint32_t last_heading_report_ms;

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

static void telemetry_send_imu(serial_port_t port)
{
  const mpu6050_data_t *imu = mpu6050_get_data();

  serial_printf(
    port,
    "IMU,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,"
    "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
    "%ld,%ld,%ld,%ld,%lu,%lu,%ld,%u,%lu\r\n",
    (unsigned long)imu->sample_time_ms,
    (unsigned long)imu->sample_sequence,
    (unsigned int)imu->state,
    imu->sample_valid ? 1U : 0U,
    imu->stationary ? 1U : 0U,
    (unsigned int)imu->address_7bit,
    (unsigned int)imu->who_am_i,
    (unsigned int)imu->calibration_samples,
    (unsigned int)imu->calibration_required,
    (long)telemetry_to_milli_units(imu->accel_x_g),
    (long)telemetry_to_milli_units(imu->accel_y_g),
    (long)telemetry_to_milli_units(imu->accel_z_g),
    (long)telemetry_to_milli_units(imu->gyro_x_dps),
    (long)telemetry_to_milli_units(imu->gyro_y_dps),
    (long)telemetry_to_milli_units(imu->gyro_z_dps),
    (long)telemetry_to_milli_units(imu->gyro_bias_x_dps),
    (long)telemetry_to_milli_units(imu->gyro_bias_y_dps),
    (long)telemetry_to_milli_units(imu->gyro_bias_z_dps),
    (long)telemetry_to_milli_units(imu->roll_rad * TELEMETRY_RAD_TO_DEG),
    (long)telemetry_to_milli_units(imu->pitch_rad * TELEMETRY_RAD_TO_DEG),
    (long)telemetry_to_milli_units(imu->yaw_rad * TELEMETRY_RAD_TO_DEG),
    (long)telemetry_to_milli_units(imu->temperature_c),
    (unsigned long)imu->io_error_count,
    (unsigned long)imu->recovery_count,
    (long)telemetry_to_milli_units(imu->yaw_rate_dps),
    imu->moving_bias_active ? 1U : 0U,
    (unsigned long)imu->moving_bias_update_count);
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

static void telemetry_send_heading(serial_port_t port)
{
  const car_heading_feedback_t *heading =
    car_control_get_heading_feedback();

  serial_printf(
    port,
    "HDG,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
    (unsigned long)heading->sample_time_ms,
    heading->active ? 1U : 0U,
    heading->imu_valid ? 1U : 0U,
    (long)telemetry_to_milli_units(
      heading->target_yaw_rad * TELEMETRY_RAD_TO_DEG),
    (long)telemetry_to_milli_units(
      heading->measured_yaw_rad * TELEMETRY_RAD_TO_DEG),
    (long)telemetry_to_milli_units(
      heading->error_rad * TELEMETRY_RAD_TO_DEG),
    (long)telemetry_to_milli_units(heading->correction_radps),
    (long)telemetry_to_milli_units(heading->requested_az_radps),
    (long)telemetry_to_milli_units(heading->controlled_az_radps));
}

void telemetry_init(void)
{
  last_control_report_ms = HAL_GetTick();
  last_imu_report_ms = last_control_report_ms;
  last_heading_report_ms = last_control_report_ms;
}

void telemetry_process(void)
{
  uint32_t now = HAL_GetTick();

  if (now - last_control_report_ms >= TELEMETRY_CONTROL_PERIOD_MS) {
    last_control_report_ms = now;
    telemetry_send_control(SERIAL_HOST);
  }

  if (now - last_imu_report_ms >= TELEMETRY_IMU_PERIOD_MS) {
    last_imu_report_ms = now;
    telemetry_send_imu(SERIAL_HOST);
  }

  if (now - last_heading_report_ms >= TELEMETRY_IMU_PERIOD_MS) {
    last_heading_report_ms = now;
    telemetry_send_heading(SERIAL_HOST);
  }

}
