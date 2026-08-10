#include "communication.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "encoder.h"
#include "serial.h"

#define COMMAND_LINE_MAX       48U
#define COMMAND_TIMEOUT_MS     500U
#define RPY_TIMEOUT_MS         500U
#define ENCODER_REPORT_PERIOD  50U

static char command_line[2][COMMAND_LINE_MAX];
static uint8_t command_length[2];
static uint32_t last_command_ms;
static uint32_t last_report_ms;
static float host_vx_mps;
static float host_az_radps;
static bool host_command_valid;
static uint32_t last_rpy_ms;
static float host_roll_rad;
static float host_pitch_rad;
static float host_yaw_rad;
static bool host_rpy_valid;

static bool communication_parse_float(char **cursor, float *value, char separator)
{
  char *end;

  *value = strtof(*cursor, &end);
  if (end == *cursor || !isfinite(*value)) {
    return false;
  }

  if (separator != '\0') {
    if (*end != separator) {
      return false;
    }
    *cursor = end + 1;
    return true;
  }

  while (*end == ' ' || *end == '\t' || *end == '\r') {
    end++;
  }
  if (*end != '\0') {
    return false;
  }
  *cursor = end;
  return true;
}

static void communication_parse_velocity(char *line)
{
  char *cursor = line;
  float vx;
  float az;

  if (!communication_parse_float(&cursor, &vx, ',') ||
      !communication_parse_float(&cursor, &az, '\0')) {
    return;
  }

  host_vx_mps = vx;
  host_az_radps = az;
  host_command_valid = true;
  last_command_ms = HAL_GetTick();
}

static void communication_parse_rpy(char *line)
{
  char *cursor = line + 4;
  float roll;
  float pitch;
  float yaw;

  if (!communication_parse_float(&cursor, &roll, ',') ||
      !communication_parse_float(&cursor, &pitch, ',') ||
      !communication_parse_float(&cursor, &yaw, '\0')) {
    return;
  }

  host_roll_rad = roll;
  host_pitch_rad = pitch;
  host_yaw_rad = yaw;
  host_rpy_valid = true;
  last_rpy_ms = HAL_GetTick();
}

static void communication_parse_line(serial_port_t port)
{
  if (strncmp(command_line[port], "RPY,", 4U) == 0) {
    communication_parse_rpy(command_line[port]);
  } else {
    communication_parse_velocity(command_line[port]);
  }
}

static void communication_receive(serial_port_t port)
{
  uint8_t byte;

  while (serial_read_byte(port, &byte)) {
    if (byte == '\n') {
      command_line[port][command_length[port]] = '\0';
      communication_parse_line(port);
      command_length[port] = 0;
    } else if (byte != '\r' && command_length[port] < COMMAND_LINE_MAX - 1U) {
      command_line[port][command_length[port]++] = (char)byte;
    } else if (command_length[port] >= COMMAND_LINE_MAX - 1U) {
      command_length[port] = 0;
    }
  }
}

static void communication_send_encoders(void)
{
  const encoder_data_t *encoder = encoder_get_data();

  serial_printf(SERIAL_DEBUG, "ENC,%lu,%lu,%ld,%ld\r\n",
                (unsigned long)encoder->sample_time_ms,
                (unsigned long)encoder->sample_sequence,
                (long)encoder->left_total,
                (long)encoder->right_total);
  serial_printf(SERIAL_AUX, "ENC,%lu,%lu,%ld,%ld\r\n",
                (unsigned long)encoder->sample_time_ms,
                (unsigned long)encoder->sample_sequence,
                (long)encoder->left_total,
                (long)encoder->right_total);
}

void communication_init(void)
{
  command_length[SERIAL_DEBUG] = 0;
  command_length[SERIAL_AUX] = 0;
  host_command_valid = false;
  host_rpy_valid = false;
  last_command_ms = HAL_GetTick();
  last_rpy_ms = HAL_GetTick();
  last_report_ms = HAL_GetTick();
}

void communication_process(void)
{
  uint32_t now = HAL_GetTick();

  communication_receive(SERIAL_DEBUG);
  communication_receive(SERIAL_AUX);

  if (now - last_report_ms >= ENCODER_REPORT_PERIOD) {
    last_report_ms = now;
    communication_send_encoders();
  }
}

bool communication_get_command(float *vx_mps, float *az_radps)
{
  communication_command_status_t status;

  communication_get_command_status(&status);
  if (!status.valid) {
    return false;
  }

  *vx_mps = status.vx_mps;
  *az_radps = status.az_radps;
  return true;
}

void communication_get_command_status(communication_command_status_t *status)
{
  uint32_t now = HAL_GetTick();

  status->received = host_command_valid;
  status->age_ms = host_command_valid ? now - last_command_ms : UINT32_MAX;
  status->valid = host_command_valid && status->age_ms <= COMMAND_TIMEOUT_MS;
  status->vx_mps = host_vx_mps;
  status->az_radps = host_az_radps;
}

bool communication_get_rpy(float *roll_rad, float *pitch_rad, float *yaw_rad)
{
  if (!host_rpy_valid || HAL_GetTick() - last_rpy_ms > RPY_TIMEOUT_MS) {
    return false;
  }

  *roll_rad = host_roll_rad;
  *pitch_rad = host_pitch_rad;
  *yaw_rad = host_yaw_rad;
  return true;
}
