#include "communication.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "encoder.h"
#include "mpu6050.h"
#include "serial.h"

#define COMMAND_LINE_MAX       48U
#define COMMAND_TIMEOUT_MS     500U
#define ENCODER_REPORT_PERIOD  50U

static char command_line[SERIAL_PORT_COUNT][COMMAND_LINE_MAX];
static uint8_t command_length[SERIAL_PORT_COUNT];
static uint32_t last_command_ms;
static uint32_t last_report_ms;
static float host_vx_mps;
static float host_az_radps;
static bool host_command_valid;

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

static void communication_parse_imu_command(serial_port_t port, char *line)
{
  if (strcmp(line, "IMU,CAL") == 0) {
    mpu6050_start_calibration();
    serial_print(port, "ACK,IMU,CAL\r\n");
  } else if (strcmp(line, "IMU,ZERO") == 0) {
    mpu6050_zero_yaw();
    serial_print(port, "ACK,IMU,ZERO\r\n");
  } else {
    serial_print(port, "ERR,IMU,CMD\r\n");
  }
}

static void communication_parse_line(serial_port_t port)
{
  if (strncmp(command_line[port], "IMU,", 4U) == 0) {
    communication_parse_imu_command(port, command_line[port]);
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

  serial_printf(SERIAL_HOST, "ENC,%lu,%lu,%ld,%ld\r\n",
                (unsigned long)encoder->sample_time_ms,
                (unsigned long)encoder->sample_sequence,
                (long)encoder->left_total,
                (long)encoder->right_total);
}

void communication_init(void)
{
  command_length[SERIAL_DEBUG] = 0;
  command_length[SERIAL_AUX] = 0;
  command_length[SERIAL_HOST] = 0;
  host_command_valid = false;
  last_command_ms = HAL_GetTick();
  last_report_ms = HAL_GetTick();
}

void communication_process(void)
{
  uint32_t now = HAL_GetTick();

  communication_receive(SERIAL_HOST);

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
