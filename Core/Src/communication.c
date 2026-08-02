#include "communication.h"

#include <stdlib.h>

#include "main.h"
#include "encoder.h"
#include "serial.h"

#define COMMAND_LINE_MAX       48U
#define COMMAND_TIMEOUT_MS     500U
#define ENCODER_REPORT_PERIOD  50U

static char command_line[2][COMMAND_LINE_MAX];
static uint8_t command_length[2];
static uint32_t last_command_ms;
static uint32_t last_report_ms;
static float host_vx_mps;
static float host_az_radps;
static bool host_command_valid;

static void communication_parse_command(serial_port_t port)
{
  char *end;
  float vx = strtof(command_line[port], &end);
  float az;

  if (*end != ',') {
    return;
  }

  az = strtof(end + 1, &end);
  while (*end == ' ' || *end == '\t' || *end == '\r') {
    end++;
  }
  if (*end != '\0') {
    return;
  }

  host_vx_mps = vx;
  host_az_radps = az;
  host_command_valid = true;
  last_command_ms = HAL_GetTick();
}

static void communication_receive(serial_port_t port)
{
  uint8_t byte;

  while (serial_read_byte(port, &byte)) {
    if (byte == '\n') {
      command_line[port][command_length[port]] = '\0';
      communication_parse_command(port);
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

  serial_printf(SERIAL_DEBUG, "ENC,%ld,%ld\r\n",
                (long)encoder->left_total, (long)encoder->right_total);
  serial_printf(SERIAL_AUX, "ENC,%ld,%ld\r\n",
                (long)encoder->left_total, (long)encoder->right_total);
}

void communication_init(void)
{
  command_length[SERIAL_DEBUG] = 0;
  command_length[SERIAL_AUX] = 0;
  host_command_valid = false;
  last_command_ms = HAL_GetTick();
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
  if (!host_command_valid || HAL_GetTick() - last_command_ms > COMMAND_TIMEOUT_MS) {
    return false;
  }

  *vx_mps = host_vx_mps;
  *az_radps = host_az_radps;
  return true;
}
