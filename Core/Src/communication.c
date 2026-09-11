#include "communication.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "encoder.h"
#include "serial.h"

#define COMMAND_LINE_MAX       48U
#define COMMAND_TIMEOUT_MS     500U
#define ENCODER_REPORT_PERIOD  50U
#define HOST_MAX_VX_MPS         2.0f
#define HOST_MAX_AZ_RADPS      10.0f
#define TURN_MIN_ANGLE_MDEG    1000L
#define TURN_MAX_ANGLE_MDEG    360000L
#define TURN_MIN_TIMEOUT_MS    1000U
#define TURN_MAX_TIMEOUT_MS    60000U

static char command_line[SERIAL_PORT_COUNT][COMMAND_LINE_MAX];
static uint8_t command_length[SERIAL_PORT_COUNT];
static bool command_discarding[SERIAL_PORT_COUNT];
static uint32_t last_command_ms;
static uint32_t last_report_ms;
static float host_vx_mps;
static float host_az_radps;
static bool host_command_valid;
static communication_turn_request_t turn_request;
static uint32_t last_turn_command_ms;
static bool turn_sequence_seen;
static bool turn_request_pending;
static bool turn_session_selected;

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

static bool communication_parse_u32(char **cursor, uint32_t *value,
                                    char separator)
{
  char *end;
  unsigned long parsed;

  if (**cursor < '0' || **cursor > '9') {
    return false;
  }
  parsed = strtoul(*cursor, &end, 10);
  if (end == *cursor || parsed > UINT32_MAX) {
    return false;
  }
  if (separator != '\0') {
    if (*end != separator) {
      return false;
    }
    *value = (uint32_t)parsed;
    *cursor = end + 1;
    return true;
  }
  while (*end == ' ' || *end == '\t' || *end == '\r') {
    end++;
  }
  if (*end != '\0') {
    return false;
  }
  *value = (uint32_t)parsed;
  *cursor = end;
  return true;
}

static bool communication_parse_i32(char **cursor, int32_t *value,
                                    char separator)
{
  char *end;
  long parsed = strtol(*cursor, &end, 10);

  if (end == *cursor || parsed < INT32_MIN || parsed > INT32_MAX) {
    return false;
  }
  if (separator != '\0') {
    if (*end != separator) {
      return false;
    }
    *cursor = end + 1;
  } else {
    while (*end == ' ' || *end == '\t' || *end == '\r') {
      end++;
    }
    if (*end != '\0') {
      return false;
    }
    *cursor = end;
  }
  *value = (int32_t)parsed;
  return true;
}

static void communication_parse_velocity(char *line)
{
  char *cursor = line;
  float vx;
  float az;

  if (!communication_parse_float(&cursor, &vx, ',') ||
      !communication_parse_float(&cursor, &az, '\0') ||
      fabsf(vx) > HOST_MAX_VX_MPS ||
      fabsf(az) > HOST_MAX_AZ_RADPS) {
    return;
  }

  host_vx_mps = vx;
  host_az_radps = az;
  host_command_valid = true;
  turn_request_pending = false;
  turn_session_selected = false;
  last_command_ms = HAL_GetTick();
}

static void communication_parse_turn_command(serial_port_t port, char *line)
{
  char *cursor;
  uint32_t sequence;
  int32_t angle_mdeg;
  uint32_t timeout_ms;
  uint32_t now = HAL_GetTick();

  if (strcmp(line, "TURN,CANCEL") == 0) {
    turn_request_pending = false;
    turn_session_selected = false;
    host_command_valid = false;
    serial_print(port, "ACK,TURN,CANCEL\r\n");
    return;
  }

  cursor = line + 5;
  if (!communication_parse_u32(&cursor, &sequence, ',') ||
      !communication_parse_i32(&cursor, &angle_mdeg, ',') ||
      !communication_parse_u32(&cursor, &timeout_ms, '\0') ||
      (angle_mdeg >= -TURN_MIN_ANGLE_MDEG &&
       angle_mdeg <= TURN_MIN_ANGLE_MDEG) ||
      angle_mdeg < -TURN_MAX_ANGLE_MDEG ||
      angle_mdeg > TURN_MAX_ANGLE_MDEG ||
      timeout_ms < TURN_MIN_TIMEOUT_MS ||
      timeout_ms > TURN_MAX_TIMEOUT_MS) {
    serial_print(port, "ERR,TURN,FORMAT\r\n");
    return;
  }

  if (turn_sequence_seen && sequence == turn_request.sequence) {
    if (angle_mdeg != turn_request.relative_angle_mdeg ||
        timeout_ms != turn_request.timeout_ms) {
      serial_print(port, "ERR,TURN,SEQUENCE\r\n");
      return;
    }
  } else {
    turn_request.sequence = sequence;
    turn_request.relative_angle_mdeg = angle_mdeg;
    turn_request.timeout_ms = timeout_ms;
    turn_sequence_seen = true;
    turn_request_pending = true;
    serial_printf(port, "ACK,TURN,%lu\r\n", (unsigned long)sequence);
  }

  host_command_valid = false;
  turn_session_selected = true;
  last_turn_command_ms = now;
}

static void communication_parse_line(serial_port_t port)
{
  if (strncmp(command_line[port], "TURN,", 5U) == 0) {
    communication_parse_turn_command(port, command_line[port]);
  } else {
    communication_parse_velocity(command_line[port]);
  }
}

static void communication_receive(serial_port_t port)
{
  uint8_t byte;

  while (serial_read_byte(port, &byte)) {
    if (command_discarding[port]) {
      if (byte == '\n') {
        command_discarding[port] = false;
        command_length[port] = 0U;
      }
      continue;
    }

    if (byte == '\n') {
      command_line[port][command_length[port]] = '\0';
      communication_parse_line(port);
      command_length[port] = 0U;
    } else if (byte == '\r') {
      continue;
    } else if (command_length[port] < COMMAND_LINE_MAX - 1U) {
      command_line[port][command_length[port]++] = (char)byte;
    } else {
      command_length[port] = 0U;
      command_discarding[port] = true;
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
  command_discarding[SERIAL_DEBUG] = false;
  command_discarding[SERIAL_AUX] = false;
  command_discarding[SERIAL_HOST] = false;
  host_vx_mps = 0.0f;
  host_az_radps = 0.0f;
  host_command_valid = false;
  turn_sequence_seen = false;
  turn_request_pending = false;
  turn_session_selected = false;
  last_command_ms = HAL_GetTick();
  last_turn_command_ms = last_command_ms;
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

bool communication_take_turn_request(communication_turn_request_t *request)
{
  if (!turn_request_pending || !turn_session_selected) {
    return false;
  }
  *request = turn_request;
  turn_request_pending = false;
  return true;
}

bool communication_turn_session_selected(void)
{
  return turn_session_selected;
}

bool communication_turn_watchdog_valid(void)
{
  return turn_session_selected &&
         HAL_GetTick() - last_turn_command_ms <= COMMAND_TIMEOUT_MS;
}

void communication_cancel_turn_session(void)
{
  turn_request_pending = false;
  turn_session_selected = false;
}

void communication_clear_commands(void)
{
  host_command_valid = false;
  host_vx_mps = 0.0f;
  host_az_radps = 0.0f;
  communication_cancel_turn_session();
}
