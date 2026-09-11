#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool received;
  bool valid;
  uint32_t age_ms;
  float vx_mps;
  float az_radps;
} communication_command_status_t;

typedef struct {
  uint32_t sequence;
  int32_t relative_angle_mdeg;
  uint32_t timeout_ms;
} communication_turn_request_t;

void communication_init(void);
void communication_process(void);
bool communication_get_command(float *vx_mps, float *az_radps);
void communication_get_command_status(communication_command_status_t *status);
bool communication_take_turn_request(communication_turn_request_t *request);
bool communication_turn_session_selected(void);
bool communication_turn_watchdog_valid(void);
void communication_cancel_turn_session(void);
void communication_clear_commands(void);

#endif
