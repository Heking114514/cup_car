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

void communication_init(void);
void communication_process(void);
bool communication_get_command(float *vx_mps, float *az_radps);
void communication_get_command_status(communication_command_status_t *status);

#endif
