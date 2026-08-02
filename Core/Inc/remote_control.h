#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

void remote_control_init(void);
void remote_control_process(void);
bool remote_control_get_command(float *vx_mps, float *az_radps);
bool remote_control_button_down(uint16_t button);

#endif
