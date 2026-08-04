#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>

void communication_init(void);
void communication_process(void);
bool communication_get_command(float *vx_mps, float *az_radps);
bool communication_get_rpy(float *roll_rad, float *pitch_rad, float *yaw_rad);

#endif
