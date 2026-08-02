#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>

void communication_init(void);
void communication_process(void);
bool communication_get_command(float *vx_mps, float *az_radps);

#endif
