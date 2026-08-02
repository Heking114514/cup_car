#ifndef ROUTE_RUN_H
#define ROUTE_RUN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  ROUTE_RUN_IDLE = 0,
  ROUTE_RUN_RUNNING,
  ROUTE_RUN_COMPLETED,
  ROUTE_RUN_FAULT
} route_run_status_t;

void route_run_init(void);
void route_run_start(int32_t left_total, int32_t right_total, uint32_t now_ms);
void route_run_update(int32_t left_total, int32_t right_total, uint32_t now_ms);
void route_run_cancel(void);
bool route_run_get_command(float *vx_mps, float *az_radps);
route_run_status_t route_run_get_status(void);

#endif
