#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int16_t left_delta;
  int16_t right_delta;
  int32_t left_total;
  int32_t right_total;
} encoder_data_t;

void encoder_init(void);
void encoder_reset(void);
void encoder_update(void);
const encoder_data_t *encoder_get_data(void);
bool encoder_has_recent_activity(uint32_t timeout_ms);

#endif
