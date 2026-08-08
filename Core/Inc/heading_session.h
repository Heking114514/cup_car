#ifndef HEADING_SESSION_H
#define HEADING_SESSION_H

#include <stdbool.h>

typedef enum {
  HEADING_SESSION_OFF = 0,
  HEADING_SESSION_ACTIVE,
  HEADING_SESSION_UNAVAILABLE,
  HEADING_SESSION_DEGRADED
} heading_session_state_t;

typedef struct {
  heading_session_state_t state;
} heading_session_t;

void heading_session_init(heading_session_t *session);
void heading_session_enter(heading_session_t *session, bool imu_valid);
void heading_session_leave(heading_session_t *session);
void heading_session_monitor_imu(heading_session_t *session, bool imu_valid);
bool heading_session_is_active(const heading_session_t *session);
heading_session_state_t heading_session_get_state(const heading_session_t *session);

#endif
