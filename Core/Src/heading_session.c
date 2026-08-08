#include "heading_session.h"

void heading_session_init(heading_session_t *session)
{
  session->state = HEADING_SESSION_OFF;
}

void heading_session_enter(heading_session_t *session, bool imu_valid)
{
  session->state = imu_valid ? HEADING_SESSION_ACTIVE : HEADING_SESSION_UNAVAILABLE;
}

void heading_session_leave(heading_session_t *session)
{
  session->state = HEADING_SESSION_OFF;
}

void heading_session_monitor_imu(heading_session_t *session, bool imu_valid)
{
  if (session->state == HEADING_SESSION_ACTIVE && !imu_valid) {
    session->state = HEADING_SESSION_DEGRADED;
  }
}

bool heading_session_is_active(const heading_session_t *session)
{
  return session->state == HEADING_SESSION_ACTIVE;
}

heading_session_state_t heading_session_get_state(const heading_session_t *session)
{
  return session->state;
}
