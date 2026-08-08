#include <assert.h>
#include <math.h>

#include "heading_control.h"
#include "heading_session.h"

static void test_session_latching(void)
{
  heading_session_t session;

  heading_session_init(&session);
  assert(heading_session_get_state(&session) == HEADING_SESSION_OFF);

  heading_session_enter(&session, false);
  assert(heading_session_get_state(&session) == HEADING_SESSION_UNAVAILABLE);
  heading_session_monitor_imu(&session, true);
  assert(!heading_session_is_active(&session));

  heading_session_leave(&session);
  heading_session_enter(&session, true);
  assert(heading_session_is_active(&session));
  heading_session_monitor_imu(&session, false);
  assert(heading_session_get_state(&session) == HEADING_SESSION_DEGRADED);
  heading_session_monitor_imu(&session, true);
  assert(!heading_session_is_active(&session));

  heading_session_leave(&session);
  heading_session_enter(&session, true);
  assert(heading_session_is_active(&session));
}

static void test_heading_controller(void)
{
  heading_controller_t controller;
  float correction;

  heading_control_init(&controller);
  assert(!heading_control_has_target(&controller));
  assert(heading_control_update(&controller, 1.0f) == 0.0f);

  heading_control_start(&controller, 0.0f);
  assert(heading_control_update(&controller, 0.002f) == 0.0f);
  assert(heading_control_update(&controller, -1.0f) == 0.5f);
  assert(heading_control_update(&controller, 1.0f) == -0.5f);

  heading_control_start(&controller, 3.13f);
  correction = heading_control_update(&controller, -3.13f);
  assert(correction < 0.0f);
  assert(fabsf(correction) < 0.1f);

  heading_control_stop(&controller);
  assert(!heading_control_has_target(&controller));
}

int main(void)
{
  test_session_latching();
  test_heading_controller();
  return 0;
}
