#include "car_control.h"

#include "chassis.h"
#include "communication.h"
#include "encoder.h"
#include "heading_control.h"
#include "heading_session.h"
#include "main.h"
#include "route_run.h"

#define CAR_CONTROL_REMOTE_ENABLED 1

#if CAR_CONTROL_REMOTE_ENABLED
#include "ps2.h"
#include "remote_control.h"
#endif

static car_mode_t car_mode;
static bool emergency_stop;
static heading_controller_t navigation_heading_controller;
static heading_session_t navigation_heading_session;
#if CAR_CONTROL_REMOTE_ENABLED
static bool select_was_down;
static bool r2_was_down;
static bool remote_armed;
#endif

static void car_control_enter_navigation(void)
{
  float roll_rad;
  float pitch_rad;
  float yaw_rad = 0.0f;
  bool imu_valid = communication_get_rpy(&roll_rad, &pitch_rad, &yaw_rad);

  heading_session_enter(&navigation_heading_session, imu_valid);
  heading_control_stop(&navigation_heading_controller);
}

static void car_control_process_navigation(void)
{
  float vx_mps;
  float az_radps;
  float roll_rad;
  float pitch_rad;
  float yaw_rad = 0.0f;
  bool imu_valid = communication_get_rpy(&roll_rad, &pitch_rad, &yaw_rad);

  heading_session_monitor_imu(&navigation_heading_session, imu_valid);
  if (!heading_session_is_active(&navigation_heading_session)) {
    heading_control_stop(&navigation_heading_controller);
  }

  if (emergency_stop) {
    heading_control_stop(&navigation_heading_controller);
    chassis_stop();
  } else if (communication_get_command(&vx_mps, &az_radps)) {
    bool straight_requested = chassis_command_is_straight(vx_mps, az_radps);

    if (heading_session_is_active(&navigation_heading_session) &&
        straight_requested) {
      if (!heading_control_has_target(&navigation_heading_controller)) {
        heading_control_start(&navigation_heading_controller, yaw_rad);
      }
      az_radps += heading_control_update(&navigation_heading_controller, yaw_rad);
    } else {
      heading_control_stop(&navigation_heading_controller);
    }
    chassis_set_velocity(vx_mps, az_radps);
  } else {
    heading_control_stop(&navigation_heading_controller);
    chassis_stop();
  }
}

void car_control_init(void)
{
  chassis_init();
  heading_control_init(&navigation_heading_controller);
  heading_session_init(&navigation_heading_session);
#if CAR_CONTROL_REMOTE_ENABLED
  remote_control_init();
#endif
  communication_init();
  route_run_init();
#if CAR_CONTROL_REMOTE_ENABLED
  car_mode = CAR_MODE_REMOTE;
#else
  car_mode = CAR_MODE_NAVIGATION;
  car_control_enter_navigation();
#endif
  emergency_stop = false;
#if CAR_CONTROL_REMOTE_ENABLED
  select_was_down = false;
  r2_was_down = false;
  remote_armed = false;
#endif
  chassis_stop();
}

void car_control_process(void)
{
  float vx_mps;
  float az_radps;

  communication_process();

#if CAR_CONTROL_REMOTE_ENABLED
  bool select_down;
  bool r1_down;
  bool r2_down;
  bool r2_pressed;

  remote_control_process();
  select_down = remote_control_button_down(PS2_BTN_SELECT);
  r1_down = remote_control_button_down(PS2_BTN_R1);
  r2_down = remote_control_button_down(PS2_BTN_R2);
  r2_pressed = r2_down && !r2_was_down;
  r2_was_down = r2_down;

  if (select_down && !select_was_down) {
    car_mode = car_mode == CAR_MODE_REMOTE ? CAR_MODE_NAVIGATION : CAR_MODE_REMOTE;
    emergency_stop = false;
    remote_armed = false;
    route_run_cancel();
    chassis_stop();
    if (car_mode == CAR_MODE_NAVIGATION) {
      car_control_enter_navigation();
    } else {
      heading_session_leave(&navigation_heading_session);
      heading_control_stop(&navigation_heading_controller);
    }
  }
  select_was_down = select_down;

  if (car_mode == CAR_MODE_NAVIGATION) {
    if (r1_down) {
      emergency_stop = true;
    }

    car_control_process_navigation();
  } else if (remote_control_get_command(&vx_mps, &az_radps)) {
    const encoder_data_t *encoder = encoder_get_data();
    bool was_running = route_run_get_status() == ROUTE_RUN_RUNNING;

    route_run_update(encoder->left_total, encoder->right_total, HAL_GetTick());
    if (was_running && route_run_get_status() != ROUTE_RUN_RUNNING) {
      remote_armed = false;
      chassis_stop();
    } else if (route_run_get_status() == ROUTE_RUN_RUNNING) {
      if (r1_down) {
        route_run_cancel();
        remote_armed = false;
        chassis_stop();
      } else if (route_run_get_command(&vx_mps, &az_radps)) {
        if (vx_mps == 0.0f && az_radps == 0.0f) {
          chassis_stop();
        } else {
          chassis_set_velocity(vx_mps, az_radps);
        }
      }
    } else if (r2_pressed &&
               (remote_armed || (vx_mps == 0.0f && az_radps == 0.0f))) {
      remote_armed = true;
      route_run_start(encoder->left_total, encoder->right_total, HAL_GetTick());
      if (route_run_get_command(&vx_mps, &az_radps)) {
        chassis_set_velocity(vx_mps, az_radps);
      }
    } else if (!remote_armed) {
      if (vx_mps == 0.0f && az_radps == 0.0f) {
        remote_armed = true;
      }
      chassis_stop();
    } else {
      chassis_set_velocity(vx_mps, az_radps);
    }
  } else {
    route_run_cancel();
    remote_armed = false;
    chassis_stop();
  }
#else
  car_control_process_navigation();
#endif

  chassis_process();
}

car_mode_t car_control_get_mode(void)
{
  return car_mode;
}

bool car_control_emergency_stopped(void)
{
  return emergency_stop;
}

bool car_control_heading_active(void)
{
  return car_mode == CAR_MODE_NAVIGATION &&
         heading_session_is_active(&navigation_heading_session);
}
