#include "car_control.h"

#include "chassis.h"
#include "communication.h"
#include "encoder.h"
#include "heading_control.h"
#include "heading_session.h"
#include "main.h"
#include "mpu6050.h"
#include "turn_control.h"

#define CAR_CONTROL_REMOTE_ENABLED 0
#define CAR_CONTROL_HEADING_HOLD_ENABLED 1
#define CAR_CONTROL_GYRO_DEG_TO_RAD 0.01745329252f
#define CAR_CONTROL_MDEG_TO_RAD      0.00001745329252f
#define CAR_CONTROL_LOW_SPEED_CORRECTION_RATIO 0.80f
#define CAR_CONTROL_PIVOT_VX_EPSILON_MPS       0.01f
#define CAR_CONTROL_PIVOT_AZ_EPSILON_RADPS     0.001f

#if CAR_CONTROL_REMOTE_ENABLED
#include "ps2.h"
#include "remote_control.h"
#include "route_run.h"
#endif

static car_mode_t car_mode;
static bool emergency_stop;
static heading_controller_t straight_heading_controller;
static heading_session_t imu_heading_session;
static car_heading_feedback_t heading_feedback;
static turn_controller_t angle_turn_controller;
static turn_controller_t rate_turn_controller;
#if CAR_CONTROL_REMOTE_ENABLED
static bool select_was_down;
static bool r2_was_down;
static bool remote_armed;
#endif

static float car_control_abs(float value)
{
  return value >= 0.0f ? value : -value;
}

static float car_control_clamp(float value, float limit)
{
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

static bool car_control_rate_turn_pending(void)
{
  return turn_control_is_active(&rate_turn_controller) ||
         rate_turn_controller.state == TURN_CONTROL_STATE_IMU_FAULT ||
         rate_turn_controller.state == TURN_CONTROL_STATE_RATE_FAULT;
}

static void car_control_release_heading(void)
{
  heading_control_stop(&straight_heading_controller);
  heading_feedback.active = false;
  heading_feedback.error_rad = 0.0f;
  heading_feedback.correction_radps = 0.0f;
}

static void car_control_sync_turn_feedback(bool imu_valid)
{
  const turn_controller_t *turn =
    turn_control_get_feedback(&angle_turn_controller);

  heading_feedback.sample_time_ms = turn->sample_time_ms;
  heading_feedback.active = turn_control_is_active(turn);
  heading_feedback.imu_valid = imu_valid;
  heading_feedback.target_yaw_rad = turn->target_yaw_rad;
  heading_feedback.measured_yaw_rad = turn->measured_yaw_rad;
  heading_feedback.error_rad = turn->error_rad;
  heading_feedback.correction_radps = turn->requested_rate_radps;
  heading_feedback.requested_az_radps = 0.0f;
  heading_feedback.controlled_az_radps = turn->target_rate_radps;
}

static void car_control_apply_turn_action(const turn_controller_t *turn)
{
  if (turn->action == TURN_CONTROL_ACTION_DRIVE) {
    chassis_set_pivot_pwm(turn->pivot_pwm);
  } else if (turn->action == TURN_CONTROL_ACTION_BRAKE) {
    chassis_brake();
  } else {
    chassis_stop();
  }
}

static void car_control_stop_motion(void)
{
  const mpu6050_data_t *imu = mpu6050_get_data();
  float yaw_rad = imu->yaw_rad;

  car_control_release_heading();
  turn_control_cancel(&angle_turn_controller);
  turn_control_init(&rate_turn_controller);
  heading_feedback.sample_time_ms = imu->sample_time_ms;
  heading_feedback.imu_valid = mpu6050_get_yaw(&yaw_rad);
  heading_feedback.target_yaw_rad = yaw_rad;
  heading_feedback.measured_yaw_rad = yaw_rad;
  heading_feedback.requested_az_radps = 0.0f;
  heading_feedback.controlled_az_radps = 0.0f;
  chassis_stop();
}

static void car_control_drive(float vx_mps, float requested_az_radps)
{
  const mpu6050_data_t *imu = mpu6050_get_data();
  float yaw_rad = imu->yaw_rad;
  float controlled_az_radps = requested_az_radps;
  bool imu_valid = mpu6050_get_yaw(&yaw_rad);

  heading_session_monitor_imu(&imu_heading_session, imu_valid);
  heading_feedback.sample_time_ms = imu->sample_time_ms;
  heading_feedback.imu_valid = imu_valid;
  heading_feedback.measured_yaw_rad = yaw_rad;
  heading_feedback.requested_az_radps = requested_az_radps;

  if (car_control_rate_turn_pending() ||
      (car_control_abs(vx_mps) <= CAR_CONTROL_PIVOT_VX_EPSILON_MPS &&
       car_control_abs(requested_az_radps) >
         CAR_CONTROL_PIVOT_AZ_EPSILON_RADPS)) {
    const turn_controller_t *turn;
    float rate_command =
      car_control_abs(vx_mps) <= CAR_CONTROL_PIVOT_VX_EPSILON_MPS
        ? requested_az_radps
        : 0.0f;

    car_control_release_heading();
    heading_feedback.target_yaw_rad = yaw_rad;
    heading_feedback.error_rad = 0.0f;
    if (!imu_valid) {
      turn_control_imu_fault(&rate_turn_controller);
      heading_feedback.active = false;
      heading_feedback.controlled_az_radps = 0.0f;
      chassis_brake();
      return;
    }

    turn_control_follow_rate(
      &rate_turn_controller,
      rate_command,
      imu->yaw_rate_dps * CAR_CONTROL_GYRO_DEG_TO_RAD,
      imu->sample_time_ms,
      HAL_GetTick());
    turn = turn_control_get_feedback(&rate_turn_controller);
    heading_feedback.active = turn_control_is_active(turn);
    heading_feedback.controlled_az_radps = turn->target_rate_radps;
    if (car_control_abs(vx_mps) <= CAR_CONTROL_PIVOT_VX_EPSILON_MPS ||
        turn_control_is_active(turn)) {
      car_control_apply_turn_action(turn);
      return;
    }
  }

  turn_control_init(&rate_turn_controller);

#if CAR_CONTROL_HEADING_HOLD_ENABLED
  if (imu_valid && heading_session_is_active(&imu_heading_session) &&
      chassis_command_is_straight(vx_mps, requested_az_radps)) {
    float correction;
    float correction_limit =
      2.0f * car_control_abs(vx_mps) /
      CHASSIS_TRACK_WIDTH_M * CAR_CONTROL_LOW_SPEED_CORRECTION_RATIO;

    if (!heading_control_has_target(&straight_heading_controller)) {
      heading_control_start(&straight_heading_controller, yaw_rad);
    }
    correction = heading_control_update(
      &straight_heading_controller,
      yaw_rad,
      imu->yaw_rate_dps * CAR_CONTROL_GYRO_DEG_TO_RAD,
      imu->sample_time_ms);
    correction = car_control_clamp(correction, correction_limit);
    controlled_az_radps += correction;
    heading_feedback.active = true;
    heading_feedback.target_yaw_rad =
      straight_heading_controller.target_yaw_rad;
    heading_feedback.error_rad = straight_heading_controller.error_rad;
    heading_feedback.correction_radps = correction;
  } else
#endif
  {
    car_control_release_heading();
    heading_feedback.target_yaw_rad = yaw_rad;
  }

  heading_feedback.controlled_az_radps = controlled_az_radps;
  chassis_set_velocity(vx_mps, controlled_az_radps);
}

static void car_control_enter_navigation(void)
{
  car_control_release_heading();
}

static void car_control_process_navigation(void)
{
  const mpu6050_data_t *imu = mpu6050_get_data();
  communication_turn_request_t turn_request;
  float vx_mps;
  float az_radps;
  float yaw_rad = imu->yaw_rad;
  bool imu_valid = mpu6050_get_yaw(&yaw_rad);

  if (emergency_stop) {
    communication_clear_commands();
    car_control_stop_motion();
    return;
  }

  if (communication_take_turn_request(&turn_request)) {
    float turn_start_yaw_rad = imu_valid ? yaw_rad : 0.0f;

    car_control_release_heading();
    turn_control_init(&rate_turn_controller);
    if (!turn_control_start(
          &angle_turn_controller,
          turn_request.sequence,
          (float)turn_request.relative_angle_mdeg * CAR_CONTROL_MDEG_TO_RAD,
          turn_request.timeout_ms,
          turn_start_yaw_rad,
          HAL_GetTick()) || !imu_valid) {
      turn_control_imu_fault(&angle_turn_controller);
    }
  }

  if (communication_turn_session_selected()) {
    if (!communication_turn_watchdog_valid()) {
      turn_control_watchdog_fault(&angle_turn_controller);
      car_control_sync_turn_feedback(imu_valid);
      chassis_brake();
      return;
    }
    if (turn_control_is_active(&angle_turn_controller)) {
      if (!imu_valid) {
        turn_control_imu_fault(&angle_turn_controller);
      } else {
        turn_control_update(
          &angle_turn_controller,
          yaw_rad,
          imu->yaw_rate_dps * CAR_CONTROL_GYRO_DEG_TO_RAD,
          imu->sample_time_ms,
          HAL_GetTick());
      }
    }
    car_control_sync_turn_feedback(imu_valid);
    car_control_apply_turn_action(
      turn_control_get_feedback(&angle_turn_controller));
    return;
  }

  if (turn_control_is_active(&angle_turn_controller)) {
    turn_control_cancel(&angle_turn_controller);
  }

  if (communication_get_command(&vx_mps, &az_radps)) {
    car_control_drive(vx_mps, az_radps);
  } else {
    car_control_stop_motion();
  }
}

void car_control_init(void)
{
  chassis_init();
  heading_control_init(&straight_heading_controller);
  turn_control_init(&angle_turn_controller);
  turn_control_init(&rate_turn_controller);
  heading_session_init(&imu_heading_session);
  heading_session_enter(&imu_heading_session, mpu6050_get_yaw(NULL));
  heading_feedback.sample_time_ms = 0U;
  heading_feedback.active = false;
  heading_feedback.imu_valid = false;
  heading_feedback.target_yaw_rad = 0.0f;
  heading_feedback.measured_yaw_rad = 0.0f;
  heading_feedback.error_rad = 0.0f;
  heading_feedback.correction_radps = 0.0f;
  heading_feedback.requested_az_radps = 0.0f;
  heading_feedback.controlled_az_radps = 0.0f;
#if CAR_CONTROL_REMOTE_ENABLED
  remote_control_init();
#endif
  communication_init();
#if CAR_CONTROL_REMOTE_ENABLED
  route_run_init();
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
  car_control_stop_motion();
}

void car_control_process(void)
{
#if CAR_CONTROL_REMOTE_ENABLED
  float vx_mps;
  float az_radps;
  bool select_down;
  bool r1_down;
  bool r2_down;
  bool r2_pressed;
#endif

  communication_process();

#if CAR_CONTROL_REMOTE_ENABLED

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
    communication_clear_commands();
    car_control_stop_motion();
    if (car_mode == CAR_MODE_NAVIGATION) {
      car_control_enter_navigation();
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

    communication_cancel_turn_session();
    route_run_update(encoder->left_total, encoder->right_total, HAL_GetTick());
    if (was_running && route_run_get_status() != ROUTE_RUN_RUNNING) {
      remote_armed = false;
      car_control_stop_motion();
    } else if (route_run_get_status() == ROUTE_RUN_RUNNING) {
      if (r1_down) {
        route_run_cancel();
        remote_armed = false;
        car_control_stop_motion();
      } else if (route_run_get_command(&vx_mps, &az_radps)) {
        if (vx_mps == 0.0f && az_radps == 0.0f) {
          car_control_stop_motion();
        } else {
          car_control_drive(vx_mps, az_radps);
        }
      }
    } else if (r2_pressed &&
               (remote_armed || (vx_mps == 0.0f && az_radps == 0.0f))) {
      remote_armed = true;
      route_run_start(encoder->left_total, encoder->right_total, HAL_GetTick());
      if (route_run_get_command(&vx_mps, &az_radps)) {
        car_control_drive(vx_mps, az_radps);
      }
    } else if (!remote_armed) {
      if (vx_mps == 0.0f && az_radps == 0.0f) {
        remote_armed = true;
      }
      car_control_stop_motion();
    } else {
      car_control_drive(vx_mps, az_radps);
    }
  } else {
    communication_cancel_turn_session();
    route_run_cancel();
    remote_armed = false;
    car_control_stop_motion();
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
  return heading_feedback.active;
}

bool car_control_straight_heading_active(void)
{
  return heading_feedback.active &&
         !turn_control_is_active(&angle_turn_controller) &&
         !turn_control_is_active(&rate_turn_controller);
}

const car_heading_feedback_t *car_control_get_heading_feedback(void)
{
  return &heading_feedback;
}
