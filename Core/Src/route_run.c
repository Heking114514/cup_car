#include "route_run.h"

#include "chassis.h"

#define ROUTE_LINEAR_SPEED_MPS        0.30f
#define ROUTE_LINEAR_SLOW_SPEED_MPS   0.15f
#define ROUTE_ANGULAR_SPEED_RADPS     1.00f
#define ROUTE_ANGULAR_SLOW_RADPS      0.50f
#define ROUTE_LINEAR_SLOW_DISTANCE_M  0.20f
#define ROUTE_ANGULAR_SLOW_ANGLE_RAD  0.26179939f
#define ROUTE_SETTLE_MS               500U
#define ROUTE_TIMEOUT_MS              60000U
#define ROUTE_IMBALANCE_MIN_COUNTS    270U
#define ROUTE_PI                      3.14159265f

typedef enum {
  ROUTE_SEGMENT_LINEAR = 0,
  ROUTE_SEGMENT_TURN
} route_segment_type_t;

typedef struct {
  route_segment_type_t type;
  float amount;
  float direction;
} route_segment_t;

static const route_segment_t route_segments[] = {
  { ROUTE_SEGMENT_LINEAR, 3.60f,  1.0f },
  { ROUTE_SEGMENT_TURN,   ROUTE_PI * 0.5f,  1.0f },
  { ROUTE_SEGMENT_LINEAR, 1.20f,  1.0f },
  { ROUTE_SEGMENT_TURN,   ROUTE_PI * 0.5f, -1.0f },
  { ROUTE_SEGMENT_LINEAR, 2.40f,  1.0f }
};

#define ROUTE_SEGMENT_COUNT (sizeof(route_segments) / sizeof(route_segments[0]))

static route_run_status_t route_status;
static uint8_t segment_index;
static bool settling;
static int32_t segment_start_left;
static int32_t segment_start_right;
static uint32_t route_start_ms;
static uint32_t settle_start_ms;
static uint32_t progress_counts;

static uint32_t route_abs_delta(int32_t current, int32_t start)
{
  int64_t delta = (int64_t)current - (int64_t)start;
  return (uint32_t)(delta >= 0 ? delta : -delta);
}

static uint32_t route_linear_counts(float distance_m)
{
  float circumference = 2.0f * ROUTE_PI * CHASSIS_WHEEL_RADIUS_M;
  return (uint32_t)(distance_m * CHASSIS_LEFT_ENCODER_COUNTS_PER_REV /
                    circumference + 0.5f);
}

static uint32_t route_turn_counts(float angle_rad)
{
  float wheel_arc = angle_rad * CHASSIS_TRACK_WIDTH_M * 0.5f;
  return route_linear_counts(wheel_arc);
}

static uint32_t route_reference_counts(uint32_t counts, float counts_per_rev)
{
  return (uint32_t)((float)counts * CHASSIS_LEFT_ENCODER_COUNTS_PER_REV /
                    counts_per_rev + 0.5f);
}

static uint32_t route_segment_target_counts(const route_segment_t *segment)
{
  return segment->type == ROUTE_SEGMENT_LINEAR
             ? route_linear_counts(segment->amount)
             : route_turn_counts(segment->amount);
}

static void route_begin_segment(int32_t left_total, int32_t right_total)
{
  segment_start_left = left_total;
  segment_start_right = right_total;
  progress_counts = 0U;
}

void route_run_init(void)
{
  route_status = ROUTE_RUN_IDLE;
  segment_index = 0U;
  settling = false;
  segment_start_left = 0;
  segment_start_right = 0;
  route_start_ms = 0U;
  settle_start_ms = 0U;
  progress_counts = 0U;
}

void route_run_start(int32_t left_total, int32_t right_total, uint32_t now_ms)
{
  segment_index = 0U;
  settling = false;
  route_start_ms = now_ms;
  route_begin_segment(left_total, right_total);
  route_status = ROUTE_RUN_RUNNING;
}

void route_run_update(int32_t left_total, int32_t right_total, uint32_t now_ms)
{
  const route_segment_t *segment;
  uint32_t left_counts;
  uint32_t right_counts;
  uint32_t larger;
  uint32_t smaller;

  if (route_status != ROUTE_RUN_RUNNING) {
    return;
  }

  if (now_ms - route_start_ms >= ROUTE_TIMEOUT_MS) {
    route_status = ROUTE_RUN_FAULT;
    return;
  }

  if (settling) {
    if (now_ms - settle_start_ms < ROUTE_SETTLE_MS) {
      return;
    }

    settling = false;
    segment_index++;
    if (segment_index >= ROUTE_SEGMENT_COUNT) {
      route_status = ROUTE_RUN_COMPLETED;
      return;
    }
    route_begin_segment(left_total, right_total);
    return;
  }

  segment = &route_segments[segment_index];
  left_counts = route_reference_counts(
    route_abs_delta(left_total, segment_start_left),
    CHASSIS_LEFT_ENCODER_COUNTS_PER_REV);
  right_counts = route_reference_counts(
    route_abs_delta(right_total, segment_start_right),
    CHASSIS_RIGHT_ENCODER_COUNTS_PER_REV);
  progress_counts = (uint32_t)(((uint64_t)left_counts + right_counts) / 2U);

  if (progress_counts >= route_segment_target_counts(segment)) {
    settling = true;
    settle_start_ms = now_ms;
    return;
  }

  larger = left_counts > right_counts ? left_counts : right_counts;
  smaller = left_counts > right_counts ? right_counts : left_counts;
  if (larger >= ROUTE_IMBALANCE_MIN_COUNTS && smaller * 4U < larger) {
    route_status = ROUTE_RUN_FAULT;
  }
}

void route_run_cancel(void)
{
  route_status = ROUTE_RUN_IDLE;
  segment_index = 0U;
  settling = false;
  progress_counts = 0U;
}

bool route_run_get_command(float *vx_mps, float *az_radps)
{
  const route_segment_t *segment;
  uint32_t target_counts;
  uint32_t slow_counts;

  if (route_status != ROUTE_RUN_RUNNING) {
    return false;
  }

  if (settling) {
    *vx_mps = 0.0f;
    *az_radps = 0.0f;
    return true;
  }

  segment = &route_segments[segment_index];
  target_counts = route_segment_target_counts(segment);

  if (segment->type == ROUTE_SEGMENT_LINEAR) {
    slow_counts = route_linear_counts(ROUTE_LINEAR_SLOW_DISTANCE_M);
    *vx_mps = segment->direction *
              (target_counts - progress_counts <= slow_counts
                   ? ROUTE_LINEAR_SLOW_SPEED_MPS
                   : ROUTE_LINEAR_SPEED_MPS);
    *az_radps = 0.0f;
  } else {
    slow_counts = route_turn_counts(ROUTE_ANGULAR_SLOW_ANGLE_RAD);
    *vx_mps = 0.0f;
    *az_radps = segment->direction *
                (target_counts - progress_counts <= slow_counts
                     ? ROUTE_ANGULAR_SLOW_RADPS
                     : ROUTE_ANGULAR_SPEED_RADPS);
  }

  return true;
}

route_run_status_t route_run_get_status(void)
{
  return route_status;
}
