#include "mpu6050.h"

#include <math.h>
#include <string.h>

#include "main.h"

#define MPU6050_ADDRESS_LOW              0x68U
#define MPU6050_ADDRESS_HIGH             0x69U
#define MPU6050_REG_SMPLRT_DIV           0x19U
#define MPU6050_REG_CONFIG               0x1AU
#define MPU6050_REG_GYRO_CONFIG          0x1BU
#define MPU6050_REG_ACCEL_CONFIG         0x1CU
#define MPU6050_REG_FIFO_EN              0x23U
#define MPU6050_REG_INT_ENABLE           0x38U
#define MPU6050_REG_ACCEL_XOUT_H         0x3BU
#define MPU6050_REG_USER_CTRL            0x6AU
#define MPU6050_REG_PWR_MGMT_1           0x6BU
#define MPU6050_REG_PWR_MGMT_2           0x6CU
#define MPU6050_REG_WHO_AM_I             0x75U

#define MPU6050_SAMPLE_PERIOD_MS         10U
#define MPU6050_POWER_UP_DELAY_MS        100U
#define MPU6050_RESET_DELAY_MS           100U
#define MPU6050_RETRY_PERIOD_MS          1000U
#define MPU6050_I2C_TIMEOUT_MS           5U
#define MPU6050_MAX_CONSECUTIVE_ERRORS   5U

#define MPU6050_ACCEL_LSB_PER_G           8192.0f
#define MPU6050_GYRO_LSB_PER_DPS          65.5f
#define MPU6050_TEMP_LSB_PER_C            340.0f
#define MPU6050_TEMP_OFFSET_C             36.53f

#define MPU6050_CAL_GYRO_LIMIT_DPS        5.0f
#define MPU6050_CAL_ACCEL_MIN_G           0.90f
#define MPU6050_CAL_ACCEL_MAX_G           1.10f
#define MPU6050_STILL_GYRO_LIMIT_DPS      0.75f
#define MPU6050_STILL_ACCEL_MIN_G         0.95f
#define MPU6050_STILL_ACCEL_MAX_G         1.05f
#define MPU6050_STILL_CONFIRM_SAMPLES     50U
#define MPU6050_BIAS_TIME_CONSTANT_S      20.0f
#define MPU6050_MOVING_BIAS_BLOCK_SAMPLES 200U
#define MPU6050_MOVING_BIAS_BLOCK_COUNT   15U
#define MPU6050_MOVING_BIAS_RATE_LIMIT_DPS 8.0f
#define MPU6050_MOVING_BIAS_ACCEPT_DPS    0.12f
#define MPU6050_MOVING_BIAS_GAIN          0.25f
#define MPU6050_MOVING_BIAS_MAX_STEP_DPS  0.01f
#define MPU6050_MOVING_ACCEL_MIN_G        0.85f
#define MPU6050_MOVING_ACCEL_MAX_G        1.15f
#define MPU6050_MOVING_HEADING_ERROR_LIMIT_RAD 0.034906585f
#define MPU6050_MOVING_CORRECTION_LIMIT_RADPS  0.08f
#define MPU6050_TILT_TIME_CONSTANT_S      0.50f
#define MPU6050_DEFAULT_DT_S              0.010f
#define MPU6050_MIN_DT_S                  0.005f
#define MPU6050_MAX_DT_S                  0.050f

#define MPU6050_DEG_TO_RAD                0.01745329252f
#define MPU6050_RAD_TO_DEG                57.2957795f
#define MPU6050_PI                        3.14159265f
#define MPU6050_TWO_PI                    6.28318531f

static I2C_HandleTypeDef *mpu_i2c;
static uint16_t mpu_address_hal;
static uint32_t next_action_ms;
static uint32_t last_sample_ms;
static uint16_t still_sample_count;
static float calibration_mean_x;
static float calibration_mean_y;
static float calibration_mean_z;
static bool vehicle_stopped_hint;
static bool straight_motion_hint;
static float heading_error_hint_rad;
static float heading_correction_hint_radps;
static bool configured_once;
static float moving_bias_block_sum_z;
static uint16_t moving_bias_block_samples;
static float moving_bias_block_means[MPU6050_MOVING_BIAS_BLOCK_COUNT];
static uint8_t moving_bias_block_count;
static mpu6050_data_t mpu_data;

static bool mpu6050_time_reached(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0;
}

static float mpu6050_wrap_pi(float angle_rad)
{
  while (angle_rad > MPU6050_PI) {
    angle_rad -= MPU6050_TWO_PI;
  }
  while (angle_rad < -MPU6050_PI) {
    angle_rad += MPU6050_TWO_PI;
  }
  return angle_rad;
}

static float mpu6050_abs(float value)
{
  return value >= 0.0f ? value : -value;
}

static float mpu6050_clamp(float value, float limit)
{
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

static void mpu6050_reset_moving_bias_window(void)
{
  moving_bias_block_sum_z = 0.0f;
  moving_bias_block_samples = 0U;
  moving_bias_block_count = 0U;
  mpu_data.moving_bias_active = false;
}

static float mpu6050_median(const float *values, uint8_t count)
{
  float sorted[MPU6050_MOVING_BIAS_BLOCK_COUNT];
  uint32_t index;

  memcpy(sorted, values, (size_t)count * sizeof(sorted[0]));
  for (index = 1U; index < count; index++) {
    float value = sorted[index];
    uint32_t position = index;

    while (position > 0U && sorted[position - 1U] > value) {
      sorted[position] = sorted[position - 1U];
      position--;
    }
    sorted[position] = value;
  }
  return sorted[count / 2U];
}

static void mpu6050_update_moving_bias(float accel_norm_squared)
{
  float residual_dps;
  float accel_min_squared = MPU6050_MOVING_ACCEL_MIN_G *
                            MPU6050_MOVING_ACCEL_MIN_G;
  float accel_max_squared = MPU6050_MOVING_ACCEL_MAX_G *
                            MPU6050_MOVING_ACCEL_MAX_G;

  if (!straight_motion_hint || vehicle_stopped_hint) {
    mpu6050_reset_moving_bias_window();
    return;
  }
  if (accel_norm_squared < accel_min_squared ||
      accel_norm_squared > accel_max_squared) {
    mpu_data.moving_bias_active = false;
    return;
  }
  if (mpu6050_abs(heading_error_hint_rad) >
        MPU6050_MOVING_HEADING_ERROR_LIMIT_RAD ||
      mpu6050_abs(heading_correction_hint_radps) >
        MPU6050_MOVING_CORRECTION_LIMIT_RADPS) {
    mpu_data.moving_bias_active = false;
    return;
  }

  mpu_data.moving_bias_active = true;
  residual_dps = mpu6050_clamp(
    mpu_data.gyro_raw_z_dps - mpu_data.gyro_bias_z_dps,
    MPU6050_MOVING_BIAS_RATE_LIMIT_DPS);
  moving_bias_block_sum_z += residual_dps;
  moving_bias_block_samples++;
  if (moving_bias_block_samples < MPU6050_MOVING_BIAS_BLOCK_SAMPLES) {
    return;
  }

  moving_bias_block_means[moving_bias_block_count] =
    moving_bias_block_sum_z / (float)moving_bias_block_samples;
  moving_bias_block_count++;
  moving_bias_block_sum_z = 0.0f;
  moving_bias_block_samples = 0U;
  if (moving_bias_block_count >= MPU6050_MOVING_BIAS_BLOCK_COUNT) {
    float median_residual = mpu6050_median(
      moving_bias_block_means,
      MPU6050_MOVING_BIAS_BLOCK_COUNT);

    if (mpu6050_abs(median_residual) <= MPU6050_MOVING_BIAS_ACCEPT_DPS) {
      float bias_step = mpu6050_clamp(
        median_residual * MPU6050_MOVING_BIAS_GAIN,
        MPU6050_MOVING_BIAS_MAX_STEP_DPS);

      mpu_data.gyro_bias_z_dps += bias_step;
      mpu_data.moving_bias_update_count++;
    }
    moving_bias_block_count = 0U;
  }
}

static int16_t mpu6050_decode_i16(const uint8_t *bytes)
{
  return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static bool mpu6050_read(uint8_t reg, uint8_t *buffer, uint16_t length)
{
  return HAL_I2C_Mem_Read(
           mpu_i2c, mpu_address_hal, reg, I2C_MEMADD_SIZE_8BIT,
           buffer, length, MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool mpu6050_write(uint8_t reg, const uint8_t *buffer, uint16_t length)
{
  return HAL_I2C_Mem_Write(
           mpu_i2c, mpu_address_hal, reg, I2C_MEMADD_SIZE_8BIT,
           (uint8_t *)buffer, length, MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool mpu6050_write_byte(uint8_t reg, uint8_t value)
{
  return mpu6050_write(reg, &value, 1U);
}

static bool mpu6050_who_am_i_valid(uint8_t value)
{
  return value == MPU6050_ADDRESS_LOW || value == MPU6050_ADDRESS_HIGH;
}

static bool mpu6050_probe(void)
{
  static const uint8_t addresses[] = {
    MPU6050_ADDRESS_LOW,
    MPU6050_ADDRESS_HIGH
  };
  uint32_t index;

  for (index = 0U; index < sizeof(addresses); index++) {
    uint8_t who_am_i = 0U;

    mpu_address_hal = (uint16_t)addresses[index] << 1;
    if (HAL_I2C_IsDeviceReady(
          mpu_i2c, mpu_address_hal, 1U, MPU6050_I2C_TIMEOUT_MS) == HAL_OK &&
        mpu6050_read(MPU6050_REG_WHO_AM_I, &who_am_i, 1U) &&
        mpu6050_who_am_i_valid(who_am_i)) {
      mpu_data.address_7bit = addresses[index];
      mpu_data.who_am_i = who_am_i;
      return true;
    }
  }

  mpu_address_hal = 0U;
  mpu_data.address_7bit = 0U;
  mpu_data.who_am_i = 0U;
  return false;
}

static bool mpu6050_configure(void)
{
  const uint8_t sample_config[4] = {
    9U,     /* 1 kHz / (1 + 9) = 100 Hz */
    4U,     /* 21 Hz gyro and accelerometer digital low-pass filter */
    0x08U,  /* +/-500 degrees/s */
    0x08U   /* +/-4 g */
  };
  uint8_t verify[4];
  uint8_t power_management;

  if (!mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x01U) ||
      !mpu6050_write_byte(MPU6050_REG_PWR_MGMT_2, 0x00U) ||
      !mpu6050_write(MPU6050_REG_SMPLRT_DIV, sample_config,
                     sizeof(sample_config)) ||
      !mpu6050_write_byte(MPU6050_REG_FIFO_EN, 0x00U) ||
      !mpu6050_write_byte(MPU6050_REG_INT_ENABLE, 0x00U) ||
      !mpu6050_write_byte(MPU6050_REG_USER_CTRL, 0x00U) ||
      !mpu6050_read(MPU6050_REG_SMPLRT_DIV, verify, sizeof(verify)) ||
      !mpu6050_read(MPU6050_REG_PWR_MGMT_1, &power_management, 1U)) {
    return false;
  }

  return memcmp(sample_config, verify, sizeof(sample_config)) == 0 &&
         (power_management & 0x47U) == 0x01U;
}

static void mpu6050_schedule_error(uint32_t now)
{
  mpu_data.state = MPU6050_STATE_ERROR;
  mpu_data.sample_valid = false;
  mpu_data.stationary = false;
  mpu6050_reset_moving_bias_window();
  next_action_ms = now + MPU6050_RETRY_PERIOD_MS;
}

static void mpu6050_begin_calibration(uint32_t now)
{
  calibration_mean_x = 0.0f;
  calibration_mean_y = 0.0f;
  calibration_mean_z = 0.0f;
  still_sample_count = 0U;
  mpu_data.calibration_samples = 0U;
  mpu_data.stationary = false;
  mpu_data.moving_bias_update_count = 0U;
  mpu6050_reset_moving_bias_window();
  mpu_data.state = MPU6050_STATE_CALIBRATING;
  last_sample_ms = now;
}

static bool mpu6050_sample_within_limits(float gyro_x_dps,
                                         float gyro_y_dps,
                                         float gyro_z_dps,
                                         float accel_norm_squared,
                                         float gyro_limit_dps,
                                         float accel_min_g,
                                         float accel_max_g)
{
  float gyro_norm_squared = gyro_x_dps * gyro_x_dps +
                            gyro_y_dps * gyro_y_dps +
                            gyro_z_dps * gyro_z_dps;

  return gyro_norm_squared <= gyro_limit_dps * gyro_limit_dps &&
         accel_norm_squared >= accel_min_g * accel_min_g &&
         accel_norm_squared <= accel_max_g * accel_max_g;
}

static void mpu6050_initialize_tilt(void)
{
  mpu_data.roll_rad = atan2f(mpu_data.accel_y_g, mpu_data.accel_z_g);
  mpu_data.pitch_rad = atan2f(
    -mpu_data.accel_x_g,
    sqrtf(mpu_data.accel_y_g * mpu_data.accel_y_g +
          mpu_data.accel_z_g * mpu_data.accel_z_g));
  mpu_data.yaw_rad = 0.0f;
  mpu_data.yaw_rate_dps = 0.0f;
}

static void mpu6050_calibrate(float accel_norm_squared)
{
  uint16_t sample_number;
  float inverse_sample_number;

  if (!vehicle_stopped_hint ||
      !mpu6050_sample_within_limits(
        mpu_data.gyro_raw_x_dps,
        mpu_data.gyro_raw_y_dps,
        mpu_data.gyro_raw_z_dps,
        accel_norm_squared,
        MPU6050_CAL_GYRO_LIMIT_DPS,
        MPU6050_CAL_ACCEL_MIN_G,
        MPU6050_CAL_ACCEL_MAX_G)) {
    calibration_mean_x = 0.0f;
    calibration_mean_y = 0.0f;
    calibration_mean_z = 0.0f;
    mpu_data.calibration_samples = 0U;
    return;
  }

  sample_number = (uint16_t)(mpu_data.calibration_samples + 1U);
  inverse_sample_number = 1.0f / (float)sample_number;
  calibration_mean_x +=
    (mpu_data.gyro_raw_x_dps - calibration_mean_x) * inverse_sample_number;
  calibration_mean_y +=
    (mpu_data.gyro_raw_y_dps - calibration_mean_y) * inverse_sample_number;
  calibration_mean_z +=
    (mpu_data.gyro_raw_z_dps - calibration_mean_z) * inverse_sample_number;
  mpu_data.calibration_samples = sample_number;

  if (sample_number < MPU6050_CALIBRATION_REQUIRED_SAMPLES) {
    return;
  }

  mpu_data.gyro_bias_x_dps = calibration_mean_x;
  mpu_data.gyro_bias_y_dps = calibration_mean_y;
  mpu_data.gyro_bias_z_dps = calibration_mean_z;
  mpu_data.gyro_x_dps =
    mpu_data.gyro_raw_x_dps - mpu_data.gyro_bias_x_dps;
  mpu_data.gyro_y_dps =
    mpu_data.gyro_raw_y_dps - mpu_data.gyro_bias_y_dps;
  mpu_data.gyro_z_dps =
    mpu_data.gyro_raw_z_dps - mpu_data.gyro_bias_z_dps;
  still_sample_count = MPU6050_STILL_CONFIRM_SAMPLES;
  mpu_data.stationary = true;
  mpu6050_initialize_tilt();
  mpu_data.state = MPU6050_STATE_READY;
}

static void mpu6050_update_orientation(float dt_s,
                                       float accel_norm_squared)
{
  bool still_candidate;
  float gyro_x_radps;
  float gyro_y_radps;
  float gyro_z_radps;
  float accel_roll_rad;
  float accel_pitch_rad;
  float blend;
  float cosine_pitch;
  float yaw_rate_radps;

  mpu_data.gyro_x_dps =
    mpu_data.gyro_raw_x_dps - mpu_data.gyro_bias_x_dps;
  mpu_data.gyro_y_dps =
    mpu_data.gyro_raw_y_dps - mpu_data.gyro_bias_y_dps;
  mpu_data.gyro_z_dps =
    mpu_data.gyro_raw_z_dps - mpu_data.gyro_bias_z_dps;

  still_candidate = vehicle_stopped_hint &&
    mpu6050_sample_within_limits(
      mpu_data.gyro_x_dps,
      mpu_data.gyro_y_dps,
      mpu_data.gyro_z_dps,
      accel_norm_squared,
      MPU6050_STILL_GYRO_LIMIT_DPS,
      MPU6050_STILL_ACCEL_MIN_G,
      MPU6050_STILL_ACCEL_MAX_G);

  if (still_candidate) {
    if (still_sample_count < MPU6050_STILL_CONFIRM_SAMPLES) {
      still_sample_count++;
    }
  } else {
    still_sample_count = 0U;
  }
  mpu_data.stationary =
    still_sample_count >= MPU6050_STILL_CONFIRM_SAMPLES;

  if (mpu_data.stationary) {
    float bias_blend = dt_s / (MPU6050_BIAS_TIME_CONSTANT_S + dt_s);

    mpu_data.gyro_bias_x_dps +=
      (mpu_data.gyro_raw_x_dps - mpu_data.gyro_bias_x_dps) * bias_blend;
    mpu_data.gyro_bias_y_dps +=
      (mpu_data.gyro_raw_y_dps - mpu_data.gyro_bias_y_dps) * bias_blend;
    mpu_data.gyro_bias_z_dps +=
      (mpu_data.gyro_raw_z_dps - mpu_data.gyro_bias_z_dps) * bias_blend;
    mpu_data.gyro_x_dps =
      mpu_data.gyro_raw_x_dps - mpu_data.gyro_bias_x_dps;
    mpu_data.gyro_y_dps =
      mpu_data.gyro_raw_y_dps - mpu_data.gyro_bias_y_dps;
    mpu_data.gyro_z_dps =
      mpu_data.gyro_raw_z_dps - mpu_data.gyro_bias_z_dps;
    mpu6050_reset_moving_bias_window();
  } else {
    mpu6050_update_moving_bias(accel_norm_squared);
    mpu_data.gyro_z_dps =
      mpu_data.gyro_raw_z_dps - mpu_data.gyro_bias_z_dps;
  }

  gyro_x_radps = mpu_data.gyro_x_dps * MPU6050_DEG_TO_RAD;
  gyro_y_radps = mpu_data.gyro_y_dps * MPU6050_DEG_TO_RAD;
  gyro_z_radps = mpu_data.gyro_z_dps * MPU6050_DEG_TO_RAD;

  if (accel_norm_squared >= 0.7225f && accel_norm_squared <= 1.3225f) {
    accel_roll_rad = atan2f(mpu_data.accel_y_g, mpu_data.accel_z_g);
    accel_pitch_rad = atan2f(
      -mpu_data.accel_x_g,
      sqrtf(mpu_data.accel_y_g * mpu_data.accel_y_g +
            mpu_data.accel_z_g * mpu_data.accel_z_g));
    blend = MPU6050_TILT_TIME_CONSTANT_S /
            (MPU6050_TILT_TIME_CONSTANT_S + dt_s);
    mpu_data.roll_rad =
      blend * (mpu_data.roll_rad + gyro_x_radps * dt_s) +
      (1.0f - blend) * accel_roll_rad;
    mpu_data.pitch_rad =
      blend * (mpu_data.pitch_rad + gyro_y_radps * dt_s) +
      (1.0f - blend) * accel_pitch_rad;
  } else {
    mpu_data.roll_rad += gyro_x_radps * dt_s;
    mpu_data.pitch_rad += gyro_y_radps * dt_s;
  }

  cosine_pitch = cosf(mpu_data.pitch_rad);
  if (mpu6050_abs(cosine_pitch) > 0.20f) {
    yaw_rate_radps =
      (gyro_y_radps * sinf(mpu_data.roll_rad) +
       gyro_z_radps * cosf(mpu_data.roll_rad)) / cosine_pitch;
  } else {
    yaw_rate_radps = gyro_z_radps;
  }
  if (mpu_data.stationary) {
    yaw_rate_radps = 0.0f;
  }
  mpu_data.yaw_rate_dps = yaw_rate_radps * MPU6050_RAD_TO_DEG;

  if (!mpu_data.stationary) {
    mpu_data.yaw_rad =
      mpu6050_wrap_pi(mpu_data.yaw_rad + yaw_rate_radps * dt_s);
  }
}

static void mpu6050_process_sample(const uint8_t sample[14], float dt_s)
{
  int16_t raw_accel_x = mpu6050_decode_i16(&sample[0]);
  int16_t raw_accel_y = mpu6050_decode_i16(&sample[2]);
  int16_t raw_accel_z = mpu6050_decode_i16(&sample[4]);
  int16_t raw_temperature = mpu6050_decode_i16(&sample[6]);
  int16_t raw_gyro_x = mpu6050_decode_i16(&sample[8]);
  int16_t raw_gyro_y = mpu6050_decode_i16(&sample[10]);
  int16_t raw_gyro_z = mpu6050_decode_i16(&sample[12]);
  float accel_norm_squared;

  /* Body frame assumes sensor X forward, Y left, and Z up. */
  mpu_data.accel_x_g = (float)raw_accel_x / MPU6050_ACCEL_LSB_PER_G;
  mpu_data.accel_y_g = (float)raw_accel_y / MPU6050_ACCEL_LSB_PER_G;
  mpu_data.accel_z_g = (float)raw_accel_z / MPU6050_ACCEL_LSB_PER_G;
  mpu_data.gyro_raw_x_dps = (float)raw_gyro_x / MPU6050_GYRO_LSB_PER_DPS;
  mpu_data.gyro_raw_y_dps = (float)raw_gyro_y / MPU6050_GYRO_LSB_PER_DPS;
  mpu_data.gyro_raw_z_dps = (float)raw_gyro_z / MPU6050_GYRO_LSB_PER_DPS;
  mpu_data.temperature_c =
    (float)raw_temperature / MPU6050_TEMP_LSB_PER_C + MPU6050_TEMP_OFFSET_C;
  mpu_data.gyro_x_dps =
    mpu_data.gyro_raw_x_dps - mpu_data.gyro_bias_x_dps;
  mpu_data.gyro_y_dps =
    mpu_data.gyro_raw_y_dps - mpu_data.gyro_bias_y_dps;
  mpu_data.gyro_z_dps =
    mpu_data.gyro_raw_z_dps - mpu_data.gyro_bias_z_dps;

  accel_norm_squared =
    mpu_data.accel_x_g * mpu_data.accel_x_g +
    mpu_data.accel_y_g * mpu_data.accel_y_g +
    mpu_data.accel_z_g * mpu_data.accel_z_g;

  if (mpu_data.state == MPU6050_STATE_CALIBRATING) {
    mpu6050_calibrate(accel_norm_squared);
  } else {
    mpu6050_update_orientation(dt_s, accel_norm_squared);
  }
}

void mpu6050_init(I2C_HandleTypeDef *i2c)
{
  uint32_t now = HAL_GetTick();

  memset(&mpu_data, 0, sizeof(mpu_data));
  mpu_i2c = i2c;
  mpu_address_hal = 0U;
  still_sample_count = 0U;
  vehicle_stopped_hint = true;
  straight_motion_hint = false;
  heading_error_hint_rad = 0.0f;
  heading_correction_hint_radps = 0.0f;
  configured_once = false;
  mpu6050_reset_moving_bias_window();
  mpu_data.state = MPU6050_STATE_PROBING;
  mpu_data.calibration_required = MPU6050_CALIBRATION_REQUIRED_SAMPLES;
  next_action_ms = now + MPU6050_POWER_UP_DELAY_MS;
  last_sample_ms = now;
}

void mpu6050_process(void)
{
  uint32_t now;

  if (mpu_i2c == NULL) {
    return;
  }

  now = HAL_GetTick();
  if (mpu_data.state == MPU6050_STATE_PROBING ||
      mpu_data.state == MPU6050_STATE_ERROR) {
    if (!mpu6050_time_reached(now, next_action_ms)) {
      return;
    }
    mpu_data.state = MPU6050_STATE_PROBING;
    if (!mpu6050_probe()) {
      next_action_ms = now + MPU6050_RETRY_PERIOD_MS;
      return;
    }
    if (!mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x80U)) {
      mpu_data.io_error_count++;
      mpu6050_schedule_error(now);
      return;
    }
    mpu_data.state = MPU6050_STATE_RESETTING;
    mpu_data.sample_valid = false;
    next_action_ms = now + MPU6050_RESET_DELAY_MS;
    return;
  }

  if (mpu_data.state == MPU6050_STATE_RESETTING) {
    if (!mpu6050_time_reached(now, next_action_ms)) {
      return;
    }
    if (!mpu6050_configure()) {
      mpu_data.io_error_count++;
      mpu6050_schedule_error(now);
      return;
    }
    if (configured_once) {
      mpu_data.recovery_count++;
    }
    configured_once = true;
    mpu_data.consecutive_errors = 0U;
    mpu6050_begin_calibration(now);
    return;
  }

  if (now - last_sample_ms >= MPU6050_SAMPLE_PERIOD_MS) {
    uint8_t sample[14];
    uint32_t elapsed_ms = now - last_sample_ms;
    float dt_s = (float)elapsed_ms * 0.001f;

    last_sample_ms = now;
    if (!mpu6050_read(MPU6050_REG_ACCEL_XOUT_H, sample, sizeof(sample))) {
      mpu_data.io_error_count++;
      mpu_data.consecutive_errors++;
      if (mpu_data.consecutive_errors >= MPU6050_MAX_CONSECUTIVE_ERRORS) {
        mpu6050_schedule_error(now);
      }
      return;
    }

    mpu_data.consecutive_errors = 0U;
    mpu_data.sample_valid = true;
    mpu_data.sample_time_ms = now;
    mpu_data.sample_sequence++;
    if (dt_s < MPU6050_MIN_DT_S || dt_s > MPU6050_MAX_DT_S) {
      dt_s = MPU6050_DEFAULT_DT_S;
    }
    mpu6050_process_sample(sample, dt_s);
  }
}

void mpu6050_set_stationary_hint(bool vehicle_stopped)
{
  vehicle_stopped_hint = vehicle_stopped;
}

void mpu6050_set_straight_motion_hint(bool straight_motion,
                                      float heading_error_rad,
                                      float heading_correction_radps)
{
  straight_motion_hint = straight_motion;
  heading_error_hint_rad = heading_error_rad;
  heading_correction_hint_radps = heading_correction_radps;
  if (!straight_motion) {
    mpu6050_reset_moving_bias_window();
  }
}

bool mpu6050_get_yaw(float *yaw_rad)
{
  if (mpu_data.state != MPU6050_STATE_READY ||
      !mpu_data.sample_valid ||
      HAL_GetTick() - mpu_data.sample_time_ms > 50U) {
    return false;
  }

  if (yaw_rad != NULL) {
    *yaw_rad = mpu_data.yaw_rad;
  }
  return true;
}

const mpu6050_data_t *mpu6050_get_data(void)
{
  return &mpu_data;
}
