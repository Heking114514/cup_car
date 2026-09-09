#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"

#define MPU6050_CALIBRATION_REQUIRED_SAMPLES 500U

typedef enum {
  MPU6050_STATE_PROBING = 0,
  MPU6050_STATE_RESETTING,
  MPU6050_STATE_CALIBRATING,
  MPU6050_STATE_READY,
  MPU6050_STATE_ERROR
} mpu6050_state_t;

typedef struct {
  mpu6050_state_t state;
  bool sample_valid;
  bool stationary;
  uint8_t address_7bit;
  uint8_t who_am_i;
  uint16_t calibration_samples;
  uint16_t calibration_required;
  uint32_t sample_time_ms;
  uint32_t sample_sequence;
  uint32_t io_error_count;
  uint32_t consecutive_errors;
  uint32_t recovery_count;
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float gyro_raw_x_dps;
  float gyro_raw_y_dps;
  float gyro_raw_z_dps;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float yaw_rate_dps;
  float gyro_bias_x_dps;
  float gyro_bias_y_dps;
  float gyro_bias_z_dps;
  float temperature_c;
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
  bool moving_bias_active;
  uint32_t moving_bias_update_count;
} mpu6050_data_t;

void mpu6050_init(I2C_HandleTypeDef *i2c);
void mpu6050_process(void);
void mpu6050_set_stationary_hint(bool vehicle_stopped);
void mpu6050_set_straight_motion_hint(bool straight_motion,
                                      float heading_error_rad,
                                      float heading_correction_radps);
void mpu6050_start_calibration(void);
void mpu6050_zero_yaw(void);
bool mpu6050_get_yaw(float *yaw_rad);
const mpu6050_data_t *mpu6050_get_data(void);

#endif
