#ifndef BMI088_DIAG_H
#define BMI088_DIAG_H

#include <stdint.h>

#define BMI088_DIAG_GYRO_VALID 0x01U
#define BMI088_DIAG_TEMP_VALID 0x02U
#define BMI088_DIAG_YAW_VALID  0x04U
#define BMI088_DIAG_BIAS_VALID 0x08U
#define BMI088_DIAG_STATIONARY 0x10U
#define BMI088_DIAG_SATURATED  0x20U
#define BMI088_DIAG_SAMPLE_TIMEOUT 0x40U

typedef struct
{
    uint32_t sample_time_ms;
    uint8_t status;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t temperature_cC;
} BMI088DiagSample;

typedef struct
{
    uint8_t status;
    float yaw_deg;
    float gyro_z_dps;
    float gyro_z_lpf_dps;
    float gyro_z_bias_dps;
    uint16_t startup_samples;
    uint32_t sample_sequence;
    uint32_t sample_time_ms;
    uint32_t last_update_ms;
} BMI088DiagState;

/* Raw gyro counts at +/-250 dps; each count is about 0.007629 dps.
 * Positive yaw follows ROS base_link +Z: counterclockwise viewed from above.
 */
void BMI088DiagRead(BMI088DiagSample *sample);
void BMI088DiagUpdate(uint8_t stationary);
uint8_t BMI088DiagGetState(BMI088DiagState *state);
void BMI088DiagResetYaw(float yaw_deg);

#endif
