#include "bmi088_diag.h"

#include "BMI088reg.h"
#include "bsp_dwt.h"
#include "main.h"
#include "spi.h"
#include <math.h>
#include <string.h>

#define BMI088_DIAG_SPI_TIMEOUT_MS 10U
#define BMI088_DIAG_RETRY_MS 1000U
#define BMI088_DIAG_SETTLE_MS 30U
#define BMI088_GYRO_RANGE_SETTING BMI088_GYRO_250
#define BMI088_GYRO_BANDWIDTH_SETTING BMI088_GYRO_200_23_HZ
#define BMI088_GYRO_FULL_SCALE_DPS 250.0f
#define BMI088_GYRO_DPS_PER_COUNT (BMI088_GYRO_FULL_SCALE_DPS / 32768.0f)
#define BMI088_GYRO_ROS_Z_SIGN 1.0f
#define BMI088_SAMPLE_TIMEOUT_S 0.030f
#define BMI088_STARTUP_BIAS_SAMPLES 600U
#define BMI088_STARTUP_RESET_MISSES 20U
#define BMI088_STARTUP_MAX_STD_DPS 0.90f
#define BMI088_STARTUP_MAX_MEAN_DPS 3.0f
#define BMI088_STILL_MIN_SAMPLES 50U
#define BMI088_STILL_MAX_STD_DPS 0.90f
#define BMI088_STILL_MAX_CORRECTED_DPS 1.2f
#define BMI088_ONLINE_BIAS_ALPHA 0.0030f
#define BMI088_GYRO_LPF_ALPHA 0.45f
#define BMI088_GYRO_SATURATION_COUNT 32000

static uint8_t initialized;
static uint8_t attempted;
static uint32_t last_attempt_ms;
static uint32_t initialized_ms;
static BMI088DiagState diag_state;
static uint8_t yaw_initialized;
static float startup_sum;
static float startup_sum_sq;
static uint16_t startup_count;
static uint16_t startup_nonstationary_count;
static float still_sum;
static float still_sum_sq;
static uint16_t still_count;
static uint8_t gyro_lpf_initialized;
static uint64_t last_update_us;
static float last_corrected_gyro_z_dps;
static uint8_t corrected_gyro_initialized;

static void BMI088DiagResetStartupBias(void)
{
    startup_sum = 0.0f;
    startup_sum_sq = 0.0f;
    startup_count = 0U;
    startup_nonstationary_count = 0U;
    diag_state.startup_samples = 0U;
}

static void BMI088DiagResetStillBias(void)
{
    still_sum = 0.0f;
    still_sum_sq = 0.0f;
    still_count = 0U;
}

static float BMI088DiagStd(float sum, float sum_sq, uint16_t count)
{
    float mean;
    float variance;

    if (count < 2U)
        return 0.0f;
    mean = sum / (float)count;
    variance = sum_sq / (float)count - mean * mean;
    if (variance < 0.0f)
        variance = 0.0f;
    return sqrtf(variance);
}

static HAL_StatusTypeDef BMI088DiagTransfer(GPIO_TypeDef *port, uint16_t pin,
                                             const uint8_t *tx, uint8_t *rx,
                                             uint16_t length)
{
    HAL_StatusTypeDef result;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    result = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, length,
                                     BMI088_DIAG_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    return result;
}

static uint8_t BMI088DiagGyroRead(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t tx[9] = {0};
    uint8_t rx[9] = {0};

    if (length == 0U || length > 8U)
        return 0U;
    tx[0] = reg | 0x80U;
    if (BMI088DiagTransfer(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, tx, rx,
                           (uint16_t)length + 1U) != HAL_OK)
        return 0U;
    memcpy(data, &rx[1], length);
    return 1U;
}

static uint8_t BMI088DiagAccelRead(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t tx[4] = {0};
    uint8_t rx[4] = {0};

    if (length == 0U || length > 2U)
        return 0U;
    tx[0] = reg | 0x80U;
    if (BMI088DiagTransfer(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin, tx, rx,
                           (uint16_t)length + 2U) != HAL_OK)
        return 0U;
    memcpy(data, &rx[2], length);
    return 1U;
}

static uint8_t BMI088DiagGyroWrite(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0};

    return BMI088DiagTransfer(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin,
                              tx, rx, sizeof(tx)) == HAL_OK;
}

static uint8_t BMI088DiagAccelWrite(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0};

    return BMI088DiagTransfer(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin,
                              tx, rx, sizeof(tx)) == HAL_OK;
}

static uint8_t BMI088DiagConfigureGyro(uint8_t reg, uint8_t value)
{
    uint8_t readback;

    return BMI088DiagGyroWrite(reg, value) &&
           BMI088DiagGyroRead(reg, &readback, 1U) && readback == value;
}

static uint8_t BMI088DiagInit(void)
{
    uint8_t accel_id;
    uint8_t gyro_id;

    /* The first accelerometer SPI access selects SPI mode after power-up. */
    if (!BMI088DiagAccelRead(BMI088_ACC_CHIP_ID, &accel_id, 1U) ||
        !BMI088DiagAccelRead(BMI088_ACC_CHIP_ID, &accel_id, 1U) ||
        accel_id != BMI088_ACC_CHIP_ID_VALUE ||
        !BMI088DiagGyroRead(BMI088_GYRO_CHIP_ID, &gyro_id, 1U) ||
        gyro_id != BMI088_GYRO_CHIP_ID_VALUE)
        return 0U;

    if (!BMI088DiagConfigureGyro(BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE) ||
        !BMI088DiagConfigureGyro(BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_SETTING) ||
        !BMI088DiagConfigureGyro(BMI088_GYRO_BANDWIDTH,
                                BMI088_GYRO_BANDWIDTH_SETTING |
                                 BMI088_GYRO_BANDWIDTH_MUST_Set))
        return 0U;

    /* Temperature belongs to the accelerometer, which powers up suspended. */
    if (!BMI088DiagAccelWrite(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON) ||
        !BMI088DiagAccelWrite(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE))
        return 0U;

    initialized_ms = HAL_GetTick();
    gyro_lpf_initialized = 0U;
    last_update_us = 0ULL;
    corrected_gyro_initialized = 0U;
    return 1U;
}

void BMI088DiagRead(BMI088DiagSample *sample)
{
    uint8_t gyro_id;
    uint8_t gyro_bytes[6];
    uint8_t temperature_bytes[2];
    int16_t raw_temperature;
    uint32_t now;

    if (sample == NULL)
        return;
    memset(sample, 0, sizeof(*sample));
    now = HAL_GetTick();
    if (!initialized)
    {
        if (attempted && now - last_attempt_ms < BMI088_DIAG_RETRY_MS)
            return;
        last_attempt_ms = now;
        attempted = 1U;
        initialized = BMI088DiagInit();
        if (!initialized)
            return;
    }

    if (now - initialized_ms < BMI088_DIAG_SETTLE_MS)
        return;

    if (!BMI088DiagGyroRead(BMI088_GYRO_CHIP_ID, &gyro_id, 1U) ||
        gyro_id != BMI088_GYRO_CHIP_ID_VALUE ||
        !BMI088DiagGyroRead(BMI088_GYRO_X_L, gyro_bytes, sizeof(gyro_bytes)))
    {
        initialized = 0U;
        return;
    }

    sample->gx = (int16_t)((uint16_t)gyro_bytes[0] |
                           ((uint16_t)gyro_bytes[1] << 8));
    sample->gy = (int16_t)((uint16_t)gyro_bytes[2] |
                           ((uint16_t)gyro_bytes[3] << 8));
    sample->gz = (int16_t)((uint16_t)gyro_bytes[4] |
                           ((uint16_t)gyro_bytes[5] << 8));
    sample->sample_time_ms = HAL_GetTick();
    sample->status |= BMI088_DIAG_GYRO_VALID;

    if (BMI088DiagAccelRead(BMI088_TEMP_M, temperature_bytes,
                            sizeof(temperature_bytes)) &&
        !(temperature_bytes[0] == 0x80U && temperature_bytes[1] == 0x00U))
    {
        raw_temperature = (int16_t)(((uint16_t)temperature_bytes[0] << 3) |
                                    (temperature_bytes[1] >> 5));
        if (raw_temperature > 1023)
            raw_temperature -= 2048;
        sample->temperature_cC = (int16_t)(2300 + raw_temperature * 25 / 2);
        sample->status |= BMI088_DIAG_TEMP_VALID;
    }
}

static void BMI088DiagUpdateStartupBias(float gyro_z_dps, uint8_t stationary)
{
    float mean;
    float std;

    if (diag_state.status & BMI088_DIAG_BIAS_VALID)
        return;
    if (!stationary)
    {
        if (startup_count == 0U)
            return;
        if (startup_nonstationary_count < UINT16_MAX)
            startup_nonstationary_count++;
        if (startup_nonstationary_count >= BMI088_STARTUP_RESET_MISSES)
            BMI088DiagResetStartupBias();
        return;
    }
    startup_nonstationary_count = 0U;

    startup_sum += gyro_z_dps;
    startup_sum_sq += gyro_z_dps * gyro_z_dps;
    if (startup_count < UINT16_MAX)
        startup_count++;
    diag_state.startup_samples = startup_count;

    if (startup_count < BMI088_STARTUP_BIAS_SAMPLES)
        return;

    mean = startup_sum / (float)startup_count;
    std = BMI088DiagStd(startup_sum, startup_sum_sq, startup_count);
    if (std <= BMI088_STARTUP_MAX_STD_DPS &&
        fabsf(mean) <= BMI088_STARTUP_MAX_MEAN_DPS)
    {
        diag_state.gyro_z_bias_dps = mean;
        diag_state.status |= BMI088_DIAG_BIAS_VALID | BMI088_DIAG_YAW_VALID;
        yaw_initialized = 1U;
        diag_state.yaw_deg = 0.0f;
        corrected_gyro_initialized = 0U;
        BMI088DiagResetStillBias();
    }
    else
    {
        BMI088DiagResetStartupBias();
    }
}

static void BMI088DiagUpdateOnlineBias(float gyro_z_dps)
{
    float mean;
    float std;
    float corrected_mean;

    still_sum += gyro_z_dps;
    still_sum_sq += gyro_z_dps * gyro_z_dps;
    if (still_count < UINT16_MAX)
        still_count++;

    if (still_count < BMI088_STILL_MIN_SAMPLES)
        return;

    mean = still_sum / (float)still_count;
    std = BMI088DiagStd(still_sum, still_sum_sq, still_count);
    corrected_mean = mean - diag_state.gyro_z_bias_dps;

    if (std <= BMI088_STILL_MAX_STD_DPS &&
        fabsf(corrected_mean) <= BMI088_STILL_MAX_CORRECTED_DPS)
    {
        diag_state.gyro_z_bias_dps +=
            (mean - diag_state.gyro_z_bias_dps) * BMI088_ONLINE_BIAS_ALPHA;
    }

    if (still_count >= 200U)
    {
        still_sum = mean;
        still_sum_sq = mean * mean;
        still_count = 1U;
    }
}

void BMI088DiagUpdate(uint8_t stationary)
{
    BMI088DiagSample sample;
    uint64_t now_us;
    float gyro_z_dps;
    float gyro_z_est_dps;
    float corrected_gyro_z_dps;
    float dt;

    BMI088DiagRead(&sample);
    if (!(sample.status & BMI088_DIAG_GYRO_VALID))
        return;

    gyro_z_dps =
        BMI088_GYRO_ROS_Z_SIGN * (float)sample.gz * BMI088_GYRO_DPS_PER_COUNT;
    if (!gyro_lpf_initialized)
    {
        diag_state.gyro_z_lpf_dps = gyro_z_dps;
        gyro_lpf_initialized = 1U;
    }
    else
    {
        diag_state.gyro_z_lpf_dps +=
            BMI088_GYRO_LPF_ALPHA *
            (gyro_z_dps - diag_state.gyro_z_lpf_dps);
    }
    gyro_z_est_dps = diag_state.gyro_z_lpf_dps;
    diag_state.gyro_z_dps = gyro_z_est_dps;
    diag_state.sample_time_ms = sample.sample_time_ms;
    diag_state.last_update_ms = sample.sample_time_ms;
    diag_state.sample_sequence++;
    diag_state.status = sample.status |
        (diag_state.status & (BMI088_DIAG_YAW_VALID | BMI088_DIAG_BIAS_VALID));
    if (stationary)
        diag_state.status |= BMI088_DIAG_STATIONARY;
    if (sample.gz > BMI088_GYRO_SATURATION_COUNT ||
        sample.gz < -BMI088_GYRO_SATURATION_COUNT)
        diag_state.status |= BMI088_DIAG_SATURATED;

    now_us = DWT_GetTimeline_us();
    if (last_update_us == 0ULL)
    {
        last_update_us = now_us;
        BMI088DiagUpdateStartupBias(gyro_z_est_dps, stationary);
        return;
    }

    dt = (float)(now_us - last_update_us) * 0.000001f;
    last_update_us = now_us;
    if (dt > BMI088_SAMPLE_TIMEOUT_S)
        diag_state.status |= BMI088_DIAG_SAMPLE_TIMEOUT;
    if (dt <= 0.0f || dt > 0.2f || !isfinite(dt))
    {
        dt = 0.0f;
        corrected_gyro_initialized = 0U;
    }

    BMI088DiagUpdateStartupBias(gyro_z_est_dps, stationary);
    if (!(diag_state.status & BMI088_DIAG_BIAS_VALID))
        return;

    if (stationary)
        BMI088DiagUpdateOnlineBias(gyro_z_est_dps);
    else
        BMI088DiagResetStillBias();

    corrected_gyro_z_dps = gyro_z_est_dps - diag_state.gyro_z_bias_dps;
    if (!corrected_gyro_initialized)
    {
        last_corrected_gyro_z_dps = corrected_gyro_z_dps;
        corrected_gyro_initialized = 1U;
    }

    if (yaw_initialized && dt > 0.0f)
    {
        diag_state.yaw_deg +=
            0.5f * (last_corrected_gyro_z_dps + corrected_gyro_z_dps) * dt;
        diag_state.status |= BMI088_DIAG_YAW_VALID;
    }
    last_corrected_gyro_z_dps = corrected_gyro_z_dps;
}

uint8_t BMI088DiagGetState(BMI088DiagState *state)
{
    if (state == NULL)
        return 0U;
    *state = diag_state;
    return (uint8_t)((diag_state.status & BMI088_DIAG_YAW_VALID) != 0U);
}

void BMI088DiagResetYaw(float yaw_deg)
{
    diag_state.yaw_deg = isfinite(yaw_deg) ? yaw_deg : 0.0f;
    corrected_gyro_initialized = 0U;
    if (diag_state.status & BMI088_DIAG_BIAS_VALID)
    {
        yaw_initialized = 1U;
        diag_state.status |= BMI088_DIAG_YAW_VALID;
    }
}
