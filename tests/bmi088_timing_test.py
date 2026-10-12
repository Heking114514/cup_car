"""Host regression check of the firmware's actual BMI088 yaw update functions.
Run: python3 tests/bmi088_timing_test.py
No STM32 hardware or DWT clock is used; only the SPI sample reader is stubbed.
"""
from pathlib import Path
import subprocess
import tempfile

PROJECT = Path(__file__).resolve().parents[1]


def function(source: str, name: str) -> str:
    start = source.index(name + '(')
    start = source.rfind('\n', 0, start) + 1
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def main() -> None:
    source = (PROJECT / 'modules/imu/bmi088_diag.c').read_text()
    macros = '\n'.join(line for line in source.splitlines() if line.startswith('#define '))
    globals_ = source[source.index('static uint8_t initialized;'):source.index('static void BMI088DiagResetStartupBias')]
    functions = '\n'.join(function(source, name) for name in (
        'BMI088DiagResetStartupBias', 'BMI088DiagResetStillBias', 'BMI088DiagStd',
        'BMI088DiagUpdateStartupBias', 'BMI088DiagUpdateOnlineBias', 'BMI088DiagUpdate',
    ))
    harness = '''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "bmi088_diag.h"
''' + macros + '\n' + globals_ + '''
static BMI088DiagSample fixture;
void BMI088DiagRead(BMI088DiagSample *sample) { *sample = fixture; }
''' + functions + '''
static void reset_ready(uint32_t stamp) {
    memset(&diag_state, 0, sizeof(diag_state));
    diag_state.status = BMI088_DIAG_BIAS_VALID;
    yaw_initialized = 1;
    update_time_initialized = 0;
    gyro_lpf_initialized = 0;
    corrected_gyro_initialized = 0;
    fixture = (BMI088DiagSample){.status = BMI088_DIAG_GYRO_VALID,
                               .gz = -786, .sample_time_ms = stamp};
    BMI088DiagUpdate(0);
}
int main(void) {
    const float speed = 786.0f * BMI088_GYRO_DPS_PER_COUNT;
    reset_ready(0);
    for (uint32_t i = 1; i <= 100; i++) {
        fixture.sample_time_ms = i * 10;
        BMI088DiagUpdate(0);
    }
    assert(fabsf(diag_state.yaw_deg - speed) < 0.0001f);
    assert(diag_state.status & BMI088_DIAG_YAW_VALID);
    float previous = diag_state.yaw_deg;
    BMI088DiagUpdate(0); /* Duplicate timestamp must not integrate. */
    assert(diag_state.yaw_deg == previous);
    assert(!(diag_state.status & BMI088_DIAG_YAW_VALID));
    assert(diag_state.status & BMI088_DIAG_SAMPLE_TIMEOUT);
    fixture.sample_time_ms += 10;
    BMI088DiagUpdate(0);
    assert(diag_state.status & BMI088_DIAG_YAW_VALID);
    previous = diag_state.yaw_deg;
    fixture.sample_time_ms += 500;
    BMI088DiagUpdate(0);
    assert(diag_state.yaw_deg == previous);
    assert(!(diag_state.status & BMI088_DIAG_YAW_VALID));
    reset_ready(UINT32_MAX - 5U);
    fixture.sample_time_ms = 4U; /* 10ms across the 32-bit rollover. */
    BMI088DiagUpdate(0);
    assert(fabsf(diag_state.yaw_deg - speed * .01f) < 0.0001f);
    diag_state.yaw_deg = NAN;
    fixture.sample_time_ms += 10;
    BMI088DiagUpdate(0);
    assert(isfinite(diag_state.yaw_deg));
    assert(!(diag_state.status & BMI088_DIAG_YAW_VALID));
    fixture.sample_time_ms += 10;
    BMI088DiagUpdate(0);
    assert(diag_state.status & BMI088_DIAG_YAW_VALID);
    memset(&diag_state, 0, sizeof(diag_state));
    BMI088DiagResetStartupBias();
    update_time_initialized = 0;
    gyro_lpf_initialized = yaw_initialized = 0;
    fixture.gz = 0;
    for (uint32_t i = 0; i < 600; i++) {
        fixture.sample_time_ms = i * 10;
        BMI088DiagUpdate(1);
    }
    assert(diag_state.status & BMI088_DIAG_BIAS_VALID);
    assert(diag_state.status & BMI088_DIAG_YAW_VALID);
    puts("PASS: actual BMI088 yaw integration without DWT; startup, duplicates, gaps, rollover, NaN recovery");
    return 0;
}
'''
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        (path / 'check.c').write_text(harness)
        subprocess.run(['cc', '-std=c11', '-I' + str(PROJECT / 'modules/imu'),
                        str(path / 'check.c'), '-lm', '-o', str(path / 'check')], check=True)
        subprocess.run([str(path / 'check')], check=True)


if __name__ == '__main__':
    main()
