#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

#define CHASSIS_JLINK_TUNE_ARM_KEY 0x4D54554EU
#define CHASSIS_JLINK_TUNE_SAMPLE_CAPACITY 160U
#define CHASSIS_JLINK_TUNE_SAMPLE_PERIOD_MS 10U

typedef enum
{
    CHASSIS_JLINK_TUNE_IDLE = 0,
    CHASSIS_JLINK_TUNE_ACTIVE,
    CHASSIS_JLINK_TUNE_COMPLETE,
    CHASSIS_JLINK_TUNE_REJECTED,
    CHASSIS_JLINK_TUNE_ABORTED,
} Chassis_JLink_Tune_State_e;

typedef enum
{
    CHASSIS_JLINK_TUNE_STOP_NONE = 0,
    CHASSIS_JLINK_TUNE_STOP_DURATION,
    CHASSIS_JLINK_TUNE_STOP_BAD_KEY,
    CHASSIS_JLINK_TUNE_STOP_BAD_REQUEST,
    CHASSIS_JLINK_TUNE_STOP_FEEDBACK_LOST,
    CHASSIS_JLINK_TUNE_STOP_CAN_FAULT,
    CHASSIS_JLINK_TUNE_STOP_OVERTEMPERATURE,
    CHASSIS_JLINK_TUNE_STOP_OVERSPEED,
    CHASSIS_JLINK_TUNE_STOP_HOST,
} Chassis_JLink_Tune_Stop_e;

typedef enum
{
    CHASSIS_JLINK_TUNE_SPEED_ONLY = 0,
    CHASSIS_JLINK_TUNE_SPEED_CURRENT = 1,
} Chassis_JLink_Tune_Control_Mode_e;

/* request_seq must be written last so the target never consumes a partial request. */
typedef struct
{
    uint32_t arm_key;
    int32_t left_ref_dps;
    int32_t right_ref_dps;
    uint32_t duration_ms;
    int32_t left_kp_milli;
    int32_t left_ki_milli;
    int32_t right_kp_milli;
    int32_t right_ki_milli;
    uint32_t control_mode;
    int32_t left_current_kp_milli;
    int32_t right_current_kp_milli;
    uint32_t request_seq;
} Chassis_JLink_Tune_Command_s;

typedef struct
{
    uint32_t time_ms;
    uint32_t left_pid_dt_us;
    uint32_t right_pid_dt_us;
    int16_t left_ref_dps;
    int16_t right_ref_dps;
    int16_t left_speed_dps;
    int16_t right_speed_dps;
    int16_t left_error_dps;
    int16_t right_error_dps;
    int16_t left_p_output;
    int16_t right_p_output;
    int16_t left_i_output;
    int16_t right_i_output;
    int16_t left_feedforward;
    int16_t right_feedforward;
    int16_t left_command;
    int16_t right_command;
    int16_t left_current;
    int16_t right_current;
    uint16_t left_feedback_age_ms;
    uint16_t right_feedback_age_ms;
    uint8_t left_online;
    uint8_t right_online;
    uint8_t drive_status;
    uint8_t can_flags;
    uint8_t can_last_error_code;
    uint8_t reserved[3];
} Chassis_JLink_Tune_Sample_s;

typedef struct
{
    uint32_t capture_seq;
    uint32_t request_seq;
    uint32_t start_time_ms;
    uint32_t end_time_ms;
    uint32_t requested_duration_ms;
    uint32_t control_mode;
    int32_t left_ref_dps;
    int32_t right_ref_dps;
    int32_t left_kp_milli;
    int32_t left_ki_milli;
    int32_t right_kp_milli;
    int32_t right_ki_milli;
    int32_t left_current_kp_milli;
    int32_t right_current_kp_milli;
    uint32_t left_feedback_start;
    uint32_t right_feedback_start;
    uint32_t left_feedback_end;
    uint32_t right_feedback_end;
    uint32_t can_tx_failure_start;
    uint32_t can_tx_failure_end;
    uint32_t can_hal_error_code;
    uint32_t can_protocol_last_error_code;
    uint16_t sample_count;
    uint16_t sample_capacity;
    uint16_t sample_period_ms;
    uint8_t state;
    uint8_t stop_reason;
    uint8_t can_bus_off;
    uint8_t can_error_passive;
    uint8_t reserved[2];
    Chassis_JLink_Tune_Sample_s samples[CHASSIS_JLINK_TUNE_SAMPLE_CAPACITY];
} Chassis_JLink_Tune_Capture_s;

extern volatile Chassis_JLink_Tune_Command_s chassis_jlink_tune_command;
extern volatile Chassis_JLink_Tune_Capture_s chassis_jlink_tune_capture;

typedef struct
{
    int32_t left_total_count;
    int32_t right_total_count;
    float target_left_mps;
    float target_right_mps;
    float measured_left_mps;
    float measured_right_mps;
    int16_t left_output;  // Zero in DM velocity mode (no external current command).
    int16_t right_output;
    int16_t left_current;
    int16_t right_current;
    int16_t left_integral_output;
    int16_t right_integral_output;
    uint8_t left_temperature;
    uint8_t right_temperature;
    uint8_t left_stop_flag;
    uint8_t right_stop_flag;
    uint8_t drive_status;
    uint8_t left_feedback_online;
    uint8_t right_feedback_online;
    uint32_t left_feedback_age_ms;
    uint32_t right_feedback_age_ms;
    uint32_t left_feedback_count;
    uint32_t right_feedback_count;
    uint32_t can_tx_success_count;
    uint32_t can_tx_failure_count;
    uint32_t can_last_tx_failure_age_ms;
    uint32_t can_hal_error_code;
    uint32_t can_protocol_last_error_code;
    uint8_t can_state;
    uint8_t can_bus_off;
    uint8_t can_error_passive;
    uint8_t can_tx_fifo_free_level;
    uint8_t heading_active;
    float heading_ref_deg;
    float heading_error_deg;
    float heading_correction_ref;
    float heading_i_term_ref;
    float heading_fused_error_deg;
    uint8_t sync_active;
    float sync_phase_error_deg;
    float sync_correction_ref;
} Chassis_Host_Feedback_s;

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

/* 获取按小车前进方向归一化的左右轮反馈。 */
void ChassisGetHostFeedback(Chassis_Host_Feedback_s *feedback);

#endif // CHASSIS_H
