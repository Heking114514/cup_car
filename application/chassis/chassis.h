#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

typedef struct
{
    int32_t left_total_count;
    int32_t right_total_count;
    float target_left_mps;
    float target_right_mps;
    float measured_left_mps;
    float measured_right_mps;
    int16_t left_output;
    int16_t right_output;
    uint8_t heading_active;
    float heading_ref_deg;
    float heading_error_deg;
    float heading_correction_ref;
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
