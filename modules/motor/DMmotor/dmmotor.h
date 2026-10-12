#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4
#define DM_MOTOR_FEEDBACK_TIMEOUT_MS 200U

/* DM 电机型号 */
typedef enum
{
    DM4310 = 0,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    DM2325,
    DM_MOTOR_TYPE_NUM
} DM_Motor_Type_e;

/* 电机控制模式 (寄存器 RID=10) */
typedef enum
{
    DM_CTRL_MIT       = 1,  // MIT 模式: 位置+速度+扭矩+Kp+Kd
    DM_CTRL_POS_VEL   = 2,  // 位置-速度模式: float pos + float vel, CAN ID += 0x100
    DM_CTRL_VEL       = 3,  // 速度模式: float vel, CAN ID += 0x200
    DM_CTRL_POS_FORCE = 4   // 位置-力模式, CAN ID += 0x300
} DM_Control_Mode_e;

/* 电机状态命令 (使能/停止/归零/清除错误)。
   命令帧 ID 与当前控制模式一致：MIT=CAN_ID，位置速度=CAN_ID+0x100，
   速度=CAN_ID+0x200。 */
typedef enum
{
    DM_CMD_CLEAR_ERROR  = 0xfb,
    DM_CMD_MOTOR_MODE   = 0xfc,  // 使能
    DM_CMD_RESET_MODE   = 0xfd,  // 停止
    DM_CMD_ZERO_POSITION = 0xfe  // 编码器归零
} DM_Motor_Cmd_e;

/* 每型号电机的位置/速度/扭矩限制 */
typedef struct
{
    float p_max;   // 位置最大值 (rad)
    float v_max;   // 速度最大值 (rad/s)
    float t_max;   // 扭矩最大值 (Nm)
} DM_Motor_Limit_s;

/* 电机测量值 */
typedef struct
{
    uint8_t id;
    volatile uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;      // MOS 温度, 部分固件版本可能为保留字节
    float T_Rotor;    // 转子温度, 部分固件版本可能为保留字节
    int32_t total_round;
    double total_position; // Unwrapped output-shaft displacement in rad, relative to first feedback.
} DM_Motor_Measure_s;

/* MIT 模式控制报文 */
typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_s;

/* 电机实例 */
typedef struct
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    volatile Motor_Working_Type_e stop_flag; // Requested state, independent of feedback.
    volatile uint8_t feedback_online;
    CANInstance *motor_can_instace;
    DaemonInstance *motor_daemon;
    DM_Motor_Type_e dm_motor_type;
    DM_Motor_Limit_s limits;
    DM_Control_Mode_e control_mode;
    uint32_t can_id;
    uint32_t lost_cnt;
    volatile uint32_t feedback_count;
    volatile uint32_t last_feedback_ms;
} DMMotorInstance;

extern const DM_Motor_Limit_s dm_motor_limits[DM_MOTOR_TYPE_NUM];
extern volatile uint32_t dm_decode_count;
extern volatile uint32_t dm_decode_reject_count;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetDMType(DMMotorInstance *motor, DM_Motor_Type_e type);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e closeloop_type);

/* Requests are applied asynchronously by the motor task; init defaults to disabled. */
void DMMotorEnable(DMMotorInstance *motor);

void DMMotorDisable(DMMotorInstance *motor);

/* Compatibility alias for DMMotorDisable(). */
void DMMotorStop(DMMotorInstance *motor);

void DMMotorCaliEncoder(DMMotorInstance *motor);

void DMMotorControlInit(void);

void DMMotorSetMode(DM_Motor_Cmd_e cmd, DMMotorInstance *motor);

void DMMotorSetControlMode(DMMotorInstance *motor, DM_Control_Mode_e mode);

float DMMotorReadRegister(DMMotorInstance *motor, uint8_t rid);

void DMMotorWriteRegister(DMMotorInstance *motor, uint8_t rid, const uint8_t data[4]);

void DMMotorSaveParams(DMMotorInstance *motor);

/* Override limits when the motor firmware/configuration differs from defaults. */
void DMMotorSetLimits(DMMotorInstance *motor, const DM_Motor_Limit_s *limits);

#endif // !DMMOTOR_H
