#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "controller.h"
#include "motor_def.h"
#include <math.h>

/* The DM2325 velocity field is reported in the motor's native scaled range.
 * Its position field is the authoritative output-shaft measurement (8192
 * counts/rev); comparing both fields on the 2026-10-11 straight test showed
 * the decoded velocity was low by 3.46x.  Keep the command limit unchanged and
 * correct only the feedback value used by telemetry and wheel synchronization.
 */
#define DM2325_FEEDBACK_VELOCITY_SCALE 3.46f

/* 各型号 DM 电机限制参数 (p_max, v_max, t_max) */
const DM_Motor_Limit_s dm_motor_limits[DM_MOTOR_TYPE_NUM] = {
    {12.5f, 30.0f,  10.0f},   // DM4310
    {12.5f, 50.0f,  10.0f},   // DM4310_48V
    {12.5f, 10.0f,  28.0f},   // DM4340
    {12.5f, 10.0f,  28.0f},   // DM4340_48V
    {12.5f, 45.0f,  12.0f},   // DM6006
    {12.5f, 45.0f,  20.0f},   // DM8006
    {12.5f, 45.0f,  54.0f},   // DM8009
    {12.5f, 25.0f, 200.0f},   // DM10010L
    {12.5f, 20.0f, 200.0f},   // DM10010
    {12.5f, 280.0f,  1.0f},   // DMH3510
    {12.5f, 45.0f,  10.0f},   // DMH6215
    {12.5f, 45.0f,  10.0f},   // DMG6220
    /* DM-2325 V1.0: output speed 560 rpm max, peak torque 5 Nm.
       Position range is a firmware/configuration item and defaults to 12.5 rad. */
    {12.5f, 560.0f * 2.0f * 3.14159265359f / 60.0f, 5.0f} // DM2325
};

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
volatile uint32_t dm_decode_count;
volatile uint32_t dm_decode_reject_count;

/* float → uint 线性映射 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/* uint → float 线性映射 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static uint16_t clamp_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    if (x <= x_min)
        return 0;
    if (x >= x_max)
        return (uint16_t)((1u << bits) - 1u);
    return float_to_uint(x, x_min, x_max, bits);
}

static uint32_t DMMotorControlFrameId(const DMMotorInstance *motor)
{
    if (motor == NULL)
        return 0u;

    /* DM-S2325 command frames use a mode-dependent ID offset.  This also
       applies to enable/disable/clear/zero commands (manual V1.1). */
    switch (motor->control_mode)
    {
    case DM_CTRL_POS_VEL:
        return motor->can_id + 0x100u;
    case DM_CTRL_VEL:
        return motor->can_id + 0x200u;
    case DM_CTRL_POS_FORCE:
        return motor->can_id + 0x300u;
    case DM_CTRL_MIT:
    default:
        return motor->can_id;
    }
}

static uint8_t DMMotorFeedbackFresh(const DMMotorInstance *motor)
{
    return motor != NULL && motor->feedback_online != 0U &&
           motor->feedback_count != 0U &&
           HAL_GetTick() - motor->last_feedback_ms <=
               DM_MOTOR_FEEDBACK_TIMEOUT_MS;
}

void DMMotorSetMode(DM_Motor_Cmd_e cmd, DMMotorInstance *motor)
{
    if (motor == NULL || motor->motor_can_instace == NULL)
        return;
    uint32_t saved_id = motor->motor_can_instace->tx_id;
    uint32_t command_id = DMMotorControlFrameId(motor);
    motor->motor_can_instace->tx_id = command_id;
#ifdef FDCAN
    uint32_t saved_identifier = motor->motor_can_instace->txconf.Identifier;
    motor->motor_can_instace->txconf.Identifier = command_id;
#else
    uint32_t saved_std_id = motor->motor_can_instace->txconf.StdId;
    motor->motor_can_instace->txconf.StdId = command_id;
#endif
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instace, 1);
    motor->motor_can_instace->tx_id = saved_id;
#ifdef FDCAN
    motor->motor_can_instace->txconf.Identifier = saved_identifier;
#else
    motor->motor_can_instace->txconf.StdId = saved_std_id;
#endif
}

void DMMotorSetDMType(DMMotorInstance *motor, DM_Motor_Type_e type)
{
    if (motor == NULL)
        return;
    if (type >= DM_MOTOR_TYPE_NUM)
        type = DM4310;
    motor->dm_motor_type = type;
    motor->limits = dm_motor_limits[type];
}

void DMMotorSetLimits(DMMotorInstance *motor, const DM_Motor_Limit_s *limits)
{
    if (motor != NULL && limits != NULL && limits->p_max > 0.0f &&
        limits->v_max > 0.0f && limits->t_max > 0.0f)
        motor->limits = *limits;
}

/* 解析 MIT 模式反馈帧 (bytes 1-5: position/velocity/torque) */
static void DMMotorDecode(CANInstance *motor_can)
{
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure);
    DM_Motor_Limit_s *lim = &(motor->limits);

    if (motor_can->rx_len < 8u || (rxbuff[0] & 0x0fu) != (motor->can_id & 0x0fu))
    {
        dm_decode_reject_count++;
        return;
    }

    DaemonReload(motor->motor_daemon);

    measure->id = (uint8_t)(rxbuff[0] & 0x0fu);
    measure->state = (uint8_t)(rxbuff[0] >> 4);
    motor->feedback_online = 1u;

    measure->last_position = measure->position;

    /* 位置 16-bit (bytes 1-2) */
    uint16_t tmp16 = (uint16_t)(rxbuff[1] << 8) | rxbuff[2];
    measure->position = uint_to_float(tmp16, -lim->p_max, lim->p_max, 16);
    if (motor->feedback_count != 0u)
    {
        float delta = measure->position - measure->last_position;
        if (delta > lim->p_max)
        {
            delta -= 2.0f * lim->p_max;
            measure->total_round--;
        }
        else if (delta < -lim->p_max)
        {
            delta += 2.0f * lim->p_max;
            measure->total_round++;
        }
        measure->total_position += delta;
    }

    /* 速度 12-bit (byte 3 全8位 + byte 4 高4位) */
    tmp16 = (uint16_t)(rxbuff[3] << 4) | (rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp16, -lim->v_max, lim->v_max, 12);
    if (motor->dm_motor_type == DM2325)
        measure->velocity *= DM2325_FEEDBACK_VELOCITY_SCALE;

    /* 扭矩 12-bit (byte 4 低4位 + byte 5 全8位) */
    tmp16 = (uint16_t)((rxbuff[4] & 0x0f) << 8) | rxbuff[5];
    measure->torque = uint_to_float(tmp16, -lim->t_max, lim->t_max, 12);

    /* bytes 6-7: 温度, 部分固件版本可能为保留字节 */
    measure->T_Mos   = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
    motor->last_feedback_ms = HAL_GetTick();
    motor->feedback_count++;
    dm_decode_count++;
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    if (motor != NULL)
    {
        motor->feedback_online = 0u;
        motor->lost_cnt++;
    }
}

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    if (motor == NULL)
        return;
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    if (config == NULL || idx >= DM_MOTOR_CNT)
        return NULL;

    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);
    if (motor->motor_can_instace == NULL)
    {
        free(motor);
        return NULL;
    }
    motor->can_id = config->can_init_config.tx_id & 0xffu;

    /* Default to the DM-2325 used by this project. */
    DMMotorSetDMType(motor, DM2325);
    motor->control_mode = DM_CTRL_MIT;

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        /* DM feedback is normally much faster than the chassis task, but a
         * short CAN/RTOS service delay must not immediately gate velocity
         * frames.  ChassisTask still applies the paired-wheel interlock. */
        .reload_count = 50,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    motor->stop_flag = MOTOR_STOP;
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    if (motor != NULL)
        motor->pid_ref = isfinite(ref) ? ref : 0.0f;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    if (motor != NULL)
        motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorDisable(DMMotorInstance *motor)
{
    if (motor != NULL)
        motor->stop_flag = MOTOR_STOP;
}

void DMMotorStop(DMMotorInstance *motor)
{
    DMMotorDisable(motor);
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    if (motor != NULL)
        motor->motor_settings.outer_loop_type = type;
}

/* 切换控制模式: 通过 RID=10 写入 mode 值 */
void DMMotorSetControlMode(DMMotorInstance *motor, DM_Control_Mode_e mode)
{
    if (motor == NULL || mode < DM_CTRL_MIT || mode > DM_CTRL_POS_FORCE)
        return;
    uint8_t data[4] = {(uint8_t)mode, 0x00, 0x00, 0x00};
    DMMotorWriteRegister(motor, 10, data);
    motor->control_mode = mode;
}

/* 读电机寄存器 (CAN ID 0x7FF, 命令 0x33) */
float DMMotorReadRegister(DMMotorInstance *motor, uint8_t rid)
{
    if (motor == NULL || motor->motor_can_instace == NULL)
        return 0.0f;
    CANInstance *can = motor->motor_can_instace;
    uint8_t *tx = can->tx_buff;

    tx[0] = (uint8_t)(motor->can_id & 0xff);
    tx[1] = (uint8_t)((motor->can_id >> 8) & 0xff);
    tx[2] = 0x33;
    tx[3] = rid;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;

    /* 通过 ID 0x7FF 发送寄存器读命令 */
    uint32_t saved_id = can->tx_id;
    can->tx_id = 0x7FF;
#ifdef FDCAN
    uint32_t saved_identifier = can->txconf.Identifier;
    can->txconf.Identifier = 0x7FF;
#else
    uint32_t saved_std_id = can->txconf.StdId;
    can->txconf.StdId = 0x7FF;
#endif
    CANTransmit(can, 1);
    can->tx_id = saved_id;
#ifdef FDCAN
    can->txconf.Identifier = saved_identifier;
#else
    can->txconf.StdId = saved_std_id;
#endif
    return 0.0f;  // 异步操作, 结果在下一帧反馈中获取
}

/* 写电机寄存器 (CAN ID 0x7FF, 命令 0x55) */
void DMMotorWriteRegister(DMMotorInstance *motor, uint8_t rid, const uint8_t data[4])
{
    if (motor == NULL || motor->motor_can_instace == NULL || data == NULL)
        return;
    CANInstance *can = motor->motor_can_instace;
    uint8_t *tx = can->tx_buff;

    tx[0] = (uint8_t)(motor->can_id & 0xff);
    tx[1] = (uint8_t)((motor->can_id >> 8) & 0xff);
    tx[2] = 0x55;
    tx[3] = rid;
    tx[4] = data[0];
    tx[5] = data[1];
    tx[6] = data[2];
    tx[7] = data[3];

    uint32_t saved_id = can->tx_id;
    can->tx_id = 0x7FF;
#ifdef FDCAN
    uint32_t saved_identifier = can->txconf.Identifier;
    can->txconf.Identifier = 0x7FF;
#else
    uint32_t saved_std_id = can->txconf.StdId;
    can->txconf.StdId = 0x7FF;
#endif
    CANTransmit(can, 1);
    can->tx_id = saved_id;
#ifdef FDCAN
    can->txconf.Identifier = saved_identifier;
#else
    can->txconf.StdId = saved_std_id;
#endif
}

/* 保存所有参数到 Flash (CAN ID 0x7FF, 命令 0xAA, RID=0x01) */
void DMMotorSaveParams(DMMotorInstance *motor)
{
    if (motor == NULL || motor->motor_can_instace == NULL)
        return;
    DMMotorSetMode(DM_CMD_RESET_MODE, motor);  // 先停电机
    DWT_Delay(0.01);

    CANInstance *can = motor->motor_can_instace;
    uint8_t *tx = can->tx_buff;

    tx[0] = (uint8_t)(motor->can_id & 0xff);
    tx[1] = (uint8_t)((motor->can_id >> 8) & 0xff);
    tx[2] = 0xAA;
    tx[3] = 0x01;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;

    uint32_t saved_id = can->tx_id;
    can->tx_id = 0x7FF;
#ifdef FDCAN
    uint32_t saved_identifier = can->txconf.Identifier;
    can->txconf.Identifier = 0x7FF;
#else
    uint32_t saved_std_id = can->txconf.StdId;
    can->txconf.StdId = 0x7FF;
#endif
    CANTransmit(can, 1);
    can->tx_id = saved_id;
#ifdef FDCAN
    can->txconf.Identifier = saved_identifier;
#else
    can->txconf.StdId = saved_std_id;
#endif

    osDelay(100);  // 等待写入完成
}

void DMMotorTask(void const *argument)
{
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    DMMotor_Send_s motor_send_mailbox;
    float set;

    while (1)
    {
        /* Keep the request separate from feedback so disable can interrupt an
           enable retry, and a reconnect can resume the requested state. */
        if (motor->stop_flag == MOTOR_STOP)
        {
            DMMotorSetMode(DM_CMD_RESET_MODE, motor);
            osDelay(20);
            continue;
        }
        /* Enable is retried independently; velocity commands require a valid
           feedback frame before they are allowed onto the bus. */
        /* A valid DM feedback frame is sufficient to permit the speed frame.
           Keep retrying enable for firmware revisions that report a non-1
           status nibble, but do not suppress the velocity command. */
        if (!DMMotorFeedbackFresh(motor))
        {
            DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
            osDelay(20);
            continue;
        }
        if (motor->measure.state != 1u)
            DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);

        float pid_ref = motor->pid_ref;

        set = pid_ref;
        if (motor->control_mode == DM_CTRL_MIT)
        {
            /* The public reference is torque in MIT mode. Position control
               is represented by DM_CTRL_POS_VEL and sent as raw floats. */
            set = pid_ref;
        }
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1.0f;

        /* 限制扭矩在电机允许范围内 */
        if (set > motor->limits.t_max)
            set = motor->limits.t_max;
        else if (set < -motor->limits.t_max)
            set = -motor->limits.t_max;

        /* MIT 模式打包: pos=0, vel=0, Kp=0, Kd=0 (纯扭矩控制) */
        motor_send_mailbox.position_des = clamp_float_to_uint(0.0f, -motor->limits.p_max, motor->limits.p_max, 16);
        motor_send_mailbox.velocity_des = clamp_float_to_uint(0.0f, -motor->limits.v_max, motor->limits.v_max, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;
        motor_send_mailbox.torque_des = clamp_float_to_uint(set, -motor->limits.t_max, motor->limits.t_max, 12);

        if (motor->stop_flag == MOTOR_STOP)
            motor_send_mailbox.torque_des = clamp_float_to_uint(0.0f, -motor->limits.t_max, motor->limits.t_max, 12);

        uint8_t *tx = motor->motor_can_instace->tx_buff;
        if (motor->control_mode == DM_CTRL_POS_VEL)
        {
            float position = motor->pid_ref;
            float velocity = motor->limits.v_max;
            if (motor->stop_flag == MOTOR_STOP)
                velocity = 0.0f;
            memcpy(tx, &position, sizeof(float));
            memcpy(tx + 4, &velocity, sizeof(float));
        }
        else if (motor->control_mode == DM_CTRL_VEL)
        {
            float velocity = motor->pid_ref;
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                velocity = -velocity;
            LIMIT_MIN_MAX(velocity, -motor->limits.v_max, motor->limits.v_max);
            if (motor->stop_flag == MOTOR_STOP)
                velocity = 0.0f;
            memcpy(tx, &velocity, sizeof(float));
            memset(tx + 4, 0, 4);
        }
        else if (motor->control_mode == DM_CTRL_POS_FORCE)
        {
            float position = motor->pid_ref;
            float force = (motor->stop_flag == MOTOR_STOP) ? 0.0f : set;
            memcpy(tx, &position, sizeof(float));
            memcpy(tx + 4, &force, sizeof(float));
        }
        else
        {
            tx[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
            tx[1] = (uint8_t)(motor_send_mailbox.position_des);
            tx[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
            tx[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | ((motor_send_mailbox.Kp >> 8) & 0xF));
            tx[4] = (uint8_t)(motor_send_mailbox.Kp);
            tx[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
            tx[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | ((motor_send_mailbox.torque_des >> 8) & 0xF));
            tx[7] = (uint8_t)(motor_send_mailbox.torque_des);
        }

        motor->motor_can_instace->tx_id = DMMotorControlFrameId(motor);

        /* Keep the configured transmit header synchronized with the selected mode. */
#ifdef FDCAN
        motor->motor_can_instace->txconf.Identifier = motor->motor_can_instace->tx_id;
#else
        motor->motor_can_instace->txconf.StdId = motor->motor_can_instace->tx_id;
#endif
        CANTransmit(motor->motor_can_instace, 1);

        osDelay(2);
    }
}

void DMMotorControlInit(void)
{
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        /* CMSIS-RTOS requires a compile-time identifier for osThreadDef. */
        switch (i)
        {
        case 0: { osThreadDef(dm0, DMMotorTask, osPriorityNormal, 0, 256); dm_task_handle[i] = osThreadCreate(osThread(dm0), dm_motor_instance[i]); break; }
        case 1: { osThreadDef(dm1, DMMotorTask, osPriorityNormal, 0, 256); dm_task_handle[i] = osThreadCreate(osThread(dm1), dm_motor_instance[i]); break; }
        case 2: { osThreadDef(dm2, DMMotorTask, osPriorityNormal, 0, 256); dm_task_handle[i] = osThreadCreate(osThread(dm2), dm_motor_instance[i]); break; }
        case 3: { osThreadDef(dm3, DMMotorTask, osPriorityNormal, 0, 256); dm_task_handle[i] = osThreadCreate(osThread(dm3), dm_motor_instance[i]); break; }
        default: break;
        }
    }
}
