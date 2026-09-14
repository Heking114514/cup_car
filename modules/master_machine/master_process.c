#include "master_process.h"
#include "robot_def.h" // 包含 robot_def.h，访问宏等
#include "daemon.h"
#include "bsp_log.h"
#include <string.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static Vision_Refree_Send_s send_data_refree;
static Nav_Recv_s recv_data_nav;
static void (*vision_application_callback)(void);

#ifdef VISION_USE_UART
#include "bsp_usart.h"

static USARTInstance *vision_usart_instance[2] = {0};
static DaemonInstance *vision_daemon_instance[2] = {0};

static void DecodeVision(USARTInstance *instance)
{
    uint16_t flag_register;
    get_protocol_info(instance->recv_buff, &flag_register, (uint8_t *)&recv_data.v);
    if (vision_application_callback)
        vision_application_callback();
}

static void DecodeNav(USARTInstance *instance)
{
    uint16_t flag_register;
    get_protocol_info(instance->recv_buff, &flag_register, (uint8_t *)&recv_data_nav.vx);
    if (vision_application_callback)
        vision_application_callback();
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle, void (*application_callback)(void))
{
    vision_application_callback = application_callback;

    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance[0] = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = NULL,
        .owner_id = vision_usart_instance[0],
        .reload_count = 100,
    };
    vision_daemon_instance[0] = DaemonRegister(&daemon_conf);

    return &recv_data;
}

Nav_Recv_s *NavInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeNav;
    conf.recv_buff_size = Nav_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance[1] = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = NULL,
        .owner_id = vision_usart_instance[1],
        .reload_count = 100,
    };
    vision_daemon_instance[1] = DaemonRegister(&daemon_conf);

    return &recv_data_nav;
}

void VisionSend(void)
{
    uint16_t flag_register = 0x1E01;
    uint8_t send_buff[VISION_SEND_SIZE];
    uint16_t tx_len;
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance[0], send_buff, tx_len, USART_TRANSFER_DMA);
}

void VisionProcess(void)
{
}

void VisionSetNavigationMode(uint8_t enabled)
{
    (void)enabled;
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP
#include "bsp_usb.h"
#if CHASSIS_USE_INS
#include "ins_task.h"
#else
#include "bmi088_diag.h"
#endif
#include "chassis.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define HOST_COMMAND_LINE_MAX        48U
#define HOST_RX_BUFFER_SIZE         256U
#define HOST_COMMAND_TIMEOUT_MS     500U
#define HOST_ENCODER_REPORT_MS       50U
#define HOST_CONTROL_REPORT_MS      100U
#define HOST_MAX_VX_MPS               2.0f
#define HOST_MAX_AZ_RADPS             10.0f
#define HOST_MILLI_LIMIT         2147483.0f

static uint8_t *vis_recv_buff = 0;
static uint8_t host_rx_buffer[HOST_RX_BUFFER_SIZE];
static volatile uint16_t host_rx_head;
static volatile uint16_t host_rx_tail;
static volatile uint8_t host_rx_overflow;
static char host_command_line[HOST_COMMAND_LINE_MAX];
static uint8_t host_command_length;
static uint8_t host_command_discarding;
static uint8_t host_command_seen;
static uint8_t host_command_valid;
static uint8_t host_navigation_mode;
static uint32_t host_last_command_ms;
static uint32_t host_last_encoder_report_ms;
static uint32_t host_last_control_report_ms;
static float host_last_vx_mps;
static float host_last_az_radps;

static void HostInvalidateCommand(void)
{
    recv_data.v = 0.0f;
    recv_data.w = 0.0f;
    host_command_valid = 0U;
}

static int32_t HostToMilli(float value)
{
    float scaled;

    if (!isfinite(value))
        return 0;
    if (value >= HOST_MILLI_LIMIT)
        return INT32_MAX;
    if (value <= -HOST_MILLI_LIMIT)
        return INT32_MIN;
    scaled = value * 1000.0f;
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static uint8_t HostParseFloat(char **cursor, float *value, char separator)
{
    char *end;

    *value = strtof(*cursor, &end);
    if (end == *cursor || !isfinite(*value))
        return 0U;

    if (separator != '\0')
    {
        if (*end != separator)
            return 0U;
        *cursor = end + 1;
        return 1U;
    }

    while (*end == ' ' || *end == '\t' || *end == '\r')
        end++;
    if (*end != '\0')
        return 0U;
    *cursor = end;
    return 1U;
}

static void HostParseVelocity(void)
{
    char *cursor = host_command_line;
    float vx;
    float az;

    if (!HostParseFloat(&cursor, &vx, ',') ||
        !HostParseFloat(&cursor, &az, '\0') ||
        fabsf(vx) > HOST_MAX_VX_MPS ||
        fabsf(az) > HOST_MAX_AZ_RADPS)
        return;

    recv_data.v = vx;
    recv_data.w = az;
    host_last_vx_mps = vx;
    host_last_az_radps = az;
    host_last_command_ms = HAL_GetTick();
    host_command_seen = 1U;
    host_command_valid = 1U;
    if (vision_application_callback)
        vision_application_callback();
}

static uint8_t HostReadByte(uint8_t *byte)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (host_rx_tail == host_rx_head)
    {
        if (primask == 0U)
            __enable_irq();
        return 0U;
    }
    *byte = host_rx_buffer[host_rx_tail];
    host_rx_tail = (uint16_t)((host_rx_tail + 1U) % HOST_RX_BUFFER_SIZE);
    if (primask == 0U)
        __enable_irq();
    return 1U;
}

static uint8_t HostRecoverRxOverflow(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t overflowed;

    __disable_irq();
    overflowed = host_rx_overflow;
    if (overflowed)
    {
        host_rx_overflow = 0U;
        host_rx_tail = host_rx_head;
    }
    if (primask == 0U)
        __enable_irq();

    if (overflowed)
    {
        host_command_length = 0U;
        host_command_discarding = 1U;
    }
    return overflowed;
}

static void DecodeVision(uint16_t len)
{
    uint16_t index;

    if (vis_recv_buff == NULL)
        return;

    for (index = 0U; index < len; index++)
    {
        uint16_t next_head = (uint16_t)((host_rx_head + 1U) % HOST_RX_BUFFER_SIZE);
        if (next_head == host_rx_tail)
        {
            host_rx_overflow = 1U;
            break;
        }
        host_rx_buffer[host_rx_head] = vis_recv_buff[index];
        host_rx_head = next_head;
    }
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle, void (*application_callback)(void))
{
    vision_application_callback = application_callback;

    USB_Init_Config_s conf = {
        .tx_cbk = NULL,
        .rx_cbk = DecodeVision
    };

    host_rx_head = 0U;
    host_rx_tail = 0U;
    host_rx_overflow = 0U;
    host_command_length = 0U;
    host_command_discarding = 0U;
    host_navigation_mode = 0U;
    host_command_seen = 0U;
    host_last_vx_mps = 0.0f;
    host_last_az_radps = 0.0f;
    HostInvalidateCommand();
    host_last_command_ms = HAL_GetTick();
    host_last_encoder_report_ms = host_last_command_ms;
    host_last_control_report_ms = host_last_command_ms;
    vis_recv_buff = USBInit(conf);

    (void)_handle; // 保留参数占位
    return &recv_data;
}

void VisionProcess(void)
{
    uint8_t byte;

    HostRecoverRxOverflow();
    for (;;)
    {
        if (HostRecoverRxOverflow())
            continue;
        if (!HostReadByte(&byte))
            break;

        if (host_command_discarding)
        {
            if (byte == '\n')
            {
                host_command_discarding = 0U;
                host_command_length = 0U;
            }
            continue;
        }

        if (byte == '\n')
        {
            if (host_command_length > 0U)
            {
                host_command_line[host_command_length] = '\0';
                HostParseVelocity();
            }
            host_command_length = 0U;
        }
        else if (byte == '\r')
        {
            continue;
        }
        else if (host_command_length < HOST_COMMAND_LINE_MAX - 1U)
        {
            host_command_line[host_command_length++] = (char)byte;
        }
        else
        {
            host_command_length = 0U;
            host_command_discarding = 1U;
        }
    }

    if (host_command_valid &&
        HAL_GetTick() - host_last_command_ms > HOST_COMMAND_TIMEOUT_MS)
        HostInvalidateCommand();
}

static int HostAppendEncoder(char *buffer, size_t capacity, uint32_t now,
                             uint32_t sequence,
                             const Chassis_Host_Feedback_s *chassis)
{
    return snprintf(buffer, capacity, "ENC,%lu,%lu,%ld,%ld\r\n",
                    (unsigned long)now,
                    (unsigned long)sequence,
                    (long)chassis->left_total_count,
                    (long)chassis->right_total_count);
}

static int HostAppendControl(char *buffer, size_t capacity, uint32_t now,
                             uint32_t sequence,
                             const Chassis_Host_Feedback_s *chassis)
{
    uint32_t age = host_command_seen
        ? now - host_last_command_ms
        : UINT32_MAX;

    return snprintf(
        buffer, capacity,
        "CTL,%lu,%lu,%u,0,%u,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d\r\n",
        (unsigned long)now,
        (unsigned long)sequence,
        (unsigned int)host_navigation_mode,
        (unsigned int)host_command_valid,
        (unsigned long)age,
        (long)HostToMilli(host_last_vx_mps),
        (long)HostToMilli(host_last_az_radps),
        (long)HostToMilli(chassis->target_left_mps),
        (long)HostToMilli(chassis->target_right_mps),
        (long)HostToMilli(chassis->measured_left_mps),
        (long)HostToMilli(chassis->measured_right_mps),
        (int)chassis->left_output,
        (int)chassis->right_output);
}

static int HostAppendAttitude(char *buffer, size_t capacity, uint32_t now,
                              uint32_t sequence)
{
#if CHASSIS_USE_INS
    return snprintf(buffer, capacity,
                    "ATT,%lu,%lu,%u,%ld,%ld,%ld,%u\r\n",
                    (unsigned long)now,
                    (unsigned long)sequence,
                    0x01U,
                    (long)HostToMilli(INS_GetYawTotalAngle()),
                    (long)HostToMilli(INS_GetGyroZDegps()),
                    (long)HostToMilli(INS_GetGyroZBiasDegps()),
                    0U);
#else
    BMI088DiagState imu_state;

    BMI088DiagGetState(&imu_state);
    return snprintf(buffer, capacity,
                    "ATT,%lu,%lu,%u,%ld,%ld,%ld,%u\r\n",
                    (unsigned long)now,
                    (unsigned long)sequence,
                    (unsigned int)imu_state.status,
                    (long)HostToMilli(imu_state.yaw_deg),
                    (long)HostToMilli(imu_state.gyro_z_dps),
                    (long)HostToMilli(imu_state.gyro_z_bias_dps),
                    (unsigned int)imu_state.startup_samples);
#endif
}

static int HostAppendHeadingHold(char *buffer, size_t capacity, uint32_t now,
                                 uint32_t sequence,
                                 const Chassis_Host_Feedback_s *chassis)
{
    return snprintf(buffer, capacity,
                    "HLD,%lu,%lu,%u,%ld,%ld,%ld\r\n",
                    (unsigned long)now,
                    (unsigned long)sequence,
                    (unsigned int)chassis->heading_active,
                    (long)HostToMilli(chassis->heading_ref_deg),
                    (long)HostToMilli(chassis->heading_error_deg),
                    (long)HostToMilli(chassis->heading_correction_ref));
}

void VisionSend(void)
{
    char send_buffer[384];
    Chassis_Host_Feedback_s chassis;
#if !CHASSIS_USE_INS
    BMI088DiagSample imu;
#endif
    uint32_t now = HAL_GetTick();
    uint32_t sequence = now / 10U;
    uint8_t encoder_due = now - host_last_encoder_report_ms >= HOST_ENCODER_REPORT_MS;
    uint8_t control_due = now - host_last_control_report_ms >= HOST_CONTROL_REPORT_MS;
    int length = 0;
    int appended;

    if (!encoder_due && !control_due)
        return;

    ChassisGetHostFeedback(&chassis);
    if (encoder_due)
    {
        appended = HostAppendEncoder(send_buffer, sizeof(send_buffer), now,
                                     sequence, &chassis);
        if (appended <= 0 || appended >= (int)sizeof(send_buffer))
            return;
        length = appended;

#if CHASSIS_USE_INS
        appended = snprintf(send_buffer + length, sizeof(send_buffer) - (size_t)length,
                            "IMU,%lu,%lu,%u,%d,%d,%d,%d\r\n",
                            (unsigned long)now, (unsigned long)sequence,
                            0x01U, 0, 0,
                            (int)HostToMilli(INS_GetGyroZDegps()), 0);
#else
        BMI088DiagRead(&imu);
        appended = snprintf(send_buffer + length, sizeof(send_buffer) - (size_t)length,
                            "IMU,%lu,%lu,%u,%d,%d,%d,%d\r\n",
                            (unsigned long)now, (unsigned long)sequence,
                            (unsigned int)imu.status, (int)imu.gx, (int)imu.gy,
                            (int)imu.gz, (int)imu.temperature_cC);
#endif
        if (appended <= 0 || appended >= (int)(sizeof(send_buffer) - (size_t)length))
            return;
        length += appended;

        appended = HostAppendAttitude(send_buffer + length,
                                      sizeof(send_buffer) - (size_t)length,
                                      now, sequence);
        if (appended <= 0 || appended >= (int)(sizeof(send_buffer) - (size_t)length))
            return;
        length += appended;

        appended = HostAppendHeadingHold(send_buffer + length,
                                         sizeof(send_buffer) - (size_t)length,
                                         now, sequence, &chassis);
        if (appended <= 0 || appended >= (int)(sizeof(send_buffer) - (size_t)length))
            return;
        length += appended;
    }
    if (control_due)
    {
        appended = HostAppendControl(send_buffer + length,
                                     sizeof(send_buffer) - (size_t)length,
                                     now, sequence, &chassis);
        if (appended <= 0 || appended >= (int)(sizeof(send_buffer) - (size_t)length))
            return;
        length += appended;
    }

    if (USBTransmit((const uint8_t *)send_buffer, (uint16_t)length) == USBD_OK)
    {
        if (encoder_due)
            host_last_encoder_report_ms = now;
        if (control_due)
            host_last_control_report_ms = now;
    }
}

void VisionSetNavigationMode(uint8_t enabled)
{
    host_navigation_mode = enabled ? 1U : 0U;
}

#endif // VISION_USE_VCP

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

void VisionRefree_SetAltitude(float battery, float life, float color, float bullet, float game_mode)
{
    send_data_refree.battery = battery;
    send_data_refree.life = life;
    send_data_refree.color = color;
    send_data_refree.bullet = bullet;
    send_data_refree.game_mode = game_mode;
}

void Vision_Refree_Send(void)
{
    uint16_t flag_register = 0x1E01;
    uint8_t send_buff[VISION_SEND_SIZE];
    uint16_t tx_len;
    get_protocol_send_data(0x20, flag_register, &send_data_refree.battery, 5, send_buff, &tx_len);

#ifdef VISION_USE_UART
    USARTSend(vision_usart_instance[0], send_buff, tx_len, USART_TRANSFER_DMA);
#endif
}

void Vision_Send_All(void)
{
    static uint8_t i = 0;
    if (i == 0)
    {
        VisionSend();
        i++;
    }
    else
    {
        Vision_Refree_Send();
        i = 0;
    }
}
