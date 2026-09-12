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

static void DecodeVision(void)
{
    uint16_t flag_register;
    get_protocol_info(vision_usart_instance[0]->recv_buff, &flag_register, (uint8_t *)&recv_data.v);
    if (vision_application_callback)
        vision_application_callback();
}

static void DecodeNav(void)
{
    uint16_t flag_register;
    get_protocol_info(vision_usart_instance[1]->recv_buff, &flag_register, (uint8_t *)&recv_data_nav.vx);
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

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP
#include "bsp_usb.h"
static uint8_t *vis_recv_buff = 0;
static DaemonInstance *vision_daemon_instance[1] = {0};

static void VisionOfflineCallback(void *id)
{
    recv_data.v=0;
    recv_data.w=0;
    LOGWARNING("[vision] vision offline, restart communication.");
}

static void DecodeVision(uint8_t *buf, uint16_t len)
{
    (void)len;
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance[0]);
    get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.v);
    if (vision_application_callback)
        vision_application_callback();
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle, void (*application_callback)(void))
{
    vision_application_callback = application_callback;

    USB_Init_Config_s conf = {
        .tx_cbk = NULL,
        .rx_cbk = DecodeVision
    };

    vis_recv_buff = USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 50,
    };
    vision_daemon_instance[0] = DaemonRegister(&daemon_conf);

    (void)_handle; // 保留参数占位
    return &recv_data;
}

void VisionSend(void)
{
    uint16_t flag_register = 0x1E01;
    uint8_t send_buff[VISION_SEND_SIZE];
    uint16_t tx_len;
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
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
#ifdef VISION_USE_VCP
    USBTransmit(send_buff, tx_len);
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