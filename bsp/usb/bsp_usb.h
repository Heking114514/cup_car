#pragma once
#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"


/* USB 初始化配置结构 */
typedef struct
{
    USBCallback tx_cbk;  // 可选：发送完成回调
    USBCallback rx_cbk;  // 必选：接收到数据回调
} USB_Init_Config_s;

/* 初始化 USB，返回接收缓冲区指针 */
uint8_t *USBInit(USB_Init_Config_s usb_conf);

/* 通过 USB 发送数据 */
void USBTransmit(uint8_t *buffer, uint16_t len);

/* 内部函数：USB 接收到数据时调用，调用上层回调 */
void CDC_Receive_HS_Callback(uint8_t *buf, uint16_t len);