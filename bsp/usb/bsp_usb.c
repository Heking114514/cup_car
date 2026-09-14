#include "bsp_usb.h"
#include "bsp_log.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

/* 静态变量保存回调和缓冲区指针 */
static uint8_t *usb_rx_buffer_ptr;

/* 初始化 USB，设置回调和接收缓冲区 */
uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    // usb的软件复位(模拟拔插)在usbd_conf.c中的HAL_PCD_MspInit()中
    usb_rx_buffer_ptr = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk); // 获取接收数据指针
    // usb的接收回调函数会在这里被设置,并将数据保存在bsp_usb_rx_buffer中
    LOGINFO("USB init success");
    return usb_rx_buffer_ptr;
}

/* USB 数据发送函数 */
uint8_t USBTransmit(const uint8_t *buffer, uint16_t len)
{
    return CDC_Transmit_FS((uint8_t *)buffer, len);
}
