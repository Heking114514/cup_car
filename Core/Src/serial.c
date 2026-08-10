#include "serial.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"

#define SERIAL_RX_BUFFER_SIZE 128U
#define SERIAL_TX_FORMAT_BUFFER_SIZE 192U

typedef struct {
  uint8_t rx_byte;
  volatile uint8_t buffer[SERIAL_RX_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
} serial_rx_buffer_t;

static serial_rx_buffer_t uart1_rx;
static serial_rx_buffer_t uart2_rx;

static UART_HandleTypeDef *serial_handle(serial_port_t port)
{
  return port == SERIAL_AUX ? &huart2 : &huart1;
}

static serial_rx_buffer_t *serial_rx_buffer(serial_port_t port)
{
  return port == SERIAL_AUX ? &uart2_rx : &uart1_rx;
}

static void serial_start_receive(serial_port_t port)
{
  serial_rx_buffer_t *rx = serial_rx_buffer(port);
  HAL_UART_Receive_IT(serial_handle(port), &rx->rx_byte, 1U);
}

void serial_init(void)
{
  serial_start_receive(SERIAL_DEBUG);
  serial_start_receive(SERIAL_AUX);
}

void serial_write(serial_port_t port, const uint8_t *data, uint16_t length)
{
  HAL_UART_Transmit(serial_handle(port), (uint8_t *)data, length, 100U);
}

void serial_print(serial_port_t port, const char *text)
{
  serial_write(port, (const uint8_t *)text, (uint16_t)strlen(text));
}

void serial_printf(serial_port_t port, const char *format, ...)
{
  char buffer[SERIAL_TX_FORMAT_BUFFER_SIZE];
  va_list args;
  int length;

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length <= 0) {
    return;
  }
  if (length >= (int)sizeof(buffer)) {
    length = sizeof(buffer) - 1;
  }
  serial_write(port, (const uint8_t *)buffer, (uint16_t)length);
}

bool serial_read_byte(serial_port_t port, uint8_t *byte)
{
  serial_rx_buffer_t *rx = serial_rx_buffer(port);

  if (rx->head == rx->tail) {
    return false;
  }

  *byte = rx->buffer[rx->tail];
  rx->tail = (rx->tail + 1U) % SERIAL_RX_BUFFER_SIZE;
  return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  serial_port_t port;
  serial_rx_buffer_t *rx;
  uint16_t next_head;

  if (huart->Instance == USART1) {
    port = SERIAL_DEBUG;
  } else if (huart->Instance == USART2) {
    port = SERIAL_AUX;
  } else {
    return;
  }

  rx = serial_rx_buffer(port);
  next_head = (rx->head + 1U) % SERIAL_RX_BUFFER_SIZE;
  if (next_head != rx->tail) {
    rx->buffer[rx->head] = rx->rx_byte;
    rx->head = next_head;
  }

  serial_start_receive(port);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    serial_start_receive(SERIAL_DEBUG);
  } else if (huart->Instance == USART2) {
    serial_start_receive(SERIAL_AUX);
  }
}
