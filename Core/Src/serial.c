#include "serial.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"

#define SERIAL_RX_BUFFER_SIZE 128U
#define SERIAL_TX_BUFFER_SIZE 1024U
#define SERIAL_TX_FORMAT_BUFFER_SIZE 256U

typedef struct {
  uint8_t rx_byte;
  volatile uint8_t buffer[SERIAL_RX_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
} serial_rx_buffer_t;

typedef struct {
  uint8_t buffer[SERIAL_TX_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint16_t in_flight;
  volatile bool active;
} serial_tx_buffer_t;

static serial_rx_buffer_t uart1_rx;
static serial_rx_buffer_t uart2_rx;
static serial_rx_buffer_t uart3_rx;
static serial_tx_buffer_t uart1_tx;
static serial_tx_buffer_t uart2_tx;
static serial_tx_buffer_t uart3_tx;

static UART_HandleTypeDef *serial_handle(serial_port_t port)
{
  if (port == SERIAL_HOST) {
    return &huart3;
  }
  return port == SERIAL_AUX ? &huart2 : &huart1;
}

static serial_rx_buffer_t *serial_rx_buffer(serial_port_t port)
{
  if (port == SERIAL_HOST) {
    return &uart3_rx;
  }
  return port == SERIAL_AUX ? &uart2_rx : &uart1_rx;
}

static serial_tx_buffer_t *serial_tx_buffer(serial_port_t port)
{
  if (port == SERIAL_HOST) {
    return &uart3_tx;
  }
  return port == SERIAL_AUX ? &uart2_tx : &uart1_tx;
}

static serial_tx_buffer_t *serial_tx_buffer_from_handle(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    return &uart1_tx;
  }
  if (huart->Instance == USART2) {
    return &uart2_tx;
  }
  if (huart->Instance == USART3) {
    return &uart3_tx;
  }
  return NULL;
}

static uint16_t serial_tx_free(const serial_tx_buffer_t *tx)
{
  if (tx->head >= tx->tail) {
    return (uint16_t)(SERIAL_TX_BUFFER_SIZE - (tx->head - tx->tail) - 1U);
  }
  return (uint16_t)(tx->tail - tx->head - 1U);
}

static void serial_start_transmit(serial_port_t port)
{
  serial_tx_buffer_t *tx = serial_tx_buffer(port);
  uint16_t length;

  if (tx->active || tx->head == tx->tail) {
    return;
  }

  length = tx->head > tx->tail
             ? (uint16_t)(tx->head - tx->tail)
             : (uint16_t)(SERIAL_TX_BUFFER_SIZE - tx->tail);
  tx->active = true;
  tx->in_flight = length;
  if (HAL_UART_Transmit_IT(serial_handle(port), &tx->buffer[tx->tail], length) != HAL_OK) {
    tx->active = false;
    tx->in_flight = 0U;
  }
}

static void serial_start_receive(serial_port_t port)
{
  serial_rx_buffer_t *rx = serial_rx_buffer(port);
  HAL_UART_Receive_IT(serial_handle(port), &rx->rx_byte, 1U);
}

void serial_init(void)
{
  uart1_tx.head = 0U;
  uart1_tx.tail = 0U;
  uart1_tx.in_flight = 0U;
  uart1_tx.active = false;
  uart2_tx.head = 0U;
  uart2_tx.tail = 0U;
  uart2_tx.in_flight = 0U;
  uart2_tx.active = false;
  uart3_tx.head = 0U;
  uart3_tx.tail = 0U;
  uart3_tx.in_flight = 0U;
  uart3_tx.active = false;
  serial_start_receive(SERIAL_DEBUG);
  serial_start_receive(SERIAL_AUX);
  serial_start_receive(SERIAL_HOST);
}

void serial_write(serial_port_t port, const uint8_t *data, uint16_t length)
{
  serial_tx_buffer_t *tx = serial_tx_buffer(port);
  uint16_t index;
  uint32_t primask;

  if (data == NULL || length == 0U || length >= SERIAL_TX_BUFFER_SIZE) {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (serial_tx_free(tx) < length) {
    if (primask == 0U) {
      __enable_irq();
    }
    return;
  }

  index = tx->head;
  while (length > 0U) {
    tx->buffer[index] = *data++;
    index = (uint16_t)((index + 1U) % SERIAL_TX_BUFFER_SIZE);
    length--;
  }
  tx->head = index;
  serial_start_transmit(port);
  if (primask == 0U) {
    __enable_irq();
  }
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
  } else if (huart->Instance == USART3) {
    port = SERIAL_HOST;
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  serial_port_t port;
  serial_tx_buffer_t *tx = serial_tx_buffer_from_handle(huart);

  if (tx == NULL) {
    return;
  }

  if (huart->Instance == USART3) {
    port = SERIAL_HOST;
  } else {
    port = huart->Instance == USART2 ? SERIAL_AUX : SERIAL_DEBUG;
  }
  tx->tail = (uint16_t)((tx->tail + tx->in_flight) % SERIAL_TX_BUFFER_SIZE);
  tx->in_flight = 0U;
  tx->active = false;
  serial_start_transmit(port);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    serial_start_receive(SERIAL_DEBUG);
  } else if (huart->Instance == USART2) {
    serial_start_receive(SERIAL_AUX);
  } else if (huart->Instance == USART3) {
    serial_start_receive(SERIAL_HOST);
  }
}
