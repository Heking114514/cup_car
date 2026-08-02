#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  SERIAL_DEBUG = 0,
  SERIAL_AUX
} serial_port_t;

void serial_init(void);
void serial_write(serial_port_t port, const uint8_t *data, uint16_t length);
void serial_print(serial_port_t port, const char *text);
void serial_printf(serial_port_t port, const char *format, ...);
bool serial_read_byte(serial_port_t port, uint8_t *byte);

#endif
