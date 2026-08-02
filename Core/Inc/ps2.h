#ifndef PS2_H
#define PS2_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool connected;
  uint8_t mode;
  uint16_t buttons;
  uint8_t right_x;
  uint8_t right_y;
  uint8_t left_x;
  uint8_t left_y;
} ps2_state_t;

#define PS2_BTN_SELECT   0x0001U
#define PS2_BTN_START    0x0008U
#define PS2_BTN_UP       0x0010U
#define PS2_BTN_RIGHT    0x0020U
#define PS2_BTN_DOWN     0x0040U
#define PS2_BTN_LEFT     0x0080U
#define PS2_BTN_L2       0x0100U
#define PS2_BTN_R2       0x0200U
#define PS2_BTN_L1       0x0400U
#define PS2_BTN_R1       0x0800U
#define PS2_BTN_TRIANGLE 0x1000U
#define PS2_BTN_CIRCLE   0x2000U
#define PS2_BTN_CROSS    0x4000U
#define PS2_BTN_SQUARE   0x8000U

void ps2_init(void);
bool ps2_read(ps2_state_t *state);
bool ps2_button_down(const ps2_state_t *state, uint16_t button);

#endif
