#ifndef RFID_READER_H
#define RFID_READER_H

#include <stdint.h>
#include "main.h"

#define RFID_UID_MAX_SIZE 16U

#define RFID_STATUS_READY          0x01U
#define RFID_STATUS_CARD_PRESENT   0x02U
#define RFID_STATUS_TIMEOUT        0x04U
#define RFID_STATUS_PROTOCOL_ERROR 0x08U
#define RFID_STATUS_READER_ONLINE  0x10U

typedef struct
{
    uint32_t update_time_ms;
    uint32_t card_sequence;
    uint8_t status;
    uint8_t protocol_status;
    uint16_t tag_type;
    uint8_t sak;
    uint8_t uid_size;
    uint8_t uid[RFID_UID_MAX_SIZE];
    char version[32];
} RFID_State_s;

void RFIDInit(UART_HandleTypeDef *uart_handle);
void RFIDTask(void);
void RFIDGetState(RFID_State_s *state);

#endif
