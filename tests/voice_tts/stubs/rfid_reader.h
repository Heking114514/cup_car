#ifndef TEST_RFID_H
#define TEST_RFID_H
#include <stdint.h>
#define RFID_UID_MAX_SIZE 10U
#define RFID_STATUS_CARD_PRESENT 1U
#define RFID_STATUS_NUMBER_VALID 2U
typedef struct {
    uint8_t status, number, uid_size, uid[RFID_UID_MAX_SIZE];
    uint32_t card_sequence;
} RFID_State_s;
void RFIDGetState(RFID_State_s *state);
#endif
