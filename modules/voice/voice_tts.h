#ifndef VOICE_TTS_H
#define VOICE_TTS_H

#include <stdint.h>

#include "main.h"

#define VOICE_TTS_STATUS_READY    0x01U
#define VOICE_TTS_STATUS_TX_OK    0x02U
#define VOICE_TTS_STATUS_TX_ERROR 0x04U

typedef struct
{
    uint32_t last_tx_time_ms;
    uint32_t tx_count;
    uint32_t tx_error_count;
    uint8_t status;
} VoiceTTS_State_s;

void VoiceTTSInit(UART_HandleTypeDef *uart_handle);
void VoiceTTSTask(void);
uint8_t VoiceTTSSpeakGBK(const uint8_t *data, uint16_t len);
uint8_t VoiceTTSSpeakASCII(const char *text);
uint8_t VoiceTTSSetVolume(uint8_t level);
uint8_t VoiceTTSSetSpeed(uint8_t level);
uint8_t VoiceTTSPlayEffect(uint8_t effect_id);
void VoiceTTSGetState(VoiceTTS_State_s *state);

#endif
