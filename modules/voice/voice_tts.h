#ifndef VOICE_TTS_H
#define VOICE_TTS_H

#include <stdint.h>

#include "main.h"

#define VOICE_TTS_STATUS_READY    0x01U
#define VOICE_TTS_STATUS_TX_OK    0x02U
#define VOICE_TTS_STATUS_TX_ERROR 0x04U
#define VOICE_TTS_STATUS_BUSY     0x08U

typedef struct
{
    uint32_t last_tx_time_ms;
    uint32_t tx_count;
    uint32_t tx_error_count;
    uint8_t status;
} VoiceTTS_State_s;

#define VOICE_TTS_HOST_MAX_BYTES 128U

typedef enum {
    VOICE_HOST_NONE = 0, VOICE_HOST_QUEUED, VOICE_HOST_PLAYING,
    VOICE_HOST_ESTIMATED_DONE, VOICE_HOST_ERROR, VOICE_HOST_BUSY,
    VOICE_HOST_INVALID
} VoiceTTS_HostStatus_e;

typedef struct {
    uint32_t id;
    uint32_t rejected_id;
    VoiceTTS_HostStatus_e status;
    VoiceTTS_HostStatus_e rejected_status;
} VoiceTTS_HostState_s;

void VoiceTTSGetHostState(VoiceTTS_HostState_s *state);
const char *VoiceTTSHostStatusName(VoiceTTS_HostStatus_e status);
uint8_t VoiceTTSParseHostLine(const uint8_t *line, uint16_t len);
uint8_t VoiceTTSQueueHostRequest(uint32_t id, const uint8_t *data, uint16_t len);
void VoiceTTSInit(UART_HandleTypeDef *uart_handle);
void VoiceTTSTask(void);
uint8_t VoiceTTSSpeakGBK(const uint8_t *data, uint16_t len);
uint8_t VoiceTTSQueueHostGBK(const uint8_t *data, uint16_t len);
uint8_t VoiceTTSIsHostReady(void);
uint8_t VoiceTTSSpeakASCII(const char *text);
uint8_t VoiceTTSSetVolume(uint8_t level);
uint8_t VoiceTTSSetSpeed(uint8_t level);
uint8_t VoiceTTSPlayEffect(uint8_t effect_id);
void VoiceTTSGetState(VoiceTTS_State_s *state);

#endif
