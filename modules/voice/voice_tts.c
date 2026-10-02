#include "voice_tts.h"

#include <stddef.h>
#include <string.h>

#include "rfid_reader.h"

#define VOICE_TTS_STARTUP_DELAY_MS      1000U
#define VOICE_TTS_MIN_INTERVAL_MS       1200U
#define VOICE_TTS_TX_BASE_TIMEOUT_MS    30U
#define VOICE_TTS_TX_PER_BYTE_TIMEOUT_MS 2U
#define VOICE_TTS_TX_BUFFER_SIZE        96U

typedef enum
{
    VOICE_TTS_STAGE_ASCII_TEST = 0,
    VOICE_TTS_STAGE_STARTUP_TEXT,
    VOICE_TTS_STAGE_READY,
} VoiceTTS_StartupStage_e;

static UART_HandleTypeDef *voice_uart;
static VoiceTTS_State_s voice_state;
static VoiceTTS_StartupStage_e voice_startup_stage;
static uint32_t voice_next_tx_time_ms;
static uint32_t voice_last_card_sequence;

static const uint8_t voice_startup_text[] = {
    0xB2, 0xA5, 0xB1, 0xA8, 0xC6, 0xF7, 0xC6, 0xF4, 0xB6, 0xAF
};

static const uint8_t voice_ascii_test_text[] = {
    '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'
};

static const uint8_t voice_rfid_identified_text[] = {
    0x52, 0x46, 0x49, 0x44, 0xD2, 0xD1, 0xCA, 0xB6, 0xB1, 0xF0
};

static uint32_t VoiceTTSTxTimeout(uint16_t len)
{
    return VOICE_TTS_TX_BASE_TIMEOUT_MS +
           (uint32_t)len * VOICE_TTS_TX_PER_BYTE_TIMEOUT_MS;
}

static uint8_t VoiceTTSWrite(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;

    if (voice_uart == NULL || data == NULL || len == 0U ||
        len > VOICE_TTS_TX_BUFFER_SIZE)
        return 0U;

    ret = HAL_UART_Transmit(voice_uart, (uint8_t *)data, len,
                            VoiceTTSTxTimeout(len));
    voice_state.last_tx_time_ms = HAL_GetTick();
    if (ret == HAL_OK)
    {
        voice_state.tx_count++;
        voice_state.status |= VOICE_TTS_STATUS_TX_OK;
        voice_state.status &= (uint8_t)~VOICE_TTS_STATUS_TX_ERROR;
        return 1U;
    }

    voice_state.tx_error_count++;
    voice_state.status |= VOICE_TTS_STATUS_TX_ERROR;
    return 0U;
}

void VoiceTTSInit(UART_HandleTypeDef *uart_handle)
{
    voice_uart = uart_handle;
    memset(&voice_state, 0, sizeof(voice_state));
    voice_state.status = VOICE_TTS_STATUS_READY;
    voice_startup_stage = VOICE_TTS_STAGE_ASCII_TEST;
    voice_next_tx_time_ms = VOICE_TTS_STARTUP_DELAY_MS;
    voice_last_card_sequence = 0U;
}

uint8_t VoiceTTSSpeakGBK(const uint8_t *data, uint16_t len)
{
    return VoiceTTSWrite(data, len);
}

uint8_t VoiceTTSSpeakASCII(const char *text)
{
    if (text == NULL)
        return 0U;

    return VoiceTTSWrite((const uint8_t *)text, (uint16_t)strlen(text));
}

uint8_t VoiceTTSSetVolume(uint8_t level)
{
    char cmd[] = {'<', 'V', '>', '4'};

    if (level < 1U)
        level = 1U;
    else if (level > 4U)
        level = 4U;

    cmd[3] = (char)('0' + level);
    return VoiceTTSWrite((const uint8_t *)cmd, (uint16_t)sizeof(cmd));
}

uint8_t VoiceTTSSetSpeed(uint8_t level)
{
    char cmd[] = {'<', 'S', '>', '2'};

    if (level < 1U)
        level = 1U;
    else if (level > 3U)
        level = 3U;

    cmd[3] = (char)('0' + level);
    return VoiceTTSWrite((const uint8_t *)cmd, (uint16_t)sizeof(cmd));
}

uint8_t VoiceTTSPlayEffect(uint8_t effect_id)
{
    char cmd[] = {'<', 'Z', '>', '0'};

    if (effect_id > 7U)
        effect_id = 7U;

    cmd[3] = (char)('0' + effect_id);
    return VoiceTTSWrite((const uint8_t *)cmd, (uint16_t)sizeof(cmd));
}

void VoiceTTSTask(void)
{
    uint32_t now = HAL_GetTick();
    RFID_State_s rfid_state;

    if (voice_uart == NULL)
        return;

    if (voice_startup_stage == VOICE_TTS_STAGE_ASCII_TEST)
    {
        if (now < voice_next_tx_time_ms)
            return;

        if (VoiceTTSSpeakGBK(voice_ascii_test_text,
                             (uint16_t)sizeof(voice_ascii_test_text)))
        {
            voice_startup_stage = VOICE_TTS_STAGE_STARTUP_TEXT;
            voice_next_tx_time_ms = now + 3000U;
        }
        return;
    }

    if (voice_startup_stage == VOICE_TTS_STAGE_STARTUP_TEXT)
    {
        if (now < voice_next_tx_time_ms)
            return;

        if (VoiceTTSSpeakGBK(voice_startup_text,
                             (uint16_t)sizeof(voice_startup_text)))
        {
            voice_startup_stage = VOICE_TTS_STAGE_READY;
            voice_next_tx_time_ms = now + VOICE_TTS_MIN_INTERVAL_MS;
        }
        return;
    }

    if (now < voice_next_tx_time_ms)
        return;

    RFIDGetState(&rfid_state);
    if ((rfid_state.status & RFID_STATUS_CARD_PRESENT) &&
        rfid_state.card_sequence != 0U &&
        rfid_state.card_sequence != voice_last_card_sequence)
    {
        if (VoiceTTSSpeakGBK(voice_rfid_identified_text,
                             (uint16_t)sizeof(voice_rfid_identified_text)))
        {
            voice_last_card_sequence = rfid_state.card_sequence;
            voice_next_tx_time_ms = now + VOICE_TTS_MIN_INTERVAL_MS;
        }
    }
}

void VoiceTTSGetState(VoiceTTS_State_s *state)
{
    if (state == NULL)
        return;

    *state = voice_state;
}
