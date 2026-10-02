#include "voice_tts.h"

#include <stddef.h>
#include <string.h>

#include "bsp_dwt.h"
#include "rfid_reader.h"

#define VOICE_TTS_STARTUP_DELAY_MS      3000U
#define VOICE_TTS_MIN_INTERVAL_MS       1200U
#define VOICE_TTS_TX_BASE_TIMEOUT_MS    300U
#define VOICE_TTS_TX_PER_BYTE_TIMEOUT_MS 20U
#define VOICE_TTS_TX_BUFFER_SIZE        96U
#define VOICE_TTS_RFID_NUMBER_MAX_BYTES 16U
#define VOICE_TTS_SAME_UID_RESET_MS     1000U
#define VOICE_TTS_BYTE_DELAY_S          0.002f
#define VOICE_TTS_END_DELAY_S           0.004f

typedef enum
{
    VOICE_TTS_STAGE_STARTUP_TEXT = 0,
    VOICE_TTS_STAGE_READY,
} VoiceTTS_StartupStage_e;

static UART_HandleTypeDef *voice_uart;
static VoiceTTS_State_s voice_state;
static VoiceTTS_StartupStage_e voice_startup_stage;
static uint32_t voice_next_tx_time_ms;
static uint32_t voice_last_card_sequence;
static uint32_t voice_card_absent_since_ms;
static uint8_t voice_card_absent_seen;
static uint8_t voice_last_uid_size;
static uint8_t voice_last_uid[RFID_UID_MAX_SIZE];

static const uint8_t voice_startup_text[] = {
    0xC4, 0xE3, 0xBA, 0xC3
};

static const uint8_t voice_rfid_prefix[] = {
    0xB5, 0xBD, 0xB4, 0xEF
};

static const uint8_t voice_rfid_suffix[] = {
    0xBA, 0xC5, 0xD1, 0xB2, 0xC2, 0xDF, 0xB5, 0xE3
};

static const uint8_t voice_number_1[] = {0xD2, 0xBB};
static const uint8_t voice_number_2[] = {0xB6, 0xFE};
static const uint8_t voice_number_3[] = {0xC8, 0xFD};
static const uint8_t voice_number_4[] = {0xCB, 0xC4};
static const uint8_t voice_number_5[] = {0xCE, 0xE5};
static const uint8_t voice_number_6[] = {0xC1, 0xF9};
static const uint8_t voice_number_7[] = {0xC6, 0xDF};
static const uint8_t voice_number_8[] = {0xB0, 0xCB};
static const uint8_t voice_number_9[] = {0xBE, 0xC5};
static const uint8_t voice_number_10[] = {0xCA, 0xAE};
static const uint8_t voice_number_11[] = {0xCA, 0xAE, 0xD2, 0xBB};
static const uint8_t voice_number_12[] = {0xCA, 0xAE, 0xB6, 0xFE};

static const uint8_t *const voice_number_texts[] = {
    voice_number_1,
    voice_number_2,
    voice_number_3,
    voice_number_4,
    voice_number_5,
    voice_number_6,
    voice_number_7,
    voice_number_8,
    voice_number_9,
    voice_number_10,
    voice_number_11,
    voice_number_12,
};

static const uint8_t voice_number_text_lens[] = {
    sizeof(voice_number_1),
    sizeof(voice_number_2),
    sizeof(voice_number_3),
    sizeof(voice_number_4),
    sizeof(voice_number_5),
    sizeof(voice_number_6),
    sizeof(voice_number_7),
    sizeof(voice_number_8),
    sizeof(voice_number_9),
    sizeof(voice_number_10),
    sizeof(voice_number_11),
    sizeof(voice_number_12),
};

static uint32_t VoiceTTSTxTimeout(uint16_t len)
{
    return VOICE_TTS_TX_BASE_TIMEOUT_MS +
           (uint32_t)len * VOICE_TTS_TX_PER_BYTE_TIMEOUT_MS;
}

static uint16_t VoiceTTSBuildRFIDNumberText(uint8_t number, uint8_t *buffer,
                                            uint16_t capacity)
{
    const uint8_t *text;
    uint16_t len;
    uint16_t total_len;

    if (buffer == NULL || capacity == 0U || number < 1U || number > 12U)
        return 0U;

    text = voice_number_texts[number - 1U];
    len = voice_number_text_lens[number - 1U];
    total_len = (uint16_t)(sizeof(voice_rfid_prefix) + len +
                           sizeof(voice_rfid_suffix));
    if (total_len > capacity)
        return 0U;

    memcpy(buffer, voice_rfid_prefix, sizeof(voice_rfid_prefix));
    memcpy(&buffer[sizeof(voice_rfid_prefix)], text, len);
    memcpy(&buffer[sizeof(voice_rfid_prefix) + len], voice_rfid_suffix,
           sizeof(voice_rfid_suffix));
    return total_len;
}

static uint8_t VoiceTTSIsNewRFIDUid(const RFID_State_s *rfid_state)
{
    if (rfid_state->uid_size == 0U)
        return 0U;

    if (voice_last_uid_size != rfid_state->uid_size)
        return 1U;

    return (uint8_t)(memcmp(voice_last_uid, rfid_state->uid,
                            rfid_state->uid_size) != 0);
}

static void VoiceTTSRememberRFIDUid(const RFID_State_s *rfid_state)
{
    if (rfid_state->uid_size == 0U || rfid_state->uid_size > RFID_UID_MAX_SIZE)
        return;

    memcpy(voice_last_uid, rfid_state->uid, rfid_state->uid_size);
    voice_last_uid_size = rfid_state->uid_size;
}

static uint8_t VoiceTTSWrite(const uint8_t *data, uint16_t len)
{
    uint16_t index;

    if (voice_uart == NULL || data == NULL || len == 0U ||
        len > VOICE_TTS_TX_BUFFER_SIZE || voice_uart->Instance == NULL)
        return 0U;

    (void)VoiceTTSTxTimeout(len);
    for (index = 0U; index < len; index++)
    {
        voice_uart->Instance->TDR = data[index];
        DWT_Delay(VOICE_TTS_BYTE_DELAY_S);
    }
    DWT_Delay(VOICE_TTS_END_DELAY_S);

    voice_state.last_tx_time_ms = HAL_GetTick();
    voice_state.tx_count++;
    voice_state.status |= VOICE_TTS_STATUS_TX_OK;
    voice_state.status &= (uint8_t)~VOICE_TTS_STATUS_TX_ERROR;
    return 1U;
}

void VoiceTTSInit(UART_HandleTypeDef *uart_handle)
{
    voice_uart = uart_handle;
    memset(&voice_state, 0, sizeof(voice_state));
    voice_state.status = VOICE_TTS_STATUS_READY;
    voice_startup_stage = VOICE_TTS_STAGE_STARTUP_TEXT;
    voice_next_tx_time_ms = VOICE_TTS_STARTUP_DELAY_MS;
    voice_last_card_sequence = 0U;
    voice_card_absent_since_ms = 0U;
    voice_card_absent_seen = 0U;
    voice_last_uid_size = 0U;
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
    uint8_t rfid_number_text[VOICE_TTS_RFID_NUMBER_MAX_BYTES];
    uint16_t rfid_number_text_len;

    if (voice_uart == NULL)
        return;

    if (voice_startup_stage == VOICE_TTS_STAGE_STARTUP_TEXT)
    {
        if (now < voice_next_tx_time_ms)
            return;

        (void)VoiceTTSSpeakGBK(voice_startup_text,
                               (uint16_t)sizeof(voice_startup_text));
        voice_startup_stage = VOICE_TTS_STAGE_READY;
        voice_next_tx_time_ms = now + VOICE_TTS_MIN_INTERVAL_MS;
        return;
    }

    if (now < voice_next_tx_time_ms)
        return;

    RFIDGetState(&rfid_state);
    if (!(rfid_state.status & RFID_STATUS_CARD_PRESENT))
    {
        if (!voice_card_absent_seen)
        {
            voice_card_absent_seen = 1U;
            voice_card_absent_since_ms = now;
        }
        else if (now - voice_card_absent_since_ms > VOICE_TTS_SAME_UID_RESET_MS)
        {
            voice_last_uid_size = 0U;
        }
        return;
    }

    voice_card_absent_seen = 0U;
    if ((rfid_state.status & RFID_STATUS_NUMBER_VALID) &&
        rfid_state.card_sequence != 0U &&
        rfid_state.card_sequence != voice_last_card_sequence &&
        VoiceTTSIsNewRFIDUid(&rfid_state))
    {
        rfid_number_text_len = VoiceTTSBuildRFIDNumberText(rfid_state.number,
                                                           rfid_number_text,
                                                           (uint16_t)sizeof(rfid_number_text));
        (void)VoiceTTSSpeakGBK(rfid_number_text, rfid_number_text_len);
        voice_last_card_sequence = rfid_state.card_sequence;
        VoiceTTSRememberRFIDUid(&rfid_state);
        voice_next_tx_time_ms = now + VOICE_TTS_MIN_INTERVAL_MS;
    }
}

void VoiceTTSGetState(VoiceTTS_State_s *state)
{
    if (state == NULL)
        return;

    *state = voice_state;
}
