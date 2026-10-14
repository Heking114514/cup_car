#include "voice_tts.h"

#include <stddef.h>
#include <string.h>

#include "bsp_dwt.h"
#include "rfid_reader.h"
#include "robot_def.h"

#define VOICE_TTS_STARTUP_DELAY_MS       3000U
#define VOICE_TTS_MIN_INTERVAL_MS        1200U
#define VOICE_TTS_TX_BUFFER_SIZE         VOICE_TTS_HOST_MAX_BYTES
#define VOICE_TTS_RFID_NUMBER_MAX_BYTES  16U
#define VOICE_TTS_SAME_UID_RESET_MS      1000U
#define VOICE_TTS_BYTE_DELAY_S           0.002f
#define VOICE_TTS_END_DELAY_S            0.004f

typedef enum
{
    VOICE_TTS_STAGE_STARTUP_TEXT = 0,
    VOICE_TTS_STAGE_READY,
} VoiceTTS_StartupStage_e;

static UART_HandleTypeDef *voice_uart;
static VoiceTTS_State_s voice_state;
static VoiceTTS_StartupStage_e voice_startup_stage;
static uint32_t voice_next_tx_time_ms;
#if !ROBOT_HOST_ONLY_SPEECH
static uint32_t voice_last_card_sequence;
static uint32_t voice_card_absent_since_ms;
static uint8_t voice_card_absent_seen;
static uint8_t voice_last_uid_size;
static uint8_t voice_last_uid[RFID_UID_MAX_SIZE];
#endif
static VoiceTTS_HostState_s voice_host_state;
static uint32_t voice_host_done_ms;
static uint32_t voice_host_estimate_ms;
static uint8_t voice_host_pending;
static uint16_t voice_host_pending_len;
/* Task-owned cursor/deadline; payload is immutable while the queue is occupied. */
static uint16_t voice_host_tx_index;
static uint32_t voice_host_tx_due_ms;
static uint8_t voice_host_pending_data[VOICE_TTS_TX_BUFFER_SIZE];

#if !ROBOT_HOST_ONLY_SPEECH
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

#endif

static uint32_t VoiceLock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

static void VoiceUnlock(uint32_t mask)
{
    if (mask == 0U) __enable_irq();
}

static uint8_t VoiceDue(uint32_t now, uint32_t deadline)
{
    return (uint8_t)((int32_t)(now - deadline) >= 0);
}

static uint8_t VoiceTTSWrite(const uint8_t *data, uint16_t len)
{
    uint16_t index;

    if (voice_uart == NULL || data == NULL || len == 0U ||
        len > VOICE_TTS_TX_BUFFER_SIZE || voice_uart->Instance == NULL)
        return 0U;

    for (index = 0U; index < len; index++)
    {
        voice_uart->Instance->TDR = data[index];
        DWT_Delay(VOICE_TTS_BYTE_DELAY_S);
    }
    DWT_Delay(VOICE_TTS_END_DELAY_S);

    uint32_t mask = VoiceLock();
    voice_state.last_tx_time_ms = HAL_GetTick();
    voice_next_tx_time_ms = voice_state.last_tx_time_ms + VOICE_TTS_MIN_INTERVAL_MS;
    voice_state.tx_count++;
    voice_state.status |= VOICE_TTS_STATUS_TX_OK;
    voice_state.status &= (uint8_t)~VOICE_TTS_STATUS_TX_ERROR;
    VoiceUnlock(mask);
    return 1U;
}

void VoiceTTSInit(UART_HandleTypeDef *uart_handle)
{
    voice_uart = uart_handle;
    memset(&voice_state, 0, sizeof(voice_state));
    voice_state.status = VOICE_TTS_STATUS_READY;
#if ROBOT_HOST_ONLY_SPEECH
    voice_startup_stage = VOICE_TTS_STAGE_READY;
    voice_next_tx_time_ms = HAL_GetTick();
#else
    voice_startup_stage = VOICE_TTS_STAGE_STARTUP_TEXT;
    voice_next_tx_time_ms = HAL_GetTick() + VOICE_TTS_STARTUP_DELAY_MS;
    voice_last_card_sequence = 0U;
    voice_card_absent_since_ms = 0U;
    voice_card_absent_seen = 0U;
    voice_last_uid_size = 0U;
#endif
    memset(&voice_host_state, 0, sizeof(voice_host_state));
    voice_host_pending = 0U;
    voice_host_pending_len = 0U;
    voice_host_tx_index = 0U;
    voice_host_tx_due_ms = 0U;
}

uint8_t VoiceTTSSpeakGBK(const uint8_t *data, uint16_t len)
{
    return VoiceTTSWrite(data, len);
}

/* ASCII counts as one character; each structurally valid GBK pair counts as one.
 * Reject controls/NUL so line framing and module commands cannot be embedded. */
static uint16_t VoiceCharacters(const uint8_t *data, uint16_t len)
{
    uint16_t count = 0U;
    if (data == NULL || len == 0U || len > VOICE_TTS_HOST_MAX_BYTES) return 0U;
    for (uint16_t i = 0U; i < len; i++, count++) {
        if (data[i] >= 0x81U && data[i] <= 0xFEU) {
            if (++i >= len || data[i] < 0x40U || data[i] > 0xFEU || data[i] == 0x7FU)
                return 0U;
        } else if (data[i] < 0x20U || data[i] > 0x7EU ||
                   data[i] == '<' || data[i] == '>') return 0U;
    }
    return count;
}

uint8_t VoiceTTSIsHostReady(void)
{
    uint32_t mask = VoiceLock();
    uint8_t ready = voice_host_pending == 0U &&
        voice_host_state.status != VOICE_HOST_PLAYING &&
        voice_startup_stage == VOICE_TTS_STAGE_READY &&
        VoiceDue(HAL_GetTick(), voice_next_tx_time_ms);
    VoiceUnlock(mask);
    return ready;
}

static uint8_t VoiceQueue(uint32_t id, const uint8_t *data, uint16_t len, uint8_t legacy)
{
    uint16_t characters = VoiceCharacters(data, len);
    uint32_t mask = VoiceLock();
    /* Latest accepted nonzero ID is idempotent even if a retry has changed text. */
    if (!legacy && id != 0U && voice_host_state.status != VOICE_HOST_NONE &&
        id == voice_host_state.id) {
        if (voice_host_state.rejected_id == id)
            voice_host_state.rejected_status = VOICE_HOST_NONE;
        VoiceUnlock(mask);
        return 1U;
    }
    VoiceTTS_HostStatus_e rejection = VOICE_HOST_NONE;
    if ((!legacy && id == 0U) || characters == 0U) rejection = VOICE_HOST_INVALID;
    else if (voice_host_pending || voice_host_state.status == VOICE_HOST_PLAYING ||
             voice_startup_stage != VOICE_TTS_STAGE_READY ||
             !VoiceDue(HAL_GetTick(), voice_next_tx_time_ms)) rejection = VOICE_HOST_BUSY;
    if (rejection != VOICE_HOST_NONE) {
        voice_host_state.rejected_id = id;
        voice_host_state.rejected_status = rejection;
        VoiceUnlock(mask);
        return 0U;
    }
    memcpy(voice_host_pending_data, data, len);
    voice_host_pending_len = len;
    voice_host_estimate_ms = (uint32_t)characters * 800U + 1500U;
    if (voice_host_estimate_ms < VOICE_TTS_MIN_INTERVAL_MS)
        voice_host_estimate_ms = VOICE_TTS_MIN_INTERVAL_MS;
    voice_host_pending = 1U;
    voice_host_state.id = id;
    voice_host_state.status = VOICE_HOST_QUEUED;
    if (voice_host_state.rejected_id == id)
        voice_host_state.rejected_status = VOICE_HOST_NONE;
    voice_state.status |= VOICE_TTS_STATUS_BUSY;
    VoiceUnlock(mask);
    return 1U;
}

uint8_t VoiceTTSQueueHostRequest(uint32_t id, const uint8_t *data, uint16_t len)
{
    return VoiceQueue(id, data, len, 0U);
}

uint8_t VoiceTTSQueueHostGBK(const uint8_t *data, uint16_t len)
{
    return VoiceQueue(0U, data, len, 1U);
}

uint8_t VoiceTTSParseHostLine(const uint8_t *line, uint16_t len)
{
    if (len >= 4U && memcmp(line, "TTS,", 4U) == 0) {
        (void)VoiceTTSQueueHostGBK(line + 4U, len - 4U);
        return 1U;
    }
    if (len < 5U || memcmp(line, "TTS2,", 5U) != 0) return 0U;
    uint32_t id = 0U;
    uint16_t pos = 5U;
    uint8_t valid = 1U, digits = 0U;
    for (; pos < len && line[pos] != ','; pos++) {
        uint8_t c = line[pos];
        if (c < '0' || c > '9' || id > (UINT32_MAX - (c - '0')) / 10U) {
            valid = 0U;
            continue;
        }
        if (valid) id = id * 10U + (c - '0');
        digits = 1U;
    }
    if (!valid || !digits || pos == len) {
        uint32_t mask = VoiceLock();
        voice_host_state.rejected_id = 0U; /* unparseable ID */
        voice_host_state.rejected_status = VOICE_HOST_INVALID;
        VoiceUnlock(mask);
    } else (void)VoiceTTSQueueHostRequest(id, line + pos + 1U, len - pos - 1U);
    return 1U;
}

void VoiceTTSGetHostState(VoiceTTS_HostState_s *state)
{
    if (state == NULL) return;
    uint32_t mask = VoiceLock();
    *state = voice_host_state;
    VoiceUnlock(mask);
}

const char *VoiceTTSHostStatusName(VoiceTTS_HostStatus_e status)
{
    static const char *const names[] = {
        "", "QUEUED", "PLAYING", "ESTIMATED_DONE", "ERROR", "BUSY", "INVALID"
    };
    return status <= VOICE_HOST_INVALID ? names[status] : "ERROR";
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
    uint32_t mask = VoiceLock();
    if (voice_host_state.status == VOICE_HOST_PLAYING && !voice_host_pending &&
        VoiceDue(now, voice_host_done_ms)) {
        voice_host_state.status = VOICE_HOST_ESTIMATED_DONE;
        voice_state.status &= (uint8_t)~VOICE_TTS_STATUS_BUSY;
    }
    if (voice_host_pending && voice_host_state.status == VOICE_HOST_QUEUED) {
        voice_host_state.status = VOICE_HOST_PLAYING;
        voice_host_tx_index = 0U;
        voice_host_tx_due_ms = now;
    }
    uint8_t transmitting = voice_host_pending;
    VoiceUnlock(mask);
    if (transmitting) {
        /* One byte per invocation, never a delay or catch-up loop. USB and
         * chassis control can run between bytes, including on a late tick. */
        if (!VoiceDue(now, voice_host_tx_due_ms)) return;
        if (voice_uart == NULL || voice_uart->Instance == NULL) {
            mask = VoiceLock();
            voice_host_pending = 0U;
            voice_host_pending_len = 0U;
            voice_host_state.status = VOICE_HOST_ERROR;
            voice_state.status |= VOICE_TTS_STATUS_TX_ERROR;
            voice_state.status &= (uint8_t)~VOICE_TTS_STATUS_BUSY;
            voice_state.tx_error_count++;
            VoiceUnlock(mask);
            return;
        }
        if (voice_host_tx_index < voice_host_pending_len) {
            voice_uart->Instance->TDR = voice_host_pending_data[voice_host_tx_index++];
            /* Retain the raw API's 2ms per byte and additional 4ms tail. */
            voice_host_tx_due_ms = now +
                (voice_host_tx_index == voice_host_pending_len ? 6U : 2U);
            return;
        }
        mask = VoiceLock();
        voice_host_pending = 0U;
        voice_host_pending_len = 0U;
        voice_state.last_tx_time_ms = now;
        voice_state.tx_count++;
        voice_state.status |= VOICE_TTS_STATUS_TX_OK;
        voice_state.status &= (uint8_t)~VOICE_TTS_STATUS_TX_ERROR;
        voice_host_done_ms = now + voice_host_estimate_ms;
        voice_next_tx_time_ms = voice_host_done_ms;
        VoiceUnlock(mask);
        return;
    }
    mask = VoiceLock();
    uint8_t occupied = voice_host_state.status == VOICE_HOST_PLAYING;
    VoiceUnlock(mask);
    if (occupied) return;
#if ROBOT_HOST_ONLY_SPEECH
    return;
#else
    RFID_State_s rfid_state;
    uint8_t rfid_number_text[VOICE_TTS_RFID_NUMBER_MAX_BYTES];
    uint16_t rfid_number_text_len;
    if (voice_uart == NULL) return;

    if (voice_startup_stage == VOICE_TTS_STAGE_STARTUP_TEXT)
    {
        if (!VoiceDue(now, voice_next_tx_time_ms))
            return;

        (void)VoiceTTSSpeakGBK(voice_startup_text,
                               (uint16_t)sizeof(voice_startup_text));
        voice_startup_stage = VOICE_TTS_STAGE_READY;
        return;
    }

    if (!VoiceDue(now, voice_next_tx_time_ms))
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
    }
#endif
}

void VoiceTTSGetState(VoiceTTS_State_s *state)
{
    if (state == NULL)
        return;

    uint32_t mask = VoiceLock();
    *state = voice_state;
    VoiceUnlock(mask);
}
