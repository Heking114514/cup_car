#include "rfid_reader.h"

#include <string.h>

#include "bsp_usart.h"

#define RFID_STX                  0xAAU
#define RFID_DEVICE_ADDR          0x0000U
#define RFID_CMD_GET_VERSION      0x0000U
#define RFID_CMD_FIND_TYPE_A      0x1000U
#define RFID_CMD_AUTHEN_CLASS     0x1001U
#define RFID_CMD_READ_CLASS       0x1002U
#define RFID_CMD_BEEP             0xFF00U
#define RFID_FIND_IDLE_MODE       0x26U
#define RFID_AUTH_KEY_A           0x60U
#define RFID_TARGET_SECTOR        0U
#define RFID_TARGET_BLOCK         1U
#define RFID_TARGET_BYTE          0U
#define RFID_BLOCK_SIZE           16U

#define RFID_RX_BUFFER_SIZE       96U
#define RFID_TX_BUFFER_SIZE       32U
#define RFID_RESPONSE_TIMEOUT_MS  400U
#define RFID_POLL_INTERVAL_MS     100U

typedef enum
{
    RFID_INIT_VERSION = 0,
    RFID_INIT_BEEP,
    RFID_FIND_CARD,
    RFID_AUTH_CARD,
    RFID_READ_NUMBER,
} RFID_InitStage_e;

typedef struct
{
    uint8_t index;
    uint16_t cmd;
    uint8_t status;
    const uint8_t *data;
    uint16_t data_len;
} RFID_Response_s;

static USARTInstance *rfid_usart;
static RFID_State_s rfid_state;
static RFID_InitStage_e rfid_init_stage;
static uint8_t rfid_tx_index;
static uint8_t rfid_pending;
static uint8_t rfid_pending_index;
static uint16_t rfid_pending_cmd;
static uint32_t rfid_pending_time_ms;
static uint32_t rfid_last_poll_ms;
static uint8_t rfid_rx_frame[RFID_RX_BUFFER_SIZE];
static volatile uint16_t rfid_rx_len;
static volatile uint8_t rfid_rx_ready;
static const uint8_t rfid_default_key[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static uint16_t RFIDFrameTotalLength(const uint8_t *frame)
{
    uint16_t payload_len;
    uint16_t total_len;

    if (frame[0] != RFID_STX)
        return 0U;

    payload_len = ((uint16_t)frame[2] << 8) | frame[3];
    total_len = (uint16_t)(1U + 1U + 2U + payload_len + 1U);
    if (payload_len < 5U || total_len > RFID_RX_BUFFER_SIZE)
        return 0U;

    return total_len;
}

static uint8_t RFIDXor(const uint8_t *data, uint16_t len)
{
    uint8_t checksum = 0U;

    while (len-- > 0U)
        checksum ^= *data++;

    return checksum;
}

static void RFIDRxCallback(USARTInstance *instance)
{
    uint16_t total_len = RFIDFrameTotalLength(instance->recv_buff);

    if (total_len == 0U)
    {
        rfid_state.status |= RFID_STATUS_PROTOCOL_ERROR;
        return;
    }

    memcpy(rfid_rx_frame, instance->recv_buff, total_len);
    rfid_rx_len = total_len;
    rfid_rx_ready = 1U;
}

static uint8_t RFIDParseResponse(const uint8_t *frame, uint16_t frame_len,
                                 RFID_Response_s *response)
{
    uint16_t payload_len;
    uint16_t expected_len;

    if (frame_len < 10U || frame[0] != RFID_STX)
        return 0U;

    payload_len = ((uint16_t)frame[2] << 8) | frame[3];
    expected_len = (uint16_t)(1U + 1U + 2U + payload_len + 1U);
    if (payload_len < 5U || expected_len != frame_len)
        return 0U;

    if (RFIDXor(&frame[1], (uint16_t)(frame_len - 2U)) != frame[frame_len - 1U])
        return 0U;

    response->index = frame[1];
    response->cmd = ((uint16_t)frame[6] << 8) | frame[7];
    response->status = frame[8];
    response->data = &frame[9];
    response->data_len = (uint16_t)(payload_len - 5U);
    return 1U;
}

static uint8_t RFIDSendCommand(uint16_t cmd, const uint8_t *data, uint16_t data_len)
{
    uint8_t frame[RFID_TX_BUFFER_SIZE];
    uint16_t payload_len;
    uint16_t frame_len;
    uint8_t index;
    uint16_t pos;

    if (rfid_usart == NULL || rfid_pending || data_len > (RFID_TX_BUFFER_SIZE - 9U))
        return 0U;

    index = rfid_tx_index++;
    payload_len = (uint16_t)(2U + 2U + data_len);

    frame[0] = RFID_STX;
    frame[1] = index;
    frame[2] = (uint8_t)(payload_len >> 8);
    frame[3] = (uint8_t)payload_len;
    frame[4] = (uint8_t)(RFID_DEVICE_ADDR >> 8);
    frame[5] = (uint8_t)RFID_DEVICE_ADDR;
    frame[6] = (uint8_t)(cmd >> 8);
    frame[7] = (uint8_t)cmd;

    pos = 8U;
    if (data_len > 0U && data != NULL)
    {
        memcpy(&frame[pos], data, data_len);
        pos = (uint16_t)(pos + data_len);
    }
    frame[pos] = RFIDXor(&frame[1], (uint16_t)(pos - 1U));
    pos++;
    frame_len = pos;

    rfid_pending = 1U;
    rfid_pending_index = index;
    rfid_pending_cmd = cmd;
    rfid_pending_time_ms = HAL_GetTick();
    USARTSend(rfid_usart, frame, frame_len, USART_TRANSFER_BLOCKING);
    return 1U;
}

static uint8_t RFIDUidChanged(const uint8_t *uid, uint8_t uid_size)
{
    if (rfid_state.uid_size != uid_size)
        return 1U;

    return (uint8_t)(memcmp(rfid_state.uid, uid, uid_size) != 0);
}

static void RFIDClearCard(uint8_t protocol_status)
{
    rfid_state.protocol_status = protocol_status;
    rfid_state.status &= (uint8_t)~(RFID_STATUS_CARD_PRESENT |
                                    RFID_STATUS_NUMBER_VALID);
    rfid_state.uid_size = 0U;
    rfid_state.number = 0U;
    rfid_state.tag_type = 0U;
    rfid_state.sak = 0U;
    rfid_state.update_time_ms = HAL_GetTick();
}

static void RFIDHandleFindCard(const RFID_Response_s *response)
{
    uint8_t uid_size;
    uint8_t uid_changed;

    if (response->status != 0U || response->data_len < 3U)
    {
        RFIDClearCard(response->status);
        rfid_init_stage = RFID_FIND_CARD;
        return;
    }

    uid_size = (uint8_t)(response->data_len - 3U);
    if (uid_size > RFID_UID_MAX_SIZE)
    {
        rfid_state.status |= RFID_STATUS_PROTOCOL_ERROR;
        RFIDClearCard(response->status);
        rfid_init_stage = RFID_FIND_CARD;
        return;
    }

    uid_changed = (uint8_t)(!(rfid_state.status & RFID_STATUS_CARD_PRESENT) ||
                            RFIDUidChanged(&response->data[3], uid_size));
    if (uid_changed)
    {
        rfid_state.card_sequence++;
        rfid_state.status &= (uint8_t)~RFID_STATUS_NUMBER_VALID;
        rfid_state.number = 0U;
    }

    rfid_state.protocol_status = response->status;
    rfid_state.tag_type = ((uint16_t)response->data[0] << 8) | response->data[1];
    rfid_state.sak = response->data[2];
    rfid_state.uid_size = uid_size;
    memcpy(rfid_state.uid, &response->data[3], uid_size);
    rfid_state.status |= RFID_STATUS_CARD_PRESENT;
    rfid_state.status &= (uint8_t)~RFID_STATUS_TIMEOUT;
    rfid_state.update_time_ms = HAL_GetTick();
    if (uid_changed || !(rfid_state.status & RFID_STATUS_NUMBER_VALID))
        rfid_init_stage = RFID_AUTH_CARD;
    else
        rfid_init_stage = RFID_FIND_CARD;
}

static void RFIDHandleAuth(const RFID_Response_s *response)
{
    if (response->status == 0U)
    {
        rfid_init_stage = RFID_READ_NUMBER;
        return;
    }

    rfid_state.status |= RFID_STATUS_PROTOCOL_ERROR;
    rfid_state.status &= (uint8_t)~RFID_STATUS_NUMBER_VALID;
    rfid_state.number = 0U;
    rfid_init_stage = RFID_FIND_CARD;
}

static uint8_t RFIDDecodeNumber(uint8_t raw_value, uint8_t *number)
{
    if (number == NULL)
        return 0U;

    if (raw_value >= 1U && raw_value <= 12U)
    {
        *number = raw_value;
        return 1U;
    }

    if (raw_value >= (uint8_t)'1' && raw_value <= (uint8_t)'9')
    {
        *number = (uint8_t)(raw_value - (uint8_t)'0');
        return 1U;
    }

    if (raw_value == (uint8_t)'A' || raw_value == (uint8_t)'a')
    {
        *number = 10U;
        return 1U;
    }

    if (raw_value == (uint8_t)'B' || raw_value == (uint8_t)'b')
    {
        *number = 11U;
        return 1U;
    }

    if (raw_value == (uint8_t)'C' || raw_value == (uint8_t)'c')
    {
        *number = 12U;
        return 1U;
    }

    return 0U;
}

static void RFIDHandleReadNumber(const RFID_Response_s *response)
{
    uint8_t number;

    if (response->status != 0U || response->data_len != RFID_BLOCK_SIZE)
    {
        rfid_state.status |= RFID_STATUS_PROTOCOL_ERROR;
        rfid_state.status &= (uint8_t)~RFID_STATUS_NUMBER_VALID;
        rfid_state.number = 0U;
        rfid_init_stage = RFID_FIND_CARD;
        return;
    }

    if (RFIDDecodeNumber(response->data[RFID_TARGET_BYTE], &number))
    {
        rfid_state.number = number;
        rfid_state.status |= RFID_STATUS_NUMBER_VALID;
    }
    else
    {
        rfid_state.number = 0U;
        rfid_state.status &= (uint8_t)~RFID_STATUS_NUMBER_VALID;
        rfid_state.status |= RFID_STATUS_PROTOCOL_ERROR;
    }

    rfid_state.update_time_ms = HAL_GetTick();
    rfid_init_stage = RFID_FIND_CARD;
}

static void RFIDHandleVersion(const RFID_Response_s *response)
{
    uint16_t copy_len;

    if (response->status != 0U)
        return;

    copy_len = response->data_len;
    if (copy_len >= sizeof(rfid_state.version))
        copy_len = (uint16_t)(sizeof(rfid_state.version) - 1U);

    memcpy(rfid_state.version, response->data, copy_len);
    rfid_state.version[copy_len] = '\0';
    rfid_state.status |= RFID_STATUS_READER_ONLINE;
}

static void RFIDHandleResponse(const RFID_Response_s *response)
{
    if (!rfid_pending ||
        response->index != rfid_pending_index ||
        response->cmd != rfid_pending_cmd)
    {
        rfid_state.status |= RFID_STATUS_PROTOCOL_ERROR;
        return;
    }

    rfid_pending = 0U;
    rfid_state.status &= (uint8_t)~(RFID_STATUS_TIMEOUT | RFID_STATUS_PROTOCOL_ERROR);
    rfid_state.protocol_status = response->status;

    switch (response->cmd)
    {
    case RFID_CMD_GET_VERSION:
        RFIDHandleVersion(response);
        rfid_init_stage = RFID_INIT_BEEP;
        break;
    case RFID_CMD_BEEP:
        rfid_init_stage = RFID_FIND_CARD;
        break;
    case RFID_CMD_FIND_TYPE_A:
        RFIDHandleFindCard(response);
        break;
    case RFID_CMD_AUTHEN_CLASS:
        RFIDHandleAuth(response);
        break;
    case RFID_CMD_READ_CLASS:
        RFIDHandleReadNumber(response);
        break;
    default:
        break;
    }
}

static void RFIDProcessRx(void)
{
    uint8_t frame[RFID_RX_BUFFER_SIZE];
    uint16_t frame_len;
    RFID_Response_s response;
    uint32_t primask;

    if (!rfid_rx_ready)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    frame_len = rfid_rx_len;
    if (frame_len <= RFID_RX_BUFFER_SIZE)
        memcpy(frame, rfid_rx_frame, frame_len);
    rfid_rx_ready = 0U;
    if (primask == 0U)
        __enable_irq();

    if (frame_len > RFID_RX_BUFFER_SIZE ||
        !RFIDParseResponse(frame, frame_len, &response))
    {
        rfid_state.status |= RFID_STATUS_PROTOCOL_ERROR;
        return;
    }

    RFIDHandleResponse(&response);
}

static void RFIDHandleTimeout(uint32_t now)
{
    if (!rfid_pending || now - rfid_pending_time_ms <= RFID_RESPONSE_TIMEOUT_MS)
        return;

    rfid_pending = 0U;
    rfid_state.status |= RFID_STATUS_TIMEOUT;
    if (rfid_pending_cmd == RFID_CMD_FIND_TYPE_A)
        RFIDClearCard(0x20U);
}

void RFIDInit(UART_HandleTypeDef *uart_handle)
{
    USART_Init_Config_s conf;

    memset(&rfid_state, 0, sizeof(rfid_state));
    rfid_state.status = RFID_STATUS_READY;
    rfid_init_stage = RFID_INIT_VERSION;
    rfid_tx_index = 0U;
    rfid_pending = 0U;
    rfid_rx_ready = 0U;
    rfid_last_poll_ms = 0U;

    conf.module_callback = RFIDRxCallback;
    conf.recv_buff_size = RFID_RX_BUFFER_SIZE;
    conf.usart_handle = uart_handle;
    rfid_usart = USARTRegister(&conf);
}

void RFIDTask(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t find_mode = RFID_FIND_IDLE_MODE;
    uint8_t beep_time = 10U;
    uint8_t read_block = RFID_TARGET_BLOCK;
    uint8_t auth_payload[8];

    if (rfid_usart == NULL)
        return;

    RFIDProcessRx();
    RFIDHandleTimeout(now);

    if (rfid_pending || now - rfid_last_poll_ms < RFID_POLL_INTERVAL_MS)
        return;

    switch (rfid_init_stage)
    {
    case RFID_INIT_VERSION:
        if (RFIDSendCommand(RFID_CMD_GET_VERSION, NULL, 0U))
            rfid_last_poll_ms = now;
        break;
    case RFID_INIT_BEEP:
        if (RFIDSendCommand(RFID_CMD_BEEP, &beep_time, 1U))
            rfid_last_poll_ms = now;
        break;
    case RFID_AUTH_CARD:
        auth_payload[0] = RFID_AUTH_KEY_A;
        auth_payload[1] = RFID_TARGET_SECTOR;
        memcpy(&auth_payload[2], rfid_default_key, sizeof(rfid_default_key));
        if (RFIDSendCommand(RFID_CMD_AUTHEN_CLASS, auth_payload,
                            (uint16_t)sizeof(auth_payload)))
            rfid_last_poll_ms = now;
        break;
    case RFID_READ_NUMBER:
        if (RFIDSendCommand(RFID_CMD_READ_CLASS, &read_block, 1U))
            rfid_last_poll_ms = now;
        break;
    case RFID_FIND_CARD:
    default:
        if (RFIDSendCommand(RFID_CMD_FIND_TYPE_A, &find_mode, 1U))
            rfid_last_poll_ms = now;
        break;
    }
}

void RFIDGetState(RFID_State_s *state)
{
    uint32_t primask;

    if (state == NULL)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(state, &rfid_state, sizeof(*state));
    if (primask == 0U)
        __enable_irq();
}
