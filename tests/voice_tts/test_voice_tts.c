/* Compile against the production implementation, without STM32 hardware. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "voice_tts.h"
#include "rfid_reader.h"

static uint32_t tick, irq_mask, delay_calls, rfid_calls;
static USART_TypeDef uart_regs;
static UART_HandleTypeDef uart = { &uart_regs };
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __enable_irq(void) { irq_mask = 0U; }
void RFIDGetState(RFID_State_s *s) {
    rfid_calls++;
    memset(s, 0, sizeof(*s));
    s->status = RFID_STATUS_CARD_PRESENT | RFID_STATUS_NUMBER_VALID;
    s->number = 1U; s->uid_size = 1U; s->uid[0] = 3U;
    s->card_sequence = rfid_calls;
}
static VoiceTTS_HostState_s host(void) {
    VoiceTTS_HostState_s s; VoiceTTSGetHostState(&s); return s;
}
static VoiceTTS_State_s stats(void) {
    VoiceTTS_State_s s; VoiceTTSGetState(&s); return s;
}
void DWT_Delay(float seconds) {
    (void)seconds;
    assert(irq_mask == 0U);
    delay_calls++;
    assert(!"host UART must never call a blocking delay");
}
/* Run simulated 1ms control ticks until the last byte plus tail is complete. */
static void finish_send(void) {
    uint32_t sent = stats().tx_count;
    for (unsigned i = 0U; i < 1000U && stats().tx_count == sent; i++) {
        uint32_t before = tick;
        VoiceTTSTask();
        assert(tick == before && delay_calls == 0U);
        if (stats().tx_count != sent) return;
        tick++;
    }
    assert(!"incremental UART failed to finish");
}
static void reset(uint32_t now) {
    tick = now; irq_mask = delay_calls = rfid_calls = 0U;
    VoiceTTSInit(&uart);
}
static void parse(const char *s) {
    assert(VoiceTTSParseHostLine((const uint8_t *)s, (uint16_t)strlen(s)));
}
int main(void) {
    reset(0U);
    for (tick = 0U; tick < 10000U; tick += 20U) VoiceTTSTask();
    assert(delay_calls == 0U && rfid_calls == 0U && stats().tx_count == 0U);
    assert(VoiceTTSIsHostReady());

    reset(0U);
    parse("TTS2,1,A");
    assert(host().id == 1U && host().status == VOICE_HOST_QUEUED);
    assert(stats().status & VOICE_TTS_STATUS_BUSY);
    parse("TTS2,2,B");
    assert(host().id == 1U && host().rejected_id == 2U);
    assert(host().rejected_status == VOICE_HOST_BUSY);
    VoiceTTSTask();
    assert(tick == 0U && delay_calls == 0U && uart_regs.TDR == 'A');
    assert(host().status == VOICE_HOST_PLAYING && stats().tx_count == 0U);
    parse("TTS2,1,retry"); parse("TTS2,2,other");
    assert(host().rejected_id == 2U && host().rejected_status == VOICE_HOST_BUSY);
    tick = 5U; VoiceTTSTask(); assert(stats().tx_count == 0U);
    tick = 6U; VoiceTTSTask();
    assert(stats().tx_count == 1U && host().status == VOICE_HOST_PLAYING);
    tick += 2299U; VoiceTTSTask();
    assert(!VoiceTTSIsHostReady() && host().status == VOICE_HOST_PLAYING);
    tick++; VoiceTTSTask();
    assert(VoiceTTSIsHostReady() && host().status == VOICE_HOST_ESTIMATED_DONE);
    assert(!(stats().status & VOICE_TTS_STATUS_BUSY));
    parse("TTS2,1,different"); VoiceTTSTask();
    assert(stats().tx_count == 1U && host().status == VOICE_HOST_ESTIMATED_DONE);
    for (unsigned i = 0U; i < 10U; i++) {
        assert(host().rejected_id == 2U && host().rejected_status == VOICE_HOST_BUSY);
    }
    parse("TTS2,2,B");
    assert(host().status == VOICE_HOST_QUEUED && host().rejected_status == VOICE_HOST_NONE);

    /* No same-tick catch-up, minimum byte spacing, late control ticks. */
    reset(0U); parse("TTS2,4,AB");
    VoiceTTSTask(); assert(uart_regs.TDR == 'A' && tick == 0U);
    VoiceTTSTask(); assert(uart_regs.TDR == 'A');
    tick = 1U; VoiceTTSTask(); assert(uart_regs.TDR == 'A');
    tick = 2U; VoiceTTSTask(); assert(uart_regs.TDR == 'B');
    tick = 7U; VoiceTTSTask(); assert(stats().tx_count == 0U);
    tick = 8U; VoiceTTSTask(); assert(stats().tx_count == 1U);
    reset(0U); parse("TTS2,5,ABC");
    VoiceTTSTask(); tick = 50U; VoiceTTSTask(); assert(uart_regs.TDR == 'B');
    VoiceTTSTask(); assert(uart_regs.TDR == 'B');
    tick = 52U; VoiceTTSTask(); assert(uart_regs.TDR == 'C');
    tick = 58U; VoiceTTSTask(); assert(stats().tx_count == 1U);
    /* Hardware pointer invalidated during a multi-byte send is an error. */
    reset(0U); parse("TTS2,6,AB"); VoiceTTSTask();
    uart.Instance = NULL; tick = 2U; VoiceTTSTask();
    assert(host().status == VOICE_HOST_ERROR && stats().tx_error_count == 1U);
    uart.Instance = &uart_regs;

    reset(0U);
    const uint8_t gbk[] = {0xC4U, 0xE3U, 0xBAU, 0xC3U, 'A'};
    assert(VoiceTTSQueueHostRequest(7U, gbk, sizeof(gbk)));
    finish_send(); assert(tick == 14U);
    tick += 3899U; VoiceTTSTask(); assert(host().status == VOICE_HOST_PLAYING);
    tick++; VoiceTTSTask(); assert(host().status == VOICE_HOST_ESTIMATED_DONE);

    reset(0U);
    uint8_t boundary[129]; memset(boundary, 'x', sizeof(boundary));
    assert(!VoiceTTSQueueHostRequest(8U, boundary, 129U));
    assert(host().rejected_id == 8U && host().rejected_status == VOICE_HOST_INVALID);
    assert(VoiceTTSQueueHostRequest(UINT32_MAX, boundary, 128U));
    finish_send(); assert(tick == 260U && stats().tx_count == 1U);
    tick += 103899U; VoiceTTSTask(); assert(host().status == VOICE_HOST_PLAYING);
    tick++; VoiceTTSTask(); assert(host().status == VOICE_HOST_ESTIMATED_DONE);
    reset(0U);
    char line[160];
    int prefix = snprintf(line, sizeof(line), "TTS2,%lu,", (unsigned long)UINT32_MAX);
    memset(line + prefix, 'x', 128U); line[prefix + 128] = '\0';
    parse(line); assert(host().id == UINT32_MAX && host().status == VOICE_HOST_QUEUED);

    reset(0U);
    const char *bad[] = {"TTS2,0,A", "TTS2,4294967296,A", "TTS2,-1,A",
                        "TTS2,,A", "TTS2,3,", "TTS2,3x,A", "TTS2,3"};
    for (unsigned i = 0U; i < sizeof(bad)/sizeof(bad[0]); i++) {
        parse(bad[i]); assert(host().status == VOICE_HOST_NONE);
        assert(host().rejected_status == VOICE_HOST_INVALID);
    }
    const uint8_t malformed[] = {0x81U, 0x7FU};
    assert(!VoiceTTSQueueHostRequest(3U, malformed, sizeof(malformed)));
    assert(!VoiceTTSQueueHostRequest(3U, gbk, 1U));
    assert(!VoiceTTSQueueHostRequest(3U, (const uint8_t *)"<S>3", 4U));
    assert(!VoiceTTSQueueHostRequest(3U, (const uint8_t *)"a\rb", 3U));
    assert(!VoiceTTSQueueHostRequest(3U, (const uint8_t *)"a\0b", 3U));
    assert(!VoiceTTSParseHostLine((const uint8_t *)"0,0", 3U));

    reset(UINT32_MAX - 3U);
    parse("TTS2,9,A"); finish_send(); assert(tick == 2U);
    tick += 2299U; VoiceTTSTask(); assert(host().status == VOICE_HOST_PLAYING);
    tick++; VoiceTTSTask(); assert(host().status == VOICE_HOST_ESTIMATED_DONE);
    reset(UINT32_MAX - 100U);
    parse("TTS2,10,A"); finish_send();
    tick += 2299U; VoiceTTSTask(); assert(host().status == VOICE_HOST_PLAYING);
    tick++; VoiceTTSTask(); assert(host().status == VOICE_HOST_ESTIMATED_DONE);

    reset(0U); VoiceTTSInit(NULL);
    parse("TTS2,11,A"); VoiceTTSTask();
    assert(host().status == VOICE_HOST_ERROR && stats().tx_error_count == 1U);
    parse("TTS2,11,A"); VoiceTTSTask(); assert(stats().tx_error_count == 1U);

    reset(0U); irq_mask = 1U;
    assert(VoiceTTSQueueHostRequest(12U, gbk, sizeof(gbk)));
    assert(irq_mask == 1U); (void)host(); assert(irq_mask == 1U);
    irq_mask = 0U;
    reset(0U); parse("TTS,legacy"); finish_send();
    assert(host().id == 0U && stats().tx_count == 1U);
    tick += 6300U; VoiceTTSTask(); parse("TTS,legacy"); finish_send();
    assert(stats().tx_count == 2U);
    assert(strcmp(VoiceTTSHostStatusName(VOICE_HOST_ESTIMATED_DONE), "ESTIMATED_DONE") == 0);
    puts("voice_tts: all host protocol tests passed");
    return 0;
}
