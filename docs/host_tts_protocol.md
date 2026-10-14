# MCU host TTS protocol

The USB command line accepts the legacy command `TTS,<GBK payload>\r\n` and the reliable command:

```text
TTS2,<uint32 nonzero id>,<GBK payload>\r\n
```

The TTS2 payload is 1 to 128 GBK bytes. ASCII bytes count as one character; a valid two byte GBK sequence counts as one character. Control bytes, NUL, ASCII `<` and `>` (the module's volume/speed/effect command delimiters), malformed GBK, zero IDs, overflowed IDs, and empty payloads are rejected. The receive line buffer is 192 bytes so a complete maximum command can be framed safely.

The MCU repeats status records with every control telemetry report (normally 100 ms):

```text
TTS,<id>,QUEUED|PLAYING|ESTIMATED_DONE|ERROR\r\n
```

The latest rejected request is also repeated while its rejection metadata remains current:

```text
TTS,<rejected_id>,BUSY|INVALID\r\n
```

`QUEUED` means accepted by the MCU queue. `PLAYING` covers transmission and the subsequent estimated occupation. The voice module has no documented receive or completion feedback in this firmware, so `ESTIMATED_DONE` is a timer estimate and never claims confirmed hardware completion. The estimate is 800 ms per GBK character plus 1500 ms after the UART send, with a minimum occupation of 1200 ms.

Host UART transmission is incremental: each task invocation writes at most one byte, with at least 2 ms between bytes and 2 ms plus the existing 4 ms tail after the final byte. It never calls a blocking delay or catches up multiple bytes on a late task tick. The Robot task currently runs with a 10 ms delay, so real byte spacing can exceed 2 ms and a 128-byte payload takes about 1.28 seconds to send. The occupation estimate starts after that send and tail; `PLAYING` remains active throughout. Direct autonomous/raw APIs retain their existing blocking 2 ms per byte behavior; host-only mode suppresses their autonomous callers.

A request with the same latest accepted nonzero ID is idempotent: it reports the current state and does not replay. A different request received while the queue is occupied is retained as `BUSY` and can be retried after the estimate. The latest rejection is retained so a dropped USB response can be recovered on a later telemetry interval.

With `ROBOT_HOST_ONLY_SPEECH=1`, startup hello and autonomous RFID speech are disabled. Host TTS requests continue to be serviced.
