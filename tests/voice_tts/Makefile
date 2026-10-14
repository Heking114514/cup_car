CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O2
ROOT := ../..
TARGET := $(ROOT)/build/test_voice_tts

.PHONY: test
test: $(TARGET)
	$(TARGET)

$(TARGET): test_voice_tts.c $(ROOT)/modules/voice/voice_tts.c $(ROOT)/modules/voice/voice_tts.h $(wildcard stubs/*.h)
	mkdir -p $(ROOT)/build
	$(CC) $(CFLAGS) -Istubs -I$(ROOT)/modules/voice test_voice_tts.c $(ROOT)/modules/voice/voice_tts.c -o $@
