CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
LIBOPENCM3 ?= ./libopencm3

HOSTCC ?= gcc

GIT_VERSION := $(shell git describe --abbrev=8 --dirty --always --tags)

# Config bits
APP_ADDRESS = 0x20000000
USR_ADDRESS = 0x08001000

PLATFORM_DEFS = -DSTM32F1 -mthumb -mcpu=cortex-m3

CFLAGS = -O2 -std=c11 -Wall -pedantic -Werror -I../ \
	-ffunction-sections -fdata-sections -Wno-address-of-packed-member \
	-I$(LIBOPENCM3)/include -DAPP_ADDRESS=$(APP_ADDRESS)   \
	-ggdb -DVERSION=\"$(GIT_VERSION)\" -flto \
	$(PLATFORM_DEFS)

LDFLAGS = -lopencm3_stm32f1 -ggdb \
	-ffunction-sections -fdata-sections \
	-Wl,-Tstm32f103-ram.ld -nostartfiles -lnosys \
	-L$(LIBOPENCM3)/lib/ -Wl,-gc-sections -flto \
	$(PLATFORM_DEFS)

all:	payload.bin upgrader.bin client.exe

payload.elf: main.o | $(LIBOPENCM3)/lib/libopencm3_stm32f1.a
	$(CC) $^ -o $@ $(LDFLAGS) -Wl,-Ttext=$(APP_ADDRESS) -Wl,-Map,payload.map

upgrader.elf:	payload.bin bundle.S
	$(CC) -o $@ bundle.S -mthumb -mcpu=cortex-m3 -nostdlib -fPIC -Wl,-Tstm32f103-rom.ld -L$(LIBOPENCM3)/lib/

$(LIBOPENCM3)/lib/libopencm3_stm32f1.a:
	$(MAKE) -C $(LIBOPENCM3) TARGETS=stm32/f1 AR=$(CC)-ar CFLAGS=-flto LDFLAGS=-flto

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@
	python3 ../checksum.py $@

%.o: %.c $(LIBOPENCM3)/lib/libopencm3_stm32f1.a
	$(CC) -c $< -o $@ $(CFLAGS)

clean:
	-rm -f *.elf *.o *.bin *.map *.exe

client.exe:	client.c
	$(HOSTCC) -o client.exe client.c -lusb-1.0 -ggdb

