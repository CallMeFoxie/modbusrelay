CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=20000000UL -mmcu=attiny414 -D__AVR_DEV_LIB_NAME__=tn414 -fdata-sections -ffunction-sections -Werror
LDFLAGS=-Os -mmcu=attiny414 -Wl,--gc-sections
ifneq ($(origin PACK_PATH),undefined)
	CFLAGS+=-B $(PACK_PATH)
endif

.PHONY: relayboard.elf

default: relayboard.elf

lib/uart.o: lib/uart.c lib/uart.c
	$(CC) -C -o lib/uart.o -c lib/uart.c $(CFLAGS)

lib/rs485.o: lib/rs485.c lib/rs485.h lib/uart.h board.h
	$(CC) -C -o lib/rs485.o -c lib/rs485.c $(CFLAGS)

boards/relayboard.o: boards/relayboard.c lib/rs485.h lib/uart.h board.h
	$(CC) -C -o boards/relayboard.o -c boards/relayboard.c $(CFLAGS)

relayboard.elf: lib/uart.o lib/rs485.o boards/relayboard.o
	$(CC) -o relayboard.elf lib/uart.o lib/rs485.o boards/relayboard.o $(LDFLAGS)
	avr-size  --format=avr relayboard.elf

relayboard.hex: relayboard.elf
	$(OBJCOPY) -O ihex app.elf flash.hex

do-flash-relayboard: relayboard.hex
	pyupdi -d attiny414 -c $(PORT) -f flash.hex

do-erase:
	pyupdi -d attiny414 -c $(PORT) -e

do-reset:
	pyupdi -d attiny414 -c $(PORT) -r

do-disassemble-relayboard: relayboard.elf
	avr-objdump -d relayboard.elf

clean:
	rm -rf lib/*.o boards/*.o *.o *.elf *.hex
