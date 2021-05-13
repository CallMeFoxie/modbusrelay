CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=20000000UL -mmcu=attiny414 -D__AVR_DEV_LIB_NAME__=tn414 -fdata-sections -ffunction-sections
LDFLAGS=-Os -mmcu=attiny414 -Wl,--gc-sections
ifneq ($(origin PACK_PATH),undefined)
	CFLAGS+=-B $(PACK_PATH)
endif

.PHONY: app.elf

default: app.elf

lib/uart.o: lib/uart.c
	$(CC) -C -o lib/uart.o -c lib/uart.c $(CFLAGS)

lib/rs485.o: lib/rs485.c
	$(CC) -C -o lib/rs485.o -c lib/rs485.c $(CFLAGS)

main.o: main.c
	$(CC) -C -o main.o -c main.c $(CFLAGS)

app.elf: lib/uart.o lib/rs485.o main.o
	$(CC) -o app.elf lib/uart.o lib/rs485.o main.o $(LDFLAGS)
	avr-size  --format=avr app.elf

flash.hex: app.elf
	$(OBJCOPY) -O ihex app.elf flash.hex

do-flash: flash.hex
	pyupdi -d attiny414 -c $(PORT) -f flash.hex

do-erase:
	pyupdi -d attiny414 -c $(PORT) -e

do-reset:
	pyupdi -d attiny414 -c $(PORT) -r

do-disassemble: app.elf
	avr-objdump -d app.elf

clean:
	rm -rf lib/*.o *.o *.elf *.hex
