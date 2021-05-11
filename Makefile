CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=20000000UL -mmcu=attiny414 -D__AVR_DEV_LIB_NAME__=tn414

.PHONY: flash.hex

default: flash.hex

lib/uart.o: lib/uart.c
	$(CC) -C -o lib/uart.o -c lib/uart.c $(CFLAGS)

lib/rs485.o: lib/rs485.c
	$(CC) -C -o lib/rs485.o -c lib/rs485.c $(CFLAGS)

app.elf: lib/uart.o lib/rs485.o main.o
	$(CC) -o app.elf lib/uart.o lib/rs485.o main.o $(CFLAGS)

flash.hex: app.elf
	$(OBJCOPY) -O ihex app.elf flash.hex

clean:
	rm -rf lib/*.o *.o *.elf *.hex


