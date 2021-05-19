#include <avr/io.h>

#include "uart.h"

void init_uart() {
	PORTB.DIRSET = 1 << 0 | 1 << 2; // XDIR | TX
	
	USART0.CTRLA = 3; // RS485 mode auto drive
	
	USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
	USART0.BAUD = (uint16_t)UART_BAUD_RATE(9600);
}

unsigned char receive_char() {
	return USART0.RXDATAL;
}

void send_message(unsigned char* message, uint8_t length) {
	while(length--) {
		while(!(USART0.STATUS & USART_DREIF_bm));
		USART0.TXDATAL = *message++;
	}
}

void send_char(unsigned char data) {
	while(!(USART0.STATUS & USART_DREIF_bm));
	USART0.TXDATAL = data;
}


void send_nibble(uint8_t nibble) {
	if (nibble < 0x0A)
	nibble += '0';
	else
	nibble += 'A' - 0x0A;
	
	send_char(nibble);
}

void send_uint16(uint16_t num) {
	send_nibble((num >> 12) & 0xF);
	send_nibble((num >> 8) & 0xF);
	send_nibble((num >> 4) & 0xF);
	send_nibble(num & 0xF);
}

void send_uint8(uint8_t num) {
	send_nibble((num >> 4) & 0x0F);
	send_nibble(num & 0x0F);
}
