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