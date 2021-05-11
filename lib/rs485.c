#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "rs485.h"
#include "uart.h"

uint8_t localModbusAddress;
volatile uint8_t modbusReceivingOffset;
uint8_t messageRead = 1;


unsigned char lastModbusMessage[MODBUS_MAX_MSG_LENGTH];
unsigned char receivingModbusMessage[MODBUS_MAX_MSG_LENGTH];

ISR(TCA0_CMP0_vect) {
	if (modbusReceivingOffset == 8 && receivingModbusMessage[0] == localModbusAddress && (receivingModbusMessage[1] == READ_HOLDING_REGISTERS || receivingModbusMessage[1] == WRITE_SINGLE_REGISTER || receivingModbusMessage[1] == READ_INPUT_REGISTERS)) {
		memcpy(lastModbusMessage, receivingModbusMessage, MODBUS_MAX_MSG_LENGTH);
		messageRead = 0;
	}
	modbusReceivingOffset = 0;
	TCA0.SINGLE.INTFLAGS |= (1 << 4); // reset int flag
}

ISR(USART0_RXC_vect) {
	USART0.STATUS |= USART_RXCIF_bm;
	TCA0.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESET_gc;
	unsigned char received = receive_char();
	if (modbusReceivingOffset > 7)
		return;
	
	receivingModbusMessage[modbusReceivingOffset] = received;
	
	if (modbusReceivingOffset < 255)
		modbusReceivingOffset++;
}

uint16_t doModbusCRC16(unsigned char* data, uint8_t length) {
	uint16_t crc = 0xffff;
	for(; length > 0; length--) {
		crc = crc ^ (uint8_t)*data;
		for(uint_fast8_t j = 0; j < 8; j++) {
			if (crc & 0x0001) {
				crc >>= 1;
				crc ^= 0xa001;
			} else {
				crc >>= 1;
			}
		}
		data++;
	}
	return crc;
}

void init_rs485(uint8_t address) {
	localModbusAddress = address;
	TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV256_gc;
	TCA0.SINGLE.CMP0 = 27;
	
	init_uart();
}

unsigned char* get_message_if_ready() {
	if (!messageRead) {
		messageRead = 1;
		return lastModbusMessage;
	} else {
		return NULL;
	}
}

void send_reply(unsigned char * data, uint8_t length) {
	cli();
	data[0] = localModbusAddress;
	uint16_t crc = doModbusCRC16(data, length - 2);
	data[length - 2] = crc >> 8;
	data[length - 1] = crc & 0xFF;
	
	send_message(data, 8);
	sei();
}

uint8_t get_modbus_message_type(unsigned char* data) {
	return data[1];
}

uint16_t get_modbus_a(unsigned char* data) {
	return (uint16_t)(data[2] | data[3] << 8);
}

uint16_t get_modbus_b(unsigned char* data) {
	return (uint16_t)(data[4] | data[5] << 8);
}