#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include "rs485.h"
#include "uart.h"
#include "../board.h"

uint8_t localModbusAddress;
volatile uint8_t modbusReceivingOffset;
volatile uint8_t messageRead = 1;

uint8_t (*write_holding_register)(volatile uint16_t[], uint16_t, uint16_t);

volatile uint16_t holding_registers[REGISTER_COUNT];

unsigned char lastModbusMessage[MODBUS_MAX_MSG_LENGTH];
unsigned char receivingModbusMessage[MODBUS_MAX_MSG_LENGTH];

volatile struct {
#ifdef DIAGNOSTICS_LISTEN_ONLY_MODE	
	unsigned char listen_only : 1;
#endif
	unsigned char allow_addr_change : 1;
} rs485_state;

#ifdef DIAGNOSTICS_STATISTICS
uint16_t bus_message_count, bus_exception_error_count, server_message_count;
#endif

ISR(TCA0_CMP0_vect) {
	if (/*modbusReceivingOffset == MODBUS_MAX_MSG_LENGTH &&*/ receivingModbusMessage[0] == localModbusAddress) {
#ifdef DIAGNOSTICS_STATISTICS
		server_message_count++;
#endif
		memcpy(lastModbusMessage, receivingModbusMessage, MODBUS_MAX_MSG_LENGTH);
		messageRead = 0;
	}
#ifdef DIAGNOSTICS_STATISTICS
	bus_message_count++;
#endif
	modbusReceivingOffset = 0;
	TCA0.SINGLE.INTFLAGS |= (1 << 4); // reset int flag
	TCA0.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESTART_gc;
}

ISR(USART0_RXC_vect) {
	USART0.STATUS |= USART_RXCIF_bm;
	TCA0.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESTART_gc;
	unsigned char received = receive_char();
	if (modbusReceivingOffset < 255)
		modbusReceivingOffset++;

	if (modbusReceivingOffset > MODBUS_MAX_MSG_LENGTH)
		return;
	
	receivingModbusMessage[modbusReceivingOffset - 1] = received;
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

void init_rs485(uint8_t (*write_holding_register_func)(volatile uint16_t[], uint16_t, uint16_t)) {
	localModbusAddress = eeprom_read_word(EEPROM_ADDRESS_RS485_SLAVEID);
	TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV256_gc;
	TCA0.SINGLE.CMP0 = 300;
	TCA0.SINGLE.INTCTRL = (1 << 4); // enable OVF
	
	init_uart();
	USART0.CTRLA |= (1 << 7);

	write_holding_register = write_holding_register_func;

	memset((void*)holding_registers, 0, sizeof(uint16_t) * REGISTER_COUNT);
}

unsigned char* get_message_if_ready() {
	if (!messageRead) {
		messageRead = 1;
		if (PORTA.OUT & PIN_LEDB != 0)
			PORTA.OUTCLR = PIN_LEDB;
		else
			PORTA.OUTSET = PIN_LEDB;
		return lastModbusMessage;
	} else {
		return NULL;
	}
}

void memcpy_and_swap(unsigned char* to, const unsigned char* from, uint8_t length) {
	for(uint8_t i = 0; i < length; i+=2) {
		to[i] = from[i+1];
		to[i+1] = from[i];
	}
}

void do_act() {
	unsigned char* msg = get_message_if_ready();
	if (msg != NULL) {
		uint16_t a = get_modbus_a(msg);
		uint16_t b = get_modbus_b(msg);
		unsigned char reply[5 + REGISTER_COUNT * 2]; // addr + func + (error/count) + CRC16.. max length
		uint8_t length;
		reply[1] = msg[1];
		switch (get_modbus_message_type(msg)) {
			case READ_HOLDING_REGISTERS: ;
				if(a > (REGISTER_COUNT - 1)) {
					length = create_modbus_error(reply, ILLEGAL_DATA_ADDRESS);
				} else if (a + b > REGISTER_COUNT) {
					length = create_modbus_error(reply, ILLEGAL_DATA_VALUE);
				} else {
					reply[2] = b * sizeof(uint16_t);
					memcpy_and_swap((void *)(reply + 3), (const void*)(holding_registers + a), b * sizeof(uint16_t));
					length = 5 + (b * sizeof(uint16_t));
				}
				send_reply(reply, length);
				break;
			case WRITE_SINGLE_REGISTER: ;
				uint8_t callback = write_holding_register(holding_registers, a, b);
				if (callback == 0) {
					send_reply(msg, MODBUS_MAX_MSG_LENGTH);
				} else {
					length = create_modbus_error(reply, callback);
					send_reply(reply, length);
				}
				break;
			case DIAGNOSTICS: ;
				// sub-function = a
				switch(msg[2]) {
#ifdef DIAGNOSTICS_SET_ADDRESS
					case SET_DEVICE_ADDRESS: ;
						if (rs485_state.allow_addr_change) {
							eeprom_write_byte(EEPROM_ADDRESS_RS485_SLAVEID, b & 0xFF);
							send_reply(msg, 7);
							localModbusAddress = b & 0xFF;
							rs485_state.allow_addr_change = 0;
						}
						break;
#endif
#ifdef DIAGNOSTICS_LISTEN_ONLY_MODE
					case FORCE_LISTEN_ONLY_MODE: ;
						rs485_state.listen_only = 1;
						break;
#endif
#ifdef DIAGNOSTICS_STATISTICS
					case RETURN_BUS_COMMUNICATION_EXCEPTION_COUNT: ;
						reply[2] = msg[2];
						reply[3] = bus_exception_error_count & 0xFF;
						reply[4] = bus_exception_error_count >> 8;
						send_reply(reply, 7);
						break;
					case RETURN_BUS_MESSAGE_COUNT: ;
						reply[2] = msg[2];
						reply[3] = bus_message_count & 0xFF;
						reply[4] = bus_message_count >> 8;
						send_reply(reply, 7);
						break;
					case RETURN_SERVER_MESSAGE_COUNT: ;
						reply[2] = msg[2];
						reply[3] = server_message_count & 0xFF;
						reply[4] = server_message_count >> 8;
						send_reply(reply, 7);
						break;
#endif
				}
				break;
			default: ;
				length = create_modbus_error(reply, ILLEGAL_FUNCTION);
				send_reply(reply, length);
				break;

		}
	}
}

void send_reply(unsigned char * data, uint8_t length) {
#ifdef DIAGNOSTICS_LISTEN_ONLY_MODE
	if (rs485_state.listen_only == 1) {
		return;
	}
#endif
	cli();
	data[0] = localModbusAddress;
	uint16_t crc = doModbusCRC16(data, length - 2);
	data[length - 1] = crc >> 8;
	data[length - 2] = crc & 0xFF;
	
	send_message(data, length);
	sei();
}

uint8_t get_modbus_message_type(unsigned char* data) {
	return data[1];
}

uint16_t get_modbus_a(unsigned char* data) {
	return (uint16_t)(data[3] | data[2] << 8);
}

uint16_t get_modbus_b(unsigned char* data) {
	return (uint16_t)(data[5] | data[4] << 8);
}

uint8_t create_modbus_error(unsigned char* buffer, uint8_t error) {
#ifdef DIAGNOSTICS_STATISTICS
	bus_exception_error_count++;
#endif
	buffer[1] |= 0x80;
	buffer[2] = error;
	return 5;
}

void write_holding_register_int(uint16_t addr, uint16_t val) {
	holding_registers[addr] = val;
}
uint16_t read_holding_register_int(uint16_t addr) {
	return holding_registers[addr];
}

void allow_addr_change() {
	rs485_state.allow_addr_change = 1;
}