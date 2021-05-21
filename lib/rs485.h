#ifndef RS485_H_
#define RS485_H_

// supported messages are read holding register (3) and write single register (6)
enum MODBUS_FUNCTION {
	READ_COILS = 0x01,
	READ_INPUTS = 0x02,
	READ_HOLDING_REGISTERS = 0x03,
	READ_INPUT_REGISTERS = 0x04,
	WRITE_SINGLE_COIL = 0x05,
	WRITE_SINGLE_REGISTER = 0x06,
	READ_EXCEPTION_STATUS = 0x07,
	DIAGNOSTICS = 0x08
	// do not care for more atm
};

enum MODBUS_EXCEPTIONS {
	NO_ERROR = 0x00,
	ILLEGAL_FUNCTION = 0x01,
	ILLEGAL_DATA_ADDRESS = 0x02,
	ILLEGAL_DATA_VALUE = 0x03,
	SLAVE_DEVICE_FAILURE = 0x04,
	ACKNOWLEDGE = 0x05,
	SLAVE_DEVICE_BUSY = 0x06,
	NEGATIVE_ACKNOWLEDGE = 0x07,
	MEMORY_PARITY_ERROR = 0x08,
	GATEWAY_PATH_UNAVAILABLE = 0x0A,
	GATEWAY_DEVICE_FAILED_TO_RESPOND = 0x0B
};

#define MODBUS_MAX_MSG_LENGTH 8

void init_rs485(uint8_t address, uint8_t (*write_holding_register_func)(volatile uint16_t[], uint16_t, uint16_t));
unsigned char *get_message_if_ready();
void send_reply(unsigned char* data, uint8_t length);
uint8_t get_modbus_message_type(unsigned char* data);
uint16_t get_modbus_a(unsigned char* data);
uint16_t get_modbus_b(unsigned char* data);
uint8_t create_modbus_error(unsigned char* buffer, uint8_t error);
void do_act();
void memcpy_and_swap(unsigned char* to, const unsigned char* from, uint8_t length);
void write_holding_register_int(uint16_t addr, uint16_t val);
uint16_t read_holding_register_int(uint16_t addr);

#endif /* RS485_H_ */