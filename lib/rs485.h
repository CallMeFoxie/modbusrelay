#ifndef RS485_H_
#define RS485_H_

// supported messages are read inputs (2) read holding register (3) and write single register (6)
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

#define MODBUS_MAX_MSG_LENGTH 8

void init_rs485(uint8_t address);
unsigned char *get_message_if_ready();
void send_reply(unsigned char* data, uint8_t length);
uint8_t get_modbus_message_type(unsigned char* data);
uint16_t get_modbus_a(unsigned char* data);
uint16_t get_modbus_b(unsigned char* data);




#endif /* RS485_H_ */