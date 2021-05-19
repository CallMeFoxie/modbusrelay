#ifndef UART_H_
#define UART_H_
#define F_CPU 20000000UL
#define UART_BAUD_RATE(BAUD_RATE) ((float)F_CPU * 64 / (16 * (float)BAUD_RATE) + 0.5)

#define MAX_MESSAGE_LENGTH 12

unsigned char uart_in_msg[MAX_MESSAGE_LENGTH];

typedef unsigned char bool;

void init_uart();
void send_char(unsigned char);
void send_message(unsigned char*, uint8_t);
unsigned char receive_char();
unsigned char *receive_message();
void send_uint16(uint16_t num);
void send_uint8(uint8_t num);
bool is_char_ready();


#endif /* UART_H_ */