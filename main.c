#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "lib/rs485.h"
#include "lib/uart.h"

#define PIN_DETECT (1 << 1)
#define PIN_RELS (1 << 2)
#define PIN_RELR (1 << 3)
#define PIN_LEDA (1 << 4)
#define PIN_LEDB (1 << 5)
#define PIN_BTNA (1 << 6)
#define PIN_BTNB (1 << 7)
#define PINCTRL_BTNA PIN6CTRL
#define PINCTRL_BTNB PIN7CTRL

#define LIGHT_ON 1
#define LIGHT_OFF 0

#define RELAY_LAST_SET 1
#define RELAY_LAST_RESET 0
#define RELAY_LAST_UNKNOWN 2

#define CURRENT_OFF_TOLERANCE 0x10

#define IS_LIGHT_ON ((adcmeasured < (0x200-CURRENT_OFF_TOLERANCE)) || (adcmeasured > (0x200+CURRENT_OFF_TOLERANCE))) && adcmeasured > CURRENT_OFF_TOLERANCE

uint16_t adcmeasured = 0;

volatile struct {
	unsigned char light_status : 1;
	unsigned char relay_last_toggle : 2;
	unsigned char first_button_ignore : 1;
} node_state;

ISR(ADC0_RESRDY_vect) {
	ADC0.INTFLAGS |= ADC_RESRDY_bm;
	adcmeasured = ADC0.RES / 8 ;

	if (IS_LIGHT_ON) {
		node_state.light_status = LIGHT_ON;
	} else {
		node_state.light_status = LIGHT_OFF;
	}
}


void set_relay(uint8_t which) {
	PORTA.OUTCLR = PIN_RELS | PIN_RELR;
	_delay_ms(10);
	PORTA.OUTSET = which;
	node_state.relay_last_toggle = which == PIN_RELR ? RELAY_LAST_RESET : RELAY_LAST_SET;
	_delay_ms(10);
	PORTA.OUTCLR = which;
}

void toggle_relay() {
	if (node_state.relay_last_toggle == RELAY_LAST_UNKNOWN) {
		unsigned char wasLightOn = IS_LIGHT_ON;
		set_relay(PIN_RELR);
		_delay_ms(100);
		if (wasLightOn == IS_LIGHT_ON) { // we did not toggle anything
			set_relay(PIN_RELS);
		}
	} else {
		if (node_state.relay_last_toggle == RELAY_LAST_SET) {
			set_relay(PIN_RELR);
		} else {
			set_relay(PIN_RELS);
		}
	}
}

ISR(PORTA_PORT_vect) {
	// soft debounce
	_delay_ms(10);
	if (PORTA.INTFLAGS & PIN_BTNA && node_state.first_button_ignore == 1) {
		toggle_relay();
	}

	PORTA.INTFLAGS = 0xFF;

	node_state.first_button_ignore = 1;
}

int main(void)
{
	// init clock. 20MHz ought to do it
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLA = 0x00;
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = 0x00;
	PORTA.DIRSET = PIN_LEDA | PIN_LEDB | PIN_RELS | PIN_RELR;
	PORTA.DIRCLR = PIN_BTNA | PIN_BTNB;
	//PORTA.OUTSET = PIN_BTNA | PIN_BTNB; // enable pullups
	PORTA.PINCTRL_BTNA = PORT_INVEN_bm | PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc; // enable pullup + sense falling edge
	PORTA.PINCTRL_BTNB = PORT_INVEN_bm | PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

	// read bus address from eeprom
	uint16_t rs485_address = eeprom_read_word(0);
	node_state.first_button_ignore = 0;
	node_state.relay_last_toggle = RELAY_LAST_UNKNOWN;
 
	// init other stuff
    init_rs485(rs485_address);	
	
	// init ADC
	ADC0.CTRLA = ADC_FREERUN_bm | ADC_ENABLE_bm;
	ADC0.CTRLB = (3 << 0); // ACC8
	ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV256_gc;
	ADC0.CTRLD = ADC_ASDV_ASVON_gc;
	ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
	ADC0.INTCTRL = ADC_RESRDY_bm;
	
	ADC0.COMMAND = ADC_STCONV_bm;

	sei();

	while(1) {
		unsigned char *msg = get_message_if_ready();
		if(msg != 0) {
			uint16_t a = get_modbus_a(msg);
			uint16_t b = get_modbus_b(msg);
			unsigned char reply[9]; // addr + func + bytes + 16bit data + crc16
			reply[1] = msg[1];
			switch(msg[1]) {
				case READ_INPUT_REGISTERS:
					break;
				case WRITE_SINGLE_REGISTER:
					if (b && !IS_LIGHT_ON)
						toggle_relay();
					else if (!b && IS_LIGHT_ON)
						toggle_relay();

					reply[2] = a >> 8;
					reply[3] = a & 0xFF;
					reply[4] = b >> 8;
					reply[5] = b & 0xFF;
					send_reply(reply, 8);
					break;
				case READ_HOLDING_REGISTERS: ;
					reply[2] = 4;
					reply[4] = adcmeasured & 0xFF;
					reply[3] = adcmeasured >> 8;
					reply[6] = IS_LIGHT_ON;
					reply[5] = 0;
					send_reply(reply, 9);
					break;
				default: ;
					reply[2] = 0xaa;
					reply[3] = 0x55;
					reply[4] = 0xaa;
					send_reply(reply, 8);
					break;
			}
		}

		if (node_state.relay_last_toggle == RELAY_LAST_RESET) {
			PORTA.OUTCLR = PIN_LEDA;
		} else if (node_state.relay_last_toggle == RELAY_LAST_SET) {
			PORTA.OUTSET = PIN_LEDA;
		}

		if (node_state.light_status == LIGHT_ON) {
			PORTA.OUTSET = PIN_LEDB;
		} else {
			PORTA.OUTCLR = PIN_LEDB;
		}
	}
}

