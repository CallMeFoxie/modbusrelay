#define F_CPU 20000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lib/rs485.h"

#define PIN_DETECT (1 << 1)
#define PIN_RELS (1 << 2)
#define PIN_RELR (1 << 3)
#define PIN_LEDA (1 << 4)
#define PIN_LEDB (1 << 5)
#define PIN_BTNA (1 << 6)
#define PIN_BTNB (1 << 7)

#define LIGHT_ON 1
#define LIGHT_OFF 0

#define RELAY_LAST_SET 1
#define RELAY_LAST_RESET 0
#define RELAY_LAST_UNKNOWN 2

#define CURRENT_OFF_TOLERANCE 0x10

#define IS_LIGHT_ON ((holding.reg_adcsum < (0x3FF-CURRENT_OFF_TOLERANCE)) || (holding.reg_adcsum > (0x3FF+CURRENT_OFF_TOLERANCE)))

typedef struct holding_registers {
	uint16_t reg_adcsum;
	uint16_t reg_relay;
} holding_registers_t;

holding_registers_t holding;

uint16_t adcresults[8];
uint8_t adcmeasured = 0;

volatile struct {
	unsigned char light_status : 1;
	unsigned char relay_last_toggle : 2;
} node_state;

ISR(ADC0_RESRDY_vect) {
	adcresults[adcmeasured] = ADC0.RES;
	ADC0.INTFLAGS |= ADC_RESRDY_bm;
	adcmeasured = (adcmeasured + 1) % 8;
	holding.reg_adcsum = 0;
	for(uint8_t i = 0; i < 8; i++) {
		holding.reg_adcsum += adcresults[i];
	}
	holding.reg_adcsum /= 8;

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
		if(wasLightOn == IS_LIGHT_ON) { // we did not toggle anything
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
	if (PORTA.INTFLAGS & PIN_BTNA) {
		toggle_relay();
	}
	_delay_ms(10);

	PORTA.INTFLAGS = 0xFF;
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
	PORTA.PIN6CTRL = (1 << 7) | (1 << 3) | (3 << 0); // enable pullup + sense falling edge
	PORTA.PIN7CTRL = (1 << 7) | (1 << 3) | (3 << 0);

	
	// init other stuff
    init_rs485(2);
	
	// init LED ports & relay ports
	
	
	// init ADC
	ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV256_gc;
	ADC0.CTRLD = ADC_ASDV_ASVON_gc;
	ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
	ADC0.INTCTRL = ADC_RESRDY_bm;
	ADC0.CTRLA = ADC_FREERUN_bm | ADC_ENABLE_bm;
	ADC0.COMMAND = ADC_STCONV_bm;

	sei();
	
	// measure for a while to see whether light is currently on
	//while(adcmeasured != 7);
	
    /*while (1) 
    {
		/*unsigned char *msg = get_message_if_ready();
		if(msg != 0) {
			uint16_t a = get_modbus_a(msg);
			uint16_t b = get_modbus_b(msg);
			switch(msg[1]) {
				case READ_INPUT_REGISTERS:
					break;
				case WRITE_SINGLE_REGISTER:
					switch(a) {
						case 1:
							holding.reg_relay = b;
							// TODO toggle relay if needed
							break;
					}
					break;
				case READ_HOLDING_REGISTERS: ;
					unsigned char reply[9]; // addr + func + bytes + 2x16bit data + crc16
					reply[1] = READ_HOLDING_REGISTERS;
					reply[2] = 2;
					reply[3] = holding.reg_adcsum & 0xFF;
					reply[4] = holding.reg_adcsum >> 8;
					reply[5] = holding.reg_relay & 0xFF;
					reply[6] = holding.reg_relay >> 8;
					send_reply(reply, 9);
				
				}	break;
			}
		
		if(node_state.light_status) {
			PORTA.OUTSET = PIN_LEDA;
		} else {
			PORTA.OUTCLR = PIN_LEDA;
		}
		
		if(node_state.relay_last_toggle == RELAY_LAST_SET || node_state.relay_last_toggle == RELAY_LAST_UNKNOWN) {
			PORTA.OUTSET = PIN_LEDB;
		} else {
			PORTA.OUTCLR = PIN_LEDB;
		}
	}*/

	while(1) {
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

