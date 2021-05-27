#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <string.h>
#include "lib/rs485.h"
#include "lib/uart.h"
#include "board.h"

#define LIGHT_ON 1
#define LIGHT_OFF 0

#define RELAY_LAST_SET 1
#define RELAY_LAST_RESET 0
#define RELAY_LAST_UNKNOWN 2

uint8_t is_light_on(uint16_t adcval) {
	return adcval < (0x200-CURRENT_OFF_TOLERANCE) || (adcval > (0x200 + CURRENT_OFF_TOLERANCE));
}

volatile struct {
	unsigned char relay_last_toggle : 2;
	unsigned char first_button_ignore : 1;
} node_state;

ISR(ADC0_RESRDY_vect) {
	ADC0.INTFLAGS |= ADC_RESRDY_bm;
	uint16_t adcval = ADC0.RES / 8;
	write_holding_register_int(REGISTER_ADC, adcval);

	if (is_light_on(adcval)) {
		write_holding_register_int(REGISTER_ISLIGHTON, LIGHT_ON);
	} else {
		write_holding_register_int(REGISTER_ISLIGHTON, LIGHT_OFF);
	}
}


void set_relay(uint8_t which) {
	PORTA.OUTCLR = PIN_RELS | PIN_RELR;
	_delay_ms(30);
	PORTA.OUTSET = which;
	node_state.relay_last_toggle = which == PIN_RELR ? RELAY_LAST_RESET : RELAY_LAST_SET;
	_delay_ms(30);
	PORTA.OUTCLR = which;
}

void toggle_relay() {
	if (node_state.relay_last_toggle == RELAY_LAST_UNKNOWN) {
		unsigned char wasLightOn = read_holding_register_int(REGISTER_ISLIGHTON);
		set_relay(PIN_RELR);
		_delay_ms(100);
		if (wasLightOn == read_holding_register_int(REGISTER_ISLIGHTON)) { // we did not toggle anything
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

uint8_t process_write_holding_register(volatile uint16_t holding_register[], uint16_t reg, uint16_t newValue) {
	if (reg == 1) {
		if (newValue != holding_register[REGISTER_ISLIGHTON]) {
			toggle_relay();
		}
		return NO_ERROR;
	}
	return ILLEGAL_DATA_ADDRESS;
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

	PORTA.PINCTRL_BTNA = PORT_INVEN_bm | PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc; // enable pullup + sense falling edge
	PORTA.PINCTRL_BTNB = PORT_INVEN_bm | PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

	// read bus address from eeprom
	
	node_state.first_button_ignore = 0;
	node_state.relay_last_toggle = RELAY_LAST_UNKNOWN;
 
	// init other stuff
	init_rs485(&process_write_holding_register);	
	
	// init ADC
	ADC0.CTRLA = ADC_FREERUN_bm | ADC_ENABLE_bm;
	ADC0.CTRLB = ADC_SAMPNUM_ACC8_gc;
	ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV256_gc;
	ADC0.CTRLD = ADC_ASDV_ASVON_gc;
	ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
	ADC0.INTCTRL = ADC_RESRDY_bm;
	
	ADC0.COMMAND = ADC_STCONV_bm;

	sei();

	while(1) {

		do_act();

		if (node_state.relay_last_toggle == RELAY_LAST_RESET) {
			PORTA.OUTCLR = PIN_LEDA;
		} else if (node_state.relay_last_toggle == RELAY_LAST_SET) {
			PORTA.OUTSET = PIN_LEDA;
		}

		if (read_holding_register_int(REGISTER_ISLIGHTON) == LIGHT_ON) {
			PORTA.OUTSET = PIN_LEDB;
		} else {
			PORTA.OUTCLR = PIN_LEDB;
		}
	}
}

