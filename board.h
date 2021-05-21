#pragma once

#define PIN_DETECT (1 << 1)
#define PIN_RELS (1 << 2)
#define PIN_RELR (1 << 3)
#define PIN_LEDA (1 << 4)
#define PIN_LEDB (1 << 5)
#define PIN_BTNA (1 << 6)
#define PIN_BTNB (1 << 7)
#define PINCTRL_BTNA PIN6CTRL
#define PINCTRL_BTNB PIN7CTRL

#define CURRENT_OFF_TOLERANCE 0x10

#define REGISTER_ADC 0
#define REGISTER_ISLIGHTON 1
#define REGISTER_COUNT 2