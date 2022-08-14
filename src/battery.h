#ifndef BATTERY_H
#define BATTERY_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#define BATTERY_DIV 2.36

void battery_init(void);
uint16_t battery_get_value(void);

#endif // BATTERY_H