#ifndef SERIAL_H
#define SERIAL_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

void serial_init(uint32_t baudrate);
void serial_print(char *s);
void serial_send(uint8_t *buf, uint16_t len);
void serial_send_ch(uint8_t ch);

#endif // SERIAL_H