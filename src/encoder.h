#ifndef ENCODER_H
#define ENCODER_H

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>


typedef struct {
    // Hardware definitions
    uint32_t timer;
    enum rcc_periph_clken rcc_timer;
    // Status
    uint32_t value;

} Encoder;

void encoder_init(Encoder *encoder);
uint32_t encoder_get_value(Encoder *encoder);

#endif // ENCODER_H