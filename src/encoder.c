#include "encoder.h"


void encoder_init(Encoder *encoder) {
    // Timer
    rcc_periph_clock_enable(encoder->rcc_timer);
    timer_set_period(encoder->timer, 0xFFFFFFFF);
    timer_slave_set_mode(encoder->timer, TIM_SMCR_SMS_EM3);
    timer_ic_set_input(encoder->timer, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(encoder->timer, TIM_IC2, TIM_IC_IN_TI2);
    timer_enable_counter(encoder->timer);

    // Status
    encoder->value = 0;
}

uint32_t encoder_get_value(Encoder *encoder) {
    encoder->value = timer_get_counter(encoder->timer);
    return encoder->value;
}
