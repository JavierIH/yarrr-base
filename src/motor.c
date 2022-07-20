#include "motor.h"


void motor_init(Motor *m){
    // GPIOs
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(m->port_in1, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, m->gpio_in1);
    gpio_set_mode(m->port_in2, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, m->gpio_in2);

    // PWMs
    rcc_periph_clock_enable(RCC_TIM4);
    gpio_set_mode(m->port_pwm, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, m->gpio_pwm);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, 0);
    timer_set_period(TIM4, 1023);
    timer_disable_preload(TIM4);
    timer_continuous_mode(TIM4);
    timer_set_oc_mode(TIM4, m->tim_oc, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM4, m->tim_oc);
    timer_enable_counter(TIM4);
}

void motor_set_speed(Motor *m, uint16_t speed){
    if (speed > 1023)
        speed = 1023;
    timer_set_oc_value(TIM4, m->tim_oc, speed);
}

void motor_set_direction(Motor *m, bool direction){
    if (direction){
        gpio_set(m->port_in1, m->gpio_in1);
        gpio_clear(m->port_in2, m->gpio_in2);
    }
    else{
        gpio_clear(m->port_in1, m->gpio_in1);
        gpio_set(m->port_in2, m->gpio_in2);
    }
}

void motor_brake(Motor *m, uint16_t brake){
    gpio_set(m->port_in1, m->gpio_in1);
    gpio_set(m->port_in2, m->gpio_in2);
    if (brake > 1023)
        brake = 1023;
    timer_set_oc_value(m->tim_pwm, m->tim_oc, brake);
}

void motor_free(Motor *m){
    gpio_clear(m->port_in1, m->gpio_in1);
    gpio_clear(m->port_in2, m->gpio_in2);
}