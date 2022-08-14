#include "motor.h"

void motor_init(Motor *motor) {
    // GPIOs
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(motor->port_in1, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, motor->gpio_in1);
    gpio_set_mode(motor->port_in2, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, motor->gpio_in2);

    // PWM
    rcc_periph_clock_enable(RCC_TIM4);
    gpio_set_mode(motor->port_pwm, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, motor->gpio_pwm);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, 0);
    timer_set_period(TIM4, 1023);
    timer_disable_preload(TIM4);
    timer_continuous_mode(TIM4);
    timer_set_oc_mode(TIM4, motor->tim_oc, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM4, motor->tim_oc);
    timer_enable_counter(TIM4);

    // Status
    motor->direction = 0;
    motor->output = 0;
}

void motor_set_output(Motor *motor, uint16_t output) {
    if (output > 1023)
        output = 1023;
    timer_set_oc_value(TIM4, motor->tim_oc, output);
    motor->output = output;
}

uint16_t motor_get_output(Motor *motor) {
    return motor->output;
}

void motor_set_direction(Motor *motor, bool direction) {
    if (direction == motor->invert) {
        gpio_set(motor->port_in1, motor->gpio_in1);
        gpio_clear(motor->port_in2, motor->gpio_in2);
    } else {
        gpio_clear(motor->port_in1, motor->gpio_in1);
        gpio_set(motor->port_in2, motor->gpio_in2);
    }
    motor->direction = direction;
}

bool motor_get_direction(Motor *motor) {
    return motor->direction;
}

void motor_brake(Motor *motor, uint16_t brake) {
    gpio_set(motor->port_in1, motor->gpio_in1);
    gpio_set(motor->port_in2, motor->gpio_in2);
    if (brake > 1023)
        brake = 1023;
    timer_set_oc_value(motor->tim_pwm, motor->tim_oc, brake);
}

void motor_stop(Motor *motor) {
    gpio_clear(motor->port_in1, motor->gpio_in1);
    gpio_clear(motor->port_in2, motor->gpio_in2);
}