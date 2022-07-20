#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include "motor.h"

volatile uint32_t ticks;

static void delay(uint32_t delta_ms){
    uint32_t final_ms = ticks + delta_ms;
    while (ticks < final_ms);
}

static void init(void){
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(71999);
    systick_interrupt_enable();
    systick_counter_enable();

    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

static void encoderInit(void){
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_period(TIM2, 0xFFFFFFFF);
    timer_slave_set_mode(TIM2, TIM_SMCR_SMS_EM3);
    timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI2);
    timer_enable_counter(TIM2);

    rcc_periph_clock_enable(RCC_TIM3);
    timer_set_period(TIM3, 0xFFFFFFFF);
    timer_slave_set_mode(TIM3, TIM_SMCR_SMS_EM3);
    timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI2);
    timer_enable_counter(TIM3);
}


volatile uint32_t pos1;
volatile uint32_t pos2;

int main(void){
    Motor motor_l = (Motor){
        .gpio_in1 = GPIO8,
        .port_in1 = GPIOB,
        .gpio_in2 = GPIO9,
        .port_in2 = GPIOB,
        .gpio_pwm = GPIO7,
        .port_pwm = GPIOB,
        .tim_pwm = TIM4,
        .tim_oc = TIM_OC2,
    };
    Motor motor_r= (Motor){
        .gpio_in1 = GPIO5,
        .port_in1 = GPIOB,
        .gpio_in2 = GPIO12,
        .port_in2 = GPIOA,
        .gpio_pwm = GPIO6,
        .port_pwm = GPIOB,
        .tim_pwm = TIM4,
        .tim_oc = TIM_OC1,
    };

    init();
    motor_init(&motor_l);
    //motor_init(&motor_r);
    encoderInit();

    motor_set_direction(&motor_l, 0);
    motor_set_speed(&motor_l, 100);
    //motor_set_speed(&motor_r, 200);
    motor_free(&motor_l);

    while (1){
        gpio_toggle(GPIOC, GPIO13);
        pos1 = timer_get_counter(TIM3);
        pos2 = timer_get_counter(TIM2);
        delay(100);
    }
}

void sys_tick_handler(void){
    ticks++;
}