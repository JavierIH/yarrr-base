#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>

#include "motor.h"
#include "encoder.h"
#include "battery.h"

volatile uint32_t ticks;

static void delay(uint32_t delta_ms) {
    uint32_t final_ms = ticks + delta_ms;
    while (ticks < final_ms)
        ;
}

static void init(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(71999); // 1ms
    systick_interrupt_enable();
    systick_counter_enable();

    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

volatile uint32_t pos1;
volatile uint32_t pos2;
volatile float battery;

Motor motor_l = (Motor){
    .gpio_in1 = GPIO9,
    .port_in1 = GPIOB,
    .gpio_in2 = GPIO8,
    .port_in2 = GPIOB,
    .gpio_pwm = GPIO7,
    .port_pwm = GPIOB,
    .tim_pwm = TIM4,
    .tim_oc = TIM_OC2,
    .invert = false};

Motor motor_r = (Motor){
    .gpio_in1 = GPIO12,
    .port_in1 = GPIOA,
    .gpio_in2 = GPIO5,
    .port_in2 = GPIOB,
    .gpio_pwm = GPIO6,
    .port_pwm = GPIOB,
    .tim_pwm = TIM4,
    .tim_oc = TIM_OC1,
    .invert = true};

Encoder encoder_l = (Encoder){
    .timer = TIM2,
    .rcc_timer = RCC_TIM2};

Encoder encoder_r = (Encoder){
    .timer = TIM3,
    .rcc_timer = RCC_TIM3};

int main(void) {

    init();
    motor_init(&motor_l);
    motor_init(&motor_r);
    encoder_init(&encoder_l);
    encoder_init(&encoder_r);
    battery_init();

    motor_set_direction(&motor_l, 0);
    motor_set_direction(&motor_r, 0);
    // motor_set_output(&motor_l, 200);
    // motor_set_output(&motor_r, 200);
    motor_stop(&motor_l);
    motor_stop(&motor_r);
    gpio_set(GPIOC, GPIO13);
    while (1) {
        // gpio_toggle(GPIOC, GPIO13);

        battery = battery_get_value();
        pos1 = encoder_get_value(&encoder_l);
        pos2 = encoder_get_value(&encoder_r);
        delay(1000);
    }
}

int motor_r_pos = 0;
int motor_r_prev_pos = 0;
int motor_r_delta_pos = 0;
int motor_r_error_pos = 0;
int motor_r_target_pos = 100;

int motor_l_pos = 0;
int motor_l_prev_pos = 0;
int motor_l_delta_pos = 0;
int motor_l_error_pos = 0;
int motor_l_target_pos = 100;
uint16_t speed = 0;

void sys_tick_handler(void) {
    ticks++;
    static uint16_t task_tick = 1;

    switch (task_tick) {
    case 100:
        break;
    case 200:
        // motor_update_pid(&motor_l);
        // motor_update_pid(&motor_r);

        motor_r_prev_pos = motor_r_pos;
        motor_r_pos = encoder_get_value(&encoder_r);
        motor_r_delta_pos = motor_r_pos - motor_r_prev_pos;
        motor_r_error_pos = motor_r_target_pos - motor_r_delta_pos;
        speed += 5 * motor_l_error_pos;
        motor_set_output(&motor_r, speed);

        break;
    default:
        break;
    }

    task_tick++;
    if (task_tick > 200)
        task_tick = 1;
}
