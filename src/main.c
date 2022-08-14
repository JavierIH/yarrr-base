#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>

#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#include "motor.h"

#define BATTERY_DIV         0.00236

volatile uint32_t ticks;

static void delay(uint32_t delta_ms) {
    uint32_t final_ms = ticks + delta_ms;
    while (ticks < final_ms)
        ;
}

static void init(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(71999);
    systick_interrupt_enable();
    systick_counter_enable();

    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

static void encoderInit(void) {
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
volatile float battery;

int main(void) {
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

    init();
    motor_init(&motor_r);
    motor_init(&motor_l);
    encoderInit();

    uint8_t channel_sequence[16];
    rcc_periph_clock_enable(RCC_ADC2);

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);

    channel_sequence[0] = ADC_CHANNEL9;
    adc_power_off(ADC2);
    adc_disable_scan_mode(ADC2);
    adc_set_single_conversion_mode(ADC2);
    adc_disable_external_trigger_regular(ADC2);
    adc_set_right_aligned(ADC2);
    adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_13DOT5CYC);
    adc_set_regular_sequence(ADC2, 1, channel_sequence);

    adc_power_on(ADC2);
    for (int i = 0; i < 800000; i++)
        __asm__("nop");
    adc_reset_calibration(ADC2);
    adc_calibrate(ADC2);

    motor_set_direction(&motor_l, 0);
    motor_set_direction(&motor_r, 0);
    motor_set_speed(&motor_l, 200);
    motor_set_speed(&motor_r, 200);
    motor_free(&motor_l);
    motor_free(&motor_r);

    while (1) {
        gpio_toggle(GPIOC, GPIO13);

        adc_start_conversion_direct(ADC2);
        while (!adc_eoc(ADC2))
            ;
        battery = adc_read_regular(ADC2) * BATTERY_DIV;

        pos1 = timer_get_counter(TIM3);
        pos2 = timer_get_counter(TIM2);
        delay(100);
    }
}

void sys_tick_handler(void) {
    ticks++;
}
