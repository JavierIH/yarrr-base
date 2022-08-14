#include "battery.h"

void battery_init(void) {
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
}

uint16_t battery_get_value(void) {
    adc_start_conversion_direct(ADC2);
    while (!adc_eoc(ADC2))
        ;
    return adc_read_regular(ADC2) * BATTERY_DIV;
}