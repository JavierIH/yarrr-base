#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
//#include <libopencm3/stm32/spi.h>
//#include <libopencm3/stm32/usart.h>

#include "yarrr/mavlink.h"

#include "motor.h"
#include "encoder.h"
#include "battery.h"
#include "serial.h"

#define MOTOR_KP 0.8
#define MOTOR_KI 0.1
#define MOTOR_KD 0
#define SAMPLE_RATE 200

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

uint16_t command_speed;

int main(void) {

    init();
    serial_init(115200);
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
    // motor_stop(&motor_r);
    
    mavlink_message_t msg;
    uint16_t len;
    uint8_t buf[1024];
    int x=0;

    while (1) {
        gpio_toggle(GPIOC, GPIO13);

        battery = battery_get_value();
        pos1 = encoder_get_value(&encoder_l);
        pos2 = encoder_get_value(&encoder_r);
        //command_speed = 100;
        //serial_print(" OK! \n");        
        delay(100);

        int ret = mavlink_msg_robot_encoders_pack(2, 3, &msg, 4, 123, 456, 78, -90);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        serial_send(buf, len);

        //char str[20];
        //sprintf(str, " [len: %d]", len);
        //serial_print(str);
    }
}

/*int motor_r_pos = 0;
int motor_r_prev_pos = 0;
int motor_r_delta_pos = 0;
int motor_r_error_pos = 0;
int motor_r_target_pos = 100;

int motor_l_pos = 0;
int motor_l_prev_pos = 0;
int motor_l_delta_pos = 0;
int motor_l_error_pos = 0;
int motor_l_target_pos = 100;*/
int output = 0;
int error;
int error_ref;

static void motor_update_pid(Motor *motor, Encoder *encoder, uint16_t speed) { // speed in ticks/s
    int prev_pos = encoder->value;
    int delta_pos = encoder_get_value(encoder) - prev_pos; // ticks in lasts 200ms
    error_ref = error;
    error = speed - (1000 / SAMPLE_RATE) * delta_pos;      // speed in ticks/200ms
    int delta_error = error - error_ref;
    output += MOTOR_KP*error + MOTOR_KI*delta_error;

    // visual check of the settling time
    //if (error == 0)
    //    gpio_clear(GPIOC, GPIO13);
    //else
    //    gpio_set(GPIOC, GPIO13);

    if (output < 0) output = 0;

    motor_set_output(motor, output);
}

void sys_tick_handler(void) {
    ticks++;
    static uint16_t task_tick = 1;

    switch (task_tick) {
    case SAMPLE_RATE:
        motor_update_pid(&motor_r, &encoder_r, command_speed);
        // motor_update_pid(&motor_l, &encoder_l, 100);
        break;
    default:
        break;
    }

    task_tick++;
    if (task_tick > SAMPLE_RATE)
        task_tick = 1;
}
