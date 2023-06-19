#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
//#include <libopencm3/stm32/spi.h>

#include "yarrr/mavlink.h"

#include "motor.h"
#include "encoder.h"
#include "battery.h"
#include "serial.h"

// Motors PID settings
#define MOTOR_PID_KP        0.8
#define MOTOR_PID_KI        0.1
#define MOTOR_PID_KD        0
#define MOTOR_PID_RATE      200

// Messages rates 
#define ODOMETRY_RATE       50 //20hz


volatile uint32_t ticks;

static void delay(uint32_t delta_ms) {
    uint32_t final_ms = ticks + delta_ms; // TODO overflow each 50 days
    while (ticks < final_ms);
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
    
    uint32_t x=0;
    mavlink_message_t output_msg;
    mavlink_robot_encoders_t encoders_msg;
    uint8_t output_buffer[256];
    uint16_t len=0;

    while (1) {
        //battery = battery_get_value();
        //command_speed = 100;
        //serial_print("\n\rOK! ");        

        encoders_msg.total_left = encoder_get_value(&encoder_l);;
        encoders_msg.total_right = encoder_get_value(&encoder_r);;
        encoders_msg.delta_left = x;
        encoders_msg.delta_right = 444;
        mavlink_msg_robot_encoders_encode(1, 1, &output_msg, &encoders_msg);
        len = mavlink_msg_to_send_buffer(output_buffer, &output_msg);
        if(len>256){
            while(1);
        }
        serial_send(output_buffer, len);
        delay(500);
        x++;
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


static int process_msg(const mavlink_message_t* input_msg){
    switch (input_msg->msgid){
        case MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED:
        {
            mavlink_robot_motors_speed_t robot_motors_speed_msg;
            mavlink_msg_robot_motors_speed_decode(input_msg, &robot_motors_speed_msg);
            //motor_set_direction(&motor_l, 0);
            //motor_set_direction(&motor_r, 0);
            //motor_set_output(&motor_l, robot_motors_speed_msg.speed_left);
            //motor_set_output(&motor_r, robot_motors_speed_msg.speed_right);
            break;
        }
        case MAVLINK_MSG_ID_ROBOT_ENCODERS:
        {
            gpio_toggle(GPIOC, GPIO13);
            break;
        }

        default:
        {
            //Received unregistered message with ID [input_msg->msgid]
            break;
        }
    }
    return 1;
}

void serial_receiver(){
    static mavlink_message_t msg;
    static mavlink_status_t status;
    // Check if it was called because of RXNE
	if(((USART_CR1(USART3)&USART_CR1_RXNEIE)!=0) && ((USART_SR(USART3)&USART_SR_RXNE)!=0)){
		uint8_t data_in = usart_recv(USART3);
        uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, data_in, &msg, &status);
            if(result){
                process_msg(&msg);
            }
	}
}



static void motor_update_pid(Motor *motor, Encoder *encoder, uint16_t speed) { // speed in ticks/s
    int prev_pos = encoder->value;
    int delta_pos = encoder_get_value(encoder) - prev_pos; // ticks in lasts 200ms
    error_ref = error;
    error = speed - (1000 / MOTOR_PID_RATE) * delta_pos;      // speed in ticks/200ms
    int delta_error = error - error_ref;
    output += MOTOR_PID_KP*error + MOTOR_PID_KI*delta_error;

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
    /*static uint16_t task_tick = 1;

    switch (task_tick) {
    case MOTOR_PID_RATE:
        motor_update_pid(&motor_r, &encoder_r, command_speed);
        // motor_update_pid(&motor_l, &encoder_l, 100);
        break;
    default:
        break;
    }

    task_tick++;
    if (task_tick > MOTOR_PID_RATE)
        task_tick = 1;*/
}
