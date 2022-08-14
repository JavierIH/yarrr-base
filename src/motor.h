#ifndef MOTOR_H
#define MOTOR_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

//typedef enum {MOTOR_R, MOTOR_L, MOTOR_ALL} motor_t;
//typedef enum {FORWARD, BACKWARD, BRAKE, FREE} motor_sense_t;

typedef struct{
    // Hardware definitions
    uint16_t gpio_in1;
    uint32_t port_in1;
    uint16_t gpio_in2;
    uint32_t port_in2;
    uint16_t gpio_pwm;
    uint32_t port_pwm;
    uint32_t tim_pwm;
    enum tim_oc_id tim_oc;
    // Configuration
    bool invert;
    // Status
    uint16_t output;
    bool direction;
} Motor;

void motor_init(Motor *motor);
void motor_set_output(Motor *motor, uint16_t output);
uint16_t motor_get_output(Motor *motor);
void motor_update_pid(Motor *motor);
void motor_set_direction(Motor *motor, bool direction);
bool motor_get_direction(Motor *motor);
void motor_brake(Motor *motor, uint16_t brake);
void motor_stop(Motor *motor);


#endif // MOTOR_H