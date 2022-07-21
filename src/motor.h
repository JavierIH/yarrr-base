#ifndef MOTOR_H
#define MOTOR_H


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

//typedef enum {MOTOR_R, MOTOR_L, MOTOR_ALL} motor_t;
//typedef enum {FORWARD, BACKWARD, BRAKE, FREE} motor_sense_t;

typedef struct{
    uint16_t gpio_in1;
    uint32_t port_in1;
    uint16_t gpio_in2;
    uint32_t port_in2;
    uint16_t gpio_pwm;
    uint32_t port_pwm;
    uint32_t tim_pwm;
    enum tim_oc_id tim_oc;
    bool invert;
} Motor;

void motor_init(Motor *m);
void motor_set_speed(Motor *m, uint16_t speed);
void motor_set_direction(Motor *m, bool direction);
void motor_brake(Motor *m, uint16_t brake);
void motor_free(Motor *m);


#endif // MOTOR_H